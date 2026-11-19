#include "ClockSynchronizer.h"

#include "utils/simulator.h"
#include "utils/vsource.h"
#include "utils/verification.h"
#include "utils/nlohmann/json.hpp"

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

// ============================================================
// 边沿记录：按线性插值记录通道的 0V（50%）、20%、80% 穿越时刻，以及电压极值
// ============================================================
SCA_TDF_MODULE(EdgeProbe)
{
    sca_tdf::sca_in<double> in;

    EdgeProbe(sc_core::sc_module_name nm, double vod)
        : in("in"), m_v20(-0.6 * vod), m_v80(0.6 * vod) {}

    void processing() override{
        const double t = get_time().to_seconds();
        const double v = in.read();
        if (m_has_prev){
            cross(t, v, 0.0,   rise50, fall50);
            cross(t, v, m_v20, rise20, fall20);
            cross(t, v, m_v80, rise80, fall80);
        }
        vmax = std::max(vmax, v);
        vmin = std::min(vmin, v);
        m_tp = t; m_vp = v; m_has_prev = true;
    }

    std::vector<double> rise50, fall50, rise20, fall20, rise80, fall80;
    double vmax = -1e9, vmin = 1e9;

private:
    double m_v20, m_v80;
    bool m_has_prev = false;
    double m_tp = 0.0, m_vp = 0.0;

    void cross(double t, double v, double th, std::vector<double>& up, std::vector<double>& dn){
        if ((m_vp < th) == (v < th)) return;
        const double tc = m_tp + (th - m_vp) / (v - m_vp) * (t - m_tp);
        (m_vp < th ? up : dn).push_back(tc);
    }
};

// ============================================================
// ClockSynchronizer 功能测试：输出波形验证（1 ps 时间步，理想锁定）
//   REFIN：100 MHz 差分方波（±350 mV）
//   0. 模型信息接口 name / version / latency / power / area
//   1. 频率（= fREFIN × NDIV / (RDIV × ODIV)）、占空比、差分摆幅、上升/下降时间（20%~80%）、抖动
//   2. 延迟步进：通道 1 依次加 1 个粗延迟码、10 个细延迟码，测量相对通道 0 的时差变化
//   仿真时长 21 us，远小于一次 TDC 测量（2.6 ms），对齐环路尚未调整任何延迟
// ============================================================
SC_MODULE(Simulator)
    , public AbstractSimulator
{
public:
    sca_tdf::sca_signal<double> refin;
    sc_core::sc_vector<sca_tdf::sca_signal<double>> bsync;

    SC_HAS_PROCESS(Simulator);
    explicit Simulator(sc_core::sc_module_name name)
        : sc_module(name)
        , m_fail(0)
    {
        const char* json = "../../json/ClockSynchronizer.json";
        load_parameters(json);

        const sc_core::sc_time dt(1, sc_core::SC_PS);
        ClockSynchronizer::configs cfgs = ClockSynchronizer::get_configs_from_json(json);
        m_sync = new ClockSynchronizer(cfgs.module_name.c_str(), cfgs, dt, false);
        m_fexp = REFIN_HZ * cfgs.ndiv / (cfgs.rdiv * cfgs.odiv);

        m_refin = new PULSE("refin", -0.35, 0.35, 0.0, 1e-12, 1e-12, 0.5 / REFIN_HZ, 1.0 / REFIN_HZ, 0, dt);
        m_refin->out(refin);
        m_sync->refin(refin);

        bsync.init(cfgs.channels);
        for (int i = 0; i < cfgs.channels; ++i)
            m_sync->bsync[i](bsync[i]);

        m_probe0 = new EdgeProbe("probe0", cfgs.vod_mv * 1e-3);
        m_probe1 = new EdgeProbe("probe1", cfgs.vod_mv * 1e-3);
        m_probe0->in(bsync[0]);
        m_probe1->in(bsync[1]);

        m_tf = sca_util::sca_create_tabular_trace_file("waveform.dat");
        m_sync->trace_me(m_tf);

        SC_THREAD(testbench);
    }

    ~Simulator(){
        sca_util::sca_close_tabular_trace_file(m_tf);
        delete m_sync;
        delete m_refin;
        delete m_probe0;
        delete m_probe1;
    }

private:
    size_t m_fail;
    sca_util::sca_trace_file* m_tf;
    ClockSynchronizer* m_sync;
    PULSE* m_refin;
    EdgeProbe* m_probe0;
    EdgeProbe* m_probe1;

    static constexpr double REFIN_HZ = 100e6;  // REFIN 频率

    double m_fexp;          // 期望的 BSYNC 频率

    // json 中的手册期望值
    double m_tr_ps, m_jitter_ps, m_vod_mv, m_fine_ps, m_coarse_ps, m_duty;

    // json 为扁平格式：手册参数为顶层字段，取 value 或 valid_range
    static double value(const nlohmann::json& j, const char* key){
        return j.at(key).at("value").get<double>();
    }
    static double range(const nlohmann::json& j, const char* key, int idx){
        return j.at(key).at("valid_range").at(idx).get<double>();
    }

    void load_parameters(const char* path){
        std::ifstream f(path);
        const nlohmann::json p = nlohmann::json::parse(f);
        m_tr_ps     = value(p, "bsync-rise-fall-time-20-80");
        m_jitter_ps = value(p, "bsync-rms-jitter");
        m_vod_mv    = value(p, "bsync-differential-output-swing");
        m_fine_ps   = value(p, "fine-delay-step");
        m_coarse_ps = value(p, "intermediate-delay-step");
        m_duty      = value(p, "bsync-duty-cycle");
    }

    static double mean(const std::vector<double>& v){
        double s = 0.0;
        for (double x : v) s += x;
        return v.empty() ? 0.0 : s / v.size();
    }

    static double stddev(const std::vector<double>& v){
        const double m = mean(v);
        double s = 0.0;
        for (double x : v) s += (x - m) * (x - m);
        return v.size() > 1 ? std::sqrt(s / v.size()) : 0.0;
    }

    // 对 end 中的每个时刻，找 start 中在它之前且最近的时刻，返回两者之差
    static std::vector<double> paired(const std::vector<double>& start, const std::vector<double>& end){
        std::vector<double> r;
        size_t j = 0;
        for (double te : end){
            while (j + 1 < start.size() && start[j + 1] < te) ++j;
            if (j < start.size() && start[j] < te) r.push_back(te - start[j]);
        }
        return r;
    }

    // [t0, t1) 内的元素
    static std::vector<double> window(const std::vector<double>& v, double t0, double t1){
        std::vector<double> r;
        for (double x : v) if (x >= t0 && x < t1) r.push_back(x);
        return r;
    }

    // 通道 1 相对通道 0 的平均上升沿时差（s），以 ref 为中心折算，避免跨周期跳变
    double mean_skew(double t0, double t1, double ref) const{
        const double T = 1.0 / m_sync->bsync_frequency();
        const std::vector<double> a = window(m_probe0->rise50, t0, t1);
        const std::vector<double> b = window(m_probe1->rise50, t0, t1);
        std::vector<double> d;
        size_t j = 0;
        for (double tb : b){
            while (j + 1 < a.size() && std::fabs(a[j + 1] - tb) < std::fabs(a[j] - tb)) ++j;
            if (a.empty()) break;
            double x = tb - a[j] - ref;
            x -= T * std::floor(x / T + 0.5);
            d.push_back(x + ref);
        }
        return mean(d);
    }

    void testbench(){
        using sc_core::sc_time;
        using sc_core::SC_US;
        using sc_core::SC_NS;

        {
            info("模型信息接口");
            info((std::string("name: ") + m_sync->get_name()).c_str());
            check("version", m_sync->get_version(), 0.1, 0, "", m_fail);
            check("latency（一次 TDC 测量时间）", m_sync->get_latency(), 2.62144, 0, "ms", m_fail);
            check("power", m_sync->get_power(), 2300, 0, "mW", m_fail);
            check("area", m_sync->get_area(), 49, 0, "mm2", m_fail);
        }

        // 只记录锁定后 200 ns 的波形，避免波形文件过大
        m_tf->disable();
        while (!m_sync->locked())
            wait(sc_time(100, SC_NS));
        info("已锁定，开始输出");
        m_tf->enable();
        wait(sc_time(200, SC_NS));
        m_tf->disable();
        const double T = 1.0 / m_sync->bsync_frequency();

        // ---- 1. 波形参数（通道 0，0 ~ 10 us）----
        wait(sc_time(10, SC_US) - sc_core::sc_time_stamp());
        info("输出波形参数（通道 0）");
        {
            const std::vector<double> r = window(m_probe0->rise50, 0.0, 10e-6);
            const std::vector<double> f = window(m_probe0->fall50, 0.0, 10e-6);

            // 频率：上升沿平均间隔
            const double fout = (r.size() - 1) / (r.back() - r.front());
            check("BSYNC 频率", fout / 1e6, m_fexp / 1e6, 0.01, "MHz", m_fail);

            // 占空比：高电平时间 / 周期
            std::vector<double> high;
            for (size_t i = 0, j = 0; i < r.size(); ++i){
                while (j < f.size() && f[j] <= r[i]) ++j;
                if (j < f.size()) high.push_back(f[j] - r[i]);
            }
            check("占空比", mean(high) / T * 100.0, m_duty, 0.5, "%", m_fail);

            // 差分摆幅
            check("差分输出幅度（正）", m_probe0->vmax * 1e3,  m_vod_mv, 1.0, "mV", m_fail);
            check("差分输出幅度（负）", m_probe0->vmin * 1e3, -m_vod_mv, 1.0, "mV", m_fail);

            // 上升/下降时间：同一边沿 20% 与 80% 穿越的时间差
            //   每个结束穿越配对它之前最近的起始穿越，不依赖两组穿越的序号对齐
            const std::vector<double> tr = paired(m_probe0->rise20, m_probe0->rise80);
            const std::vector<double> tf = paired(m_probe0->fall80, m_probe0->fall20);
            check("上升时间（20%~80%）", mean(tr) * 1e12, m_tr_ps, 1.0, "ps", m_fail);
            check("下降时间（80%~20%）", mean(tf) * 1e12, m_tr_ps, 1.0, "ps", m_fail);

            // 抖动：上升沿时刻相对线性拟合（理想时刻）的残差 rms
            const size_t n = r.size();
            double sx = 0, sy = 0, sxx = 0, sxy = 0;
            for (size_t i = 0; i < n; ++i){ sx += i; sy += r[i]; sxx += 1.0 * i * i; sxy += i * r[i]; }
            const double k = (n * sxy - sx * sy) / (n * sxx - sx * sx);
            const double b = (sy - k * sx) / n;
            std::vector<double> tie;
            for (size_t i = 0; i < n; ++i) tie.push_back(r[i] - (k * i + b));
            check("抖动（rms，上升沿 TIE）", stddev(tie) * 1e12, m_jitter_ps, 0.3, "ps", m_fail);
        }

        // ---- 2. 延迟步进（通道 1 相对通道 0）----
        info("延迟步进（通道 1）");
        const double skew0 = mean_skew(1e-6, 10e-6, 0.0);

        m_sync->set_delay(1, 1, 0);         // +1 个粗延迟码
        wait(sc_time(15, SC_US) - sc_core::sc_time_stamp());
        const double skew1 = mean_skew(10.1e-6, 15e-6, skew0);

        m_sync->set_delay(1, 1, 10);        // 再 +10 个细延迟码
        wait(sc_time(20, SC_US) - sc_core::sc_time_stamp());
        const double skew2 = mean_skew(15.1e-6, 20e-6, skew1);

        check("粗延迟步进（1 个码）", (skew1 - skew0) * 1e12, m_coarse_ps, 1.0, "ps", m_fail);
        check("细延迟步进（10 个码 / 10）", (skew2 - skew1) * 1e12 / 10.0, m_fine_ps, 0.1, "ps", m_fail);

        info(("验证结束，失败项数：" + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, 21, sc_core::SC_US);
}
