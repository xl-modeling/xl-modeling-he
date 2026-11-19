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
// 上升沿记录：按线性插值记录信号过 0 V 的上升沿时刻
// ============================================================
SCA_TDF_MODULE(RiseProbe)
{
    sca_tdf::sca_in<double> in;

    RiseProbe(sc_core::sc_module_name nm) : in("in") {}

    void processing() override{
        const double t = get_time().to_seconds();
        const double v = in.read();
        if (m_has_prev && m_vp <= 0.0 && v > 0.0)
            rise.push_back(m_tp + (0.0 - m_vp) / (v - m_vp) * (t - m_tp));
        m_tp = t; m_vp = v; m_has_prev = true;
    }

    std::vector<double> rise;

private:
    bool m_has_prev = false;
    double m_tp = 0.0, m_vp = 0.0;
};

// ============================================================
// ClockSynchronizer 锁相测试（10 ps 时间步，CPPLL）
//   REFIN：100 MHz 差分方波（±350 mV）
//   1. PLL 锁定，锁定后保持锁定
//   2. PFD、VCO、BSYNC 频率在手册范围内，且 BSYNC 频率 = fREFIN × NDIV / (RDIV × ODIV)
//   3. BSYNC 与 REFIN 相位锁定：参考通道上升沿相对 REFIN 上升沿的时差不随时间漂移
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

        const sc_core::sc_time dt(10, sc_core::SC_PS);
        m_cfgs = ClockSynchronizer::get_configs_from_json(json);
        m_sync = new ClockSynchronizer(m_cfgs.module_name.c_str(), m_cfgs, dt, true);

        m_refin = new PULSE("refin", -0.35, 0.35, 0.0, 1e-12, 1e-12, 0.5 / REFIN_HZ, 1.0 / REFIN_HZ, 0, dt);
        m_refin->out(refin);
        m_sync->refin(refin);

        bsync.init(m_cfgs.channels);
        for (int i = 0; i < m_cfgs.channels; ++i)
            m_sync->bsync[i](bsync[i]);

        m_probe_ref = new RiseProbe("probe_refin");
        m_probe_out = new RiseProbe("probe_bsync");
        m_probe_ref->in(refin);
        m_probe_out->in(bsync[m_cfgs.ref_channel]);

        m_tf = sca_util::sca_create_tabular_trace_file("pll.dat");
        m_tf->set_mode(sca_util::sca_decimation(10));
        m_sync->trace_me(m_tf);

        SC_THREAD(testbench);
    }

    ~Simulator(){
        sca_util::sca_close_tabular_trace_file(m_tf);
        delete m_sync;
        delete m_refin;
        delete m_probe_ref;
        delete m_probe_out;
    }

    sc_core::sc_time sim_time() const{
        return LOCK_TIMEOUT + SETTLE + WINDOW + sc_core::sc_time(1, sc_core::SC_US);
    }

private:
    static constexpr double REFIN_HZ = 100e6;                                   // REFIN 频率
    const sc_core::sc_time LOCK_TIMEOUT = sc_core::sc_time(20, sc_core::SC_US);  // 等待锁定的上限
    const sc_core::sc_time SETTLE = sc_core::sc_time(1, sc_core::SC_US);         // 锁定后等待输出稳定
    const sc_core::sc_time WINDOW = sc_core::sc_time(10, sc_core::SC_US);        // 测量窗口

    size_t m_fail;
    sca_util::sca_trace_file* m_tf;
    ClockSynchronizer::configs m_cfgs;
    ClockSynchronizer* m_sync;
    PULSE* m_refin;
    RiseProbe* m_probe_ref;
    RiseProbe* m_probe_out;

    // json 中的手册范围
    double m_pfd_min, m_pfd_max, m_vco_min, m_vco_max, m_bsync_min, m_bsync_max;

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
        // 手册范围单位为 MHz，换算为 Hz
        m_pfd_min   = range(p, "pfd-frequency", 0) * 1e6;
        m_pfd_max   = range(p, "pfd-frequency", 1) * 1e6;
        m_vco_min   = range(p, "vco-frequency", 0) * 1e6;
        m_vco_max   = range(p, "vco-frequency", 1) * 1e6;
        m_bsync_min = range(p, "bsync-frequency", 0) * 1e6;
        m_bsync_max = range(p, "bsync-frequency", 1) * 1e6;
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

    static std::vector<double> window(const std::vector<double>& v, double t0, double t1){
        std::vector<double> r;
        for (double x : v) if (x >= t0 && x < t1) r.push_back(x);
        return r;
    }

    // 在 [lo, hi] 内记 1，否则记 0
    void check_range(const char* what, double v, double lo, double hi, const char* unit){
        check(what, (v >= lo && v <= hi) ? 1 : 0, 1, 0, "", m_fail);
        info((std::string("  ") + what + " = " + std::to_string(v) + " " + unit +
              "（手册范围 " + std::to_string(lo) + " ~ " + std::to_string(hi) + "）").c_str());
    }

    void testbench(){
        using sc_core::sc_time;

        // ---- 1. 锁定 ----
        while (!m_sync->locked() && sc_core::sc_time_stamp() < LOCK_TIMEOUT)
            wait(sc_time(100, sc_core::SC_NS));
        check("PLL 锁定", m_sync->locked(), 1, 0, "", m_fail);
        if (!m_sync->locked()){
            info(("验证结束，失败项数：" + std::to_string(m_fail)).c_str());
            return;
        }
        info(("锁定时刻：" + m_sync->lock_time().to_string()).c_str());

        const double t0 = (m_sync->lock_time() + SETTLE).to_seconds();
        const double t1 = t0 + WINDOW.to_seconds();
        wait(sc_time(t1, sc_core::SC_SEC) - sc_core::sc_time_stamp());
        check("测量窗口内保持锁定", m_sync->locked(), 1, 0, "", m_fail);

        // ---- 2. 频率 ----
        info("频率");
        const double fexp = REFIN_HZ * m_cfgs.ndiv / (m_cfgs.rdiv * m_cfgs.odiv);
        const std::vector<double> r = window(m_probe_out->rise, t0, t1);
        const double fout = (r.size() - 1) / (r.back() - r.front());
        check("BSYNC 频率（实测）", fout / 1e6, fexp / 1e6, 0.01, "MHz", m_fail);
        check_range("PFD 频率在手册范围内", m_sync->pfd_frequency(), m_pfd_min, m_pfd_max, "Hz");
        check_range("VCO 频率在手册范围内", m_sync->vco_frequency(), m_vco_min, m_vco_max, "Hz");
        check_range("BSYNC 频率在手册范围内", fout, m_bsync_min, m_bsync_max, "Hz");

        // ---- 3. 相位锁定：BSYNC 上升沿相对之前最近的 REFIN 上升沿的时差 ----
        info("BSYNC 与 REFIN 相位锁定");
        const std::vector<double> a = window(m_probe_ref->rise, t0 - 1.0 / REFIN_HZ, t1);
        const double Tref = 1.0 / REFIN_HZ;
        //   时差统一折算到第一个样本附近（±Tref/2），避免在 0 / Tref 处来回折返
        std::vector<double> d1, d2;         // 窗口前半段 / 后半段
        size_t j = 0;
        bool has_x0 = false;
        double x0 = 0.0;
        for (double tb : r){
            while (j + 1 < a.size() && a[j + 1] <= tb) ++j;
            if (a.empty() || a[j] > tb) continue;
            double x = tb - a[j];
            if (!has_x0){ x0 = x; has_x0 = true; }
            x -= Tref * std::floor((x - x0) / Tref + 0.5);
            (tb < 0.5 * (t0 + t1) ? d1 : d2).push_back(x);
        }
        const double m1 = mean(d1);
        const double drift = (mean(d2) - m1) * 1e12;
        std::vector<double> all(d1);
        all.insert(all.end(), d2.begin(), d2.end());
        info(("  时差均值 " + std::to_string(m1 * 1e12) + " ps，标准差 " + std::to_string(stddev(all) * 1e12) + " ps").c_str());
        check("相位漂移（后半段均值 - 前半段均值）", drift, 0.0, 1.0, "ps", m_fail);

        info(("验证结束，失败项数：" + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, sim.sim_time().to_seconds(), sc_core::SC_SEC);
}
