#include "ClockDistribution.h"
#include "ClockDistribution_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include <algorithm>
#include <cmath>
#include <string>

// ============================================================
// ClockDistribution 功能测试（步长 10ps）
//   激励：ICLK = VBB + A·sin(2π·f·t)，VBB = 3.70V（判决阈值）
//     阶段 A [0, 100ns)   ：f = 100MHz， A = 0.40V（单端摆幅 800mV，正常翻转）
//     阶段 B [100, 150ns) ：f = 100MHz， A = 0.04V（±40mV < 50mV 迟滞，输出应保持）
//     阶段 C [150, 200ns) ：f = 100MHz， A = 0.06V（±60mV > 50mV 迟滞，输出恢复翻转）
//     阶段 D [200, 220ns) ：f = 1.25GHz，A = 0.40V（手册 fMAX，半周期 400ps 短于传播延迟）
//     220ns 之后          ：保持 VBB（落在迟滞内，不再翻转）
//   检查：
//     0. 模型信息接口 name / version / latency / power / area
//     1. 各路输出电平 VOH / VOL
//     2. 频率跟随：每路每个输入判决沿对应一个输出沿，输出周期 10ns
//     3. 10%~90% 上升 / 下降时间 = 0.8 × tr_tf
//     4. ±50mV 迟滞：小于迟滞的输入不翻转，大于迟滞的输入恢复翻转
//     5. fMAX：每路不丢沿；tr_tf 长于半周期，峰峰值摆幅 = (VOH - VOL)·a/(2 - a)，a = 半周期 / tr_tf
//   传播延迟、skew 与抖动需要更小步长，见 ClockDistribution_timing_tb.cpp
// ============================================================

// 分段激励
SCA_TDF_MODULE(PhasedSine)
{
    sca_tdf::sca_out<double> out;

    PhasedSine(sc_core::sc_module_name nm, double offset, sc_core::sc_time timestep)
        : out("out"), m_offset(offset), m_timestep(timestep) {}

    void set_attributes() override{ set_timestep(m_timestep); }

    void processing() override{
        double t = get_time().to_seconds();
        double v;
        if(t < 100e-9)      v = 0.40 * std::sin(2.0 * M_PI * 100e6 * t);
        else if(t < 150e-9) v = 0.04 * std::sin(2.0 * M_PI * 100e6 * t);
        else if(t < 200e-9) v = 0.06 * std::sin(2.0 * M_PI * 100e6 * t);
        else if(t < 220e-9) v = 0.40 * std::sin(2.0 * M_PI * 1.25e9 * (t - 200e-9));
        else                v = 0.0;
        out.write(m_offset + v);
    }
private:
    const double m_offset;
    const sc_core::sc_time m_timestep;
};

SC_MODULE(Simulator)
    , public AbstractSimulator
{
public:
    sca_tdf::sca_signal<double> iclk;
    sc_core::sc_vector<sca_tdf::sca_signal<double>> oclks;

	SC_HAS_PROCESS(Simulator);
	explicit Simulator(sc_core::sc_module_name name)
		: sc_module(name)
        , iclk("iclk")
        , m_fail(0)
	{
        m_cfgs = ClockDistribution::get_configs_from_json("../../json/ClockDistribution.json");
        oclks.init(m_cfgs.output_n);

        m_src = new PhasedSine("src", m_cfgs.vth_v, sc_core::sc_time(10, sc_core::SC_PS));
        m_src->out(iclk);

        m_dist = new ClockDistribution(m_cfgs.module_name.c_str(), m_cfgs);
        m_dist->iclk(iclk);
        m_dist->oclks(oclks);

        m_mon = new ClockDistributionMonitor("mon", m_cfgs.output_n, m_cfgs.voh_v, m_cfgs.vol_v, m_cfgs.vth_v, 0.05,
                                             205e-9, 220e-9);   // 阶段 D 稳态窗口
        m_mon->iclk(iclk);
        m_mon->oclks(oclks);

        tf = sca_util::sca_create_tabular_trace_file("wave.dat");
        m_dist->trace_me(tf);

        SC_THREAD(testbench);
	}

	~Simulator()
    {
        sca_util::sca_close_tabular_trace_file(tf);
        delete m_mon;
        delete m_dist;
        delete m_src;
    }
private:
    sca_util::sca_trace_file* tf;
    ClockDistribution::configs m_cfgs;
    PhasedSine* m_src;
    ClockDistribution* m_dist;
    ClockDistributionMonitor* m_mon;
    size_t m_fail;

    using Mon = ClockDistributionMonitor;
    using Edges = Mon::Edges;

    // 两组边沿数量相同且方向逐一一致
    static bool same_direction(const Edges& a, const Edges& b){
        if(a.size() != b.size()) return false;
        for(size_t i = 0; i < a.size(); i++)
            if(a[i].rise != b[i].rise) return false;
        return true;
    }

    // 成对边沿的平均间隔 (ps)：from 中第 i 个 dir 方向的沿 → to 中第 i 个 dir 方向的沿
    static double mean_gap_ps(const Edges& from, const Edges& to, bool dir){
        std::vector<double> a, b;
        for(const auto& e : from) if(e.rise == dir) a.push_back(e.t);
        for(const auto& e : to)   if(e.rise == dir) b.push_back(e.t);
        size_t n = std::min(a.size(), b.size());
        double sum = 0.0;
        for(size_t i = 0; i < n; i++) sum += b[i] - a[i];
        return n ? sum / n * 1e12 : NAN;
    }

    // 各路结果均满足时为 1
    template<typename F>
    bool all_outputs(F f){
        for(size_t k = 0; k < m_cfgs.output_n; k++)
            if(!f(k)) return false;
        return true;
    }

    void testbench(){
        const double ns = 1e-9;
        const size_t n = m_cfgs.output_n;
        const double voh = m_cfgs.voh_v, vol = m_cfgs.vol_v;
        const double tr_ps = m_cfgs.tr_tf_ps;
        // 输入窗口 [t0, t1) 的边沿对应的输出沿落在 [t0 + dmin, t1 + dmin)，dmin 为最短可能延迟
        const double dmin = (m_cfgs.propagation_delay_ps - 6 * m_cfgs.jitter_ps) * 1e-12;
        auto out_win = [&](size_t k, double t0, double t1){ return Mon::window(m_mon->o50[k], t0 + dmin, t1 + dmin); };

        {
            info("模型信息接口");
            info((std::string("name: ") + m_dist->get_name()).c_str());
            check("version", m_dist->get_version(), 0.1, 0, "", m_fail);
            check("latency", m_dist->get_latency(), 570.0, 0, "ps", m_fail);
            check("power", m_dist->get_power(), 125.0, 0, "mW", m_fail);
            check("area", m_dist->get_area(), 38.61, 0, "mm2", m_fail);
            check("输出路数（1:4）", n, 4, 0, "", m_fail);
        }

        wait(sc_core::sc_time(222, sc_core::SC_NS));

        {
            info("输出电平：各路在 VOL ~ VOH 之间摆动");
            for(size_t k = 0; k < n; k++){
                std::string s = "OCLK" + std::to_string(k);
                check((s + " 最大值 = VOH").c_str(), m_mon->o_max[k], voh, 1e-9, "V", m_fail);
                check((s + " 最小值 = VOL").c_str(), m_mon->o_min[k], vol, 1e-9, "V", m_fail);
            }
        }

        {
            info("阶段 A [0, 100ns)：100MHz、VBB ± 0.4V，各路频率跟随");
            Edges in = Mon::window(m_mon->in_edges, 0, 100 * ns);
            check("输入判决沿数", in.size(), 20, 0, "", m_fail);
            check("各路输出沿数 = 输入判决沿数，方向一致",
                  all_outputs([&](size_t k){ return same_direction(in, out_win(k, 0, 100 * ns)); }), 1, 0, "", m_fail);
            for(size_t k = 0; k < n; k++){
                std::vector<double> rise;
                for(const auto& e : out_win(k, 0, 100 * ns)) if(e.rise) rise.push_back(e.t);
                check(("OCLK" + std::to_string(k) + " 平均周期").c_str(), (rise.back() - rise.front()) / (rise.size() - 1) / ns, 10.0, 0.01, "ns", m_fail);
            }
        }

        {
            info("阶段 A：上升 / 下降时间（10%~90%）= 0.8 × tr_tf");
            for(size_t k = 0; k < n; k++){
                Edges o10 = Mon::window(m_mon->o10[k], dmin, 100 * ns + dmin);
                Edges o90 = Mon::window(m_mon->o90[k], dmin, 100 * ns + dmin);
                std::string s = "OCLK" + std::to_string(k);
                check((s + " 上升时间").c_str(), mean_gap_ps(o10, o90, true), 0.8 * tr_ps, 1.0, "ps", m_fail);
                check((s + " 下降时间").c_str(), mean_gap_ps(o90, o10, false), 0.8 * tr_ps, 1.0, "ps", m_fail);
            }
        }

        {
            info("阶段 B [100, 150ns)：VBB ± 40mV < 50mV 迟滞，输出保持");
            check("输入判决沿数", Mon::window(m_mon->in_edges, 100 * ns, 150 * ns).size(), 0, 0, "", m_fail);
            check("各路输出沿数为 0",
                  all_outputs([&](size_t k){ return out_win(k, 100 * ns, 150 * ns).empty(); }), 1, 0, "", m_fail);
        }

        {
            info("阶段 C [150, 200ns)：VBB ± 60mV > 50mV 迟滞，输出恢复翻转");
            Edges in = Mon::window(m_mon->in_edges, 150 * ns, 200 * ns);
            check("输入判决沿数", in.size(), 10, 0, "", m_fail);
            check("各路输出沿数 = 输入判决沿数，方向一致",
                  all_outputs([&](size_t k){ return same_direction(in, out_win(k, 150 * ns, 200 * ns)); }), 1, 0, "", m_fail);
        }

        {
            info("阶段 D [200, 220ns)：fMAX 1.25GHz，半周期 400ps 短于传播延迟，各路不丢沿");
            Edges in = Mon::window(m_mon->in_edges, 200 * ns, 220 * ns);
            check("输入判决沿数", in.size(), 50, 0, "", m_fail);
            check("各路输出沿数 = 输入判决沿数，方向一致",
                  all_outputs([&](size_t k){ return same_direction(in, out_win(k, 200 * ns, 220 * ns)); }), 1, 0, "", m_fail);

            // Transition 被中途打断时，从当前值出发、仍以 tr_tf 到达新目标（与 Verilog-AMS transition 一致），
            // 设 a = 半周期 / tr_tf，稳态下输出在中间电平附近往复，峰峰值 = (VOH - VOL) · a / (2 - a)
            const double half_ps = 1e12 / 1.25e9 / 2;
            const double a = std::min(1.0, half_ps / tr_ps);
            const double exp_pp = (voh - vol) * a / (2 - a);
            info("tr_tf 541.67ps 长于半周期 400ps，线性斜坡走不完，峰峰值摆幅 = (VOH - VOL)·a/(2 - a)，a = 400 / 541.67");
            for(size_t k = 0; k < n; k++)
                check(("OCLK" + std::to_string(k) + " 峰峰值摆幅").c_str(), m_mon->w_max[k] - m_mon->w_min[k], exp_pp, 0.05 * exp_pp, "V", m_fail);
        }

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, 223, sc_core::SC_NS);
}
