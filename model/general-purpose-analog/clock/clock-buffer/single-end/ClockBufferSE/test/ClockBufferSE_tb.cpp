#include "ClockBufferSE.h"
#include "ClockBufferSE_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include <cmath>
#include <string>

// ============================================================
// ClockBufferSE 功能测试（步长 10ps）
//   激励：ICLK = 1.65 + A·sin(2π·100MHz·t)（100MHz 在手册 fin ≤ 200MHz 范围内）
//     阶段 A [0, 100ns)   ：A = 1.65V（0 ~ 3.3V 满摆幅，正常翻转）
//     阶段 B [100, 150ns) ：A = 0.60V（1.05 ~ 2.25V，处于 VIL 0.99V 与 VIH 2.31V 之间，输出应保持）
//     阶段 C [150, 200ns) ：A = 0.70V（0.95 ~ 2.35V，越过 VIL / VIH，输出恢复翻转）
//   检查：
//     0. 模型信息接口 name / version / latency / power / area
//     1. 输出电平 VOH / VOL
//     2. 频率跟随：每个输入判决沿对应一个输出沿，输出周期 10ns
//     3. 10%~90% 上升 / 下降时间 = 0.8 × tr_tf；传播延迟 - (propagation_delay + tr_tf / 2) ∈ [0, 1 个步长)
//     4. VIH / VIL 双阈值：阈值之间的输入不翻转，越过阈值的输入恢复翻转
//   传播延迟的精确核对见 ClockBufferSE_timing_tb.cpp（步长 0.1ps）
// ============================================================

// 分段幅度的单端正弦激励
SCA_TDF_MODULE(PhasedSine)
{
    sca_tdf::sca_out<double> out;

    PhasedSine(sc_core::sc_module_name nm, double freq, sc_core::sc_time timestep)
        : out("out"), m_freq(freq), m_timestep(timestep) {}

    void set_attributes() override{ set_timestep(m_timestep); }

    void processing() override{
        double t = get_time().to_seconds();
        double a = t < 100e-9 ? 1.65 : (t < 150e-9 ? 0.60 : 0.70);
        out.write(1.65 + a * std::sin(2.0 * M_PI * m_freq * t));
    }
private:
    const double m_freq;
    const sc_core::sc_time m_timestep;
};

SC_MODULE(Simulator)
    , public AbstractSimulator
{
public:
    sca_tdf::sca_signal<double> in;
    sca_tdf::sca_signal<double> out;

	SC_HAS_PROCESS(Simulator);
	explicit Simulator(sc_core::sc_module_name name)
		: sc_module(name)
        , in("in"), out("out")
        , m_fail(0)
	{
        m_cfgs = ClockBufferSE::get_configs_from_json("../../json/ClockBufferSE.json");

        m_src = new PhasedSine("src", 100e6, sc_core::sc_time(10, sc_core::SC_PS));
        m_src->out(in);

        m_buf = new ClockBufferSE(m_cfgs.module_name.c_str(), m_cfgs);
        m_buf->iclk(in);
        m_buf->oclk(out);

        m_mon = new ClockBufferSEMonitor("mon", m_cfgs.voh_v, m_cfgs.vol_v, m_cfgs.vih_v, m_cfgs.vil_v);
        m_mon->iclk(in);
        m_mon->oclk(out);

        tf = sca_util::sca_create_tabular_trace_file("wave.dat");
        m_buf->trace_me(tf);

        SC_THREAD(testbench);
	}

	~Simulator()
    {
        sca_close_tabular_trace_file(tf);
        delete m_mon;
        delete m_buf;
        delete m_src;
    }
private:
    sca_util::sca_trace_file* tf;
    ClockBufferSE::configs m_cfgs;
    PhasedSine* m_src;
    ClockBufferSE* m_buf;
    ClockBufferSEMonitor* m_mon;
    size_t m_fail;

    using Edges = std::vector<ClockBufferSEMonitor::Edge>;

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

    void testbench(){
        const double ns = 1e-9;
        const double voh = m_cfgs.voh_v, vol = m_cfgs.vol_v;
        const double tr_ps = m_cfgs.tr_tf_ps;
        const double settle = (m_cfgs.propagation_delay_ps + 6 * m_cfgs.jitter_ps + m_cfgs.tr_tf_ps) * 1e-12;

        {
            info("模型信息接口");
            info((std::string("name: ") + m_buf->get_name()).c_str());
            check("version", m_buf->get_version(), 0.1, 0, "", m_fail);
            check("latency", m_buf->get_latency(), 2200.0, 0, "ps", m_fail);
            check("power", m_buf->get_power(), 990.0, 0, "mW", m_fail);
            check("area", m_buf->get_area(), 1.875, 0, "mm2", m_fail);
        }

        wait(sc_core::sc_time(205, sc_core::SC_NS));

        {
            info("输出电平：OCLK 在 VOL ~ VOH 之间摆动");
            check("OCLK 最大值 = VOH", m_mon->o_max, voh, 1e-9, "V", m_fail);
            check("OCLK 最小值 = VOL", m_mon->o_min, vol, 1e-9, "V", m_fail);
        }

        {
            info("阶段 A [0, 100ns)：100MHz、0 ~ 3.3V，频率跟随");
            Edges in  = ClockBufferSEMonitor::window(m_mon->in_edges, 0, 100 * ns);
            Edges out = ClockBufferSEMonitor::window(m_mon->o50, 0, 100 * ns + settle);
            check("输入判决沿数", in.size(), 20, 0, "", m_fail);
            check("输出沿数 = 输入判决沿数", out.size(), in.size(), 0, "", m_fail);
            check("输出沿方向与输入一致", same_direction(in, out), 1, 0, "", m_fail);

            std::vector<double> rise;
            for(const auto& e : out) if(e.rise) rise.push_back(e.t);
            double period = (rise.back() - rise.front()) / (rise.size() - 1);
            check("输出平均周期", period / ns, 10.0, 0.01, "ns", m_fail);
        }

        {
            info("阶段 A：上升 / 下降时间（10%~90%）= 0.8 × tr_tf");
            Edges o10 = ClockBufferSEMonitor::window(m_mon->o10, 0, 100 * ns + settle);
            Edges o90 = ClockBufferSEMonitor::window(m_mon->o90, 0, 100 * ns + settle);
            check("上升时间", mean_gap_ps(o10, o90, true), 0.8 * tr_ps, 1.0, "ps", m_fail);
            check("下降时间", mean_gap_ps(o90, o10, false), 0.8 * tr_ps, 1.0, "ps", m_fail);
        }

        {
            info("阶段 A：传播延迟（输入越过 VIH / VIL → OCLK 50%），模型在越阈后的下一个采样点判决，额外滞后 0 ~ 1 个步长");
            Edges in  = ClockBufferSEMonitor::window(m_mon->in_edges, 0, 100 * ns);
            Edges out = ClockBufferSEMonitor::window(m_mon->o50, 0, 100 * ns + settle);
            double sum = 0.0;
            size_t n = std::min(in.size(), out.size());
            for(size_t i = 0; i < n; i++) sum += out[i].t - in[i].t;
            const double step_ps = 10.0;
            const double exp_ps = m_cfgs.propagation_delay_ps + tr_ps / 2;
            check("延迟均值 - (propagation_delay + tr_tf/2) ∈ [0, 步长)", sum / n * 1e12 - exp_ps, step_ps / 2, step_ps / 2, "ps", m_fail);
        }

        {
            info("阶段 B [100, 150ns)：1.05 ~ 2.25V，处于 VIL / VIH 之间，输出保持");
            Edges in  = ClockBufferSEMonitor::window(m_mon->in_edges, 100 * ns, 150 * ns);
            Edges out = ClockBufferSEMonitor::window(m_mon->o50, 100 * ns + settle, 150 * ns);
            check("输入判决沿数", in.size(), 0, 0, "", m_fail);
            check("输出沿数", out.size(), 0, 0, "", m_fail);
        }

        {
            info("阶段 C [150, 200ns)：0.95 ~ 2.35V，越过 VIL / VIH，输出恢复翻转");
            Edges in  = ClockBufferSEMonitor::window(m_mon->in_edges, 150 * ns, 200 * ns);
            Edges out = ClockBufferSEMonitor::window(m_mon->o50, 150 * ns, 200 * ns + settle);
            check("输入判决沿数", in.size(), 10, 0, "", m_fail);
            check("输出沿数 = 输入判决沿数", out.size(), in.size(), 0, "", m_fail);
            check("输出沿方向与输入一致", same_direction(in, out), 1, 0, "", m_fail);
        }

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, 206, sc_core::SC_NS);
}
