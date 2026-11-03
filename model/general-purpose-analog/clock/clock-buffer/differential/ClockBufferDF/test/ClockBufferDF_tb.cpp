#include "ClockBufferDF.h"
#include "ClockBufferDF_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include <cmath>
#include <numeric>
#include <string>

// ============================================================
// ClockBufferDF 功能测试（步长 10ps）
//   激励：DI = A·sin(2π·1GHz·t)，DN = -DI，差分峰值 2A
//     阶段 A [0, 10ns) ：A = 1.0V   （差分 ±2V，正常翻转）
//     阶段 B [10, 15ns)：A = 0.02V  （差分 ±40mV < 50mV 迟滞，输出应保持）
//     阶段 C [15, 20ns)：A = 0.03V  （差分 ±60mV > 50mV 迟滞，输出恢复翻转）
//   检查：
//     0. 模型信息接口 name / version / latency / power / area
//     1. 输出电平 VOH / VOL，Q / QN 互补
//     2. 频率跟随：每个输入判决沿对应一个输出沿，输出周期 1ns
//     3. 10%~90% 上升 / 下降时间 = 0.8 × tr_tf
//     4. ±50mV 迟滞：小于迟滞的输入不翻转，大于迟滞的输入恢复翻转
//   传播延迟与抖动需要更小步长，见 ClockBufferDF_timing_tb.cpp
// ============================================================

// 分段幅度的差分正弦激励
SCA_TDF_MODULE(PhasedDiffSine)
{
    sca_tdf::sca_out<double> p;
    sca_tdf::sca_out<double> n;

    PhasedDiffSine(sc_core::sc_module_name nm, double freq, sc_core::sc_time timestep)
        : p("p"), n("n"), m_freq(freq), m_timestep(timestep) {}

    void set_attributes() override{ set_timestep(m_timestep); }

    void processing() override{
        double t = get_time().to_seconds();
        double a = t < 10e-9 ? 1.0 : (t < 15e-9 ? 0.02 : 0.03);
        double v = a * std::sin(2.0 * M_PI * m_freq * t);
        p.write(v);
        n.write(-v);
    }
private:
    const double m_freq;
    const sc_core::sc_time m_timestep;
};

SC_MODULE(Simulator)
    , public AbstractSimulator
{
public:
    sca_tdf::sca_signal<double> d;
    sca_tdf::sca_signal<double> dn;
    sca_tdf::sca_signal<double> q;
    sca_tdf::sca_signal<double> qn;

	SC_HAS_PROCESS(Simulator);
	explicit Simulator(sc_core::sc_module_name name)
		: sc_module(name)
        , d("d"), dn("dn"), q("q"), qn("qn")
        , m_fail(0)
	{
        m_cfgs = ClockBufferDF::get_configs_from_json("../../json/ClockBufferDF.json");

        m_src = new PhasedDiffSine("src", 1e9, sc_core::sc_time(10, sc_core::SC_PS));
        m_src->p(d);
        m_src->n(dn);

        m_buf = new ClockBufferDF(m_cfgs.module_name.c_str(), m_cfgs);
        m_buf->di(d);
        m_buf->dn(dn);
        m_buf->q(q);
        m_buf->qn(qn);

        m_mon = new ClockBufferDFMonitor("mon", m_cfgs.voh_v, m_cfgs.vol_v, 0.05);
        m_mon->di(d);
        m_mon->dn(dn);
        m_mon->q(q);
        m_mon->qn(qn);

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
    ClockBufferDF::configs m_cfgs;
    PhasedDiffSine* m_src;
    ClockBufferDF* m_buf;
    ClockBufferDFMonitor* m_mon;
    size_t m_fail;

    using Edges = std::vector<ClockBufferDFMonitor::Edge>;

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
        const double tr_ps = m_cfgs.tr_tf_ns * 1e3;
        const double settle = (m_cfgs.propagation_delay_ps + 6 * m_cfgs.jitter_ps) * 1e-12 + m_cfgs.tr_tf_ns * ns;

        {
            info("模型信息接口");
            info((std::string("name: ") + m_buf->get_name()).c_str());
            check("version", m_buf->get_version(), 0.1, 0, "", m_fail);
            check("latency", m_buf->get_latency(), 100.0, 0, "ps", m_fail);
            check("power", m_buf->get_power(), 3200.0, 0, "mW", m_fail);
            check("area", m_buf->get_area(), 2.966, 0, "mm2", m_fail);
        }

        wait(sc_core::sc_time(20.5, sc_core::SC_NS));

        {
            info("输出电平与互补：Q / QN 在 VOL ~ VOH 之间摆动，Q + QN = VOH + VOL");
            check("Q 最大值 = VOH", m_mon->q_max, voh, 1e-9, "V", m_fail);
            check("Q 最小值 = VOL", m_mon->q_min, vol, 1e-9, "V", m_fail);
            check("QN 最大值 = VOH", m_mon->qn_max, voh, 1e-9, "V", m_fail);
            check("QN 最小值 = VOL", m_mon->qn_min, vol, 1e-9, "V", m_fail);
            check("max|Q + QN - (VOH + VOL)|", m_mon->compl_err, 0.0, 1e-9, "V", m_fail);
        }

        {
            info("阶段 A [0, 10ns)：1GHz、差分 ±2V，频率跟随");
            Edges in  = ClockBufferDFMonitor::window(m_mon->in_edges, 0, 10 * ns);
            Edges out = ClockBufferDFMonitor::window(m_mon->q50, 0, 10 * ns + settle);
            check("输入判决沿数", in.size(), 20, 0, "", m_fail);
            check("Q 输出沿数 = 输入判决沿数", out.size(), in.size(), 0, "", m_fail);
            check("Q 输出沿方向与输入一致", same_direction(in, out), 1, 0, "", m_fail);

            std::vector<double> rise;
            for(const auto& e : out) if(e.rise) rise.push_back(e.t);
            double period = (rise.back() - rise.front()) / (rise.size() - 1);
            check("Q 平均周期", period / ns, 1.0, 0.01, "ns", m_fail);
        }

        {
            info("阶段 A：上升 / 下降时间（10%~90%）= 0.8 × tr_tf");
            Edges q10 = ClockBufferDFMonitor::window(m_mon->q10, 0, 10 * ns + settle);
            Edges q90 = ClockBufferDFMonitor::window(m_mon->q90, 0, 10 * ns + settle);
            check("上升时间", mean_gap_ps(q10, q90, true), 0.8 * tr_ps, 1.0, "ps", m_fail);
            check("下降时间", mean_gap_ps(q90, q10, false), 0.8 * tr_ps, 1.0, "ps", m_fail);
        }

        {
            info("阶段 B [10, 15ns)：差分 ±40mV < 50mV 迟滞，输出保持");
            Edges in  = ClockBufferDFMonitor::window(m_mon->in_edges, 10 * ns, 15 * ns);
            Edges out = ClockBufferDFMonitor::window(m_mon->q50, 10 * ns + settle, 15 * ns);
            check("输入判决沿数", in.size(), 0, 0, "", m_fail);
            check("Q 输出沿数", out.size(), 0, 0, "", m_fail);
        }

        {
            info("阶段 C [15, 20ns)：差分 ±60mV > 50mV 迟滞，输出恢复翻转");
            Edges in  = ClockBufferDFMonitor::window(m_mon->in_edges, 15 * ns, 20 * ns);
            Edges out = ClockBufferDFMonitor::window(m_mon->q50, 15 * ns, 20 * ns + settle);
            check("输入判决沿数", in.size(), 10, 0, "", m_fail);
            check("Q 输出沿数 = 输入判决沿数", out.size(), in.size(), 0, "", m_fail);
            check("Q 输出沿方向与输入一致", same_direction(in, out), 1, 0, "", m_fail);
        }

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, 21, sc_core::SC_NS);
}
