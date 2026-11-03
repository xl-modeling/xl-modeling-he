#include "ClockBufferSE.h"
#include "ClockBufferSE_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include <algorithm>
#include <cmath>
#include <string>

// ============================================================
// ClockBufferSE 时序测试（步长 0.1ps，时间分辨率 1fs）
//   模型的延迟线按 TDF 步长取整，ps 级的传播延迟必须用远小于 1ps 的步长才能精确核对
//   激励：ICLK = 1.65 + 1.65·sin(2π·200MHz·t)（手册最高输入频率 200MHz，0 ~ 3.3V），仿真 100ns，40 个边沿
//   检查：
//     1. 每个输入判决沿对应一个输出沿，方向一致
//     2. 传播延迟（输入越过 VIH / VIL → OCLK 50%）均值 - (propagation_delay + tr_tf / 2) ∈ [0, 1 个步长)
//        （模型先延迟 propagation_delay，再以 tr_tf 线性成形，50% 点再晚 tr_tf / 2；
//          模型在越阈后的下一个采样点才判决，额外滞后 0 ~ 1 个步长）
//     3. 抖动为 0：所有边沿的传播延迟一致（标准差 < 1 个步长）
// ============================================================
SCA_TDF_MODULE(Sine)
{
    sca_tdf::sca_out<double> out;

    Sine(sc_core::sc_module_name nm, double offset, double amp, double freq, sc_core::sc_time timestep)
        : out("out"), m_offset(offset), m_amp(amp), m_freq(freq), m_timestep(timestep) {}

    void set_attributes() override{ set_timestep(m_timestep); }

    void processing() override{
        double t = get_time().to_seconds();
        out.write(m_offset + m_amp * std::sin(2.0 * M_PI * m_freq * t));
    }
private:
    const double m_offset, m_amp, m_freq;
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

        m_src = new Sine("src", 1.65, 1.65, 200e6, sc_core::sc_time(0.1, sc_core::SC_PS));
        m_src->out(in);

        m_buf = new ClockBufferSE(m_cfgs.module_name.c_str(), m_cfgs);
        m_buf->iclk(in);
        m_buf->oclk(out);

        m_mon = new ClockBufferSEMonitor("mon", m_cfgs.voh_v, m_cfgs.vol_v, m_cfgs.vih_v, m_cfgs.vil_v);
        m_mon->iclk(in);
        m_mon->oclk(out);

        SC_THREAD(testbench);
	}

	~Simulator()
    {
        delete m_mon;
        delete m_buf;
        delete m_src;
    }
private:
    ClockBufferSE::configs m_cfgs;
    Sine* m_src;
    ClockBufferSE* m_buf;
    ClockBufferSEMonitor* m_mon;
    size_t m_fail;

    void testbench(){
        // 统计 [0, 100ns) 内的输入判决沿及其对应的输出沿（[0, 100ns + 传播延迟 + tr_tf)），多等 3ns 让最后一个沿传到输出
        wait(sc_core::sc_time(103, sc_core::SC_NS));

        // 第 i 个输出沿与第 i 个输入判决沿配对（传播延迟 2.2ns 小于半周期 2.5ns，不会错配）
        const auto in = ClockBufferSEMonitor::window(m_mon->in_edges, 0, 100e-9);
        const double settle = (m_cfgs.propagation_delay_ps + m_cfgs.tr_tf_ps) * 1e-12;
        const auto o = ClockBufferSEMonitor::window(m_mon->o50, 0, 100e-9 + settle);
        size_t n = std::min(in.size(), o.size());
        bool dir_ok = in.size() == o.size();
        std::vector<double> delay_ps;
        for(size_t i = 0; i < n; i++){
            dir_ok = dir_ok && in[i].rise == o[i].rise;
            delay_ps.push_back((o[i].t - in[i].t) * 1e12);
        }

        double mean = 0.0, var = 0.0;
        for(double x : delay_ps) mean += x;
        mean /= n;
        for(double x : delay_ps) var += (x - mean) * (x - mean);
        double sigma = std::sqrt(var / n);
        auto mm = std::minmax_element(delay_ps.begin(), delay_ps.end());

        const double exp_mean = m_cfgs.propagation_delay_ps + m_cfgs.tr_tf_ps / 2;

        info((std::string("输入判决沿数: ") + std::to_string(in.size())
              + "，输出沿数: " + std::to_string(o.size())).c_str());
        check("输入判决沿数（200MHz × 100ns）", in.size(), 40, 0, "", m_fail);
        check("输出沿数 = 输入判决沿数，方向一致", dir_ok, 1, 0, "", m_fail);
        const double step_ps = 0.1;
        check("延迟均值 - (propagation_delay + tr_tf/2) ∈ [0, 步长)", mean - exp_mean, step_ps / 2, step_ps / 2, "ps", m_fail);
        check("传播延迟标准差（抖动为 0）", sigma, 0.0, 0.1, "ps", m_fail);
        info((std::string("延迟范围: ") + std::to_string(*mm.first) + " ~ " + std::to_string(*mm.second) + " ps").c_str());

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    sc_core::sc_set_time_resolution(1, sc_core::SC_FS);     // 0.1ps 步长需要 fs 级时间分辨率
    Simulator sim("sim");
    SIMULATOR_START(sim, 104, sc_core::SC_NS);
}
