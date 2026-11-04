#include "ClockDistribution.h"
#include "ClockDistribution_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include <algorithm>
#include <cmath>
#include <string>

// ============================================================
// ClockDistribution 时序测试（步长 0.1ps，时间分辨率 1fs）
//   模型的延迟线按 TDF 步长取整，ps 级的 skew 与抖动必须用远小于 1ps 的步长才能核对
//   激励：ICLK = VBB + 0.4·sin(2π·500MHz·t)（半周期 1ns 长于 tr_tf，输出满摆幅），仿真 200ns，200 个边沿
//   检查（各路传播延迟 = 输入判决沿 → OCLK[k] 50%）：
//     1. 每路每个输入判决沿对应一个输出沿，方向一致
//     2. 各路延迟均值落在 [propagation_delay + tr_tf/2, propagation_delay + skew + tr_tf/2 + 1 个步长) 内
//        （模型先延迟 propagation_delay + 该路 skew（U[0, skew)），再以 tr_tf 线性成形，50% 点再晚 tr_tf/2；
//          模型在越阈后的下一个采样点判决，额外滞后 0 ~ 1 个步长）
//     3. 片内 skew：各路延迟均值的极差 < skew
//     4. 各路抖动（延迟标准差）= jitter（误差 < 15%）
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

        m_src = new Sine("src", m_cfgs.vth_v, 0.40, 500e6, sc_core::sc_time(0.1, sc_core::SC_PS));
        m_src->out(iclk);

        m_dist = new ClockDistribution(m_cfgs.module_name.c_str(), m_cfgs);
        m_dist->iclk(iclk);
        m_dist->oclks(oclks);

        m_mon = new ClockDistributionMonitor("mon", m_cfgs.output_n, m_cfgs.voh_v, m_cfgs.vol_v, m_cfgs.vth_v, 0.05);
        m_mon->iclk(iclk);
        m_mon->oclks(oclks);

        SC_THREAD(testbench);
	}

	~Simulator()
    {
        delete m_mon;
        delete m_dist;
        delete m_src;
    }
private:
    ClockDistribution::configs m_cfgs;
    Sine* m_src;
    ClockDistribution* m_dist;
    ClockDistributionMonitor* m_mon;
    size_t m_fail;

    using Mon = ClockDistributionMonitor;

    void testbench(){
        const double T = 200e-9;
        const double step_ps = 0.1;
        const double tr_ps = m_cfgs.tr_tf_ps;
        // 输入窗口 [0, T) 的边沿对应的输出沿落在 [dmin, T + dmin)，dmin 为最短可能延迟
        const double dmin = (m_cfgs.propagation_delay_ps - 6 * m_cfgs.jitter_ps) * 1e-12;
        wait(sc_core::sc_time(T + 3e-9, sc_core::SC_SEC));

        const auto in = Mon::window(m_mon->in_edges, 0, T);
        const size_t n = m_cfgs.output_n;
        info((std::string("输入判决沿数: ") + std::to_string(in.size())).c_str());
        check("输入判决沿数（500MHz × 200ns）", in.size(), 200, 0, "", m_fail);

        const double lo = m_cfgs.propagation_delay_ps + tr_ps / 2;
        const double hi = m_cfgs.propagation_delay_ps + m_cfgs.skew_ps + tr_ps / 2 + step_ps;
        std::vector<double> means;
        bool dir_ok = true;
        for(size_t k = 0; k < n; k++){
            // 第 i 个输出沿与第 i 个输入判决沿配对（延迟 < 0.9ns 短于半周期 1ns，不会错配）
            const auto o = Mon::window(m_mon->o50[k], dmin, T + dmin);
            dir_ok = dir_ok && o.size() == in.size();
            std::vector<double> d;
            for(size_t i = 0; i < std::min(in.size(), o.size()); i++){
                dir_ok = dir_ok && in[i].rise == o[i].rise;
                d.push_back((o[i].t - in[i].t) * 1e12);
            }
            double mean = 0.0, var = 0.0;
            for(double x : d) mean += x;
            mean /= d.size();
            for(double x : d) var += (x - mean) * (x - mean);
            double sigma = std::sqrt(var / d.size());
            means.push_back(mean);

            std::string s = "OCLK" + std::to_string(k);
            check((s + " 延迟均值 ∈ [tpd + tr/2, tpd + skew + tr/2 + 步长)").c_str(), mean, (lo + hi) / 2, (hi - lo) / 2, "ps", m_fail);
            check((s + " 抖动（延迟标准差）= jitter").c_str(), sigma, m_cfgs.jitter_ps, 0.15 * m_cfgs.jitter_ps, "ps", m_fail);
        }
        check("各路输出沿数 = 输入判决沿数，方向一致", dir_ok, 1, 0, "", m_fail);

        auto mm = std::minmax_element(means.begin(), means.end());
        info((std::string("各路延迟均值范围: ") + std::to_string(*mm.first) + " ~ " + std::to_string(*mm.second) + " ps").c_str());
        check("片内 skew（各路延迟均值极差）< skew", *mm.second - *mm.first, m_cfgs.skew_ps / 2, m_cfgs.skew_ps / 2, "ps", m_fail);

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    sc_core::sc_set_time_resolution(1, sc_core::SC_FS);     // 0.1ps 步长需要 fs 级时间分辨率
    Simulator sim("sim");
    SIMULATOR_START(sim, 204, sc_core::SC_NS);
}
