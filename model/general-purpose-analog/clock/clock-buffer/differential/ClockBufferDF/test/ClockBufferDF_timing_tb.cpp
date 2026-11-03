#include "ClockBufferDF.h"
#include "ClockBufferDF_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"
#include "utils/vsource.h"

#include <cmath>
#include <string>

// ============================================================
// ClockBufferDF 时序测试（步长 0.1ps，时间分辨率 1fs）
//   模型的延迟线按 TDF 步长取整，ps 级抖动必须用远小于抖动的步长才能体现
//   激励：DI / DN 为反相的 1GHz、1V 正弦（差分 ±2V），仿真 200ns，约 400 个边沿
//   检查：
//     1. 传播延迟（输入判决 → Q 50%）均值 = propagation_delay + tr_tf / 2
//        （模型先延迟 propagation_delay，再以 tr_tf 线性成形，50% 点再晚 tr_tf / 2）
//     2. 抖动（传播延迟标准差）= jitter
//     3. 所有边沿延迟落在均值 ±6σ 内
// ============================================================
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
        const sc_core::sc_time timestep(0.1, sc_core::SC_PS);
        sine = new SINE("sine", 0, 1.0, 1e9, 0, 0, 0, 0, timestep);
        sinen = new SINE("sinen", 0, 1.0, 1e9, 0, 0, 180, 0, timestep);
        sine->out(d);
        sinen->out(dn);

        m_cfgs = ClockBufferDF::get_configs_from_json("../../json/ClockBufferDF.json");
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

        SC_THREAD(testbench);
	}

	~Simulator()
    {
        delete m_mon;
        delete m_buf;
        delete sine;
        delete sinen;
    }
private:
    ClockBufferDF::configs m_cfgs;
    SINE* sine;
    SINE* sinen;
    ClockBufferDF* m_buf;
    ClockBufferDFMonitor* m_mon;
    size_t m_fail;

    void testbench(){
        wait(sc_core::sc_time(200, sc_core::SC_NS));

        // 每个 Q 50% 沿与其之前最近的同方向输入判决沿配对
        std::vector<double> delay_ps;
        const auto& in = m_mon->in_edges;
        for(const auto& o : m_mon->q50){
            for(size_t i = in.size(); i-- > 0; ){
                if(in[i].t < o.t && in[i].rise == o.rise){
                    delay_ps.push_back((o.t - in[i].t) * 1e12);
                    break;
                }
            }
        }

        double n = delay_ps.size();
        double mean = 0.0, var = 0.0, dmin = 1e9, dmax = -1e9;
        for(double x : delay_ps){ mean += x; dmin = std::min(dmin, x); dmax = std::max(dmax, x); }
        mean /= n;
        for(double x : delay_ps) var += (x - mean) * (x - mean);
        double sigma = std::sqrt(var / n);

        const double exp_mean = m_cfgs.propagation_delay_ps + m_cfgs.tr_tf_ns * 1e3 / 2;
        const double exp_sigma = m_cfgs.jitter_ps;

        info((std::string("统计边沿数: ") + std::to_string(delay_ps.size())
              + "，输入判决沿数: " + std::to_string(in.size())).c_str());
        check("输出沿数 = 输入判决沿数", n, in.size(), 0, "", m_fail);
        check("传播延迟均值 = propagation_delay + tr_tf/2", mean, exp_mean, 0.5, "ps", m_fail);
        check("抖动（延迟标准差）= jitter", sigma, exp_sigma, 0.1 * exp_sigma, "ps", m_fail);
        check("最小延迟 >= 均值 - 6σ", dmin >= exp_mean - 6 * exp_sigma, 1, 0, "", m_fail);
        check("最大延迟 <= 均值 + 6σ", dmax <= exp_mean + 6 * exp_sigma, 1, 0, "", m_fail);
        info((std::string("延迟范围: ") + std::to_string(dmin) + " ~ " + std::to_string(dmax) + " ps").c_str());

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    sc_core::sc_set_time_resolution(1, sc_core::SC_FS);     // 0.1ps 步长需要 fs 级时间分辨率
    Simulator sim("sim");
    SIMULATOR_START(sim, 201, sc_core::SC_NS);
}
