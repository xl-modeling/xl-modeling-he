#include "ClockGen.h"
#include "ClockGen_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include <algorithm>
#include <cmath>
#include <string>

// ============================================================
// ClockGen 时序测试（模型步长 10ps，仿真 100µs，3200 个周期）
//   沿时刻精确到亚步长，无需减小步长
//   各沿 50% 点理想时刻：上升沿 skew + k·T + tr_tf/2，下降沿再加 duty·T
//   检查（上升沿、下降沿分别统计）：
//     1. 沿数 = 3200
//     2. 沿时刻误差均值 = 0（含时钟偏斜 skew），容差约 5 倍均值标准误
//     3. 沿时刻误差标准差 = jitter（误差 < 5%）
//     4. 前后两半误差均值之差 ≈ 0（无长时间漂移）
//     5. 周期抖动（相邻上升沿间隔的标准差）= √2·jitter（各沿抖动独立）
// ============================================================
SC_MODULE(Simulator)
    , public AbstractSimulator
{
public:
    sca_tdf::sca_signal<double> clk;

    SC_HAS_PROCESS(Simulator);
    explicit Simulator(sc_core::sc_module_name name)
        : sc_module(name)
        , clk("clk")
        , m_fail(0)
    {
        m_cfgs = ClockGen::get_configs_from_json("../../json/ClockGen.json");
        m_clkgen = new ClockGen(m_cfgs.module_name.c_str(), m_cfgs);
        m_clkgen->clk(clk);

        m_mon = new ClockGenMonitor("mon", m_cfgs.voh_v, m_cfgs.vol_v);
        m_mon->clk(clk);

        SC_THREAD(testbench);
    }

    ~Simulator(){
        delete m_mon;
        delete m_clkgen;
    }

private:
    ClockGen::configs m_cfgs;
    ClockGen* m_clkgen;
    ClockGenMonitor* m_mon;
    size_t m_fail;

    static void stats(const std::vector<double>& x, double& mean, double& sigma){
        mean = 0.0;
        for(double v : x) mean += v;
        mean /= x.size();
        double var = 0.0;
        for(double v : x) var += (v - mean) * (v - mean);
        sigma = std::sqrt(var / x.size());
    }

    void check_edges(const char* name, const std::vector<double>& edges, double offset){
        const double T = 1.0 / m_cfgs.freq_hz;
        const double base = m_cfgs.skew_ps * 1e-12 + offset + m_cfgs.tr_tf_ps * 1e-12 / 2;
        const double sigma_ps = m_cfgs.jitter_ps;

        std::vector<double> err_ps;
        for(size_t k = 0; k < edges.size(); k++)
            err_ps.push_back((edges[k] - (base + k * T)) * 1e12);

        double mean, sigma, m1, s1, m2, s2;
        size_t h = err_ps.size() / 2;
        stats(err_ps, mean, sigma);
        stats(std::vector<double>(err_ps.begin(), err_ps.begin() + h), m1, s1);
        stats(std::vector<double>(err_ps.begin() + h, err_ps.end()), m2, s2);
        auto mm = std::minmax_element(err_ps.begin(), err_ps.end());

        std::string s(name);
        check((s + "数").c_str(), edges.size(), 3200, 0, "", m_fail);
        check((s + "时刻误差均值（含 skew）").c_str(), mean, 0.0, 5 * sigma_ps / std::sqrt(err_ps.size()), "ps", m_fail);
        check((s + "时刻误差标准差 = jitter").c_str(), sigma, sigma_ps, 0.05 * sigma_ps, "ps", m_fail);
        check((s + "前后两半误差均值之差（漂移）").c_str(), m2 - m1, 0.0, 5 * sigma_ps * std::sqrt(2.0 / h), "ps", m_fail);
        info((s + "时刻误差范围: " + std::to_string(*mm.first) + " ~ " + std::to_string(*mm.second) + " ps").c_str());
    }

    void testbench(){
        const double T = 1.0 / m_cfgs.freq_hz;
        wait(sc_core::sc_time(100, sc_core::SC_US));

        auto rise = ClockGenMonitor::times(m_mon->e50, true);
        auto fall = ClockGenMonitor::times(m_mon->e50, false);

        info("上升沿：理想 50% 时刻 = skew + k·T + tr_tf/2");
        check_edges("上升沿", rise, 0.0);
        info("下降沿：理想 50% 时刻 = skew + k·T + duty·T + tr_tf/2");
        check_edges("下降沿", fall, m_cfgs.duty_cycle * T);

        {
            info("周期抖动：相邻上升沿间隔的标准差 = √2·jitter");
            std::vector<double> per_ps;
            for(size_t i = 1; i < rise.size(); i++) per_ps.push_back((rise[i] - rise[i - 1]) * 1e12);
            double mean, sigma;
            stats(per_ps, mean, sigma);
            check("平均周期", mean / 1e3, T * 1e9, 1e-4, "ns", m_fail);
            check("周期抖动", sigma, std::sqrt(2.0) * m_cfgs.jitter_ps, 0.05 * std::sqrt(2.0) * m_cfgs.jitter_ps, "ps", m_fail);
        }

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, 100.001, sc_core::SC_US);
}
