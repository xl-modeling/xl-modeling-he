#include "ClockGen.h"
#include "ClockGen_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include <cmath>
#include <string>

// ============================================================
// ClockGen 功能测试（模型步长 10ps，仿真 1000ns，32 个周期）
//   检查：
//     0. 模型信息接口 name / version / latency / power / area
//     1. 上电初始电平为 VOL，输出在 VOL ~ VOH 之间摆动
//     2. 频率：32 个上升沿、32 个下降沿，平均周期 = 1 / freq
//     3. 占空比 = duty_cycle
//     4. 30%~70% 上升 / 下降时间 = 0.4 × tr_tf（模型边沿为 0~100% 线性斜坡，起点在亚步长时刻，
//        30% / 70% 点离斜坡拐点 ≥ 1 个步长，采样插值精确；10% / 90% 点可能与拐点同在一个采样区间）
//   沿时刻误差、抖动与长时间稳定性见 ClockGen_timing_tb.cpp
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
        m_tf = sca_util::sca_create_tabular_trace_file("wave.dat");

        m_cfgs = ClockGen::get_configs_from_json("../../json/ClockGen.json");
        m_clkgen = new ClockGen(m_cfgs.module_name.c_str(), m_cfgs);
        m_clkgen->clk(clk);

        m_mon = new ClockGenMonitor("mon", m_cfgs.voh_v, m_cfgs.vol_v);
        m_mon->clk(clk);

        m_clkgen->trace_me(m_tf);

        SC_THREAD(testbench);
    }

    ~Simulator(){
        sca_util::sca_close_tabular_trace_file(m_tf);
        delete m_mon;
        delete m_clkgen;
    }

private:
    sca_util::sca_trace_file* m_tf;
    ClockGen::configs m_cfgs;
    ClockGen* m_clkgen;
    ClockGenMonitor* m_mon;
    size_t m_fail;

    static double mean_gap_ps(const std::vector<double>& from, const std::vector<double>& to){
        size_t n = std::min(from.size(), to.size());
        double sum = 0.0;
        for(size_t i = 0; i < n; i++) sum += to[i] - from[i];
        return sum / n * 1e12;
    }

    void testbench(){
        const double T = 1.0 / m_cfgs.freq_hz;
        const double sigma_ps = m_cfgs.jitter_ps;

        {
            info("模型信息接口");
            info((std::string("name: ") + m_clkgen->get_name()).c_str());
            check("version", m_clkgen->get_version(), 0.1, 0, "", m_fail);
            check("latency（上电稳定时间）", m_clkgen->get_latency(), 1.0, 0, "ms", m_fail);
            check("power", m_clkgen->get_power(), 0.264, 0, "mW", m_fail);
            check("area", m_clkgen->get_area(), 5.76, 0, "mm2", m_fail);
        }

        wait(sc_core::sc_time(1000, sc_core::SC_NS));

        {
            info("输出电平：上电为 VOL，在 VOL ~ VOH 之间摆动");
            check("上电初始电平 = VOL", m_mon->v_first, m_cfgs.vol_v, 1e-9, "V", m_fail);
            check("最大值 = VOH", m_mon->v_max, m_cfgs.voh_v, 1e-9, "V", m_fail);
            check("最小值 = VOL", m_mon->v_min, m_cfgs.vol_v, 1e-9, "V", m_fail);
        }

        auto rise = ClockGenMonitor::times(m_mon->e50, true);
        auto fall = ClockGenMonitor::times(m_mon->e50, false);
        {
            info("频率：1000ns 内 32 个周期");
            check("上升沿数", rise.size(), 32, 0, "", m_fail);
            check("下降沿数", fall.size(), 32, 0, "", m_fail);
            // 首末上升沿各带 σ 抖动，平均周期误差 σ = √2·σ / 31，容差取约 5.5 倍
            double period = (rise.back() - rise.front()) / (rise.size() - 1);
            check("平均周期", period * 1e9, T * 1e9, 5e-3, "ns", m_fail);
            check("频率", 1.0 / period / 1e6, m_cfgs.freq_hz / 1e6, 5e-3, "MHz", m_fail);
        }

        {
            info("占空比 = duty_cycle");
            double sum = 0.0;
            for(size_t i = 0; i < rise.size(); i++) sum += (fall[i] - rise[i]) / T;
            // 每个周期占空比误差 σ = √2·σ / T，32 个周期取均值，容差取约 5 倍
            double tol = 5 * std::sqrt(2.0) * sigma_ps * 1e-12 / T / std::sqrt(32.0);
            check("平均占空比", sum / rise.size(), m_cfgs.duty_cycle, tol, "", m_fail, 40, 6);
        }

        {
            info("上升 / 下降时间（30%~70%）= 0.4 × tr_tf");
            auto r30 = ClockGenMonitor::times(m_mon->e30, true),  r70 = ClockGenMonitor::times(m_mon->e70, true);
            auto f70 = ClockGenMonitor::times(m_mon->e70, false), f30 = ClockGenMonitor::times(m_mon->e30, false);
            check("上升时间（30%~70%）", mean_gap_ps(r30, r70), 0.4 * m_cfgs.tr_tf_ps, 0.01, "ps", m_fail);
            check("下降时间（70%~30%）", mean_gap_ps(f70, f30), 0.4 * m_cfgs.tr_tf_ps, 0.01, "ps", m_fail);
        }

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, 1001, sc_core::SC_NS);
}
