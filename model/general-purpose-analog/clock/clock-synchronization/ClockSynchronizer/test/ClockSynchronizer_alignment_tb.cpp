#include "ClockSynchronizer.h"

#include "utils/simulator.h"
#include "utils/vsource.h"
#include "utils/verification.h"
#include "utils/nlohmann/json.hpp"

#include <cmath>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

// ============================================================
// ClockSynchronizer 通道对齐测试（1 ns 时间步，理想锁定）
//   对齐只依赖模型内部的精确边沿时刻，不依赖输出波形，因此可用粗步长缩短仿真时间
//   REFIN：100 MHz 差分方波（±350 mV）
//   1. 对齐前：上电随机相位使各通道相对参考通道存在明显偏移
//   2. 对齐耗时：(通道数 - 1) × 迭代次数 × AVG 个 BSYNC 周期
//   3. 对齐后：各通道相对参考通道的时差与目标时差（TDC_OFFSETx）之差在 ±5 ps 以内
//        - 目标时差：部分通道设为非零（含负值、超过一个粗延迟步进），其余为 0
//        - 理想时差：只由延迟设置决定，不含抖动
//        - 平均时差：对齐后 1000 个实际上升沿（含抖动）的平均
//        - 模型的 TDC 含系统误差，残差主要来自它（两通道之差在 ±tdc_accuracy 以内）
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

        const sc_core::sc_time dt(1, sc_core::SC_NS);
        m_cfgs = ClockSynchronizer::get_configs_from_json(json);
        m_sync = new ClockSynchronizer(m_cfgs.module_name.c_str(), m_cfgs, dt, false);

        m_refin = new PULSE("refin", -0.35, 0.35, 0.0, 1e-12, 1e-12, 0.5 / REFIN_HZ, 1.0 / REFIN_HZ, 0, dt);
        m_refin->out(refin);
        m_sync->refin(refin);

        bsync.init(m_cfgs.channels);
        for (int i = 0; i < m_cfgs.channels; ++i)
            m_sync->bsync[i](bsync[i]);

        // 目标时差（TDC_OFFSETx 码值，步进 ≈ 0.78125 ps）
        for (const auto& o : OFFSETS)
            m_sync->set_tdc_offset(o.first, o.second);

        SC_THREAD(testbench);
    }

    ~Simulator(){
        delete m_sync;
        delete m_refin;
    }

    // 仿真时长：锁定前的 REFIN 测量 + 对齐预计耗时 + 平均窗口 + 余量
    sc_core::sc_time sim_time() const{
        return expected_align_time() * 1.1 + sc_core::sc_time(30, sc_core::SC_US);
    }

private:
    size_t m_fail;
    ClockSynchronizer::configs m_cfgs;
    ClockSynchronizer* m_sync;
    PULSE* m_refin;

    static constexpr double REFIN_HZ = 100e6;  // REFIN 频率

    double m_align_err_ps;      // json: bsync-alignment-error-max

    static constexpr int NAVG = 1000;   // 对齐后平均的上升沿数

    // 目标时差：{通道, TDC_OFFSETx 码值}
    //   通道 2：+128 码 ≈ +100 ps（两个粗延迟步进）
    //   通道 5：-64 码  ≈ -50 ps
    //   通道 7：+9 码   ≈ +7.0 ps（不是粗延迟步进的整数倍，需要用到细延迟）
    const std::vector<std::pair<int, int>> OFFSETS = { {2, 128}, {5, -64}, {7, 9} };

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
        m_align_err_ps = value(p, "bsync-alignment-error-max");
    }

    // 预计对齐耗时：(通道数 - 1) × 迭代次数 × AVG 个 BSYNC 周期
    sc_core::sc_time expected_align_time() const{
        const double T = m_cfgs.rdiv * m_cfgs.odiv / (REFIN_HZ * m_cfgs.ndiv);
        const double avg = static_cast<double>(64L << m_cfgs.avgexp);
        return sc_core::sc_time((m_cfgs.channels - 1) * m_cfgs.align_cycles * avg * T, sc_core::SC_SEC);
    }

    void testbench(){
        const int ref = m_sync->ref_channel();

        // ---- 1. 对齐前（锁定后 1 us）----
        while (!m_sync->locked())
            wait(sc_core::sc_time(100, sc_core::SC_NS));
        wait(sc_core::sc_time(1, sc_core::SC_US));
        const double T = 1.0 / m_sync->bsync_frequency();
        info("对齐前：各通道相对参考通道的时差");
        double max_before = 0.0;
        for (int i = 0; i < m_sync->channels(); ++i){
            if (i == ref) continue;
            const double s = m_sync->ideal_skew(i) * 1e12;
            max_before = std::max(max_before, std::fabs(s));
            info(("  通道 " + std::to_string(i) + "：" + std::to_string(s) + " ps").c_str());
        }
        check("对齐前最大时差（应明显大于对齐误差）", max_before > 10.0 * m_align_err_ps, 1, 0, "", m_fail);

        // ---- 2. 等待对齐完成 ----
        while (!m_sync->aligned())
            wait(m_sync->tdc_time());
        info("对齐完成");
        const double t_align = (m_sync->align_done_time() - m_sync->align_start_time()).to_seconds();
        const double t_exp = expected_align_time().to_seconds();
        check("对齐耗时（相对预计值）", t_align / t_exp, 1.0, 0.01, "", m_fail);

        // ---- 3. 对齐后：理想时差 ----
        info("对齐后：各通道相对参考通道的时差（期望值为目标时差）");
        for (int i = 0; i < m_sync->channels(); ++i){
            if (i == ref) continue;
            const double tgt = m_sync->tdc_offset(i) * 1e12;
            const double tdc_err = (m_sync->tdc_error(i) - m_sync->tdc_error(ref)) * 1e12;
            info(("  通道 " + std::to_string(i) + "：目标 " + std::to_string(tgt) +
                  " ps，TDC 系统误差 " + std::to_string(tdc_err) + " ps").c_str());
            check(("通道 " + std::to_string(i) + " 理想时差").c_str(),
                  m_sync->ideal_skew(i) * 1e12, tgt, m_align_err_ps, "ps", m_fail);
        }

        // ---- 4. 对齐后：实际上升沿平均时差 ----
        std::vector<double> sum(m_sync->channels(), 0.0);
        for (int k = 0; k < NAVG; ++k){
            wait(sc_core::sc_time(T, sc_core::SC_SEC));
            for (int i = 0; i < m_sync->channels(); ++i)
                sum[i] += m_sync->edge_skew(i);
        }
        for (int i = 0; i < m_sync->channels(); ++i){
            if (i == ref) continue;
            check(("通道 " + std::to_string(i) + " 平均时差（" + std::to_string(NAVG) + " 个上升沿）").c_str(),
                  sum[i] / NAVG * 1e12, m_sync->tdc_offset(i) * 1e12, m_align_err_ps, "ps", m_fail);
        }

        info(("验证结束，失败项数：" + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, sim.sim_time().to_seconds(), sc_core::SC_SEC);
}
