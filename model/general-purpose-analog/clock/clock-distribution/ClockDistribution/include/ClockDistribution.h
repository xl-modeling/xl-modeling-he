#ifndef CLOCKDISTRIBUTION_H
#define CLOCKDISTRIBUTION_H

#include <systemc-ams>
#include <algorithm>
#include <vector>

#include "utils/absdelay.h"
#include "utils/configurable.h"
#include "utils/traceable.h"
#include "utils/randomtime.h"
#include "utils/transition.h"

// ============================================================
// 单路输入，多路输出的时钟分配器（扇出缓冲器）
//   - 带迟滞的比较器检测输入边沿，各路输出再生（可配置电平与上升/下降时间）
//   - 每路输出有一个固定的 skew，构造时服从 U[0, skew) 采样
//   - 每路输出在每个边沿独立采样 RMS 抖动（正态分布）
//   - 传播延迟 + skew + 抖动：每路一条 Absdelay 延迟线；上升/下降时间：Transition 边沿成形
//     （延迟线保证每个输入边沿都延迟输出，输入半周期短于传播延迟时也不丢沿）
//   - 对外提供 name / version / latency / power / area 读取接口（数值来自 json）
//
//   注意：
//     - 输出为单端同相 Q；tr_tf 为 0~100% 线性斜坡时间，半周期短于 tr_tf 时输出摆幅按比例减小
//     - 延迟线按 TDF 步长取整，ps 级抖动与 skew 须用小步长评估，见 test/ClockDistribution_timing_tb.cpp
//
// 数据来源：ON Semiconductor MC10EL15/D Rev. 8（2021-03，doc/MC10EL.pdf），10EL 系列，PECL 模式 VCC = 5.0V，25°C
//   - 1:4 扇出、fMAX 1.25GHz、片内 skew 50ps、RMS 抖动 2.6ps：Description、Table 9 AC Characteristics
//   - 传播延迟 CLK→Q（差分）470~670ps，取中值 570ps：Table 9
//   - 上升/下降时间 20%~80% 325~575ps，取最小值 325ps 换算为 0~100%：325 / 0.6 = 541.67ps：Table 9
//   - VOH 4.105V、VOL 3.210V（典型）、判决阈值取 VBB 3.70V（3.65~3.75V 中值）：Table 5 10EL PECL DC Characteristics
//   - 功耗 VCC × IEE = 5V × 25mA = 125mW（不含输出端接）：Table 5；面积取 SOIC-16 本体 9.9 × 3.9mm：封装尺寸
//   - 输入迟滞 ±50mV 手册未给出，沿用 xl-modeling
// ============================================================
SCA_TDF_MODULE(ClockDistribution)
    , public Configurable<ClockDistribution>
    , public TraceableSCA
{
public:
    using clk_in_type = sca_tdf::sca_in<double>;
    using clk_out_type = sca_tdf::sca_out<double>;
    using clk_outs_type = sc_core::sc_vector<clk_out_type>;

    clk_in_type     iclk;
    clk_outs_type   oclks;

    CFGS_DECLARE(
        std::string module_name;
        double version;
        double latency_ps;            // 传播延迟 (ps)
        double power_mw;              // 功耗 (mW)
        double area_mm2;              // 面积 (mm²)

        size_t output_n;
        double skew_ps;
        double jitter_ps;
        double propagation_delay_ps;
        double tr_tf_ps;
        double voh_v;
        double vol_v;
        double vth_v;
    );

    // json 为扁平格式（各字段 {type, value, unit, editable}），不含 meta/parameter/configuration，
    // 因此不使用 CFGS_BEGIN/CFGS_END，直接读取顶层字段
    CFGS_FUNC_DECLARE{
        std::ifstream f(json_path, std::ios::in);
        if (!f.is_open())
            throw std::runtime_error(std::string("Cannot open file: ") + json_path);

        nlohmann::json j = nlohmann::json::parse(f);
        configs cfgs;
        cfgs.module_name = j.at("name").at("value");
        cfgs.version = j.at("version").at("value");
        cfgs.latency_ps = j.at("latency").at("value");
        cfgs.power_mw = j.at("power").at("value");
        cfgs.area_mm2 = j.at("area").at("value");
        cfgs.output_n = j.at("output_n").at("value");
        cfgs.skew_ps = j.at("skew_ps").at("value");
        cfgs.jitter_ps = j.at("jitter_ps").at("value");
        cfgs.propagation_delay_ps = j.at("propagation_delay_ps").at("value");
        cfgs.tr_tf_ps = j.at("tr_tf_ps").at("value");
        cfgs.voh_v = j.at("voh_v").at("value");
        cfgs.vol_v = j.at("vol_v").at("value");
        cfgs.vth_v = j.at("vth_v").at("value");
        return cfgs;
    }

    const std::string& get_name() const { return m_cfgs.module_name; }
    double get_version() const { return m_cfgs.version; }
    double get_latency() const { return m_cfgs.latency_ps; }   // ps
    double get_power() const { return m_cfgs.power_mw; }       // mW
    double get_area() const { return m_cfgs.area_mm2; }        // mm²

    ClockDistribution(sc_core::sc_module_name name, const configs& cfgs)
    : iclk("iclk"), oclks("oclk", cfgs.output_n)
    , m_cfgs(cfgs)
    , m_propagation_delay(cfgs.propagation_delay_ps, sc_core::SC_PS)
    , m_tr_tf(cfgs.tr_tf_ps, sc_core::SC_PS)
    , m_voh(cfgs.voh_v)
    , m_vol(cfgs.vol_v)
    , m_vth(cfgs.vth_v)
    , m_jitter(sc_core::SC_ZERO_TIME, sc_core::sc_time(cfgs.jitter_ps, sc_core::SC_PS))
    , m_maxdelay((cfgs.propagation_delay_ps + cfgs.skew_ps + 6.0 * cfgs.jitter_ps) * 1e-12)   // 传播延迟 + 最大 skew + 6σ 抖动
    , m_last_state(false)
    {
        UniformRandomTime gen_skew(sc_core::SC_ZERO_TIME, sc_core::sc_time(cfgs.skew_ps, sc_core::SC_PS));
        m_skews.reserve(cfgs.output_n);
        m_tds.reserve(cfgs.output_n);
        m_delays.reserve(cfgs.output_n);
        m_trans.reserve(cfgs.output_n);
        for(size_t i = 0; i < cfgs.output_n; i++){
            m_skews.emplace_back(gen_skew(), sc_core::SC_SEC);     // gen_skew() 返回值单位为秒
            m_tds.push_back(m_propagation_delay.to_seconds() + m_skews.back().to_seconds());
            m_delays.emplace_back(this, m_maxdelay);
            m_trans.emplace_back(this, cfgs.vol_v);
        }
    }

    ~ClockDistribution() {}

    void processing() override{
        // 比较器（带迟滞）
        double vin = iclk.read();
        bool state = m_last_state;
        if (vin > m_vth + hysteresis) {
            state = true;
        } else if (vin < m_vth - hysteresis) {
            state = false;
        }

        // 状态翻转时，各路独立采样一次抖动
        if (state != m_last_state) {
            for(size_t i = 0; i < m_tds.size(); i++)
                m_tds[i] = std::min(std::max(m_propagation_delay.to_seconds() + m_skews[i].to_seconds() + m_jitter(), 0.0), m_maxdelay);
        }
        m_last_state = state;

        // 各路：延迟线延迟（传播延迟 + skew + 抖动），再做上升/下降沿成形
        double target = state ? m_voh : m_vol;
        for(size_t i = 0; i < oclks.size(); i++)
            oclks[i].write(m_trans[i](m_delays[i](target, m_tds[i]), 0.0, m_tr_tf.to_seconds()));
    }

    void trace_me(sca_util::sca_trace_file* tf) override{
        TRACE_SCA_SIGNAL(iclk);
        for(const auto& oclk : oclks)
            sca_util::sca_trace( tf, oclk, oclk.name() );
    }
private:
    configs m_cfgs;
    sc_core::sc_time m_propagation_delay;
    sc_core::sc_time m_tr_tf;
    double m_voh;                                       // 输出高电平
    double m_vol;                                       // 输出低电平
    double m_vth;                                       // 输入判决阈值
    NormalRandomTime m_jitter;
    std::vector<sc_core::sc_time> m_skews;              // 各路固定偏斜
    std::vector<double> m_tds;                          // 各路当前边沿的总延迟（秒）
    double m_maxdelay;                                  // 延迟线最大延迟（秒）
    std::vector<Absdelay> m_delays;                     // 各路延迟线
    std::vector<Transition> m_trans;                    // 各路输出过渡（仅边沿成形）
    bool m_last_state;

    static constexpr double hysteresis = 0.05;  // 50mV 迟滞
};


#endif  // CLOCK_DISTRIBUTION_H
