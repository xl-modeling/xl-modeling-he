#ifndef CLOCKBUFFERSE_H
#define CLOCKBUFFERSE_H

#include <systemc-ams>

#include <algorithm>

#include "utils/absdelay.h"
#include "utils/configurable.h"
#include "utils/traceable.h"
#include "utils/randomtime.h"
#include "utils/transition.h"

// ============================================================
// 单路输入，单路输出的单端时钟缓冲器
//   - 输入判决：按 VIH / VIL 阈值，两者之间保持上一状态
//   - 传播延迟 + 抖动：Absdelay 延迟线（每个输入边沿抽一次抖动）
//   - 上升/下降时间：Transition 边沿成形
//   - 对外提供 name / version / latency / power / area 读取接口（数值来自 json）
//
//   注意：模型在 TDF 固定步长上工作，延迟线按步长取整，传播延迟的分辨率受仿真步长限制，
//        精确评估传播延迟见 test/ClockBufferSE_timing_tb.cpp（步长 0.1ps，时间分辨率 fs）
//
// 数据来源：BXL-R12 数据手册（北京微电子技术研究所，2026-04-21，doc/BXL-R12-芯粒数据手册（公开）.pdf）
//   - 5 路 LVCMOS 输出、fin / fout 200MHz、IDD 298mA、PWd 0.99W：表 1、表 4 电特性典型测试结果
//   - 芯粒尺寸 1203×1559µm（面积 1.875mm²）：表 5 物理尺寸典型测试结果
//   - 传播延迟、上升/下降时间、抖动、输入阈值 VIH/VIL、输出电平 VOH/VOL 手册未给出，沿用 xl-modeling BXL_R12.perf.json
// ============================================================
SCA_TDF_MODULE(ClockBufferSE)
    , public Configurable<ClockBufferSE>
    , public TraceableSCA
{
    // 输入端口
    sca_tdf::sca_in<double> iclk;

    // 输出端口
    sca_tdf::sca_out<double> oclk;

    CFGS_DECLARE(
        std::string module_name;
        double version;
        double latency_ps;            // 传播延迟 (ps)
        double power_mw;              // 功耗 (mW)
        double area_mm2;              // 面积 (mm²)
        double tr_tf_ps;
        double jitter_ps;
        double propagation_delay_ps;
        double voh_v;
        double vol_v;
        double vih_v;
        double vil_v;
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
        cfgs.tr_tf_ps = j.at("tr_tf_ps").at("value");
        cfgs.jitter_ps = j.at("jitter_ps").at("value");
        cfgs.propagation_delay_ps = j.at("propagation_delay_ps").at("value");
        cfgs.voh_v = j.at("voh_v").at("value");
        cfgs.vol_v = j.at("vol_v").at("value");
        cfgs.vih_v = j.at("vih_v").at("value");
        cfgs.vil_v = j.at("vil_v").at("value");
        return cfgs;
    }

    const std::string& get_name() const { return m_cfgs.module_name; }
    double get_version() const { return m_cfgs.version; }
    double get_latency() const { return m_cfgs.latency_ps; }   // ps
    double get_power() const { return m_cfgs.power_mw; }       // mW
    double get_area() const { return m_cfgs.area_mm2; }        // mm²

    void trace_me(sca_util::sca_trace_file* tf) override{
        using namespace sca_util;
        sca_trace(tf, iclk, iclk.name());
        sca_trace(tf, oclk, oclk.name());
    }

    // 构造函数
    ClockBufferSE(sc_core::sc_module_name nm, configs cfgs)
        : iclk("iclk")
        , oclk("oclk")
        , m_cfgs(cfgs)
        , m_tr_tf(cfgs.tr_tf_ps, sc_core::SC_PS)
        , m_propagation_delay(cfgs.propagation_delay_ps, sc_core::SC_PS)
        , m_voh(cfgs.voh_v)
        , m_vol(cfgs.vol_v)
        , m_vih(cfgs.vih_v)
        , m_vil(cfgs.vil_v)
        , m_jitter(sc_core::SC_ZERO_TIME, sc_core::sc_time(cfgs.jitter_ps, sc_core::SC_PS))
        , m_maxdelay((cfgs.propagation_delay_ps + 6.0 * cfgs.jitter_ps) * 1e-12)   // 传播延迟 + 6σ 抖动
        , m_delay(this, m_maxdelay)
        , m_tran(this, cfgs.vol_v)
        , m_last_state(false)
        , m_edge_jitter(0.0)
    {}

    void processing() override{
        // 1. 输入判决：高于 VIH 为高，低于 VIL 为低，两者之间保持上一状态
        double vin = iclk.read();
        bool input_state = m_last_state;
        if (vin > m_vih) {
            input_state = true;
        } else if (vin < m_vil) {
            input_state = false;
        }

        // 2. 每个输入边沿抽一次抖动，保持到下一个边沿
        if (input_state != m_last_state)
            m_edge_jitter = m_jitter();
        m_last_state = input_state;

        // 3. 传播延迟 + 抖动（限制在延迟线范围内）
        double target_output = input_state ? m_voh : m_vol;
        double tdelay = std::min(std::max(m_propagation_delay.to_seconds() + m_edge_jitter, 0.0), m_maxdelay);
        double delayed_output = m_delay(target_output, tdelay);

        // 4. 上升/下降沿成形
        oclk.write(m_tran(delayed_output, 0.0, m_tr_tf.to_seconds()));
    }
private:
    configs m_cfgs;

    // 配置参数
    sc_core::sc_time m_tr_tf;                 // 上升/下降时间
    sc_core::sc_time m_propagation_delay;     // 传播延迟

    double m_voh;                           // 输出高电平
    double m_vol;                           // 输出低电平
    double m_vih;                           // 输入高电平阈值
    double m_vil;                           // 输入低电平阈值

    // 抖动模型参数
    NormalRandomTime m_jitter;

    // 延迟与边沿成形
    double m_maxdelay;                      // 延迟线最大延迟 (s)
    Absdelay m_delay;
    Transition m_tran;

    // 状态
    bool m_last_state;                      // 上一次输入判决结果
    double m_edge_jitter;                   // 当前边沿的抖动 (s)
};

#endif // CLOCKBUFFERSE_H
