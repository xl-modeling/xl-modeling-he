#ifndef CLOCKGEN_H
#define CLOCKGEN_H

#include <systemc-ams>

#include <algorithm>
#include <cmath>

#include "utils/configurable.h"
#include "utils/traceable.h"
#include "utils/randomtime.h"
#include "utils/transition.h"

// ============================================================
// 单端口输出的时钟发生器
//   - 可配置频率、占空比、时钟偏斜、RMS 抖动、上升/下降时间与输出电平
//   - 第 k 个沿的理想时刻直接计算（上升沿 skew + n·T，下降沿再加 duty·T），不做累加，避免浮点误差积累
//   - 每个沿独立抽一次抖动；沿落在 (t, t + dt] 内时翻转，亚步长偏移作为 Transition 延迟，沿时刻不受步长量化
//   - 对外提供 name / version / latency / power / area 读取接口（数值来自 json）
//
// 数据来源：32M-频率源芯粒手册 V1.1（清华大学，2024-12-10，doc/32M 频率源 数据手册.pdf）
//   - 输出频率 32MHz ± 0.1MHz、抖动 < 20ps（取 20ps 作为每个沿的 RMS 抖动）：表 1 芯粒参数指标
//   - 输出电平 0 ~ 3.3V：表 3 芯粒引脚清单（CLK）
//   - 功耗：简介中电流约 80µA，按 3.3V 计 0.264mW（表 1 写 < 100mA，与简介不符，未采用）
//   - 面积 2.4 × 2.4mm：第四节引脚与封装信息；latency 取上电稳定时间 1ms（模型未建模上电过程）
//   - 占空比、时钟偏斜、上升/下降时间手册未给出，沿用 xl-modeling FREF32M.perf.json
// ============================================================
SCA_TDF_MODULE(ClockGen)
    , public Configurable<ClockGen>
    , public TraceableSCA
{
public:
    // 输出端口
    sca_tdf::sca_out<double> clk;

    CFGS_DECLARE(
        std::string module_name;
        double version;
        double latency_ms;      // 上电稳定时间 (ms)
        double power_mw;        // 功耗 (mW)
        double area_mm2;        // 面积 (mm²)
        double freq_hz;         // 时钟频率
        double duty_cycle;      // 占空比 [0, 1]
        double skew_ps;         // 时钟偏斜
        double jitter_ps;       // RMS 抖动
        double tr_tf_ps;        // 上升/下降时间
        double voh_v;           // 输出高电平
        double vol_v;           // 输出低电平
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
        cfgs.latency_ms = j.at("latency").at("value");
        cfgs.power_mw = j.at("power").at("value");
        cfgs.area_mm2 = j.at("area").at("value");
        cfgs.freq_hz = j.at("freq_hz").at("value");
        cfgs.duty_cycle = j.at("duty_cycle").at("value");
        cfgs.skew_ps = j.at("skew_ps").at("value");
        cfgs.jitter_ps = j.at("jitter_ps").at("value");
        cfgs.tr_tf_ps = j.at("tr_tf_ps").at("value");
        cfgs.voh_v = j.at("voh_v").at("value");
        cfgs.vol_v = j.at("vol_v").at("value");
        return cfgs;
    }

    const std::string& get_name() const { return m_cfgs.module_name; }
    double get_version() const { return m_cfgs.version; }
    double get_latency() const { return m_cfgs.latency_ms; }   // ms
    double get_power() const { return m_cfgs.power_mw; }       // mW
    double get_area() const { return m_cfgs.area_mm2; }        // mm²

    // 构造函数
    ClockGen(sc_core::sc_module_name nm, configs cfgs)
        : clk("clk")
        , m_cfgs(cfgs)
        , m_freq(cfgs.freq_hz)
        , m_duty(cfgs.duty_cycle)
        , m_skew(cfgs.skew_ps, sc_core::SC_PS)
        , m_tr_tf(cfgs.tr_tf_ps, sc_core::SC_PS)
        , m_voh(cfgs.voh_v)
        , m_vol(cfgs.vol_v)
        , m_jitter(sc_core::SC_ZERO_TIME, sc_core::sc_time(cfgs.jitter_ps, sc_core::SC_PS))
        , m_level(false)
        , m_edge_index(0)
        , m_t_next_edge(0.0)
        , m_tran(this, cfgs.vol_v)
    {}

    void set_attributes() override{
        set_timestep(10, sc_core::SC_PS);
    }

    void initialize() override{
        // 首个上升沿：理想时刻 = 偏斜，实际时刻 = 偏斜 + 独立抖动
        m_level = false;
        m_edge_index = 0;
        m_t_next_edge = nominal_edge(m_edge_index) + m_jitter();
    }

    void processing() override{
        double t = get_time().to_seconds();
        double dt = get_timestep().to_seconds();

        // 占空比退化情形：输出恒定电平
        if (m_duty <= 0.0){ clk.write(m_vol); return; }
        if (m_duty >= 1.0){ clk.write(m_voh); return; }

        // 下一个沿落在 (t, t + dt] 内：本步翻转，沿相对本步的偏移作为过渡延迟（要求半周期大于步长）
        double td = 0.0;
        if (t + dt > m_t_next_edge){
            m_level = !m_level;
            td = std::max(m_t_next_edge - t, 0.0);
            m_t_next_edge = nominal_edge(++m_edge_index) + m_jitter();
        }

        double target = m_level ? m_voh : m_vol;
        clk.write(m_tran(target, td, m_tr_tf.to_seconds()));
    }

    void trace_me(sca_util::sca_trace_file* tf) override{
        using namespace sca_util;
        sca_trace(tf, clk, clk.name());
    }

private:
    // 第 k 个沿（k 为偶数是上升沿，奇数是下降沿）的理想时刻 (s)
    double nominal_edge(unsigned long long k) const {
        double T = 1.0 / m_freq;
        double t = m_skew.to_seconds() + static_cast<double>(k / 2) * T;
        return (k % 2) ? t + m_duty * T : t;
    }

    // 配置参数
    configs m_cfgs;
    double m_freq;                 // 时钟频率 (Hz)
    double m_duty;                 // 占空比 [0, 1]
    sc_core::sc_time m_skew;       // 时钟偏斜
    sc_core::sc_time m_tr_tf;      // 上升/下降时间
    double m_voh;                  // 输出高电平
    double m_vol;                  // 输出低电平

    // 抖动模型
    NormalRandomTime m_jitter;     // RMS 抖动源 N(0, jitter_ps)

    // 波形状态
    bool m_level;                       // 当前逻辑电平 (true=高, false=低)
    unsigned long long m_edge_index;    // 下一个沿的序号
    double m_t_next_edge;               // 下一个沿的实际时刻（理想时刻 + 抖动）(s)

    Transition m_tran;
};

#endif // CLOCKGEN_H
