#ifndef ANALOGDELAYLINE_H
#define ANALOGDELAYLINE_H

#include <systemc-ams>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "utils/configurable.h"
#include "utils/traceable.h"
#include "utils/absdelay.h"

// ============================================================
// 电调模拟延迟线（参照 SIP066）
//   H(s; Vt) = G0 · e^{-s·τ(Vt)} / (1 + s/(Q·ω0(Vt)) + s²/ω0(Vt)²)
//   - di → Absdelay 延迟 τ(Vt) → 二阶低通（截止频率 f0(Vt)，各 Vt 共用 Q）→ × G0 → dout
//   - τ(Vt)、f0(Vt) 由各 Vt 标定点线性插值，Vt 超出标定范围时钳位；Vt 变化即时生效（未建模切换时间）
//   - 二阶低通采用梯形积分（双线性变换）状态变量滤波器，系数随 Vt 逐步更新时状态连续、输出不跳变
//   - 延迟线按 TDF 步长取整，延迟分辨率等于步长；低通要求 f0 < 1/(2·步长)，超出时钳位到 0.45/步长
//   - 对外提供 name / version / latency / power / area 读取接口（数值来自 json）
//
//   精度说明：τ、f0、Q、G0 由手册曲线拟合（仅按插损曲线拟合 f0，τ 使 1GHz 相对延时与手册一致）
//     - 插损：≤ 20GHz 误差 ≤ 0.41dB
//     - 群延时：≤ 10GHz 误差 ≤ 2.9ps；10~20GHz 误差随延迟增大，Vt = 0V 时 20GHz 处约 18.6ps
//       （二阶低通无法同时复现手册中随频率上升的群延时，高频宽带信号的延时偏小）
//     - 群延时以 Vt = 15V 为基准归一化（与手册一致），器件绝对插入延迟手册未给出
//
// 数据来源：SIP066 电调延迟线 V0.2 2004（成都仕芯半导体，doc/SIP066 延迟线 产品手册（V0.2 2004）.pdf）
//   - 频率范围 DC~20GHz、可调延时 0~25ps、插损 3dB、Vt 0~15V / 10µA、芯片尺寸 2.285 × 0.77mm：电性能表
//   - 各 Vt 下的插损、归一化群延时：测试曲线（插损 VS 频率、归一化群延时 VS 频率，PDF 矢量曲线提取）
//   - 功耗按 Vt 端口上限 15V × 10µA = 0.15mW；切换时间 120ns 未建模
// ============================================================
SCA_TDF_MODULE(AnalogDelayLine)
    , public Configurable<AnalogDelayLine>
    , public TraceableSCA
{
    sca_tdf::sca_in<double> di;     // 射频输入 RFIN
    sca_tdf::sca_in<double> vt;     // 延迟控制电压 VT (V)
    sca_tdf::sca_out<double> dout;  // 射频输出 RFOUT

    CFGS_DECLARE(
        std::string module_name;
        double version;
        double latency_ps;                  // 最大可调延时 (ps)
        double power_mw;                    // 功耗 (mW)
        double area_mm2;                    // 面积 (mm²)
        std::vector<double> vt_points_v;    // 标定点 Vt (V)，升序
        std::vector<double> delay_ps;       // 各标定点延迟线延迟 τ (ps)
        std::vector<double> f0_ghz;         // 各标定点低通截止频率 f0 (GHz)
        double q;                           // 低通品质因数（各标定点共用）
        double gain_db;                     // 低频增益 G0 (dB)
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
        cfgs.vt_points_v = j.at("vt_points_v").at("value").get<std::vector<double>>();
        cfgs.delay_ps = j.at("delay_ps").at("value").get<std::vector<double>>();
        cfgs.f0_ghz = j.at("f0_ghz").at("value").get<std::vector<double>>();
        cfgs.q = j.at("q").at("value");
        cfgs.gain_db = j.at("gain_db").at("value");
        return cfgs;
    }

    const std::string& get_name() const { return m_cfgs.module_name; }
    double get_version() const { return m_cfgs.version; }
    double get_latency() const { return m_cfgs.latency_ps; }   // ps
    double get_power() const { return m_cfgs.power_mw; }       // mW
    double get_area() const { return m_cfgs.area_mm2; }        // mm²

    void trace_me(sca_util::sca_trace_file* tf) override{
        using namespace sca_util;
        sca_trace(tf, di, di.name());
        sca_trace(tf, vt, vt.name());
        sca_trace(tf, dout, dout.name());
    }

    AnalogDelayLine(sc_core::sc_module_name nm, configs cfgs)
        : di("di"), vt("vt"), dout("dout")
        , m_cfgs(cfgs)
        , m_vt(cfgs.vt_points_v)
        , m_tau(to_seconds(cfgs.delay_ps, 1e-12))
        , m_f0(to_seconds(cfgs.f0_ghz, 1e9))
        , m_k(1.0 / cfgs.q)
        , m_gain(std::pow(10.0, cfgs.gain_db / 20.0))
        , m_absdelay(this, *std::max_element(m_tau.begin(), m_tau.end()))
        , m_ic1(0.0), m_ic2(0.0), m_init(false)
    {
        if (m_vt.empty() || m_vt.size() != m_tau.size() || m_vt.size() != m_f0.size())
            throw std::invalid_argument("AnalogDelayLine: vt_points_v / delay_ps / f0_ghz 长度必须一致且非空");
        if (!std::is_sorted(m_vt.begin(), m_vt.end()))
            throw std::invalid_argument("AnalogDelayLine: vt_points_v 必须升序");
    }

    void processing() override {
        double dt = get_timestep().to_seconds();
        double v = std::min(std::max(vt.read(), m_vt.front()), m_vt.back());

        // 1. 延迟线：延迟 τ(Vt)
        double x = m_absdelay(di.read(), interp(m_tau, v));

        // 2. 二阶低通（梯形积分状态变量滤波器），首个采样按稳态初始化
        double f0 = std::min(interp(m_f0, v), 0.45 / dt);
        if (!m_init) { m_ic1 = 0.0; m_ic2 = x; m_init = true; }
        double g = std::tan(M_PI * f0 * dt);
        double a1 = 1.0 / (1.0 + g * (g + m_k));
        double a2 = g * a1;
        double a3 = g * a2;
        double v3 = x - m_ic2;
        double v1 = a1 * m_ic1 + a2 * v3;
        double v2 = m_ic2 + a2 * m_ic1 + a3 * v3;
        m_ic1 = 2.0 * v1 - m_ic1;
        m_ic2 = 2.0 * v2 - m_ic2;

        // 3. 低频插损
        dout.write(m_gain * v2);
    }

private:
    static std::vector<double> to_seconds(const std::vector<double>& v, double scale){
        std::vector<double> out(v.size());
        std::transform(v.begin(), v.end(), out.begin(), [scale](double x){ return x * scale; });
        return out;
    }

    // 按 Vt 在标定点间线性插值（v 已钳位到标定范围内）
    double interp(const std::vector<double>& y, double v) const {
        if (m_vt.size() == 1) return y[0];
        size_t i = std::upper_bound(m_vt.begin(), m_vt.end(), v) - m_vt.begin();
        i = std::min(std::max(i, size_t(1)), m_vt.size() - 1);
        double t = (v - m_vt[i - 1]) / (m_vt[i] - m_vt[i - 1]);
        return y[i - 1] + t * (y[i] - y[i - 1]);
    }

    configs m_cfgs;
    std::vector<double> m_vt;       // 标定点 Vt (V)
    std::vector<double> m_tau;      // 各标定点延迟 (s)
    std::vector<double> m_f0;       // 各标定点截止频率 (Hz)
    double m_k;                     // 1 / Q
    double m_gain;                  // 低频增益（线性）
    Absdelay m_absdelay;
    double m_ic1, m_ic2;            // 状态变量滤波器的两个积分器状态
    bool m_init;
};

#endif
