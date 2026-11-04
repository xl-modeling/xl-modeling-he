#ifndef ANALOGDELAYLINE_MONITOR_H
#define ANALOGDELAYLINE_MONITOR_H

#include <systemc-ams>

#include <cmath>

// ============================================================
// AnalogDelayLine 测试用单频响应测量器（TDF）
//   在 [t0, t1) 内对输出做最小二乘拟合 y ≈ a·sin(ωt) + b·cos(ωt)，
//   得到幅度 A = √(a² + b²) 与相位 φ（y = A·sin(ωt + φ)）
//   输入为 sin(ωt) 时：插损 = 20·log10(A)，相位延时 = -φ / ω
// ============================================================
SCA_TDF_MODULE(SineFitter)
{
    sca_tdf::sca_in<double> y;

    SineFitter(sc_core::sc_module_name nm, double freq, double t0, double t1)
        : y("y"), m_w(2.0 * M_PI * freq), m_t0(t0), m_t1(t1) {}

    void processing() override{
        double t = get_time().to_seconds();
        if(t < m_t0 || t >= m_t1) return;
        double sn = std::sin(m_w * t), cs = std::cos(m_w * t), v = y.read();
        m_ss += sn * sn; m_cc += cs * cs; m_sc += sn * cs; m_ys += v * sn; m_yc += v * cs;
    }

    double gain_db() const { double a, b; coef(a, b); return 20.0 * std::log10(std::hypot(a, b)); }
    double phase_delay_ps() const { double a, b; coef(a, b); return -std::atan2(b, a) / m_w * 1e12; }

private:
    void coef(double& a, double& b) const {
        double det = m_ss * m_cc - m_sc * m_sc;
        a = (m_ys * m_cc - m_yc * m_sc) / det;
        b = (m_yc * m_ss - m_ys * m_sc) / det;
    }

    const double m_w, m_t0, m_t1;
    double m_ss = 0, m_cc = 0, m_sc = 0, m_ys = 0, m_yc = 0;
};

#endif // ANALOGDELAYLINE_MONITOR_H
