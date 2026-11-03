#ifndef CLOCKDISTRIBUTION_MONITOR_H
#define CLOCKDISTRIBUTION_MONITOR_H

#include <systemc-ams>

#include <algorithm>
#include <vector>

// ============================================================
// ClockDistribution 测试用监视器（TDF）
//   - 每个步长采样 ICLK / OCLK[k]，相邻采样点间线性插值求越阈时刻
//   - 输入判决沿：与模型一致的迟滞判决（低→高越过 vth + hys 为上升沿，高→低越过 vth - hys 为下降沿）
//   - 每路输出沿：OCLK[k] 越过 10% / 50% / 90% 电平
//   - 每路输出全程最大最小值，以及窗口 [win_t0, win_t1) 内的最大最小值
// ============================================================
SCA_TDF_MODULE(ClockDistributionMonitor)
{
    sca_tdf::sca_in<double> iclk;
    sc_core::sc_vector<sca_tdf::sca_in<double>> oclks;

    struct Edge {
        double t;       // 越阈时刻 (s)
        bool rise;      // true: 上升沿
    };
    using Edges = std::vector<Edge>;

    Edges in_edges;                         // 输入判决沿
    std::vector<Edges> o50, o10, o90;       // 各路输出 50% / 10% / 90% 沿
    std::vector<double> o_max, o_min;       // 各路全程最大 / 最小值
    std::vector<double> w_max, w_min;       // 各路窗口内最大 / 最小值

    ClockDistributionMonitor(sc_core::sc_module_name nm, size_t n, double voh, double vol, double vth, double hys,
                             double win_t0 = 0.0, double win_t1 = 0.0)
        : iclk("iclk"), oclks("oclk", n)
        , o50(n), o10(n), o90(n)
        , o_max(n, -1e9), o_min(n, 1e9), w_max(n, -1e9), w_min(n, 1e9)
        , m_voh(voh), m_vol(vol), m_vth(vth), m_hys(hys), m_win_t0(win_t0), m_win_t1(win_t1)
        , m_vo(n, 0.0)
    {}

    void processing() override{
        double t = get_time().to_seconds();
        double vi = iclk.read();

        if(m_init){
            if(!m_state && vi > m_vth + m_hys){
                in_edges.push_back({interp(m_t, t, m_vi, vi, m_vth + m_hys), true});
                m_state = true;
            } else if(m_state && vi < m_vth - m_hys){
                in_edges.push_back({interp(m_t, t, m_vi, vi, m_vth - m_hys), false});
                m_state = false;
            }
        }

        for(size_t k = 0; k < oclks.size(); k++){
            double vo = oclks[k].read();
            o_max[k] = std::max(o_max[k], vo);
            o_min[k] = std::min(o_min[k], vo);
            if(t >= m_win_t0 && t < m_win_t1){
                w_max[k] = std::max(w_max[k], vo);
                w_min[k] = std::min(w_min[k], vo);
            }
            if(m_init){
                cross(o50[k], t, m_vo[k], vo, m_vol + 0.5 * (m_voh - m_vol));
                cross(o10[k], t, m_vo[k], vo, m_vol + 0.1 * (m_voh - m_vol));
                cross(o90[k], t, m_vo[k], vo, m_vol + 0.9 * (m_voh - m_vol));
            }
            m_vo[k] = vo;
        }
        m_init = true;
        m_t = t; m_vi = vi;
    }

    // [t0, t1) 区间内的边沿
    static Edges window(const Edges& edges, double t0, double t1){
        Edges out;
        for(const Edge& e : edges)
            if(e.t >= t0 && e.t < t1) out.push_back(e);
        return out;
    }

private:
    static double interp(double t0, double t1, double v0, double v1, double lvl){
        return t0 + (lvl - v0) / (v1 - v0) * (t1 - t0);
    }
    void cross(Edges& edges, double t, double v0, double v1, double lvl){
        if(v0 < lvl && v1 >= lvl) edges.push_back({interp(m_t, t, v0, v1, lvl), true});
        if(v0 > lvl && v1 <= lvl) edges.push_back({interp(m_t, t, v0, v1, lvl), false});
    }

    const double m_voh, m_vol, m_vth, m_hys, m_win_t0, m_win_t1;
    bool m_init = false;
    bool m_state = false;           // 与模型初始判决状态一致（低）
    double m_t = 0.0, m_vi = 0.0;
    std::vector<double> m_vo;
};

#endif // CLOCKDISTRIBUTION_MONITOR_H
