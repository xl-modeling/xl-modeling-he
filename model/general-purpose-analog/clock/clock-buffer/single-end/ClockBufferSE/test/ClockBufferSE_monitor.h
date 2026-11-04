#ifndef CLOCKBUFFERSE_MONITOR_H
#define CLOCKBUFFERSE_MONITOR_H

#include <systemc-ams>

#include <algorithm>
#include <vector>

// ============================================================
// ClockBufferSE 测试用监视器（TDF）
//   - 每个步长采样 ICLK / OCLK，相邻采样点间线性插值求越阈时刻
//   - 输入判决沿：与模型一致的 VIH / VIL 双阈值判决（低→高越过 VIH 为上升沿，高→低越过 VIL 为下降沿）
//   - 输出沿：OCLK 越过 10% / 50% / 90% 电平
//   - 统计 OCLK 最大最小值
// ============================================================
SCA_TDF_MODULE(ClockBufferSEMonitor)
{
    sca_tdf::sca_in<double> iclk;
    sca_tdf::sca_in<double> oclk;

    struct Edge {
        double t;       // 越阈时刻 (s)
        bool rise;      // true: 上升沿
    };

    std::vector<Edge> in_edges;     // 输入判决沿
    std::vector<Edge> o50;          // OCLK 50% 沿
    std::vector<Edge> o10;          // OCLK 10% 沿
    std::vector<Edge> o90;          // OCLK 90% 沿

    double o_max = -1e9, o_min = 1e9;

    ClockBufferSEMonitor(sc_core::sc_module_name nm, double voh, double vol, double vih, double vil)
        : iclk("iclk"), oclk("oclk")
        , m_voh(voh), m_vol(vol), m_vih(vih), m_vil(vil)
    {}

    void processing() override{
        double t = get_time().to_seconds();
        double vi = iclk.read();
        double vo = oclk.read();

        o_max = std::max(o_max, vo);
        o_min = std::min(o_min, vo);

        if(m_init){
            if(!m_state && vi > m_vih){
                in_edges.push_back({interp(m_t, t, m_vi, vi, m_vih), true});
                m_state = true;
            } else if(m_state && vi < m_vil){
                in_edges.push_back({interp(m_t, t, m_vi, vi, m_vil), false});
                m_state = false;
            }
            cross(o50, t, vo, m_vol + 0.5 * (m_voh - m_vol));
            cross(o10, t, vo, m_vol + 0.1 * (m_voh - m_vol));
            cross(o90, t, vo, m_vol + 0.9 * (m_voh - m_vol));
        }
        m_init = true;
        m_t = t; m_vi = vi; m_vo = vo;
    }

    // [t0, t1) 区间内的边沿
    static std::vector<Edge> window(const std::vector<Edge>& edges, double t0, double t1){
        std::vector<Edge> out;
        for(const Edge& e : edges)
            if(e.t >= t0 && e.t < t1) out.push_back(e);
        return out;
    }

private:
    static double interp(double t0, double t1, double v0, double v1, double lvl){
        return t0 + (lvl - v0) / (v1 - v0) * (t1 - t0);
    }
    void cross(std::vector<Edge>& edges, double t, double v, double lvl){
        if(m_vo < lvl && v >= lvl) edges.push_back({interp(m_t, t, m_vo, v, lvl), true});
        if(m_vo > lvl && v <= lvl) edges.push_back({interp(m_t, t, m_vo, v, lvl), false});
    }

    const double m_voh, m_vol, m_vih, m_vil;
    bool m_init = false;
    bool m_state = false;           // 与模型初始判决状态一致（低）
    double m_t = 0.0, m_vi = 0.0, m_vo = 0.0;
};

#endif // CLOCKBUFFERSE_MONITOR_H
