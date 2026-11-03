#ifndef CLOCKGEN_MONITOR_H
#define CLOCKGEN_MONITOR_H

#include <systemc-ams>

#include <algorithm>
#include <vector>

// ============================================================
// ClockGen 测试用监视器（TDF）
//   - 每个步长采样 CLK，相邻采样点间线性插值求越过 30% / 50% / 70% 电平的时刻
//     （模型边沿起点在亚步长时刻，10% / 90% 点距斜坡拐点仅 0.1·tr_tf，可能与拐点落在同一采样区间，
//      线性插值跨过拐点会有偏差；30% / 70% 点距拐点 0.3·tr_tf ≥ 1 个步长时插值精确）
//   - 记录首个采样值（上电初始电平）与全程最大最小值
// ============================================================
SCA_TDF_MODULE(ClockGenMonitor)
{
    sca_tdf::sca_in<double> clk;

    struct Edge {
        double t;       // 越阈时刻 (s)
        bool rise;      // true: 上升沿
    };
    using Edges = std::vector<Edge>;

    Edges e50, e30, e70;                // 50% / 30% / 70% 沿
    double v_first = 0.0;               // 首个采样值
    double v_max = -1e9, v_min = 1e9;

    ClockGenMonitor(sc_core::sc_module_name nm, double voh, double vol)
        : clk("clk"), m_voh(voh), m_vol(vol) {}

    void processing() override{
        double t = get_time().to_seconds();
        double v = clk.read();
        v_max = std::max(v_max, v);
        v_min = std::min(v_min, v);
        if(m_init){
            cross(e50, t, v, m_vol + 0.5 * (m_voh - m_vol));
            cross(e30, t, v, m_vol + 0.3 * (m_voh - m_vol));
            cross(e70, t, v, m_vol + 0.7 * (m_voh - m_vol));
        } else {
            v_first = v;
        }
        m_init = true;
        m_t = t; m_v = v;
    }

    // 指定方向的沿时刻
    static std::vector<double> times(const Edges& edges, bool rise){
        std::vector<double> out;
        for(const Edge& e : edges)
            if(e.rise == rise) out.push_back(e.t);
        return out;
    }

private:
    void cross(Edges& edges, double t, double v, double lvl){
        if(m_v < lvl && v >= lvl) edges.push_back({m_t + (lvl - m_v) / (v - m_v) * (t - m_t), true});
        if(m_v > lvl && v <= lvl) edges.push_back({m_t + (lvl - m_v) / (v - m_v) * (t - m_t), false});
    }

    const double m_voh, m_vol;
    bool m_init = false;
    double m_t = 0.0, m_v = 0.0;
};

#endif // CLOCKGEN_MONITOR_H
