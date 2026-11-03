#ifndef CLOCKBUFFERDF_MONITOR_H
#define CLOCKBUFFERDF_MONITOR_H

#include <systemc-ams>

#include <algorithm>
#include <cmath>
#include <vector>

// ============================================================
// ClockBufferDF 测试用监视器（TDF）
//   - 每个步长采样 DI/DN/Q/QN，相邻采样点间线性插值求越阈时刻
//   - 输入判决沿：差分越过 +hys（上升）/ -hys（下降），与模型迟滞比较器一致
//   - 输出沿：Q 越过 10% / 50% / 90% 电平
//   - 统计 Q/QN 最大最小值、|Q + QN - (VOH + VOL)| 最大值
// ============================================================
SCA_TDF_MODULE(ClockBufferDFMonitor)
{
    sca_tdf::sca_in<double> di;
    sca_tdf::sca_in<double> dn;
    sca_tdf::sca_in<double> q;
    sca_tdf::sca_in<double> qn;

    struct Edge {
        double t;       // 越阈时刻 (s)
        bool rise;      // true: 上升沿
    };

    std::vector<Edge> in_edges;     // 输入判决沿
    std::vector<Edge> q50;          // Q 50% 沿
    std::vector<Edge> q10;          // Q 10% 沿
    std::vector<Edge> q90;          // Q 90% 沿

    double q_max = -1e9, q_min = 1e9;
    double qn_max = -1e9, qn_min = 1e9;
    double compl_err = 0.0;         // max |Q + QN - (VOH + VOL)|

    ClockBufferDFMonitor(sc_core::sc_module_name nm, double voh, double vol, double hys)
        : di("di"), dn("dn"), q("q"), qn("qn")
        , m_voh(voh), m_vol(vol), m_hys(hys)
    {}

    void processing() override{
        double t = get_time().to_seconds();
        double d = di.read() - dn.read();
        double vq = q.read();
        double vqn = qn.read();

        q_max = std::max(q_max, vq);   q_min = std::min(q_min, vq);
        qn_max = std::max(qn_max, vqn); qn_min = std::min(qn_min, vqn);
        compl_err = std::max(compl_err, std::fabs(vq + vqn - (m_voh + m_vol)));

        if(m_init){
            if(m_d < m_hys && d >= m_hys)   in_edges.push_back({interp(m_t, t, m_d, d, m_hys), true});
            if(m_d > -m_hys && d <= -m_hys) in_edges.push_back({interp(m_t, t, m_d, d, -m_hys), false});
            cross(q50, t, vq, m_vol + 0.5 * (m_voh - m_vol));
            cross(q10, t, vq, m_vol + 0.1 * (m_voh - m_vol));
            cross(q90, t, vq, m_vol + 0.9 * (m_voh - m_vol));
        }
        m_init = true;
        m_t = t; m_d = d; m_q = vq;
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
        if(m_q < lvl && v >= lvl) edges.push_back({interp(m_t, t, m_q, v, lvl), true});
        if(m_q > lvl && v <= lvl) edges.push_back({interp(m_t, t, m_q, v, lvl), false});
    }

    const double m_voh, m_vol, m_hys;
    bool m_init = false;
    double m_t = 0.0, m_d = 0.0, m_q = 0.0;
};

#endif // CLOCKBUFFERDF_MONITOR_H
