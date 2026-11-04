/**
 * @file transition.h
 * @brief 为信号提供过渡功能
 *
 * @author jcl
 * @date 2026-09-18
 */

#ifndef TRANSITION_H
#define TRANSITION_H

#include <systemc-ams>

#include <utility>
#include <queue>

/**
 * @class Transition
 * @brief 为分段常数信号提供过渡功能
 * 
 * @note 信号必须是分段常数（如数字信号），不应用于连续变化的信号
 */

class Transition{
public:
    /**
     * @brief Transition构造函数
     * @param tdf_module TDF模块指针
     */
    Transition(sca_tdf::sca_module* tdf_module) 
    : m_module(tdf_module), m_vcur(0.0), m_tlocal(0.0), m_ss({0.0, 0.0, 0.0, 0.0, 0.0}) {}

    /**
     * @brief Transition构造函数
     * @param tdf_module TDF模块指针
     * @param vcur 过渡信号初始值
     */
    Transition(sca_tdf::sca_module* tdf_module, double vcur)
    : m_module(tdf_module), m_vcur(vcur), m_tlocal(0.0), m_ss({vcur, 0.0, 0.0, 0.0, 0.0}) {}

    double operator()(const sca_tdf::sca_in<double>& expr, double td, double rise_time) {
        double fall_time = rise_time;
        return operator()(expr.read(), td, rise_time, fall_time);
    }

    double operator()(const sca_tdf::sca_in<double>& expr, double td, double rise_time, double fall_time) {
        return operator()(expr.read(), td, rise_time, fall_time);
    }   

    double operator()(double expr, double td, double rise_time){
        double fall_time = rise_time;
        return operator()(expr, td, rise_time, fall_time);
    }

    double operator()(double expr, double td, double rise_time, double fall_time){
        double dt = m_module->get_timestep().to_seconds();

        /* input变化时重置局部仿真时间并更新快照 */
        if(expr != m_ss.vinss){
            m_tlocal = 0;

            bool is_rise;
            double trf, kslope, vstart;
            is_rise = m_vcur < expr ? true : false; 
            trf = is_rise ? rise_time : fall_time;

            vstart = m_vcur;
            kslope = (expr - vstart) / trf;
            m_ss = snapshot{expr, td, trf, kslope, vstart};
            return m_vcur;
        }
        
        m_tlocal += dt;
        if(m_tlocal < m_ss.td){
            return m_vcur;
        } else if(m_tlocal < m_ss.td + m_ss.trf){
            m_vcur = m_ss.vstart + m_ss.kslope * (m_tlocal - m_ss.td);
        } else {
            m_vcur = m_ss.vinss;
        }

        return m_vcur;
    }

    /* Movable*/
    Transition(Transition&& other) noexcept = default;
    Transition& operator=(Transition&& other) noexcept = default;
private:
    struct snapshot{
        double vinss;
        double td;
        double trf;

        double kslope;
        double vstart;
    };

    const sca_tdf::sca_module* m_module;

    double m_vcur;
    double m_tlocal;
    snapshot m_ss;

    Transition(const Transition&) = delete;
    Transition& operator=(const Transition&) = delete;
};
#endif