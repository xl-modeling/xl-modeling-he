/**
 * @file vsource.h
 * @brief 提供SINE和PULSE在内的电压源模块
 *
 * @author jcl
 * @date 2026-09-21
 */

#ifndef VSOURCES_H
#define VSOURCES_H

#include <systemc-ams>

#include <cmath>

/**
 * @class VSource
 * @brief 所有电压源模块的基类
 * 
 * VSource为所有电压源模块的提供了统一的输出端口
 */

struct VSource{
    VSource() : out("out") {}
    virtual ~VSource() = default;
    sca_tdf::sca_out<double> out;
};

/**
 * @class DC
 * @brief DC是一个直流电压源，用于生成一个恒定的直流电压
 */
SCA_TDF_MODULE(DC)
    , VSource
{
    /**
     * @brief DC构造函数
     * @param nm 模块名称
     * @param v 直流电压，单位：v
     * @param timestep 仿真时间步长，默认10ps
     */
    DC(sc_core::sc_module_name nm, double v, sc_core::sc_time timestep = sc_core::sc_time(10, sc_core::SC_PS))
        : sca_tdf::sca_module(nm), VSource(), m_v(v), m_timestep(timestep)
    {}

    void set_attributes() override{
        set_timestep(m_timestep);
    }

    void processing() override{
        out.write(m_v);
    }

private:
    const double m_v;
    const sc_core::sc_time m_timestep;
};

/**
 * @class SINE
 * @brief SINE是一个正弦波电压源，用于生成一个正弦波信号
 *
 */
SCA_TDF_MODULE(SINE)
    , VSource
{    
    /**
     * @brief SINE构造函数
     * @param nm 模块名称
     * @param offset 偏置，单位：v
     * @param amplitude 振幅，单位：v
     * @param freq 频率，单位：Hz
     * @param tdelay 延迟时间，单位：s
     * @param theta 衰减因子，单位：s
     * @param phi 相位，单位：degree
     * @param ncycles 周期数
     * @param timestep 仿真时间步长，默认10ps
     */
    SINE(sc_core::sc_module_name nm, double offset, double amplitude, double freq, double tdelay, double theta, double phi, size_t ncycles, sc_core::sc_time timestep = sc_core::sc_time(10, sc_core::SC_PS))
    : sca_tdf::sca_module(nm)
    , m_offset(offset), m_amplitude(amplitude), m_freq(freq), m_tdelay(tdelay), m_theta(theta), m_phase(phi * M_PI/180), m_ncycles(ncycles), m_timestep(timestep) 
    , m_tcur(0), m_cycles(0)
    {}
    
    void set_attributes() override{
        set_timestep(m_timestep);
    }

    void processing() override{
        double vt;
        double t;

        t = get_time().to_seconds();
        if(t < m_tdelay){
            out.write(m_offset);
            return;
        }

        if(m_ncycles == 0 || m_cycles < m_ncycles){
            vt = m_offset + m_amplitude * sin(2.0 * M_PI * m_freq * (t - m_tdelay) + m_phase) * exp((-m_theta) * (t - m_tdelay));
            update_next();
        }else
            vt = m_offset;

        out.write(vt);
    }
private:
    const double m_offset;    
    const double m_amplitude;     
    const double m_freq;        
    const double m_tdelay;
    const double m_theta;
    const double m_phase;
    const double m_ncycles;
    const sc_core::sc_time m_timestep;

    double m_tcur;
    size_t m_cycles;

    void update_next(){
        const double dt = get_timestep().to_seconds();
        const double tcycle = 1 / m_freq;
        
        m_tcur += dt;
        if(m_tcur >= tcycle){
            m_tcur -= tcycle;
            m_cycles++;
        }
    }
};

/**
 * @class 周期脉冲电压源(PULSE)
 * @brief PULSE是一个脉冲电压源，用于生成周期性脉冲波形
 */
SCA_TDF_MODULE(PULSE)
    , VSource
{
    /**
     * @brief PULSE构造函数
     * @param nm 模块名
     * @param vinitial 初始值，单位：v
     * @param von 导通值，单位：v
     * @param tdelay 延迟时间，单位：s
     * @param trise 上升时间，单位：s
     * @param tfall 下降时间，单位：s
     * @param ton 导通持续时间，单位：s
     * @param tperiod 周期，单位：s
     * @param ncycles 循环次数
     * @param timestep 仿真时间步长，默认为10ps
     */
    PULSE(sc_core::sc_module_name nm, double vinitial, double von, double tdelay, double trise, double tfall, double ton, double tperiod, size_t ncycles, sc_core::sc_time timestep = sc_core::sc_time(10, sc_core::SC_PS))
        : sca_tdf::sca_module(nm), VSource()
        , m_vinitial(vinitial), m_von(von)
        , m_tdelay(tdelay), m_trise(trise), m_tfall(tfall), m_ton(ton), m_tperiod(tperiod), m_ncycles(ncycles), m_timestep(timestep) 
        , m_dvrise(0.0), m_dvfall(0.0)
        , m_vcur(0.0), m_tcur(0.0), m_cycles(0)
        , m_is_vinital_lt_von(vinitial < von)
    {}

    void set_attributes() override{
        set_timestep(m_timestep);
    }

    void initialize() override{
        double dt = get_timestep().to_seconds();
        double v_abs = std::abs(m_von - m_vinitial);
        m_dvrise = (m_trise > dt) ? v_abs / m_trise * dt : v_abs;     // 防止梯度爆炸
        m_dvfall = (m_tfall > dt) ? -(v_abs / m_tfall * dt) : -(v_abs);

        m_vcur = m_vinitial;
    }

    void processing() override{
        double vt;
        double t, tdelay;

        t = get_time().to_seconds();
        tdelay = m_tdelay;

        if(t < tdelay){
            out.write(m_vinitial);
            return;
        }

        if(m_ncycles == 0 || m_cycles <= m_ncycles){
            vt = m_vcur;
            update_next();
        }else
            vt = m_vinitial;
        
        out.write(vt);
    }
private:
    const double m_vinitial;
    const double m_von;
    const double m_tdelay;
    const double m_trise;
    const double m_tfall;
    const double m_ton;
    const double m_tperiod;
    const size_t m_ncycles;
    const sc_core::sc_time m_timestep;

    double m_dvrise;
    double m_dvfall;

    double m_vcur;
    double m_tcur;
    size_t m_cycles;
    const bool m_is_vinital_lt_von;

    void update_next(){
        if(m_is_vinital_lt_von)
            update_lt_path();
        else
            update_gt_path();
    }

    void update_lt_path(){
        const double dt = get_timestep().to_seconds();
        double trise, tfall, ton;

        m_tcur += dt;
        if(m_tcur < (trise = m_trise))
            m_vcur += m_dvrise;
        else if(m_tcur < trise + (ton = m_ton))
            m_vcur = m_von;
        else if(m_tcur < trise + ton + (tfall = m_tfall))
            m_vcur += m_dvfall;
        else if(m_tcur < m_tperiod)
            m_vcur = m_vinitial; 
        else{
            m_tcur = 0;
            m_vcur = m_vinitial;
            m_cycles++;
        }
    }

    void update_gt_path(){
        const double dt = get_timestep().to_seconds();
        double trise, tfall, ton;

        m_tcur += dt;
        if(m_tcur < (tfall = m_tfall))
            m_vcur += m_dvfall;
        else if(m_tcur < tfall + (ton = m_ton))
            m_vcur = m_von;
        else if(m_tcur < tfall + ton + (trise = m_trise))
            m_vcur += m_dvrise;
        else if(m_tcur < m_tperiod)
            m_vcur = m_vinitial; 
        else{
            m_tcur = 0;
            m_vcur = m_vinitial;
            m_cycles++;
        }
    }
};

#endif