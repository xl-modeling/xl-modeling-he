#ifndef MODULES_CPPLL_H
#define MODULES_CPPLL_H

#include <systemc-ams>

#include <cassert>

#include "utils/traceable.h"

/**
 * @struct CPPLL
 * @brief Charge Pump Phase Locked Loop（电荷泵锁相环）模块
 * 
 * 结构：vref → PFD → CP → LPF → VCO ─┬→ MFD(÷mdiv) → out
 *              ↑                     │
 *              └──── NFD(÷ndiv) ←────┘
 *
 * 锁定后：f_vco = ndiv · f_ref，f_out = f_vco / mdiv
 * 
 */
SC_MODULE(CPPLL)
    , public TraceableSCA
{
    /**
     * @brief CPPLL构造函数
     * @param name 模块名称
     * @param pd_vih PFD 输入判决阈值（vref、vfb 超过该电压视为高电平），单位：V
     * @param cp_icp 电荷泵电流，单位：A
     * @param lpf_cap 环路滤波器积分电容 C1，单位：F
     * @param lpf_res 环路滤波器串联电阻 R，单位：Ω。为环路提供零点，闭环稳定所必需，不应为 0
     * @param lpf_c2 环路滤波器并联纹波电容 C2，单位：F。为 0 时退化为 R + C1，一般取 C1/10 ~ C1/20
     * @param vco_kvco VCO 压控增益，单位：Hz/V
     * @param vco_f0 VCO 自由振荡频率（控制电压为 0 时），单位：Hz
     * @param vco_vh VCO 及分频器输出高电平，单位：V
     * @param vco_vl VCO 及分频器输出低电平，单位：V
     * @param ndiv 反馈分频比 N（≥ 1），决定 f_vco = N · f_ref。N 增大会使环路等效增益变为 Kvco/N，环路变慢、阻尼减小
     * @param mdiv 输出分频比 M（≥ 1），f_out = f_vco / M，不影响环路动态
     * @param lock_phase_threshold lock 相位误差阈值，单位：rad。每个参考周期内 up/down 脉冲宽度折算的相位误差
     *                             不超过该值视为达标；应明显大于量化误差 2π·dt/Tref（建议 ≥ 3 倍）
     * @param lock_cycle_threshold lock 周期阈值。连续达标的参考周期数达到该值后 lock 置 1，任一时刻超出阈值立即置 0；
     *                             建议 该值 × Tref 覆盖环路振荡周期（约 1/fn）
     */
    CPPLL(sc_core::sc_module_name name, double pd_vih, double cp_icp, double lpf_cap, double lpf_res, double lpf_c2, double vco_kvco, double vco_f0
        , double vco_vh, double vco_vl, size_t ndiv, size_t mdiv, double lock_phase_threshold, size_t lock_cycle_threshold)
    : m_pfd("PFD", pd_vih, lock_phase_threshold, lock_cycle_threshold)
    , m_cp("CP", cp_icp)
    , m_lpf("LPF", lpf_cap, lpf_res, lpf_c2)
    , m_vco("VCO", vco_kvco, vco_f0, vco_vh, vco_vl)
    , m_ndiv("NFD", vco_vh, vco_vl, ndiv)
    , m_mdiv("MFD", vco_vh, vco_vl, mdiv)
    {
        m_pfd.vfb(vfb);
        m_pfd.vup(vup);
        m_pfd.vdown(vdown);

        m_cp.vup(vup);
        m_cp.vdown(vdown);
        m_cp.iout(iout);

        m_lpf.iin(iout);
        m_lpf.vout(vout);

        m_vco.vin(vout);
        m_vco.vout(vco_out);

        m_ndiv.vin(vco_out);
        m_ndiv.vout(vfb);

        m_mdiv.vin(vco_out);
    }

    void trace_me(sca_util::sca_trace_file* tf) override{
        m_pfd.trace_me(tf);
        m_cp.trace_me(tf);
        m_lpf.trace_me(tf);
        m_vco.trace_me(tf);
        m_ndiv.trace_me(tf);
        m_mdiv.trace_me(tf);
    }

    void in(sca_tdf::sca_signal<double>& sig) { m_pfd.vref(sig); }
    void out(sca_tdf::sca_signal<double>& sig) { m_mdiv.vout(sig); }
    void lock(sca_tdf::sca_signal<bool>& sig) { m_pfd.lock(sig); }

    /**
     * @struct PFD
     * @brief Phase Frequency Detector（相位频率检测器）模块
     */
    SCA_TDF_MODULE(PFD)
        , public TraceableSCA
    {
        /**
         * @brief PFD构造函数
         * @param name 模块名称
         * @param vih 输入高电平电压，单位：V
         * @param phase_threshold lock 相位误差阈值，单位：rad。每个参考周期内 up/down 脉冲宽度折算的相位误差
         *                        （2π·τ/Tref）不超过该值视为达标；应明显大于量化误差 2π·dt/Tref（建议 ≥ 3 倍）
         * @param cycle_threshold lock 周期阈值。连续达标的参考周期数达到该值后 lock 置 1，任一时刻超出阈值立即置 0
         *                        （vref 消失时 up/down 持续为高，可据此检测失锁）；建议 该值 × Tref 覆盖环路振荡周期（约 1/fn）
         */
        PFD(sc_core::sc_module_name name, double vih, double phase_threshold, size_t cycle_threshold) 
        : m_up(false), m_down(false)
        , m_vref_last(0.0), m_vfb_last(0.0)
        , m_vih(vih)
        , m_locker(phase_threshold, cycle_threshold)
        {}

        sca_tdf::sca_in<double> vref;
        sca_tdf::sca_in<double> vfb;
        sca_tdf::sca_out<bool> vup;
        sca_tdf::sca_out<bool> vdown;
        sca_tdf::sca_out<bool> lock;

        void processing() override{
            double vref_cur = vref.read();
            double vfb_cur = vfb.read();
            double tcur = this->get_time().to_seconds();
            double dt = this->get_timestep().to_seconds();
            bool up = m_vref_last < m_vih && vref_cur > m_vih;
            bool down = m_vfb_last < m_vih && vfb_cur > m_vih;
            
            m_up = m_up || up;
            m_down = m_down || down;
            
            bool up_tmp = m_up && !m_down;
            bool down_tmp = m_down && !m_up;
            
            m_up = up_tmp;
            m_down = down_tmp;
            
            bool islock = m_locker.update_lock(up, tcur, dt, m_up, m_down);
            m_vref_last = vref_cur;
            m_vfb_last = vfb_cur;

            lock.write(islock);
            vup.write(m_up);
            vdown.write(m_down);
        }

        void trace_me(sca_util::sca_trace_file* tf) override{
            TRACE_SCA_SIGNAL(vref);
            TRACE_SCA_SIGNAL(vfb);
            TRACE_SCA_SIGNAL(vup);
            TRACE_SCA_SIGNAL(vdown);
            TRACE_SCA_SIGNAL(lock);
        }
    private:
        struct LockDetector{
            static constexpr double PI = 3.14159265358979323846;
            LockDetector(double phase_threshold, size_t cycle_threshold)
            : m_phase_threshold(phase_threshold), m_cycle_threshold(cycle_threshold)
            , m_cycle(0),m_phase(0.0), m_tlast(0.0), m_tref(0.0), m_lock(false) 
            {}

            bool update_lock(bool rise, double tcur, double dt, bool up, bool down){
                if(m_tref > 0.0) { m_phase = (up || down) ? (m_phase + 2 * PI * dt / m_tref) : m_phase; }

                if(m_phase > m_phase_threshold){ m_cycle = 0; m_lock = false; }

                if(rise){
                    m_tref = tcur - m_tlast;
                    if(m_phase <= m_phase_threshold){
                        m_cycle++;
                        if(m_cycle >= m_cycle_threshold){
                            m_lock = true;
                            m_cycle = m_cycle_threshold;
                        }
                    }
                    m_tlast = tcur;
                    m_phase = 0.0;
                }

                return m_lock;
            }
        private:
            const double m_phase_threshold;
            const size_t m_cycle_threshold;
            size_t m_cycle;
            double m_phase;
            double m_tlast;
            double m_tref;
            bool m_lock;
        };
        bool m_up;
        bool m_down;
        double m_vref_last;
        double m_vfb_last;
        const double m_vih;
        LockDetector m_locker;
    };

    /**
     * @struct CP
     * @brief Charge Pump（电荷泵）模块
     */
    SCA_TDF_MODULE(CP)
        , public TraceableSCA
    {
        /**
         * @brief CP构造函数
         * @param name 模块名称
         * @param icp 电荷泵电流，单位：A
         */
        CP(sc_core::sc_module_name name, double icp) : m_icp(icp) {}

        sca_tdf::sca_in<bool> vup;
        sca_tdf::sca_in<bool> vdown;
        sca_tdf::sca_out<double> iout;

        void processing() override{
            bool up = vup.read();
            bool down = vdown.read();
            iout.write( !(up ^ down) ? 0.0 : (up ? m_icp : (down ? -m_icp : 0.0)) );
        }

        void trace_me(sca_util::sca_trace_file* tf) override{
            TRACE_SCA_SIGNAL(vup);
            TRACE_SCA_SIGNAL(vdown);
            TRACE_SCA_SIGNAL(iout);
        }
    private:
        const double m_icp;
    };

    /**
     * @struct LPF
     * @brief Loop Filter（环路滤波器）模块
     * 二阶低通滤波器：(R 串联 C1) 并联 C2，输出取 C2 两端电压
     *
     *   iin ──┬──────┬── vout
     *         R      C2
     *         C1     │
     *         ┴      ┴
     */
    SCA_TDF_MODULE(LPF)
        , public TraceableSCA
    {
        /**
         * @brief LPF构造函数
         * @param name 模块名称
         * @param cap 积分电容 C1，单位：F
         * @param res 与 C1 串联的电阻 R，单位：Ω（默认 0）。R 为环路提供零点，闭环 CPPLL 稳定所必需。
         * @param c2  并联纹波电容 C2，单位：F（默认 0，退化为 R + C1）。
         *            用于抑制 Icp·R 引起的控制电压跳变，一般取 C1/10 ~ C1/20。
         */
        LPF(sc_core::sc_module_name name, double cap, double res = 0.0, double c2 = 0.0)
        : m_cap(cap), m_res(res), m_c2(c2), m_vc1(0.0), m_vc2(0.0) {}

        sca_tdf::sca_in<double> iin;
        sca_tdf::sca_out<double> vout;

        void processing() override{
            double dt = this->get_timestep().to_seconds();
            double i = iin.read();

            if(m_c2 <= 0.0){
                // R + C1：输出 = 电容电压 + 电阻压降
                m_vc1 += i / m_cap * dt;
                vout.write(m_vc1 + i * m_res);
            }
            else if(m_res <= 0.0){
                // R = 0：C1 与 C2 直接并联
                m_vc2 += i / (m_cap + m_c2) * dt;
                m_vc1 = m_vc2;
                vout.write(m_vc2);
            }
            else{
                // 二阶：前向欧拉，要求 dt << R*C1*C2/(C1+C2)
                double ir = (m_vc2 - m_vc1) / m_res;   // 流经 R、C1 支路的电流
                m_vc1 += ir / m_cap * dt;
                m_vc2 += (i - ir) / m_c2 * dt;
                vout.write(m_vc2);
            }
        }

        void trace_me(sca_util::sca_trace_file* tf) override{
            TRACE_SCA_SIGNAL(iin);
            TRACE_SCA_SIGNAL(vout);
        }
    private:
        const double m_cap;
        const double m_res;
        const double m_c2;
        double m_vc1;
        double m_vc2;
    };

    /**
     * @struct VCO
     * @brief Voltage Controlled Oscillator（压控振荡器）模块
     */
    SCA_TDF_MODULE(VCO)
        , public TraceableSCA
    {
        /**
         * @brief VCO构造函数
         * @param name 模块名称
         * @param kvco 压控增益，单位：Hz/V
         * @param default_freq 自由振荡频率 f0（vin = 0 时），单位：Hz
         * @param vh 输出高电平，单位：V
         * @param vl 输出低电平，单位：V
         */
        VCO(sc_core::sc_module_name name, double kvco, double default_freq, double vh, double vl)
        : m_kvco(kvco), m_vh(vh), m_vl(vl)
        , m_f0(default_freq), m_freq(default_freq), m_phase(0.0), m_vout(false) {}

        sca_tdf::sca_in<double> vin;
        sca_tdf::sca_out<double> vout;

        void set_attributes() override{
            vout.set_delay(1);
        }

        void processing() override{
            double dt = this->get_timestep().to_seconds();
            // 线性 VCO：f = f0 + Kvco * vin，并限制 f > 0
            m_freq = m_f0 + m_kvco * vin.read();
            if(m_freq < 1.0) m_freq = 1.0;

            // 对频率积分得到相位（单位：周期），保证 vin 变化时相位连续；
            // 每累积半个周期翻转一次输出
            m_phase += m_freq * dt;
            if(m_phase >= 0.5){
                m_phase -= 0.5;
                m_vout = !m_vout;
            }

            vout.write(m_vout ? m_vh : m_vl);
        }

        void trace_me(sca_util::sca_trace_file* tf) override{
            TRACE_SCA_SIGNAL(vin);
            TRACE_SCA_SIGNAL(vout);
        }
    private:
        const double m_kvco;
        const double m_vh;
        const double m_vl;
        const double m_f0;
        double m_freq;
        double m_phase;
        bool m_vout;
    };

    /**
     * @struct DIV
     * @brief Frequency Divider（频率分频器）模块
     */
    SCA_TDF_MODULE(DIV)
        , public TraceableSCA
    {
        sca_tdf::sca_in<double> vin;
        sca_tdf::sca_out<double> vout;

        DIV(sc_core::sc_module_name name, double voh, double vol, size_t div)
        : m_voh(voh), m_vol(vol), m_div(div)
        , m_vin_last(vol), m_cycle(m_div - 1), m_vout(false)
        {
            assert(div > 0);
        }

        void processing() override{
            double vi = vin.read();
            if(m_vin_last != vi){
                m_cycle++;
                if(m_cycle >= m_div){
                    m_cycle = 0;
                    m_vout = !m_vout;
                }
            }
            m_vin_last = vi;
            vout.write(m_vout ? m_voh : m_vol);
        }

        void trace_me(sca_util::sca_trace_file* tf) override{
            TRACE_SCA_SIGNAL(vin);
            TRACE_SCA_SIGNAL(vout);
        }
    private:
        const double m_voh;
        const double m_vol;
        const size_t m_div;
        double m_vin_last;
        size_t m_cycle;
        bool m_vout;
    };
private:
    PFD m_pfd;
    CP m_cp;
    LPF m_lpf;
    VCO m_vco;
    DIV m_ndiv;
    DIV m_mdiv;

    /* PFD */
    sca_tdf::sca_signal<double> vfb;
    sca_tdf::sca_signal<bool> vup;
    sca_tdf::sca_signal<bool> vdown;

    /* CP */
    sca_tdf::sca_signal<double> iout;

    /* LPF */
    sca_tdf::sca_signal<double> vout;

    /* VCO */
    sca_tdf::sca_signal<double> vco_out;
};

#endif 