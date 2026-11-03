/**
 * @file absdelay.h
 * @brief 绝对传输延迟发生器头文件
 * 
 * @author jcl
 * @date 2026-09-30
 */

#ifndef ABSDELAY_H
#define ABSDELAY_H

#include <systemc-ams>
#include <deque>

/**
 * @class Absdelay
 * @brief 绝对传输延迟发生器
 * 
 * 把输入信号原样延迟指定时间后输出，不改变信号的幅度、上升/下降时间或波形形状，只是简单地把整条波形在时间轴上平移
 * 
 * @note 默认情况下，Absdelay 允许输入延迟时间大于最大延迟时间，此时会返回最早的延迟值。若认定这是错误，则因在构造函数中设置 throw_on_overflow 为 true，此时若溢出，则会抛出异常。
 */
class Absdelay{
public:
    /**
     * @brief Absdelay构造函数
     * @param tdf_module TDF模块指针
     */
    Absdelay(sca_tdf::sca_module* tdf_module) : m_module(tdf_module), m_rb(), m_maxdelay(0.0), m_time(0.0), m_init(false), m_throw(false) {}

    /**
     * @brief Absdelay构造函数
     * @param tdf_module TDF模块指针
     * @param maxdelay 最大延迟时间
     * @param throw_on_overflow 是否在溢出时抛出异常
     */
    Absdelay(sca_tdf::sca_module* tdf_module, double maxdelay, bool throw_on_overflow = false) : m_module(tdf_module), m_rb(), m_maxdelay(maxdelay), m_time(0.0), m_init(false), m_throw(throw_on_overflow) {}

    double operator()(double expression, double tdelay) {
        double delay_time;
        double dt = m_module->get_timestep().to_seconds();
        if(!m_init){
            if(m_maxdelay == 0.0) m_maxdelay = tdelay;
            m_rb.resize(static_cast<size_t>(m_maxdelay / dt + 1 + 1e-6), expression);
            m_init = true;
            return expression;
        }

        if(tdelay > m_maxdelay){
            if(m_throw)
                throw std::runtime_error("tdelay(" + std::to_string(tdelay) + ")is greater than maxdelay(" + std::to_string(m_maxdelay) + ")");
            else
                tdelay = m_maxdelay;
        }
        
        m_time += dt;
        m_rb.push(expression);
        
        delay_time = m_time - tdelay;
        if(delay_time < 0.0)
            return m_rb[0];
        else
            return m_rb[m_rb.size() - static_cast<size_t>(tdelay / dt + 1e-6) - 1];
    }

private:
    template<typename T>
    class RingBuffer{
        std::deque<T> deque;
    public:
        using iterator = typename std::deque<T>::iterator;
        using const_iterator = typename std::deque<T>::const_iterator;
        using reverse_iterator = typename std::deque<T>::reverse_iterator;
        using const_reverse_iterator = typename std::deque<T>::const_reverse_iterator;

        RingBuffer() {};
        RingBuffer(size_t size) : deque(size) {}

        void push(T value){
            deque.pop_front();
            deque.push_back(value);
        }

        void resize(size_t size, T value){
            deque.resize(size, value);
        }

        size_t size() const { return deque.size(); }

        iterator begin() { return deque.begin(); }
        reverse_iterator rbegin() { return deque.rbegin(); }

        iterator end() { return deque.end(); }
        reverse_iterator rend() { return deque.rend(); }

        T operator[](size_t index) { return deque[index]; }
    };

    const sca_tdf::sca_module* m_module;
    RingBuffer<double> m_rb;
    double m_maxdelay;
    double m_time;
    bool m_init;
    bool m_throw;
};

#endif 