/**
 * @file randomtime.h
 * @brief 随机时间生成器
 * 
 * 如果不提供种子，则使用默认种子42
 * 
 * @author jcl
 * @date 2026-09-30
 */

#ifndef RANDOMTIME_H
#define RANDOMTIME_H

#include <random>
#include <systemc>

#include "common.h"

/**
 * @class NormalRandomTime
 * @brief 正态随机时间发生器
 * 
 * 生成符合正态分布的时间，适合模拟jitter这类随机值
 */
class NormalRandomTime{
public:
    /**
     * @brief NormalRandomTime构造函数
     * @param mean 均值
     * @param stddev 标准差
     * @param seed 随机数种子，默认42
     */    
    NormalRandomTime(sc_core::sc_time mean, sc_core::sc_time stddev, int seed = DEFAULT_SEED) 
    : m_is_zero_jitter(stddev == sc_core::SC_ZERO_TIME)
    , m_mean(mean.to_seconds())
    , m_engine(seed)
    , m_dist(mean.to_seconds(), m_is_zero_jitter ? 1.0 /* 占位 */ : stddev.to_seconds()) 
    {}

    double next() { return m_is_zero_jitter ? m_mean : m_dist(m_engine); }
    double operator()() { return next(); }
    
private:
    // 抖动模型参数
    bool m_is_zero_jitter;
    double m_mean;
    std::default_random_engine m_engine;
    std::normal_distribution<double> m_dist;

    NormalRandomTime(const NormalRandomTime&)               =               delete;
    NormalRandomTime(NormalRandomTime&&)                    =               delete;
    NormalRandomTime& operator=(const NormalRandomTime&)    =               delete;
    NormalRandomTime& operator=(NormalRandomTime&&)         =               delete;
};

/**
 * @class UniformRandomTime
 * @brief 均匀随机时间发生器
 * 
 * 生成符合均匀分布的随机时间
 * @note 整数区间：[min, max]；浮点数区间：[min, max)
 */
class UniformRandomTime{
public:
    /**
     * @brief UniformRandomTime构造函数
     * @param min 最小值
     * @param max 最大值
     * @param seed 随机数种子，默认42
     */
    UniformRandomTime(sc_core::sc_time min, sc_core::sc_time max, int seed = DEFAULT_SEED)
    : m_engine(seed)
    , m_dist(min.to_seconds(), max.to_seconds())
    {}

    double next() { return m_dist(m_engine); }
    double operator()() { return next(); }
private:
    std::default_random_engine m_engine;
    std::uniform_real_distribution<double> m_dist;

    UniformRandomTime(const UniformRandomTime&)               =               delete;
    UniformRandomTime(UniformRandomTime&&)                    =               delete;
    UniformRandomTime& operator=(const UniformRandomTime&)    =               delete;
    UniformRandomTime& operator=(UniformRandomTime&&)         =               delete;
};
#endif 