/**
 * @file randomvalue.h
 * @brief 随机数值生成器
 * 
 * 如果不提供种子，则使用默认种子42
 * 
 * @author jcl
 * @date 2026-09-30
 */

#ifndef RANDOMVALUE_H
#define RANDOMVALUE_H

#include <random>
#include <type_traits>

#include "common.h"

/**
 * @class NormalRandomValue
 * @brief 正态随机数值发生器
 * 
 * 生成符合正态分布的随机数
 * @note: 仅支持浮点数
 */
template<typename T, typename = void>
class NormalRandomValue;

template<typename T>
class NormalRandomValue<T, std::enable_if_t<std::is_floating_point<T>::value>>{
public:
    /**
     * @brief NormalRandomValue构造函数
     * @param mean 均值
     * @param stddev 标准差
     * @param seed 随机数种子，默认42
     */    
    NormalRandomValue(T mean, T stddev, int seed = DEFAULT_SEED)
    : m_is_zero_jitter(stddev == 0.0)
    , m_mean(mean)
    , m_engine(seed)
    , m_dist(m_mean, m_is_zero_jitter ? T(1) /* 占位 */ : stddev )
    {}

    T next() { return m_is_zero_jitter ?  m_mean : m_dist(m_engine); }
    T operator()() { return next(); }
    
private:
    // 抖动模型参数
    bool m_is_zero_jitter;
    T m_mean;
    std::default_random_engine m_engine;
    std::normal_distribution<T> m_dist;

    NormalRandomValue(const NormalRandomValue&)               =               delete;
    NormalRandomValue(NormalRandomValue&&)                    =               delete;
    NormalRandomValue& operator=(const NormalRandomValue&)    =               delete;
    NormalRandomValue& operator=(NormalRandomValue&&)         =               delete;
};

/**
 * @class UniformRandomValue
 * @brief 均匀随机数值发生器
 * 
 * 生成符合均匀分布的随机数
 * @note 整数区间：[min, max]；浮点数区间：[min, max)
 */
template<typename T, typename = void>
class UniformRandomValue;

template<typename T>
class UniformRandomValue<T, std::enable_if_t<std::is_integral<T>::value>>{
public:
    /**
     * @brief UniformRandomValue构造函数
     * @param min 最小值
     * @param max 最大值
     * @param seed 随机数种子，默认42
     */
    UniformRandomValue(T min, T max, int seed = DEFAULT_SEED) : m_engine(seed), m_dist(min, max) {}

    T next() { return m_dist(m_engine); }
    T operator()() { return next(); }
private:
    std::default_random_engine m_engine;
    std::uniform_int_distribution<T> m_dist;

    UniformRandomValue(const UniformRandomValue&)               =               delete;
    UniformRandomValue(UniformRandomValue&&)                    =               delete;
    UniformRandomValue& operator=(const UniformRandomValue&)    =               delete;
    UniformRandomValue& operator=(UniformRandomValue&&)         =               delete;
};

template<typename T>
class UniformRandomValue<T, std::enable_if_t<std::is_floating_point<T>::value>>{
public:
    /**
     * @brief UniformRandomValue构造函数
     * @param min 最小值
     * @param max 最大值
     * @param seed 随机数种子，默认42
     */
    UniformRandomValue(T min, T max, int seed = DEFAULT_SEED) : m_engine(seed), m_dist(min, max) {}

    T next() { return m_dist(m_engine); }
    T operator()() { return next(); }
private:
    std::default_random_engine m_engine;
    std::uniform_real_distribution<T> m_dist;

    UniformRandomValue(const UniformRandomValue&)               =               delete;
    UniformRandomValue(UniformRandomValue&&)                    =               delete;
    UniformRandomValue& operator=(const UniformRandomValue&)    =               delete;
    UniformRandomValue& operator=(UniformRandomValue&&)         =               delete;
};
#endif