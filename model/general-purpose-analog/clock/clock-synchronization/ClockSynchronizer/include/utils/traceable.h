/**
 * @file traceable.h
 * @brief 提供基类TraceableSC和TraceableSCA和关联宏，提供统一的信号跟踪和性能打印功能
 * 
 * @author jcl
 * @date 2026-09-18
 */

#ifndef TRACEABLE_H
#define TRACEABLE_H

#define TRACE_SC_SIGNAL_IMPL_DEFERRED(x, x_str, y) \
    sc_core::sc_trace( tf, x, std::string(name()) + y x_str )

#define TRACE_SC_SIGNAL_IMPL(x, str) \
    TRACE_SC_SIGNAL_IMPL_DEFERRED(x, str, ".")

#define TRACE_SC_SIGNAL(signal) \
    TRACE_SC_SIGNAL_IMPL(signal, #signal)

#define TRACE_SCA_SIGNAL_IMPL_DEFERRED(x, x_str, y) \
    sca_util::sca_trace( tf, x, std::string(name()) + y x_str )

#define TRACE_SCA_SIGNAL_IMPL(x, str) \
    TRACE_SCA_SIGNAL_IMPL_DEFERRED(x, str, ".")

#define TRACE_SCA_SIGNAL(signal) \
    TRACE_SCA_SIGNAL_IMPL(signal, #signal)

#define DUMP_BEGIN \
    std::cout << "[" << this->name() << "]" << " Dump begin" << '\n';
#define DUMP_VAR(var) \
    std::cout << #var << ": " << var << '\n'
#define DUMP_END \
    std::cout << "[" << this->name() << "]" << " Dump end" << '\n';

namespace sc_core{
    class sc_trace_file;
}

namespace sca_util{
    class sca_trace_file;
}

/**
 * @struct TraceableSC
 * @brief SyStemC 模块的可跟踪基类
 */

struct TraceableSC{
    virtual void dump_performance() {};
    virtual void trace_me(sc_core::sc_trace_file* tf) {};
};

/**
 * @struct TraceableSCA
 * @brief SystemC-AMS 模块的可跟踪基类
 */

struct TraceableSCA{
    virtual void dump_performance() {};
    virtual void trace_me(sca_util::sca_trace_file* tf) {};
};

#endif