// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#ifndef MCP1623_H
#define MCP1623_H
// 单头文件接口：配置、跟踪和仿真辅助定义已内置；模型行为位于 source/。
#include <systemc-ams>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
// ---- Integrated utils/configurable.h ----
/**
 * @file configurable.h
 * @brief 定义Configurable类和相关宏，提供统一从 perf.json 中的配置方式
 * 
 * 用法示例：
 * SC_MODULE(Example)
 *      , Configurable<Example> {
 * public:
 *      CFGS_DECLARE(
 *          std::string meta_value;
 *          int parameter_value;
 *          double configuartion_value;
 *      );
 *      CFGS_FUNC_DECLARE{
 *          CFGS_BEGIN(cfgs)
 *              cfgs.meta_value = meta["value"];
 *              cfgs.parameter_value = parameter["value"];
 *              cfgs.configuartion_value = configuration["value"];  
 *          CFGS_END(cfgs) 
 *      }
 *      ... 
 * };
 * 
 * @author jcl
 * @date 2026-09-18
 */

#ifndef CONFIGABLE_H
#define CONFIGABLE_H

#include "nlohmann/json.hpp"

#include <fstream>
#include <type_traits>

/**
 * @brief CFGS_DECLARE宏：声明配置结构体
 * 
 * 在类中声明一个struct configs结构体
 */
#define CFGS_DECLARE(...)   \
    struct configs{         \
        __VA_ARGS__         \
    }           

/**
 * @brief CFGS_FUNC_DECLARE宏：声明静态成员配置函数
 * 
 * 在类中声明一个返回值为configs，静态成员函数get_configs_from_json
 */

#define CFGS_FUNC_DECLARE       \
    static configs get_configs_from_json(const char* json_path) 

/**
 * @brief CFGS_BEGIN 宏：配置函数
 * 
 * 宏简化了读取json文件的过程，并预先提取了 perf.json 中的三个字段：
 *   - meta: 包含元数据的字段
 *   - parameter: 包含参数的字段
 *   - configuration: 包含配置信息的字段
 * 
 * @note 需要在 CFGS_FUC_DECLARE 中使用，与 CFGS_END 配对
 */

#define CFGS_BEGIN(ret)                 \
    do{                                 \
        using json = nlohmann::json;    \
        configs ret;                    \
        std::ifstream f;                \
        json j;                         \
                                        \
        f.open(json_path, std::ios::in);        \
        if (!f.is_open()){                      \
            std::string err = std::string("Cannot open file: ") + json_path;  \
            throw std::runtime_error(err);      \
        }                                       \
                                                \
        j = json::parse(f);                     \
        const json& meta = j.at("meta");                        \
        const json& parameter = j.at("parameter");              \
        const json& configuration = j.at("configuration");      \
        (void) meta;                                            \
        (void) parameter;                                       \
        (void) configuration;                                   \

/**
 * @brief CFGS_END 宏：声明配置结束
 * @note 需要在 CFGS_FUC_DECLARE 中使用，与 CFGS_BEGIN 配对
 */

#define CFGS_END(ret)           \
        return ret;             \
    }while(0);


template <typename ...>
using void_t = void;

template <typename T, typename = void>
struct has_impl : std::false_type {};

template <typename T>
struct has_impl<T, void_t<decltype(T::get_configs_from_json(nullptr))>> : std::true_type {};


/**
 * @class Configurable
 * @brief 统一模型配置接口
 *
 * 继承Configurable后，将强制子类实现静态成员函数 `get_configs_from_json(const char*)`，否则编译不通过
 */

template<typename Derived>
class Configurable {
public:
    Configurable(){ static_assert(has_impl<Derived>::value == true, "Derived class must implement get_configs_from_json"); }
private:
    auto get_configs_impl(const char* s) const {
        return Derived::get_configs_from_json(s);
    }
};

#endif
// ---- Integrated utils/traceable.h ----
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
 * @class TraceableSC
 * @brief SyStemC 模块的可跟踪基类
 */

class TraceableSC{
    virtual void dump_performance() {};
    virtual void trace_me(sc_core::sc_trace_file* tf) {};
};

/**
 * @class TraceableSCA
 * @brief SystemC-AMS 模块的可跟踪基类
 */

class TraceableSCA{
    virtual void dump_performance() {};
    virtual void trace_me(sca_util::sca_trace_file* tf) {};
};

#endif

// ---- Integrated utils/simulator.h ----
/**
 * @file simulator.h
 * @brief 定义基类AbstractSimualtor和相关宏，提供基础 SYSTEMC/SYSTEMC-AMS 仿真框架 
 *
 * 用法示例：
 * SC_MODULE(Simulator)
 *     , public AbstractSimulator
 * {
 *      SC_HAS_PROCESS(Simulator);
 *      Simulator(sc_module_name name) {
 *          // 初始化过程
 *          ...
 *      }
 *      ~Simulator(){
 *          // 清理过程
 *          ...
 *      }
 * };
 * 
 * int sc_main(int argc, char* argv[]){
 *      Simulator sim("sim");
 *      SIMULATOR_START(sim, 10, SC_SEC);
 * }
 *
 * @author jcl
 * @date 2026-09-18
 */

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <systemc>

#include <exception>

#define SIM_OK      0
#define SIM_FAILURE -1

struct AbstractSimulator{
    virtual void operator()(sc_core::sc_time sim_time){
        sc_start(sim_time);
    }
};

#define SIMULATOR_START(simulator, time, unit)                  \
do{                                                             \
    int sim_flag = SIM_OK;                                      \
    AbstractSimulator& absim = simulator;                       \
                                                                \
    try{                                                        \
        std::cout << "--- Simulation begin ---" << std::endl;   \
        absim(sc_core::sc_time(time, unit));                    \
    }                                                           \
    catch (const sc_core::sc_report& report) {                  \
        std::cerr << "SystemC simulation failed: "              \
                  << report.get_msg() << std::endl;             \
        sim_flag = SIM_FAILURE;                                 \
    }                                                           \
    catch (const std::exception& e) {                                   \
        std::cerr << "Standard exception: " << e.what() << std::endl;   \
        sim_flag = SIM_FAILURE;                                         \
    }                                                                   \
    catch (...) {                                                       \
        std::cerr << "Unknown exception caught" << std::endl;           \
        sim_flag = SIM_FAILURE;                                         \
    }                                                                   \
                                                                        \
    std::cout << "--- Simulation finished ---" << std::endl;            \
    return sim_flag;                                                    \
}while(0)

#define ANSI_RESET "\033[0m"
#define ANSI_RED "\033[31m"
#define ANSI_GREEN "\033[32m"

// ---------------- 检查 ----------------
inline void check(const char* what, double got, double exp, double tol, const char* unit, size_t fail_counter){
    bool ok = std::fabs(got - exp) <= tol;
    std::printf("%s %s  %-40s %10.4f %s (report %.4f ± %.4f)\n", ok ? ANSI_GREEN "[PASS]" ANSI_RESET: ANSI_RED "[FAIL]" ANSI_RESET,
                sc_core::sc_time_stamp().to_string().c_str(), what, got, unit, exp, tol);
    if (!ok) ++fail_counter;
}

inline void info(const char* what){
    std::printf("[INFO] %s  %-40s\n", sc_core::sc_time_stamp().to_string().c_str(), what);
}
#endif
// 六引脚封装：PGND=SGND=EP，VOUTP=VOUTS；DFN按同名端口连接。
// SW是外部网络求得的采样电压，电流正向VIN经L流入SW。
class MCP1623:public sca_tdf::sca_module,public Configurable<MCP1623>,public TraceableSCA {
public:
 sca_tdf::sca_in<double> VIN,PGND,SGND,EP,VOUTP,VOUTS,SW,EN,VFB,inductor_current,rectifier_current;
 sca_tdf::sca_out<double> low_conductance,damping_conductance,rectifier_enabled,precharge_enabled,input_bias_current,output_sink_current,feedback_current,enable_current;
 CFGS_DECLARE(nlohmann::json parameter;double timestep_s;std::string module_name,category;double version;);
 CFGS_FUNC_DECLARE;
 MCP1623(sc_core::sc_module_name,const configs&);
 void set_attributes() override;
 void processing() override;
 std::string get_module_name() const { return c.module_name; }
 double get_version() const { return c.version; }
 std::string get_category() const { return c.category; }
 double get_timestep_s() const { return c.timestep_s; }
 double get_latency() const;
 double get_area() const;
 double get_power_dissipation() const;
 void trace_me(sca_util::sca_trace_file*) override;
 void dump_performance() override {}
 bool active=false,gate=false,damp=false,cycle_on=false;
 int mode=0;
 double command=0,limit=0,slope=0,reference=0,integrator=0;
private:
 double last_power_w_=0;
 configs c;double p(const char*)const;
 double delivered=0,start=0,cycle=0,next=0;
};
#endif
