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

#endif