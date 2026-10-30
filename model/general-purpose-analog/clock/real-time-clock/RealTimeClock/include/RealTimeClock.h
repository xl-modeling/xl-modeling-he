#ifndef REALTIMECLOCK_H
#define REALTIMECLOCK_H

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>

#include "utils/configurable.h"
#include "utils/traceable.h"

#include "RtcRegisters.h"
#include "RtcDivider.h"
#include "RtcControl.h"

// ============================================================
// 实时时钟（RTC）顶层，功能参照 DS1307
//
//   x1 (32.768kHz) ──▶ RtcDivider ──1Hz──▶ RtcControl ◀── i2c_slave (TLM)
//                         │   
//                         └▶ SQW   
//   - DS1307为I2C接口，用TLM-2.0模拟，按字节访问，支持多字节突发（地址递增）
//       - 延迟按 I2C 帧折算：写 (2 + n) 字节、读 (3 + n) 字节，每字节 9 个 SCL 周期（8 位数据 + ACK）
//   - 写秒寄存器时，复位分频链
//   - 仅建模时间/控制寄存器 0x00~0x07，访问越界返回 TLM_ADDRESS_ERROR_RESPONSE（片内 RAM 0x08~0x3F 未建模）
//   - 对外提供 name / version / latency / power / area 读取接口（数值来自 json）
//
// 数据来源：Maxim Integrated, DS1307 64 x 8, Serial, I2C Real-Time Clock, REV 3/15（doc/DS1307.pdf）
//   - 引脚与结构：Pin Description、Figure 1 Block Diagram
//   - I2C：fSCL 0~100kHz（AC Electrical Characteristics）；读写帧格式见 Figure 4/5/6
//   - 上电默认值：Clock and Calendar（01/01/00 01 00:00:00，CH=1）、Control Register（OUT=0, SQWE=0, RS=11）
//   - 功耗：ICCA = 1.5mA @ fSCL=100kHz（DC Electrical Characteristics）
// ============================================================

SC_MODULE(RealTimeClock)
    , public Configurable<RealTimeClock>
    , public TraceableSC
{
public:
    sc_core::sc_in<bool> xtal;                // 外接 32.768kHz 晶振
    sc_core::sc_out<bool> sqw;              // 方波输出（开漏，释放时为 true）
    tlm_utils::simple_target_socket<RealTimeClock> i2c_slave;

    CFGS_DECLARE(
        std::string module_name;
        double version;
        double i2c_scl_frequency_hz;  // I2C SCL 时钟频率，用于折算访问延迟
        double latency_us;            // 单字节 I2C 访问延迟 (us)
        double power_mw;              // 功耗 (mW)
        double area_mm2;              // 面积 (mm²)
    );

    // json 为扁平格式（各字段 {type, value, unit, editable}），不含 meta/parameter/configuration，
    // 因此不使用 CFGS_BEGIN/CFGS_END，直接读取顶层字段
    CFGS_FUNC_DECLARE{
        std::ifstream f(json_path, std::ios::in);
        if (!f.is_open())
            throw std::runtime_error(std::string("Cannot open file: ") + json_path);

        nlohmann::json j = nlohmann::json::parse(f);
        configs cfgs;
        cfgs.module_name = j.at("name").at("value");
        cfgs.version = j.at("version").at("value");
        cfgs.i2c_scl_frequency_hz = j.at("i2c_scl_frequency_hz").at("value");
        cfgs.latency_us = j.at("latency").at("value");
        cfgs.power_mw = j.at("power").at("value");
        cfgs.area_mm2 = j.at("area").at("value");
        return cfgs;
    }

    RealTimeClock(sc_core::sc_module_name nm, const configs& cfgs)
        : xtal("xtal"), sqw("sqw"), i2c_slave("i2c_slave"), m_cfgs(cfgs), i2c_latency(9 / cfgs.i2c_scl_frequency_hz, sc_core::SC_SEC)
        , m_reginf("RtcRegInf"), m_clk_1hz("clk_1hz")
        , m_control("RtcControl")
        , m_divider("RtCDivider")
        , m_regfile("RtCRegfile", {0x80, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x03} /* 上电默认 2000-01-01 星期 1 00:00:00 24进制 */)
    {
        i2c_slave.register_b_transport(this, &RealTimeClock::b_transport);
        m_reginf.bind(m_regfile.inf);
        
        m_control.clk_1hz(m_clk_1hz);
        m_control.inf.bind(m_regfile.inf);

        m_divider.xtal(xtal);
        m_divider.clk_1hz(m_clk_1hz);
        m_divider.sqw(sqw);
        m_divider.inf.bind(m_regfile.inf);
    }

    const std::string& get_name() const { return m_cfgs.module_name; }
    double get_version() const { return m_cfgs.version; }
    double get_latency() const { return m_cfgs.latency_us; }   // us
    double get_power() const { return m_cfgs.power_mw; }       // mW
    double get_area() const { return m_cfgs.area_mm2; }        // mm²

    void trace_me(sc_core::sc_trace_file* tf) override{
        TRACE_SC_SIGNAL(xtal);
        TRACE_SC_SIGNAL(sqw);
        m_control.trace_me(tf);
        m_divider.trace_me(tf);
        m_regfile.trace_me(tf);
    }

    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay){
        uint64_t addr = trans.get_address();
        uint8_t* data = trans.get_data_ptr();
        uint32_t len = trans.get_data_length();
        uint32_t width = trans.get_streaming_width();

        assert(width == 1);
        /* 越界访问 */
        if(addr + len > RtcRegFile::Table::REG_CONTROL + 1){
            trans.set_response_status(tlm::tlm_response_status::TLM_ADDRESS_ERROR_RESPONSE);
            return;
        }

        if(trans.get_command() == tlm::tlm_command::TLM_WRITE_COMMAND){
            delay += 2 * i2c_latency;
            for(size_t i = addr; i < addr + len; i++){
                m_reginf->write(i, data[i - addr]);
                delay += i2c_latency;
            }
            /* 写秒寄存器复位分频链 */
            if(addr == RtcRegFile::REG_SEC){
                if(m_clk_1hz.read())            // 复位产生的下降沿不推进日历
                    m_control.skip_next();
                m_divider.reset();   
            }
                
            trans.set_response_status(tlm::tlm_response_status::TLM_OK_RESPONSE);
        }

        if(trans.get_command() == tlm::tlm_command::TLM_READ_COMMAND){
            delay += 3 * i2c_latency;
            for(size_t i = addr; i < addr + len; i++){
                data[i - addr] = m_reginf->read(i);
                delay += i2c_latency;
            }
            trans.set_response_status(tlm::tlm_response_status::TLM_OK_RESPONSE);
        }
    }
private:
    configs m_cfgs;
    sc_core::sc_time i2c_latency;

    sc_core::sc_port<RtcRegFile::IF> m_reginf;  
    sc_core::sc_signal<bool> m_clk_1hz;
    // 子模块
    RtcControl m_control;
    RtcDivider m_divider;
    RtcRegFile m_regfile;
};

#endif // REALTIMECLOCK_H
