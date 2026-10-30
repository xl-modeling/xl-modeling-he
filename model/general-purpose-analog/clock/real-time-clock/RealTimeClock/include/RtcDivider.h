#ifndef RTC_DIVIDER_H
#define RTC_DIVIDER_H

#include <systemc>

#include "utils/traceable.h"
#include "RtcRegisters.h"

// ============================================================
// RTC 分频链 + 可编程方波
//   - 外接晶振 x1 
//   - 方波（SQW）：SQWE=1 时按 RS 输出 1Hz / 4.096kHz / 8.192kHz / 32.768kHz；SQWE=0 时输出 OUT 电平
//   - 停振（CH=1）：停止计数，方波保持当前电平
//
// 数据来源：Maxim Integrated, DS1307 64 x 8, Serial, I2C Real-Time Clock, REV 3/15（doc/DS1307.pdf）
//   - 方波频率选择与 OUT 电平：Control Register（RS1/RS0、SQWE、OUT）
//   - 写秒寄存器复位分频链：Clock and Calendar
//   - 晶振：32.768kHz，Table 1. Crystal Specifications
// ============================================================
SC_MODULE(RtcDivider)
    , public TraceableSC
{
public:
    sc_core::sc_in<bool>    xtal;       // 外接 32.768kHz 晶振
    sc_core::sc_out<bool>   clk_1hz;    // 1Hz，下降沿触发秒更新
    sc_core::sc_out<bool>   sqw;        // 方波输出（开漏，释放时为 true）
    sc_core::sc_port<RtcRegFile::IF> inf;

    SC_HAS_PROCESS(RtcDivider);
    explicit RtcDivider(sc_core::sc_module_name nm)
        : xtal("xtal"), clk_1hz("clk_1hz"), sqw("sqw"), inf("inf")
    {}

    void end_of_elaboration() override{
        SC_METHOD(posedge_xtal_or_reset);
        sensitive << xtal.posedge_event() << m_reset;

        SC_METHOD(any_cnt)
        sensitive << m_cnt.value_changed_event() << xtal;

        SC_METHOD(any_inf);
        sensitive << inf->default_event() << clk_1hz.value_changed_event() << m_clk_4_096khz.value_changed_event() << m_clk_8_192khz.value_changed_event() << m_clk_32_768khz.value_changed_event();
    }

    void trace_me(sc_core::sc_trace_file* tf) override{
        TRACE_SC_SIGNAL(clk_1hz);
        TRACE_SC_SIGNAL(sqw);
        TRACE_SC_SIGNAL(m_cnt);
        TRACE_SC_SIGNAL(m_clk_4_096khz);
        TRACE_SC_SIGNAL(m_clk_8_192khz);
        TRACE_SC_SIGNAL(m_clk_32_768khz);
    }

    void reset(){ m_reset.notify(); }
private:
    void posedge_xtal_or_reset(){
        /**
         * always@(posedge xtal or rstn) begin
         *      if(rstn)
         *          m_cnt <= 0;
         *      else if(!CH)
         *          m_cnt <= m_cnt + 1;
         *      else
         *          m_cnt <= m_cnt;
         * end
         */
        sc_dt::sc_uint<8> in = inf->read(RtcRegFile::Table::REG_SEC);
        if(m_reset.triggered())
            m_cnt = 0;
        else if(!in[7] /* bit: CH */ )
            m_cnt = m_cnt.read() + 1;
    }

    void any_cnt(){
        /**
         * always@(*) begin
         *      clk_1hz = m_cnt >> 14;
         *      clk_4_096khz = (m_cnt >> 2) & 0x01;
         *      clk_8_192khz = (m_cnt >> 1) & 0x01;
         *      clk_32_768khz = !CH ? xtal : m_clk_32_768khz;
         * end
         */
        sc_dt::sc_uint<8> in = inf->read(RtcRegFile::Table::REG_SEC);
        clk_1hz = (m_cnt.read() >> 14) & 0x01;
        m_clk_4_096khz = (m_cnt.read() >> 2) & 0x01;
        m_clk_8_192khz = (m_cnt.read() >> 1) & 0x01;
        m_clk_32_768khz = !in[7] ? xtal : m_clk_32_768khz; // 锁存
    }

    void any_inf(){
        /**
         * always@(*) begin
         *      casez({RS1, RS0, SQWE, OUT})
         *          4'b000?: sqw = clk_1hz;
         *          4'b011?: sqw = clk_4_096khz;
         *          4'b101?: sqw = clk_8_192khz;
         *          4'b111?: sqw = clk_32_768khz;
         *          4'b??00: sqw = 0;
         *          4'b??01: sqw = 1;
         *          default: sqw = 0;
         *      endcase 
         * end
         */
        sc_dt::sc_uint<8> in = inf->read(RtcRegFile::Table::REG_CONTROL);
        sc_dt::sc_uint<4> control_bits = in[1] << 3 | in[0] << 2 | in[4] << 1 | in[7];
        switch (control_bits)
        {
            case 0b0010:
            case 0b0011:
                sqw = clk_1hz;
                break;
            case 0b0110:
            case 0b0111:
                sqw = m_clk_4_096khz;
                break;
            case 0b1010:
            case 0b1011:
                sqw = m_clk_8_192khz;
                break;
            case 0b1110:
            case 0b1111:
                sqw = m_clk_32_768khz;
                break;
            case 0b0000:
            case 0b0100:
            case 0b1000:
            case 0b1100:
                sqw = 0;
                break;
            case 0b0001:
            case 0b0101:
            case 0b1001:
            case 0b1101:
                sqw = 1;
                break;
            default:  sqw = 0; break;
        }
    }

    sc_core::sc_event m_reset;
    sc_core::sc_signal<sc_dt::sc_uint<16>> m_cnt;
    sc_core::sc_signal<bool> m_clk_4_096khz;
    sc_core::sc_signal<bool> m_clk_8_192khz;
    sc_core::sc_signal<bool> m_clk_32_768khz;
};

#endif // RTC_DIVIDER_H
