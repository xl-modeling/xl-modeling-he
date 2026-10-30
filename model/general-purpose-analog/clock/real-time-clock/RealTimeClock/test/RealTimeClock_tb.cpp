#include "RealTimeClock.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include "tlm_utils/simple_initiator_socket.h"

using namespace sc_core;

// ============================================================
// Simulator: 32.768kHz 晶振激励（+20ppm）+ TLM 主机 + 场景激励 + 自检
//   1. 上电默认值，CH=1 时间不走
//   2. 闰年跨日、跨年、平年 2 月、30 天月份
//   3. 12 小时模式：11AM→12PM、11PM→12AM（跨日）、12→1
//   4. CH 停振 / 恢复
//   5. 方波 1Hz / 4.096kHz / 8.192kHz / 32.768kHz、OUT 电平、停振时方波停止
// ============================================================
SC_MODULE(Simulator)
    , public AbstractSimulator
{
public:
    sc_signal<bool> x1;
    sc_signal<bool> sqw;

    tlm_utils::simple_initiator_socket<Simulator> i2c_master;
    sc_core::sc_clock clk;

    SC_HAS_PROCESS(Simulator);
    explicit Simulator(sc_module_name name)
        : sc_module(name)
        , x1("x1"), sqw("sqw"), i2c_master("i2c_master")
        , clk("clk", sc_core::sc_time(30.5175, sc_core::SC_US))     // 32.768 kHz 时钟
        , m_fail(0)
    {
        m_tf = sc_create_vcd_trace_file("wave");

        RealTimeClock::configs cfgs = RealTimeClock::get_configs_from_json("../../json/RealTimeClock.json");
        m_rtc = new RealTimeClock(cfgs.module_name.c_str(), cfgs);

        m_rtc->xtal(clk);
        m_rtc->sqw(sqw);
        i2c_master.bind(m_rtc->i2c_slave);
        
        m_rtc->trace_me(m_tf);

        SC_THREAD(testbench);
    }

    ~Simulator(){
        sc_close_vcd_trace_file(m_tf);
        delete m_rtc;
    }
private:
    // I2C 访问辅助：返回响应状态，delay 由调用者传入以便检查访问延迟
    tlm::tlm_response_status i2c_access(tlm::tlm_command cmd, uint64_t addr, uint8_t* data, uint32_t len, sc_core::sc_time& delay){
        tlm::tlm_generic_payload payload;
        payload.set_command(cmd);
        payload.set_address(addr);
        payload.set_streaming_width(sizeof(uint8_t));
        payload.set_data_ptr(data);
        payload.set_data_length(len);
        payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);
        i2c_master->b_transport(payload, delay);
        return payload.get_response_status();
    }
    tlm::tlm_response_status i2c_write(uint64_t addr, uint8_t* data, uint32_t len){
        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
        return i2c_access(tlm::tlm_command::TLM_WRITE_COMMAND, addr, data, len, delay);
    }
    tlm::tlm_response_status i2c_read(uint64_t addr, uint8_t* data, uint32_t len){
        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
        return i2c_access(tlm::tlm_command::TLM_READ_COMMAND, addr, data, len, delay);
    }

    // 测量 sqw 两个相邻上升沿的间隔，返回频率 (Hz)
    double measure_sqw_freq(){
        wait(sqw.posedge_event());
        sc_core::sc_time t0 = sc_core::sc_time_stamp();
        wait(sqw.posedge_event());
        return 1.0 / (sc_core::sc_time_stamp() - t0).to_seconds();
    }

    size_t m_fail;
    sc_trace_file* m_tf;
    RealTimeClock* m_rtc;

    void testbench(){
        [[maybe_unused]] bool ok = false;
        tlm::tlm_generic_payload payload;
        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
        
        {
            info("模型信息接口");
            info((std::string("name: ") + m_rtc->get_name()).c_str());
            check("version", m_rtc->get_version(), 0.1, 0, "", m_fail);
            check("latency", m_rtc->get_latency(), 90.0, 0, "us", m_fail);
            check("power", m_rtc->get_power(), 7.5, 0, "mW", m_fail);
            check("area", m_rtc->get_area(), 19.11, 0, "mm2", m_fail);
        }

        {
            info("上电默认值：CH=1，01/01/00 星期1 00:00:00，控制寄存器 0x03");
            uint8_t read[8] = {0};
            assert(i2c_read(0x00, read, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            uint8_t gold[8] = {0x80, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x03};
            check_vec("检查上电默认寄存器", read, gold, 8, 0, "", m_fail);

            info("CH=1 等待 2s，时间不走");
            wait(2.5, sc_core::SC_SEC);
            assert(i2c_read(0x00, read, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            check_vec("检查停振时时间不变", read, gold, 8, 0, "", m_fail);
            check("检查停振时sqw=OUT=0", sqw.read(), 0, 0, "", m_fail);
        }

        {
            info("设定时间： 2026-10-20-02-23:59:58");
            uint8_t write [] = {0x58 | 0x80, 0x59, 0x23, 0x02, 0x20, 0x10, 0x26, 0x00};
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00); 
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(write);
            payload.set_data_length(sizeof(write));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE); 

            uint8_t read [8] = {0};
            payload.set_command(tlm::tlm_command::TLM_READ_COMMAND);
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE); 

            check_vec("设置时钟：2026-10-20-02-23:59:58", read, write, 8, 0, "", m_fail);
            wait(sc_core::sc_time(10, sc_core::SC_SEC));

            info("开时钟");
            uint8_t sec = 0x58 & 0x7f;
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(&sec);
            payload.set_data_length(1);
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            uint8_t ctrl = 0x10;
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x07);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(&ctrl);
            payload.set_data_length(1);
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);  
            
            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            wait(2.5, sc_core::SC_SEC);

            payload.set_command(tlm::tlm_command::TLM_READ_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            uint8_t gold [] = {0x00, 0x00, 0x00, 0x03, 0x21, 0x10, 0x26, 0x10};
            check_vec("检查时间是否是：2026-10-21-03-00:00:00", read, gold, 8, 0, "", m_fail);
        }
        
        {
            info("设置闰年：2024-2-28-03-23:59:58");
            uint8_t write [] = {0x58, 0x59, 0x23, 0x03, 0x28, 0x02, 0x24, 0x10};
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(write);
            payload.set_data_length(sizeof(write));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            wait(2.5, sc_core::SC_SEC);
            
            uint8_t read[8] = {0};
            payload.set_command(tlm::tlm_command::TLM_READ_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            uint8_t gold [] = {0x00, 0x00, 0x00, 0x04, 0x29, 0x02, 0x24, 0x10};
            check_vec("检查时间是否是：2024-2-29-04-00:00:00", read, gold, 8, 0, "", m_fail);

            info("设置RS: 0b11");
            read[7] = {0b11};
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            wait(2.5, sc_core::SC_SEC);

            info("设置OUT: 0");
            read[7] = {0x00};
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            wait(2.5, sc_core::SC_SEC);
            check("检查sqw是否为0", sqw.read(), 0, 0, "", m_fail);

            info("设置OUT: 1");
            read[7] = {0x80};
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            wait(2.5, sc_core::SC_SEC);
            check("检查sqw是否为1", sqw.read(), 1, 0, "", m_fail);
        }
            info("测试AM/PM功能");
        {
            info("设置时间为12小时制,时间:2024-2-29-04-11:59:58 am");
            uint8_t write[8] = {0x58, 0x59, 0x11 | 0x40 /* 设置12小时制 */, 0x04, 0x29, 0x02, 0x24, 0x10};
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(write);
            payload.set_data_length(sizeof(write));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            wait(2.5, sc_core::SC_SEC);

            uint8_t read[8] = {0};
            payload.set_command(tlm::tlm_command::TLM_READ_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            uint8_t gold[8] = {0x00, 0x00, 0x32 | 0x40, 0x04, 0x29, 0x02, 0x24, 0x10};
            check_vec("检查时间是否是：2024-2-29-04-12:00:00 pm", read, gold, 8, 0, "", m_fail);
        }

        {
            info("设置时间为12小时制,时间:2024-2-29-04-12:59:58 pm, 检查在am/pm下, 小时是否会从12跳到1");
            uint8_t write[8] = {0x58, 0x59, 0x32 | 0x40, 0x04, 0x29, 0x02, 0x24, 0x10};
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(write);
            payload.set_data_length(sizeof(write));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            wait(2.5, sc_core::SC_SEC);

            uint8_t read[8] = {0};
            payload.set_command(tlm::tlm_command::TLM_READ_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            uint8_t gold[8] = {0x00, 0x00, 0x21 | 0x40, 0x04, 0x29, 0x02, 0x24, 0x10};
            check_vec("检查时间是否是：2024-2-29-04-01:00:00 pm", read, gold, 8, 0, "", m_fail);
        }

        {
            info("设置时间:2024-2-29-04-11:59:58 pm。");
            uint8_t write[8] = {0x58, 0x59, 0x31 | 0x40, 0x04, 0x29, 0x02, 0x24, 0x10};
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(write);
            payload.set_data_length(sizeof(write));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            wait(2.5, sc_core::SC_SEC);

            uint8_t read[8] = {0};
            payload.set_command(tlm::tlm_command::TLM_READ_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            uint8_t gold[8] = {0x00, 0x00, 0x12 | 0x40, 0x05, 0x01, 0x03, 0x24, 0x10};
            check_vec("检查时间是否是：2024-3-01-05-12:00:00 am", read, gold, 8, 0, "", m_fail);
        }

        {
            info("设置时间:2024-2-29-04-12:59:58 am, 检查在am/pm下, 小时是否会从12调到1");
            uint8_t write[8] = {0x58, 0x59, 0x12 | 0x40, 0x04, 0x29, 0x02, 0x24, 0x10};
            payload.set_command(tlm::tlm_command::TLM_WRITE_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(write);
            payload.set_data_length(sizeof(write));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            wait(2.5, sc_core::SC_SEC);

            uint8_t read[8] = {0};
            payload.set_command(tlm::tlm_command::TLM_READ_COMMAND);
            payload.set_address(0x00);
            payload.set_streaming_width(sizeof(uint8_t));
            payload.set_data_ptr(read);
            payload.set_data_length(sizeof(read));
            payload.set_response_status(tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

            i2c_master->b_transport(payload, delay);
            assert(payload.get_response_status() == tlm::tlm_response_status::TLM_OK_RESPONSE);

            uint8_t gold[8] = {0x00, 0x00, 0x01 | 0x40, 0x04, 0x29, 0x02, 0x24, 0x10};
            check_vec("检查时间是否是：2024-2-29-04-01:00:00 am", read, gold, 8, 0, "", m_fail);
        }

        {
            info("平年 2 月：2025-2-28-05-23:59:58");
            uint8_t write[8] = {0x58, 0x59, 0x23, 0x05, 0x28, 0x02, 0x25, 0x10};
            assert(i2c_write(0x00, write, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            wait(2.5, sc_core::SC_SEC);
            uint8_t read[8] = {0};
            assert(i2c_read(0x00, read, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            uint8_t gold[8] = {0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x25, 0x10};
            check_vec("检查时间是否是：2025-3-01-06-00:00:00", read, gold, 8, 0, "", m_fail);
        }

        {
            info("30 天月份：2026-4-30-04-23:59:58");
            uint8_t write[8] = {0x58, 0x59, 0x23, 0x04, 0x30, 0x04, 0x26, 0x10};
            assert(i2c_write(0x00, write, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            wait(2.5, sc_core::SC_SEC);
            uint8_t read[8] = {0};
            assert(i2c_read(0x00, read, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            uint8_t gold[8] = {0x00, 0x00, 0x00, 0x05, 0x01, 0x05, 0x26, 0x10};
            check_vec("检查时间是否是：2026-5-01-05-00:00:00", read, gold, 8, 0, "", m_fail);
        }

        {
            info("跨年与世纪回绕：2099-12-31-07-23:59:58");
            uint8_t write[8] = {0x58, 0x59, 0x23, 0x07, 0x31, 0x12, 0x99, 0x10};
            assert(i2c_write(0x00, write, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            wait(2.5, sc_core::SC_SEC);
            uint8_t read[8] = {0};
            assert(i2c_read(0x00, read, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            uint8_t gold[8] = {0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x10};
            check_vec("检查时间是否是：2000-1-01-01-00:00:00", read, gold, 8, 0, "", m_fail);
        }

        {
            info("CH 停振 / 恢复：设置 2026-10-28-03-12:00:00 且 CH=1");
            uint8_t write[8] = {0x00 | 0x80, 0x00, 0x12, 0x03, 0x28, 0x10, 0x26, 0x10};
            assert(i2c_write(0x00, write, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            wait(3, sc_core::SC_SEC);
            uint8_t read[8] = {0};
            assert(i2c_read(0x00, read, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            check_vec("检查停振 3s 后时间不变", read, write, 8, 0, "", m_fail);

            info("清 CH 恢复走时，等待 2s");
            uint8_t sec = 0x00;
            assert(i2c_write(0x00, &sec, 1) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            wait(2.5, sc_core::SC_SEC);
            assert(i2c_read(0x00, read, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            uint8_t gold[8] = {0x02, 0x00, 0x12, 0x03, 0x28, 0x10, 0x26, 0x10};
            check_vec("检查恢复后时间是否是：12:00:02", read, gold, 8, 0, "", m_fail);
        }

        {
            info("写秒寄存器复位分频链：SQWE=1、RS=00（1Hz），在秒更新后 0.5s 写秒 = 10");
            auto bcd2bin = [](uint8_t v){ return 10 * ((v >> 4) & 0x7) + (v & 0x0f); };
            wait(sqw.negedge_event());
            wait(0.5, sc_core::SC_SEC);
            uint8_t sec = 0x10;
            assert(i2c_write(0x00, &sec, 1) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            sc_core::sc_time t_write = sc_core::sc_time_stamp();
            wait(0.9, sc_core::SC_SEC);
            uint8_t read = 0;
            assert(i2c_read(0x00, &read, 1) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            check("检查写入后 0.9s 秒寄存器仍为 10（未复位则 0.5s 时已更新）", bcd2bin(read), 10, 0, "", m_fail);
            wait(sqw.negedge_event());
            check("检查写入后首次秒更新间隔为 1s", (sc_core::sc_time_stamp() - t_write).to_seconds(), 1.0, 1e-4, "s", m_fail);
            wait(1, sc_core::SC_MS);
            assert(i2c_read(0x00, &read, 1) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            check("检查首次秒更新后秒寄存器为 11", bcd2bin(read), 11, 0, "", m_fail);

            info("1Hz 下降沿推进日历：上升沿不更新，下降沿更新");
            wait(sqw.posedge_event());
            wait(1, sc_core::SC_MS);
            uint8_t s_pos = 0;
            assert(i2c_read(0x00, &s_pos, 1) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            check("检查 1Hz 上升沿后秒寄存器不变", bcd2bin(s_pos), 11, 0, "", m_fail);
            wait(sqw.negedge_event());
            wait(1, sc_core::SC_MS);
            uint8_t s_neg = 0;
            assert(i2c_read(0x00, &s_neg, 1) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            check("检查 1Hz 下降沿后秒寄存器加 1", bcd2bin(s_neg), 12, 0, "", m_fail);
        }

        {
            info("方波输出频率：SQWE=1，RS=00/01/10/11");
            const uint8_t ctrl[4] = {0x10, 0x11, 0x12, 0x13};
            const double  freq[4] = {1.0, 4096.0, 8192.0, 32768.0};
            const char*   what[4] = {"检查sqw频率 1Hz", "检查sqw频率 4.096kHz", "检查sqw频率 8.192kHz", "检查sqw频率 32.768kHz"};
            for(size_t i = 0; i < 4; i++){
                uint8_t c = ctrl[i];
                assert(i2c_write(0x07, &c, 1) == tlm::tlm_response_status::TLM_OK_RESPONSE);
                double f = measure_sqw_freq();
                check(what[i], f, freq[i], freq[i] * 1e-4, "Hz", m_fail);   // 激励晶振有约 +2.5ppm 偏差，容差 100ppm
            }

            info("SQWE=1 时设置 CH=1，方波停止");
            uint8_t sec = 0x80;
            assert(i2c_write(0x00, &sec, 1) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            // 分频链输出比计数器晚一个晶振边沿（寄存器打拍），停振后最多再跳变一次，等待 1ms 后再检查
            wait(sc_core::sc_time(1, sc_core::SC_MS));
            bool level = sqw.read();
            sc_core::sc_time t0 = sc_core::sc_time_stamp();
            wait(sc_core::sc_time(1, sc_core::SC_SEC), sqw.value_changed_event());   // 超时 1s 或 sqw 跳变
            check("检查停振 1s 内sqw无跳变", (sc_core::sc_time_stamp() - t0).to_seconds(), 1.0, 0, "s", m_fail);
            check("检查停振后sqw电平保持", sqw.read(), level, 0, "", m_fail);
        }

        {
            info("越界访问：读 0x08、写 0x06 起 4 字节");
            uint8_t read[1] = {0};
            check("检查读 0x08 返回地址错误", i2c_read(0x08, read, 1), tlm::tlm_response_status::TLM_ADDRESS_ERROR_RESPONSE, 0, "", m_fail);

            uint8_t before[8] = {0};
            assert(i2c_read(0x00, before, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            uint8_t write[4] = {0x11, 0x22, 0x33, 0x44};
            check("检查写 0x06~0x09 返回地址错误", i2c_write(0x06, write, 4), tlm::tlm_response_status::TLM_ADDRESS_ERROR_RESPONSE, 0, "", m_fail);
            uint8_t after[8] = {0};
            assert(i2c_read(0x00, after, 8) == tlm::tlm_response_status::TLM_OK_RESPONSE);
            check_vec("检查越界写未修改寄存器", after, before, 8, 0, "", m_fail);
        }

        {
            info("I2C 访问延迟：SCL=100kHz，每字节 9 个 SCL 周期 = 90us");
            uint8_t buf[8] = {0};
            sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
            i2c_access(tlm::tlm_command::TLM_READ_COMMAND, 0x00, buf, 8, delay);
            check("检查读 8 字节延迟 (3+8)*90us", delay.to_seconds() * 1e6, 990.0, 1e-6, "us", m_fail);
            delay = sc_core::SC_ZERO_TIME;
            i2c_access(tlm::tlm_command::TLM_WRITE_COMMAND, 0x00, buf, 8, delay);
            check("检查写 8 字节延迟 (2+8)*90us", delay.to_seconds() * 1e6, 900.0, 1e-6, "us", m_fail);
        }

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, 80, sc_core::SC_SEC);
}
