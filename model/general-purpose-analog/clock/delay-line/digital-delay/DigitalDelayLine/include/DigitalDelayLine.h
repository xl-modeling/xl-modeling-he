#ifndef DIGITALDELAYLINE_H
#define DIGITALDELAYLINE_H

#include <systemc>

#include <deque>
#include <utility>
#include <vector>

#include "utils/configurable.h"
#include "utils/traceable.h"

// ============================================================
// 多抽头数字延迟线（参照 DS1100 5-Tap Silicon Delay Line）
//   - N 个抽头输出等间隔分布，第 k 个抽头（k = 1..N）延迟 = k / N × 总延迟 delay_ns
//   - 传输延迟：输入的每个变化都按（当前时刻 + 抽头延迟）入队，到时刻后依次写出，
//     输入变化快于延迟时不丢沿（器件允许的最小脉宽、最小周期见数据手册）
//   - 上电时各抽头输出为低电平，输入初值在一个抽头延迟后到达对应抽头
//   - 对外提供 name / version / latency / power / area 读取接口（数值来自 json）
//
// 数据来源：DS1100 5-Tap Economy Timing Element（Maxim Integrated，19-5735 Rev 3/11，doc/2623.pdf）
//   - 5 抽头等间隔、DS1100-100 抽头延迟 20/40/60/80/100ns：General Description、Table 1
//   - 最小输入脉宽 tWI = 20% × TAP5 延迟、最小周期 2·tWI：AC Electrical Characteristics
//   - 功耗 ICC × VCC = 30mA × 5V = 150mW（1MHz，输出开路）：DC Electrical Characteristics
//   - 面积按 SO-8（150mil）本体 4.9 × 3.9mm 估算；延迟容差 ±5%（> 40ns，25℃）未建模，取标称值
// ============================================================
SC_MODULE(DigitalDelayLine)
    , public Configurable<DigitalDelayLine>
    , public TraceableSC
{
    sc_core::sc_in<bool> in;                            // 输入 IN
    sc_core::sc_vector<sc_core::sc_out<bool>> taps;     // 抽头输出 TAP1 ~ TAPN

    CFGS_DECLARE(
        std::string module_name;
        double version;
        double latency_ns;      // 总延迟 (ns)
        double power_mw;        // 功耗 (mW)
        double area_mm2;        // 面积 (mm²)
        double delay_ns;        // 总延迟（最后一个抽头的延迟）
        size_t taps;            // 抽头数
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
        cfgs.latency_ns = j.at("latency").at("value");
        cfgs.power_mw = j.at("power").at("value");
        cfgs.area_mm2 = j.at("area").at("value");
        cfgs.delay_ns = j.at("delay_ns").at("value");
        cfgs.taps = j.contains("taps") ? j.at("taps").at("value").get<size_t>() : 5;
        return cfgs;
    }

    const std::string& get_name() const { return m_cfgs.module_name; }
    double get_version() const { return m_cfgs.version; }
    double get_latency() const { return m_cfgs.latency_ns; }   // ns
    double get_power() const { return m_cfgs.power_mw; }       // mW
    double get_area() const { return m_cfgs.area_mm2; }        // mm²

    SC_HAS_PROCESS(DigitalDelayLine);
    DigitalDelayLine(sc_core::sc_module_name name, const configs& cfgs)
        : in("in")
        , taps("tap", cfgs.taps)
        , m_cfgs(cfgs)
        , m_queues(cfgs.taps)
    {
        for(size_t k = 1; k <= cfgs.taps; k++)
            m_delays.push_back(sc_core::sc_time(cfgs.delay_ns * k / cfgs.taps, sc_core::SC_NS));

        SC_METHOD(on_input);
        sensitive << in;        // 端口敏感（值变化），绑定后生效

        SC_METHOD(on_output);
        sensitive << m_eq;
        dont_initialize();
    }

    ~DigitalDelayLine() {}

    void trace_me(sc_core::sc_trace_file* tf) override{
        TRACE_SC_SIGNAL(in);
        for(const auto& tap : taps)
            sc_core::sc_trace(tf, tap, tap.name());
    }

private:
    // 输入变化（含初始化时的初值）：各抽头按各自延迟入队
    void on_input(){
        const sc_core::sc_time now = sc_core::sc_time_stamp();
        const bool v = in.read();
        for(size_t k = 0; k < m_delays.size(); k++){
            m_queues[k].emplace_back(now + m_delays[k], v);
            m_eq.notify(m_delays[k]);
        }
    }

    // 到期：各抽头写出所有已到时刻的值（同一时刻多个值时最后一个生效）
    void on_output(){
        const sc_core::sc_time now = sc_core::sc_time_stamp();
        for(size_t k = 0; k < m_queues.size(); k++){
            auto& q = m_queues[k];
            while(!q.empty() && q.front().first <= now){
                taps[k].write(q.front().second);
                q.pop_front();
            }
        }
    }

    configs m_cfgs;
    std::vector<sc_core::sc_time> m_delays;                                 // 各抽头延迟
    std::vector<std::deque<std::pair<sc_core::sc_time, bool>>> m_queues;    // 各抽头待输出的（时刻, 值）
    sc_core::sc_event_queue m_eq;                                           // 到期事件
};

#endif // DIGITALDELAYLINE_H
