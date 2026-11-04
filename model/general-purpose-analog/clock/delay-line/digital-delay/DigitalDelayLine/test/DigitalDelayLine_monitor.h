#ifndef DIGITALDELAYLINE_MONITOR_H
#define DIGITALDELAYLINE_MONITOR_H

#include <systemc>

#include <string>
#include <utility>
#include <vector>

// ============================================================
// DigitalDelayLine 测试用事件记录器：记录输入与各抽头的每一次值变化（时刻, 新值）
// ============================================================
SC_MODULE(EdgeRecorder)
{
    using Events = std::vector<std::pair<sc_core::sc_time, bool>>;

    sc_core::sc_in<bool> in;
    sc_core::sc_vector<sc_core::sc_in<bool>> taps;

    Events in_events;
    std::vector<Events> tap_events;
    std::vector<bool> tap_initial;      // 各抽头 0 时刻的值

    SC_HAS_PROCESS(EdgeRecorder);
    EdgeRecorder(sc_core::sc_module_name nm, size_t n)
        : in("in"), taps("tap", n), tap_events(n), tap_initial(n, true)
    {
        SC_METHOD(on_in);
        sensitive << in;
        dont_initialize();
        SC_METHOD(on_taps);
        for(size_t k = 0; k < n; k++) sensitive << taps[k];
        dont_initialize();
        SC_METHOD(sample_initial);
    }

    // 期望：抽头事件 = 输入事件整体平移 delay（只取 t + delay < t_end 的事件）
    static bool shifted_equal(const Events& in, const Events& tap, sc_core::sc_time delay, sc_core::sc_time t_end){
        Events exp;
        for(const auto& e : in)
            if(e.first + delay < t_end) exp.emplace_back(e.first + delay, e.second);
        return exp == tap;
    }

private:
    void on_in(){ in_events.emplace_back(sc_core::sc_time_stamp(), in.read()); }
    // 任一抽头变化时被唤醒：逐个检查本 delta 内发生变化的抽头
    void on_taps(){
        for(size_t k = 0; k < taps.size(); k++)
            if(taps[k]->event()) tap_events[k].emplace_back(sc_core::sc_time_stamp(), taps[k].read());
    }
    void sample_initial(){
        for(size_t k = 0; k < taps.size(); k++) tap_initial[k] = taps[k].read();
    }
};

#endif // DIGITALDELAYLINE_MONITOR_H
