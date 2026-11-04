#include "DigitalDelayLine.h"
#include "DigitalDelayLine_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include <string>

// ============================================================
// DigitalDelayLine 功能测试（DS1100-100，5 抽头，总延迟 100ns）
//   激励：1MHz 方波（与数据手册 TEST CONDITIONS 一致：脉宽 500ns、周期 1µs），仿真 5µs
//   检查：
//     0. 模型信息接口 name / version / latency / power / area，抽头数 = 5
//     1. 上电时各抽头输出为低电平
//     2. 各抽头的每个上升 / 下降沿 = 输入对应沿平移 k × 20ns（tPLH = tPHL，前后沿精度相同）
//     3. 抽头延迟单调递增、等间隔：TAP(k+1) − TAP(k) = 20ns
// ============================================================
SC_MODULE(Simulator)
    , public AbstractSimulator
{
public:
    sc_core::sc_signal<bool> iclk;
    sc_core::sc_vector<sc_core::sc_signal<bool>> taps;

    SC_HAS_PROCESS(Simulator);
    explicit Simulator(sc_core::sc_module_name name)
        : sc_module(name)
        , iclk("iclk")
        , m_fail(0)
    {
        m_tf = sc_core::sc_create_vcd_trace_file("wave");

        m_cfgs = DigitalDelayLine::get_configs_from_json("../../json/DigitalDelayLine.json");
        m_ddl = new DigitalDelayLine(m_cfgs.module_name.c_str(), m_cfgs);
        taps.init(m_cfgs.taps);
        m_ddl->in(iclk);
        m_ddl->taps(taps);

        m_rec = new EdgeRecorder("rec", m_cfgs.taps);
        m_rec->in(iclk);
        m_rec->taps(taps);

        m_ddl->trace_me(m_tf);

        SC_THREAD(stimulus);
        SC_THREAD(testbench);
    }

    ~Simulator(){
        sc_core::sc_close_vcd_trace_file(m_tf);
        delete m_rec;
        delete m_ddl;
    }

private:
    sc_core::sc_trace_file* m_tf;
    DigitalDelayLine::configs m_cfgs;
    DigitalDelayLine* m_ddl;
    EdgeRecorder* m_rec;
    size_t m_fail;

    const sc_core::sc_time T_END = sc_core::sc_time(5, sc_core::SC_US);

    // 1MHz 方波：0 → 500ns 拉高 → 1000ns 拉低 → ...
    void stimulus(){
        iclk.write(false);
        while(true){
            wait(500, sc_core::SC_NS); iclk.write(true);
            wait(500, sc_core::SC_NS); iclk.write(false);
        }
    }

    sc_core::sc_time tap_delay(size_t k) const {
        return sc_core::sc_time(m_cfgs.delay_ns * (k + 1) / m_cfgs.taps, sc_core::SC_NS);
    }

    void testbench(){
        {
            info("模型信息接口");
            info((std::string("name: ") + m_ddl->get_name()).c_str());
            check("version", m_ddl->get_version(), 0.1, 0, "", m_fail);
            check("latency（总延迟）", m_ddl->get_latency(), 100.0, 0, "ns", m_fail);
            check("power", m_ddl->get_power(), 150.0, 0, "mW", m_fail);
            check("area", m_ddl->get_area(), 19.11, 0, "mm2", m_fail);
            check("抽头数", m_cfgs.taps, 5, 0, "", m_fail);
        }

        wait(T_END);

        {
            info("上电：各抽头输出为低电平");
            bool all_low = true;
            for(size_t k = 0; k < m_cfgs.taps; k++) all_low = all_low && !m_rec->tap_initial[k];
            check("TAP1 ~ TAP5 初始电平为低", all_low, 1, 0, "", m_fail);
        }

        info("1MHz 方波：各抽头 = 输入平移 k × 20ns（上升沿与下降沿）");
        check("输入沿数（5µs）", m_rec->in_events.size(), 9, 0, "", m_fail);
        for(size_t k = 0; k < m_cfgs.taps; k++){
            const auto& ev = m_rec->tap_events[k];
            std::string s = "TAP" + std::to_string(k + 1) + "（" + tap_delay(k).to_string() + "）";
            check((s + " 各沿与输入平移一致").c_str(),
                  EdgeRecorder::shifted_equal(m_rec->in_events, ev, tap_delay(k), T_END), 1, 0, "", m_fail);
            if(!ev.empty()){
                // sc_time 换算为 ns 有浮点舍入，容差取 1fs（逐沿的精确比较见上一项）
                double plh = (ev[0].first - m_rec->in_events[0].first).to_seconds() * 1e9;
                double phl = (ev[1].first - m_rec->in_events[1].first).to_seconds() * 1e9;
                check((s + " tPLH").c_str(), plh, 20.0 * (k + 1), 1e-6, "ns", m_fail);
                check((s + " tPHL").c_str(), phl, 20.0 * (k + 1), 1e-6, "ns", m_fail);
            }
        }

        {
            info("抽头间隔：TAP(k+1) − TAP(k) = 总延迟 / 5");
            bool equal_step = true;
            for(size_t k = 1; k < m_cfgs.taps; k++)
                equal_step = equal_step && (m_rec->tap_events[k][0].first - m_rec->tap_events[k - 1][0].first
                                            == sc_core::sc_time(m_cfgs.delay_ns / m_cfgs.taps, sc_core::SC_NS));
            check("相邻抽头间隔均为 20ns", equal_step, 1, 0, "", m_fail);
        }

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, 5.001, sc_core::SC_US);    // 略长于 T_END，保证 testbench 在 T_END 处的检查得以执行
}
