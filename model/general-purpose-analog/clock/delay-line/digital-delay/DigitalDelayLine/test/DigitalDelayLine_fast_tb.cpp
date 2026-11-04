#include "DigitalDelayLine.h"
#include "DigitalDelayLine_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"

#include <random>
#include <string>

// ============================================================
// DigitalDelayLine 快速输入测试（输入变化快于总延迟，验证传输延迟不丢沿）
//   激励：
//     阶段 A [0, 2µs)   ：手册最小脉宽 tWI = 20ns、最小周期 40ns 的方波（半周期 20ns，远短于总延迟 100ns）
//     阶段 B [2µs, 6µs) ：随机脉宽 20 ~ 200ns 的不规则脉冲串（固定种子，可复现）
//   检查：
//     1. 各阶段、各抽头的每个沿 = 输入对应沿平移 k × 20ns，沿数与输入一致
//     2. 任一时刻同时在途的沿数远多于 1（证明延迟期间的输入变化未丢失）
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
        m_cfgs = DigitalDelayLine::get_configs_from_json("../../json/DigitalDelayLine.json");
        m_ddl = new DigitalDelayLine(m_cfgs.module_name.c_str(), m_cfgs);
        taps.init(m_cfgs.taps);
        m_ddl->in(iclk);
        m_ddl->taps(taps);

        m_rec = new EdgeRecorder("rec", m_cfgs.taps);
        m_rec->in(iclk);
        m_rec->taps(taps);

        SC_THREAD(stimulus);
        SC_THREAD(testbench);
    }

    ~Simulator(){
        delete m_rec;
        delete m_ddl;
    }

private:
    DigitalDelayLine::configs m_cfgs;
    DigitalDelayLine* m_ddl;
    EdgeRecorder* m_rec;
    size_t m_fail;

    const sc_core::sc_time T_A = sc_core::sc_time(2, sc_core::SC_US);
    const sc_core::sc_time T_END = sc_core::sc_time(6, sc_core::SC_US);

    void stimulus(){
        const double tw_min_ns = 0.2 * m_cfgs.delay_ns;      // 手册：tWI = 20% × TAP5 延迟
        iclk.write(false);
        bool v = false;
        while(sc_core::sc_time_stamp() + sc_core::sc_time(tw_min_ns, sc_core::SC_NS) < T_A){
            wait(tw_min_ns, sc_core::SC_NS); v = !v; iclk.write(v);
        }
        wait(T_A - sc_core::sc_time_stamp());
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> width_ns(static_cast<int>(tw_min_ns), 200);
        while(true){
            v = !v; iclk.write(v);
            wait(width_ns(rng), sc_core::SC_NS);
        }
    }

    sc_core::sc_time tap_delay(size_t k) const {
        return sc_core::sc_time(m_cfgs.delay_ns * (k + 1) / m_cfgs.taps, sc_core::SC_NS);
    }

    static EdgeRecorder::Events window(const EdgeRecorder::Events& ev, sc_core::sc_time t0, sc_core::sc_time t1){
        EdgeRecorder::Events out;
        for(const auto& e : ev) if(e.first >= t0 && e.first < t1) out.push_back(e);
        return out;
    }

    void testbench(){
        wait(T_END);
        const sc_core::sc_time zero = sc_core::SC_ZERO_TIME;

        struct Phase { const char* name; sc_core::sc_time t0, t1; };
        const Phase phases[2] = {{"阶段 A（20ns 脉宽方波）", zero, T_A}, {"阶段 B（随机脉宽 20~200ns）", T_A, T_END}};
        for(const Phase& ph : phases){
            auto in = window(m_rec->in_events, ph.t0, ph.t1);
            info((std::string(ph.name) + "：输入沿数 " + std::to_string(in.size())).c_str());
            for(size_t k = 0; k < m_cfgs.taps; k++){
                auto tap = window(m_rec->tap_events[k], ph.t0 + tap_delay(k), ph.t1 + tap_delay(k));
                std::string s = "TAP" + std::to_string(k + 1) + "（" + tap_delay(k).to_string() + "）";
                size_t expected = 0;        // 平移后仍在仿真结束前的输入沿数
                for(const auto& e : in) if(e.first + tap_delay(k) < T_END) expected++;
                check((s + " 沿数 = 输入沿数").c_str(), tap.size(), expected, 0, "", m_fail);
                check((s + " 各沿与输入平移一致").c_str(), EdgeRecorder::shifted_equal(in, tap, tap_delay(k), T_END), 1, 0, "", m_fail);
            }
        }

        {
            // 同时在途沿数：输入沿已发生、但 TAP5 尚未输出的沿的最大个数
            size_t max_inflight = 0;
            const auto& in = m_rec->in_events;
            for(size_t i = 0; i < in.size(); i++){
                size_t n = 0;
                for(size_t j = 0; j <= i; j++) if(in[j].first + tap_delay(m_cfgs.taps - 1) > in[i].first) n++;
                max_inflight = std::max(max_inflight, n);
            }
            info("阶段 A 中延迟线内同时存在多个在途沿（输入半周期 20ns < 总延迟 100ns）");
            check("TAP5 最大同时在途沿数", max_inflight, 5, 0, "", m_fail);
        }

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    Simulator sim("sim");
    SIMULATOR_START(sim, 6.001, sc_core::SC_US);    // 略长于 T_END，保证 testbench 在 T_END 处的检查得以执行
}
