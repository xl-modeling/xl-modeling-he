#include "AnalogDelayLine.h"
#include "AnalogDelayLine_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"
#include "utils/vsource.h"

#include <algorithm>
#include <cmath>
#include <string>

// ============================================================
// AnalogDelayLine 电调（Vt 动态）测试（步长 0.1ps，时间分辨率 1fs）
//   DI 均为 5GHz、1V 正弦，并行例化 5 个模型：
//     U0：Vt 恒为 0V          U1：Vt 恒为 15V
//     U2：Vt 恒为 -2V（低于标定范围）    U3：Vt 恒为 20V（高于标定范围）
//     U4：Vt 在 5ns 处由 0V 阶跃到 15V（切换时间未建模，Vt 变化即时生效）
//   检查：
//     1. Vt 钳位：U2 输出与 U0 逐点相同，U3 输出与 U1 逐点相同
//     2. 阶跃前 [2, 4.8ns) U4 的插损 / 相位延时与 U0 相同，阶跃后 [6, 9.8ns) 与 U1 相同
//     3. 阶跃过程输出无过冲：|DOUT| ≤ 输入幅度 1V
//     4. 阶跃处输出连续：延迟线瞬间跳过 Δτ = τ(0V) - τ(15V)，未滤波时单步跳变可达 2·sin(π·f·Δτ)，
//        经二阶低通后阶跃附近最大单步变化应小于其 10%
// ============================================================

// 两路信号逐点比较
SCA_TDF_MODULE(Compare)
{
    sca_tdf::sca_in<double> a, b;
    double max_diff = 0.0;
    Compare(sc_core::sc_module_name nm) : a("a"), b("b") {}
    void processing() override{ max_diff = std::max(max_diff, std::fabs(a.read() - b.read())); }
};

// 输出幅度与单步变化
SCA_TDF_MODULE(StepMonitor)
{
    sca_tdf::sca_in<double> y;
    double max_abs = 0.0;
    double max_jump_near = 0.0;     // [t0, t1) 内最大单步变化
    double max_jump_before = 0.0;   // 阶跃前稳态最大单步变化
    StepMonitor(sc_core::sc_module_name nm, double t0, double t1) : y("y"), m_t0(t0), m_t1(t1) {}
    void processing() override{
        double t = get_time().to_seconds(), v = y.read();
        max_abs = std::max(max_abs, std::fabs(v));
        if(m_init){
            double d = std::fabs(v - m_last);
            if(t >= m_t0 && t < m_t1) max_jump_near = std::max(max_jump_near, d);
            if(t >= 2e-9 && t < 4.8e-9) max_jump_before = std::max(max_jump_before, d);
        }
        m_init = true; m_last = v;
    }
private:
    const double m_t0, m_t1;
    bool m_init = false;
    double m_last = 0.0;
};

SC_MODULE(Simulator)
    , public AbstractSimulator
{
public:
    sca_tdf::sca_signal<double> di[5], vt[5], dout[5];

    SC_HAS_PROCESS(Simulator);
    explicit Simulator(sc_core::sc_module_name name)
        : sc_module(name)
        , m_fail(0)
    {
        m_cfgs = AnalogDelayLine::get_configs_from_json("../../json/AnalogDelayLine.json");
        const sc_core::sc_time ts(0.1, sc_core::SC_PS);
        const double vts[4] = {0.0, 15.0, -2.0, 20.0};

        for(int i = 0; i < 5; i++){
            std::string n = std::to_string(i);
            m_src[i] = new SINE(("src" + n).c_str(), 0.0, 1.0, F, 0.0, 0.0, 0.0, 0, ts);
            m_src[i]->out(di[i]);
            m_dut[i] = new AnalogDelayLine(("adl" + n).c_str(), m_cfgs);
            m_dut[i]->di(di[i]);
            m_dut[i]->vt(vt[i]);
            m_dut[i]->dout(dout[i]);
        }
        for(int i = 0; i < 4; i++){
            m_dc[i] = new DC(("vt" + std::to_string(i)).c_str(), vts[i], ts);
            m_dc[i]->out(vt[i]);
        }
        m_step = new PULSE("vt_step", 0.0, 15.0, 5e-9, 1e-12, 1e-12, 1.0, 2.0, 0, ts);
        m_step->out(vt[4]);

        m_cmp_lo = new Compare("cmp_lo"); m_cmp_lo->a(dout[2]); m_cmp_lo->b(dout[0]);
        m_cmp_hi = new Compare("cmp_hi"); m_cmp_hi->a(dout[3]); m_cmp_hi->b(dout[1]);

        m_fit_ref_before = new SineFitter("fit_ref_before", F, 2e-9, 4.8e-9); m_fit_ref_before->y(dout[0]);
        m_fit_ref_after  = new SineFitter("fit_ref_after",  F, 6e-9, 9.8e-9); m_fit_ref_after->y(dout[1]);
        m_fit_before     = new SineFitter("fit_before",     F, 2e-9, 4.8e-9); m_fit_before->y(dout[4]);
        m_fit_after      = new SineFitter("fit_after",      F, 6e-9, 9.8e-9); m_fit_after->y(dout[4]);
        m_mon = new StepMonitor("mon", 4.99e-9, 5.1e-9); m_mon->y(dout[4]);

        m_tf = sca_util::sca_create_tabular_trace_file("wave.dat");
        sca_util::sca_trace(m_tf, di[4], "di");
        sca_util::sca_trace(m_tf, vt[4], "vt");
        sca_util::sca_trace(m_tf, dout[4], "dout");
        sca_util::sca_trace(m_tf, dout[0], "dout_vt0");
        sca_util::sca_trace(m_tf, dout[1], "dout_vt15");

        SC_THREAD(testbench);
    }

    ~Simulator(){
        sca_util::sca_close_tabular_trace_file(m_tf);
        delete m_mon; delete m_fit_after; delete m_fit_before; delete m_fit_ref_after; delete m_fit_ref_before;
        delete m_cmp_hi; delete m_cmp_lo; delete m_step;
        for(int i = 0; i < 4; i++) delete m_dc[i];
        for(int i = 0; i < 5; i++){ delete m_dut[i]; delete m_src[i]; }
    }

private:
    static constexpr double F = 5e9;

    AnalogDelayLine::configs m_cfgs;
    SINE* m_src[5];
    DC* m_dc[4];
    PULSE* m_step;
    AnalogDelayLine* m_dut[5];
    Compare *m_cmp_lo, *m_cmp_hi;
    SineFitter *m_fit_ref_before, *m_fit_ref_after, *m_fit_before, *m_fit_after;
    StepMonitor* m_mon;
    sca_util::sca_trace_file* m_tf;
    size_t m_fail;

    void testbench(){
        wait(sc_core::sc_time(10, sc_core::SC_NS));

        info("Vt 钳位：超出标定范围时按 0V / 15V 处理");
        check("Vt=-2V 与 Vt=0V 输出最大差值", m_cmp_lo->max_diff, 0.0, 0.0, "V", m_fail, 40, 6);
        check("Vt=20V 与 Vt=15V 输出最大差值", m_cmp_hi->max_diff, 0.0, 0.0, "V", m_fail, 40, 6);

        info("Vt 阶跃 0V → 15V（5ns）：阶跃前后与静态 Vt 一致");
        check("阶跃前插损（与 Vt=0V 之差）", m_fit_before->gain_db() - m_fit_ref_before->gain_db(), 0.0, 1e-3, "dB", m_fail);
        check("阶跃前相位延时（与 Vt=0V 之差）", m_fit_before->phase_delay_ps() - m_fit_ref_before->phase_delay_ps(), 0.0, 1e-3, "ps", m_fail);
        check("阶跃后插损（与 Vt=15V 之差）", m_fit_after->gain_db() - m_fit_ref_after->gain_db(), 0.0, 1e-3, "dB", m_fail);
        check("阶跃后相位延时（与 Vt=15V 之差）", m_fit_after->phase_delay_ps() - m_fit_ref_after->phase_delay_ps(), 0.0, 1e-3, "ps", m_fail);
        info((std::string("阶跃前 / 后相位延时: ") + std::to_string(m_fit_before->phase_delay_ps()) + " / "
              + std::to_string(m_fit_after->phase_delay_ps()) + " ps").c_str());

        info("阶跃过程：无过冲，输出连续");
        check("|DOUT| 最大值 ≤ 输入幅度", m_mon->max_abs <= 1.0, 1, 0, "", m_fail);
        const double dtau = (m_cfgs.delay_ps.front() - m_cfgs.delay_ps.back()) * 1e-12;
        const double raw_jump = 2.0 * std::fabs(std::sin(M_PI * F * dtau));
        info((std::string("阶跃前稳态最大单步变化: ") + std::to_string(m_mon->max_jump_before) + " V；"
              + "未滤波时延迟跳变可引起的最大单步跳变: " + std::to_string(raw_jump) + " V").c_str());
        check("阶跃附近最大单步变化 < 未滤波跳变的 10%", m_mon->max_jump_near, 0.05 * raw_jump, 0.05 * raw_jump, "V", m_fail);

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }
};

int sc_main(int argc, char* argv[]) {
    sc_core::sc_set_time_resolution(1, sc_core::SC_FS);     // 0.1ps 步长需要 fs 级时间分辨率
    Simulator sim("sim");
    SIMULATOR_START(sim, 10.001, sc_core::SC_NS);
}
