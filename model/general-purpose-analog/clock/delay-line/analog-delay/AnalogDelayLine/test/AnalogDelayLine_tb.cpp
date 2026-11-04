#include "AnalogDelayLine.h"
#include "AnalogDelayLine_monitor.h"

#include "utils/simulator.h"
#include "utils/verification.h"
#include "utils/vsource.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <memory>
#include <string>
#include <vector>

// ============================================================
// AnalogDelayLine 频率响应测试（步长 0.1ps，时间分辨率 1fs）
//   并行例化 5 个 Vt × 5 个频率 = 25 个模型，DI = sin(2π·f·t)，VT 恒定
//   在 [1ns, 21ns) 内拟合输出幅度与相位，得到插损与相位延时
//   检查：
//     0. 模型信息接口 name / version / latency / power / area
//     1. 与模型传递函数 H(s) = G0·e^{-sτ}/(1 + s/(Qω0) + s²/ω0²)（τ、f0 按 Vt 插值）一致：
//        插损误差 ≤ 0.01dB，相位延时误差 ≤ 1 个步长（延迟线按步长取整）；Vt = 4.5V 为标定点间插值
//     2. 与数据手册（插损 VS 频率、归一化群延时 VS 频率曲线）一致：
//        1~20GHz 插损误差 ≤ 0.5dB；1GHz 相对延时（相对 Vt = 15V）误差 ≤ 0.3ps
//   高频群延时与手册的偏差见自测试报告
// ============================================================
SC_MODULE(Simulator)
    , public AbstractSimulator
{
public:
    SC_HAS_PROCESS(Simulator);
    explicit Simulator(sc_core::sc_module_name name)
        : sc_module(name)
        , m_fail(0)
    {
        m_cfgs = AnalogDelayLine::get_configs_from_json("../../json/AnalogDelayLine.json");
        const sc_core::sc_time ts(STEP_PS, sc_core::SC_PS);

        for(size_t i = 0; i < VTS.size(); i++){
            for(size_t k = 0; k < FREQS.size(); k++){
                std::string tag = "_" + std::to_string(i) + "_" + std::to_string(k);
                Unit u;
                u.di.reset(new sca_tdf::sca_signal<double>(("di" + tag).c_str()));
                u.vt.reset(new sca_tdf::sca_signal<double>(("vt" + tag).c_str()));
                u.dout.reset(new sca_tdf::sca_signal<double>(("dout" + tag).c_str()));
                u.src.reset(new SINE(("src" + tag).c_str(), 0.0, 1.0, FREQS[k], 0.0, 0.0, 0.0, 0, ts));
                u.vsrc.reset(new DC(("vsrc" + tag).c_str(), VTS[i], ts));
                u.dut.reset(new AnalogDelayLine(("adl" + tag).c_str(), m_cfgs));
                u.fit.reset(new SineFitter(("fit" + tag).c_str(), FREQS[k], 1e-9, 21e-9));
                u.src->out(*u.di);
                u.vsrc->out(*u.vt);
                u.dut->di(*u.di);
                u.dut->vt(*u.vt);
                u.dut->dout(*u.dout);
                u.fit->y(*u.dout);
                m_units.push_back(std::move(u));
            }
        }
        SC_THREAD(testbench);
    }

private:
    static constexpr double STEP_PS = 0.1;
    const std::vector<double> VTS   = {0.0, 3.0, 4.5, 9.0, 15.0};
    const std::vector<double> FREQS = {1e9, 5e9, 10e9, 15e9, 20e9};

    // 数据手册插损（dB）：Vt = 0 / 3 / 9 / 15V，频率 1 / 5 / 10 / 15 / 20GHz（由手册矢量曲线提取）
    const std::vector<double> DS_VT = {0.0, 3.0, 9.0, 15.0};
    const std::vector<std::vector<double>> DS_IL = {
        {-0.41, -1.01, -1.70, -2.96, -5.00},
        {-0.36, -0.66, -1.01, -1.70, -2.78},
        {-0.36, -0.54, -0.69, -0.93, -1.42},
        {-0.36, -0.60, -0.57, -0.93, -0.85},
    };
    // 数据手册 1GHz 归一化群延时（ps，相对 Vt = 15V）
    const std::vector<double> DS_GD1G = {26.03, 15.36, 4.16, 0.0};

    struct Unit {
        std::unique_ptr<sca_tdf::sca_signal<double>> di, vt, dout;
        std::unique_ptr<SINE> src;
        std::unique_ptr<DC> vsrc;
        std::unique_ptr<AnalogDelayLine> dut;
        std::unique_ptr<SineFitter> fit;
    };

    AnalogDelayLine::configs m_cfgs;
    std::vector<Unit> m_units;
    size_t m_fail;

    const Unit& unit(size_t i, size_t k) const { return m_units[i * FREQS.size() + k]; }

    // 模型传递函数（与 AnalogDelayLine 相同的插值规则）
    std::complex<double> model_h(double vt, double f) const {
        const auto& x = m_cfgs.vt_points_v;
        size_t j = std::upper_bound(x.begin(), x.end(), vt) - x.begin();
        j = std::min(std::max(j, size_t(1)), x.size() - 1);
        double t = (vt - x[j - 1]) / (x[j] - x[j - 1]);
        double tau = (m_cfgs.delay_ps[j - 1] + t * (m_cfgs.delay_ps[j] - m_cfgs.delay_ps[j - 1])) * 1e-12;
        double f0  = (m_cfgs.f0_ghz[j - 1]   + t * (m_cfgs.f0_ghz[j]   - m_cfgs.f0_ghz[j - 1])) * 1e9;
        std::complex<double> s(0.0, 2.0 * M_PI * f);
        double w0 = 2.0 * M_PI * f0;
        return std::pow(10.0, m_cfgs.gain_db / 20.0) * std::exp(-s * tau) / (1.0 + s / (m_cfgs.q * w0) + s * s / (w0 * w0));
    }

    // 把测得的相位延时换算到与参考值相差不足半个周期的分支上（去相位卷绕）
    static double unwrap_ps(double meas, double ref, double f){
        double period = 1e12 / f;
        return meas + period * std::round((ref - meas) / period);
    }

    void testbench(){
        {
            info("模型信息接口");
            info((std::string("name: ") + m_units[0].dut->get_name()).c_str());
            check("version", m_units[0].dut->get_version(), 0.1, 0, "", m_fail);
            check("latency（最大可调延时）", m_units[0].dut->get_latency(), 25.0, 0, "ps", m_fail);
            check("power", m_units[0].dut->get_power(), 0.15, 0, "mW", m_fail);
            check("area", m_units[0].dut->get_area(), 1.759, 0, "mm2", m_fail);
        }

        wait(sc_core::sc_time(21, sc_core::SC_NS));

        info("与模型传递函数一致（1 / 5 / 10 / 15 / 20GHz 中的最大偏差）");
        for(size_t i = 0; i < VTS.size(); i++){
            double dil = 0.0, dpd = 0.0;
            for(size_t k = 0; k < FREQS.size(); k++){
                std::complex<double> h = model_h(VTS[i], FREQS[k]);
                double il_ref = 20.0 * std::log10(std::abs(h));
                double pd_ref = -std::arg(h) / (2.0 * M_PI * FREQS[k]) * 1e12;
                double pd = unwrap_ps(unit(i, k).fit->phase_delay_ps(), pd_ref, FREQS[k]);
                dil = std::max(dil, std::fabs(unit(i, k).fit->gain_db() - il_ref));
                dpd = std::max(dpd, std::fabs(pd - pd_ref));
            }
            std::string s = "Vt=" + fmt_v(VTS[i]) + " ";
            check((s + "插损最大偏差").c_str(), dil, 0.0, 0.01, "dB", m_fail);
            check((s + "相位延时最大偏差").c_str(), dpd, 0.0, STEP_PS, "ps", m_fail);
        }

        info("与数据手册一致：1~20GHz 插损");
        for(size_t d = 0; d < DS_VT.size(); d++){
            size_t i = std::find(VTS.begin(), VTS.end(), DS_VT[d]) - VTS.begin();
            double dev = 0.0;
            for(size_t k = 0; k < FREQS.size(); k++)
                dev = std::max(dev, std::fabs(unit(i, k).fit->gain_db() - DS_IL[d][k]));
            check(("Vt=" + fmt_v(DS_VT[d]) + " 插损最大偏差").c_str(), dev, 0.0, 0.5, "dB", m_fail);
        }

        info("与数据手册一致：1GHz 相对延时（相对 Vt = 15V）");
        size_t i15 = std::find(VTS.begin(), VTS.end(), 15.0) - VTS.begin();
        double pd15 = unit(i15, 0).fit->phase_delay_ps();
        for(size_t d = 0; d + 1 < DS_VT.size(); d++){
            size_t i = std::find(VTS.begin(), VTS.end(), DS_VT[d]) - VTS.begin();
            check(("Vt=" + fmt_v(DS_VT[d]) + " 1GHz 相对延时").c_str(), unit(i, 0).fit->phase_delay_ps() - pd15, DS_GD1G[d], 0.3, "ps", m_fail);
        }

        info((std::string("FAIL 数量: ") + std::to_string(m_fail)).c_str());
    }

    static std::string fmt_v(double v){
        char buf[16]; std::snprintf(buf, sizeof(buf), "%gV", v); return buf;
    }
};

int sc_main(int argc, char* argv[]) {
    sc_core::sc_set_time_resolution(1, sc_core::SC_FS);     // 0.1ps 步长需要 fs 级时间分辨率
    Simulator sim("sim");
    SIMULATOR_START(sim, 21.001, sc_core::SC_NS);
}
