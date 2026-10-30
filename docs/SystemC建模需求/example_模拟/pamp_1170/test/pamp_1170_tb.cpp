#include <systemc>
#include <systemc-ams>
#include <fstream>
#include <iostream>
#include <cmath>
#include "pamp_1170.h"

using namespace sc_core;
using namespace sca_tdf;

// 正弦波信号源（实信号，频率可调）
SCA_TDF_MODULE(SineSource) {
    sca_tdf::sca_out<double> out;
    double amplitude;      // 峰值电压 (V)
    double freq_hz;        // 信号频率 (Hz)
    double phase_offset;

    SCA_CTOR(SineSource)
        : amplitude(0.05), freq_hz(38.5e9), phase_offset(0.0) {}

    void set_attributes()  {
        set_timestep(1, SC_PS);   // 与功放步长一致
    }

    void processing() {
        double t = get_time().to_seconds();
        double value = amplitude * std::sin(2.0 * M_PI * freq_hz * t + phase_offset);
        out.write(value);
    }
};

// 监控模块：记录输入/输出电压
SCA_TDF_MODULE(VoltageMonitor) {
    sca_tdf::sca_in<double> vin;
    sca_tdf::sca_in<double> vout;
    std::ofstream csv;

    SCA_CTOR(VoltageMonitor) {
        csv.open("pamp_1170_output.csv");
        csv << "time_s,input_V,output_V\n";
    }
    ~VoltageMonitor() { if (csv.is_open()) csv.close(); }

    void set_attributes()  {
        set_timestep(1, SC_PS);
    }

    void processing()  {
        double t = get_time().to_seconds();
        double in = vin.read();
        double out = vout.read();
        if (csv.is_open()) {
            csv << t << "," << in << "," << out << "\n";
        }
    }
};

int sc_main(int argc, char* argv[]) {
    sc_set_time_resolution(1, SC_PS);

    sca_tdf::sca_signal<double> src_sig, amp_out;

    // 信号源：38.5 GHz，小信号幅度 0.05 V (~ -10 dBm 输入)
    SineSource source("source");
    source.amplitude = 1;      // 小信号，线性区
    source.freq_hz = 40e9;
    source.out(src_sig);

    // 待测放大器
    PowerAmplifier_MAAP011170 dut("dut");
    dut.pa_in(src_sig);
    dut.pa_out(amp_out);

    // 监控
    VoltageMonitor monitor("monitor");
    monitor.vin(src_sig);
    monitor.vout(amp_out);

    // VCD 波形
    sca_trace_file* vcd = sca_create_vcd_trace_file("pamp_1170_waveform.vcd");
    sca_trace(vcd, src_sig, "input_voltage");
    sca_trace(vcd, amp_out, "output_voltage");

    // 仿真时长：500 ps (覆盖约 20 个射频周期)
    sc_start(1, SC_NS);

    sca_close_vcd_trace_file(vcd);
    std::cout << "\n仿真完成，输出文件：\n"
              << "  - pamp_1170_output.csv\n"
              << "  - pamp_1170_waveform.vcd\n";

    return 0;
}