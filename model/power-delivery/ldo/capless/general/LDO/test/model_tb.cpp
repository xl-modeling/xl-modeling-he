#include "LDO.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>
#include <complex>
#include <vector>

struct Point {
    std::string name;
    double vdd, vref, load, temperature;
};

std::vector<Point> points = {
    {"nominal", 1.2, 1.0, .001, 27},
    {"outline_output", 1.2, 1.0, .001, 27},
    {"full_load_high_ref", 1.2, 1.0, .1, 27},
    {"full_load_low_ref", 1.4, .9, .1, 27},
    {"no_load_low_ref", 1.4, .9, 0, 27},
    {"no_load_high_ref", 1.4, 1.2, 0, 27},
    {"line_low", 1.2, 1.0, .1, 27},
    {"line_high", 1.4, 1.0, .1, 27},
    {"load_zero", 1.2, 1.0, 0, 27},
    {"load_full", 1.2, 1.0, .1, 27},
    {"dropout_high_supply", 1.4, 1.0, .1, 27},
    {"validation_0",1.225,1.0,.025,27},
    {"validation_1",1.325,1.0,.025,27},
    {"validation_2",1.4,1.05,.025,27},
    {"validation_3",1.3,1.0,.0005,27}

};

struct Stimulus : sca_tdf::sca_module {
    sca_tdf::sca_out<double> bias, ground, reference, supply, analog_supply;
    sca_tdf::sca_out<double> load, conductance, capacitance, temperature;
    std::string mode;
    double frequency, step;
    double ac_amplitude = 1, dc_supply = 1.2, ripple_amplitude = .001;
    double ac_temperature = 27;
    Stimulus(sc_core::sc_module_name n, std::string m, double f, double dt)
        : sca_tdf::sca_module(n), mode(m), frequency(f), step(dt) {}
    void set_attributes() override { set_timestep(step, sc_core::SC_SEC); }
    void processing() override {
        double t = get_time().to_seconds();
        Point p = points.front();
        if (mode == "dc") {
            size_t index = static_cast<size_t>(frequency);
            p = points[index];
        }
        if (mode.rfind("ac", 0) == 0 || mode == "noise") {
            p.vdd = dc_supply; p.vref = 1.0; p.load = .1; p.temperature = ac_temperature;
        }
        if (mode == "psrr") {
            p.load = .1;
            p.vdd = dc_supply;
            p.vdd += ripple_amplitude * std::sin(2 * std::acos(-1.) * frequency * t);
        }
        if (mode == "transient") {
            p.vdd = dc_supply; p.vref = 1.0;
            double up = std::max(0., std::min((t - 5e-6) / 100e-9, 1.));
            double down = std::max(0., std::min((t - 10e-6) / 100e-9, 1.));
            p.load = .001 + .099 * (up - down);
        }
        if (mode == "line") {
            double rise = std::clamp((t-4e-6)/1e-6,0.0,1.0);
            double fall = std::clamp((t-9e-6)/1e-6,0.0,1.0);
            p.vdd = 1.2 + .2*(rise-fall); p.vref=1.0; p.load=.1;
        }
        if (mode == "startup") { p.vdd=dc_supply; p.vref=1.0; }
        double ramp = mode == "startup" ? std::min(t / 1e-6, 1.) : 1.;
        bias.write(1e-5 * ramp);
        ground.write(0);
        reference.write(p.vref * ramp);
        supply.write(p.vdd * ramp);
        analog_supply.write(2.5 * ramp);
        load.write(mode == "startup" ? 0 : p.load);
        conductance.write(mode == "startup" ? .001 : 0);
        capacitance.write(mode == "transient" ? frequency : 0);
        temperature.write(p.temperature);
    }
    void ac_processing() override {
        using sca_ac_analysis::sca_ac;
        sca_ac(supply) = mode == "noise" ? 0 : ac_amplitude;
        sca_ac(bias) = 0; sca_ac(ground) = 0; sca_ac(reference) = 0;
        sca_ac(analog_supply) = 0; sca_ac(load) = 0;
        sca_ac(conductance) = 0; sca_ac(capacitance) = 0; sca_ac(temperature) = 0;
    }
};

struct Monitor : sca_tdf::sca_module {
    sca_tdf::sca_in<double> output, supply_current, analog_current, bias_current, pass;
    sca_tdf::sca_in<double> supply, load, conductance;
    std::string mode;
    double frequency;
    std::ofstream csv;
    std::vector<double> sum_v, sum_i, sum_q;
    std::vector<unsigned long> counts;
    double sine_v = 0, cosine_v = 0, sine_in = 0, cosine_in = 0;
    double max_residual = 0, previous_v = 0, step;
    unsigned long samples = 0, tick = 0, stride;
    Monitor(sc_core::sc_module_name n, std::string m, double f, double dt, const char* path)
        : sca_tdf::sca_module(n), mode(m), frequency(f), csv(path),
          sum_v(points.size()), sum_i(points.size()), sum_q(points.size()), counts(points.size()), step(dt) {
        stride = mode == "psrr" ? std::max(1UL, static_cast<unsigned long>(1. / (f * dt * 500))) : 10;
        csv << "time_s,vdd_v,vout_v,load_a,pass_a,iq_a,kcl_residual_a\n" << std::setprecision(16);
    }
    void processing() override {
        double t = get_time().to_seconds();
        double v = output.read(), current = pass.read();
        double external_load = load.read() + conductance.read() * v;
        double iq = supply_current.read() + analog_current.read() + bias_current.read() - external_load;
        double total_capacitance = 100e-12 + (mode == "transient" ? frequency : 0);
        double residual = current - external_load - total_capacitance * (v - previous_v) / step;
        previous_v = v;
        max_residual = std::max(max_residual, std::abs(residual));
        if (tick++ % stride == 0) {
            csv << t << ',' << supply.read() << ',' << v << ',' << external_load << ',' << current << ',' << iq << ',' << residual << '\n';
        }
        if (mode == "dc") {
            size_t i = static_cast<size_t>(frequency);
            if (i < points.size() && t >= 9e-6) {
                sum_v[i] += v; sum_i[i] += current; sum_q[i] += iq; ++counts[i];
            }
        }
        if (mode == "psrr" && t >= 10 / frequency) {
            double phase = 2 * std::acos(-1.) * frequency * t;
            sine_v += v * std::sin(phase); cosine_v += v * std::cos(phase);
            sine_in += supply.read() * std::sin(phase); cosine_in += supply.read() * std::cos(phase);
            ++samples;
        }
    }
    void ac_processing() override {}
    void report() {
        std::cout << std::setprecision(12);
        if (mode == "dc") {
            for (size_t i = 0; i < points.size(); ++i)
                if (counts[i]) std::cout << "DC " << points[i].name << ' ' << sum_v[i] / counts[i] << ' ' << sum_i[i] / counts[i] << ' ' << sum_q[i] / counts[i] << '\n';
        }
        if (mode == "psrr") {
            double input = 2 * std::hypot(sine_in, cosine_in) / samples;
            double output_amplitude = 2 * std::hypot(sine_v, cosine_v) / samples;
            std::cout << "RIPPLE " << frequency << ' ' << input << ' ' << output_amplitude << ' ' << 20 * std::log10(input / output_amplitude) << '\n';
        }
        std::cout << "KCL_MAX_A " << max_residual << '\n';
    }
};

int sc_main(int argc, char** argv) {
    if (argc < 6 || argc > 9) return 2;
    std::string mode = argv[2];
    double frequency = std::stod(argv[3]);
    double step = std::stod(argv[4]);
    bool point_mode = mode == "point";
    if (point_mode) {
        if (argc != 7) return 2;
        std::ifstream condition_file(argv[6]);
        nlohmann::json condition; condition_file >> condition;
        points = {{condition.value("name",std::string("custom")),condition.at("vdd"),condition.at("vref"),condition.at("load"),condition.value("temperature",27.0)}};
        mode="dc"; frequency=0;
    }
    bool ac_mode = mode == "ac" || mode == "ac_sweep" || mode == "ac_points" || mode == "noise";
    if (!(ac_mode || mode == "dc" || mode == "psrr" || mode == "transient" || mode == "startup" || mode == "line")) return 2;
    if (mode == "dc" && (frequency < 0 || frequency >= points.size() || std::floor(frequency) != frequency)) return 2;
    if (mode == "ac_sweep" && (frequency < 1 || frequency > 10000 || std::floor(frequency) != frequency)) return 2;
    if ((mode == "ac" || mode == "psrr") && frequency <= 0) return 2;
    auto cfg = LDO::get_configs_from_json(argv[1]);
    cfg.timestep_s = step;
    sca_tdf::sca_signal<double> bias, ground, reference, supply, analog_supply;
    sca_tdf::sca_signal<double> load, conductance, capacitance, temperature;
    sca_tdf::sca_signal<double> output, supply_current, analog_current, bias_current, pass;
    Stimulus stimulus("stimulus", mode, frequency, step);
    if (argc >= 7 && !point_mode) stimulus.ac_amplitude = std::stod(argv[6]);
    if (argc >= 7 && !point_mode) stimulus.ripple_amplitude = std::stod(argv[6]);
    if (argc >= 8 && !point_mode) stimulus.dc_supply = std::stod(argv[7]);
    if (argc >= 9 && !point_mode) stimulus.ac_temperature = std::stod(argv[8]);
    if (!std::isfinite(stimulus.ac_amplitude) || stimulus.ac_amplitude <= 0 || !std::isfinite(stimulus.dc_supply)) return 2;
    stimulus.bias(bias); stimulus.ground(ground); stimulus.reference(reference);
    stimulus.supply(supply); stimulus.analog_supply(analog_supply); stimulus.load(load);
    stimulus.conductance(conductance); stimulus.capacitance(capacitance); stimulus.temperature(temperature);
    LDO dut("dut", cfg);
    dut.IBIAS(bias); dut.AVSS(ground); dut.VREF(reference); dut.VDD(supply); dut.AVDD(analog_supply);
    dut.load_current(load); dut.load_conductance(conductance); dut.external_capacitance(capacitance);
    dut.temperature(temperature); dut.VOUT(output); dut.vdd_current(supply_current);
    dut.avdd_current(analog_current); dut.ibias_current(bias_current); dut.pass_current(pass);
    std::string time_path = std::string(argv[5]) + (ac_mode ? ".op.csv" : "");
    Monitor monitor("monitor", mode, frequency, step, time_path.c_str());
    monitor.output(output); monitor.supply_current(supply_current); monitor.analog_current(analog_current);
    monitor.bias_current(bias_current); monitor.pass(pass); monitor.supply(supply);
    monitor.load(load); monitor.conductance(conductance);

    double duration = mode == "dc" ? 10e-6 : 15e-6;
    if (mode == "psrr") duration = 20 / frequency;
    sc_core::sc_start(duration, sc_core::SC_SEC);
    monitor.report();
    const auto& op = dut.get_dc_operating_point();
    std::cout << std::setprecision(16) << "CONFIG " << op.vdd << ' ' << op.vref << ' ' << op.avdd << ' ' << op.bias << ' ' << op.load << ' ' << op.temperature << '\n';
    std::cout << "OP " << op.voltage << ' ' << op.current << ' ' << op.capacitive_current << '\n';
    if (ac_mode) {
        std::string trace_name = std::string(argv[5]) + ".ams";
        auto* trace = sca_util::sca_create_tabular_trace_file(trace_name.c_str());
        if(mode!="noise") sca_util::sca_trace(trace, supply, "VDD");
        sca_util::sca_trace(trace, output, "VOUT");
        if(mode!="noise") sca_util::sca_trace(trace, pass, "IPASS");
        if (mode == "noise") {
            sca_ac_analysis::sca_ac_noise_start(10,100000,161);
            sca_util::sca_close_tabular_trace_file(trace);
            return 0;
        }
        if (mode == "ac_sweep") sca_ac_analysis::sca_ac_start(1000, 1000000, 3 * static_cast<unsigned long>(frequency) + 1);
        else {
            sca_util::sca_vector<double> frequencies;
            if (mode == "ac_points") { frequencies(0) = 1000; frequencies(1) = 100000; frequencies(2) = 1000000; }
            else frequencies(0) = frequency;
            sca_ac_analysis::sca_ac_start(frequencies);
        }
        sca_util::sca_close_tabular_trace_file(trace);
        std::ifstream native(trace_name);
        std::ofstream exported(argv[5]);
        if (!native || !exported) return 4;
        exported << "frequency_hz,vdd_real_v,vdd_imag_v,vout_real_v,vout_imag_v,input_amplitude_v,output_amplitude_v,psrr_db,pass_real_a,pass_imag_a\n" << std::setprecision(16);
        std::string line;
        while (std::getline(native, line)) {
            if (line.empty() || line[0]=='%') continue;
            std::istringstream row(line);
            double f, vr, vi, outr, outi, ir, ii;
            if (!(row >> f >> vr >> vi >> outr >> outi >> ir >> ii)) return 4;
            double vin=std::hypot(vr,vi), vout=std::hypot(outr,outi), psrr=20*std::log10(vin/vout);
            exported << f << ',' << vr << ',' << vi << ',' << outr << ',' << outi << ',' << vin << ',' << vout << ',' << psrr << ',' << ir << ',' << ii << '\n';
            for (double target : {1000.,100000.,1000000.})
                if (std::abs(f-target)<target*1e-10) std::cout << "AC " << f << ' ' << vin << ' ' << vout << ' ' << psrr << '\n';
        }
        std::cout << "AC_REGION " << dut.get_ac_region() << " BOUNDARY " << dut.get_ac_branch_boundary() << '\n';
        if (dut.get_ac_branch_boundary()) return 3;
    }
    return 0;
}
