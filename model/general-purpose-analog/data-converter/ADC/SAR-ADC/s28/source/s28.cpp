#include "s28.h"
#include "detail/json_value.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace s28 {
Config Config::load(const std::string& path) {
    const auto j = s28_json::load(path);
    Config c;
    const std::set<std::string> allowed = {"name","version","category","adc_bits","sample_rate_hz",
        "timestep_s","reference_v","common_mode_v","supply_v","pad_ring_v","decision_time_s","clockout_width_s",
        "offset_lsb","span_error_percent","noise_rms_lsb","quadratic_v_inv","cubic_v_inv2","seed",
        "transition_error_lsb","power","area","latency","dimensions","ports","evidence",
        "vcmp_center_v","vcmp_gain_v_per_v","ctl_center_v","ctl_aperture_s_per_v"};
    for (const auto& item : j.object) if (!allowed.count(item.first)) throw std::runtime_error("Unknown model field: " + item.first);
    auto value = [&](const std::string& k) -> const s28_json::Value& {
        const auto& entry = j.at(k);
        const auto type = entry.at("type").as_string();
        const auto& v = entry.at("value");
        entry.at("editable").as_bool();
        entry.at("unit").as_string();
        entry.at("source").as_string();
        entry.at("group").as_string();
        const bool numeric = type == "int" || type == "float";
        const bool nullable = k == "power" || k == "area";
        if (!((nullable && v.kind == s28_json::Value::Null) ||
              (numeric && v.kind == s28_json::Value::Number &&
               (type != "int" || std::floor(v.number) == v.number)) ||
              (type == "string" && v.kind == s28_json::Value::String) ||
              (type == "array" && v.kind == s28_json::Value::Array) ||
              (type == "object" && v.kind == s28_json::Value::Object)))
            throw std::runtime_error("Configuration type/value mismatch: " + k);
        return v;
    };
    auto num = [&](const std::string& k) { return value(k).as_number(); };
    c.name = value("name").as_string(); c.version = value("version").as_string();
    c.category = value("category").as_string();
    if (num("adc_bits") != 6) throw std::runtime_error("S28 is a fixed 6-bit ADC");
    c.sample_rate_hz = num("sample_rate_hz"); c.timestep_s = num("timestep_s");
    c.reference_v = num("reference_v"); c.common_mode_v = num("common_mode_v"); c.supply_v = num("supply_v");
    c.pad_ring_v = num("pad_ring_v");
    c.decision_time_s = num("decision_time_s"); c.clockout_width_s = num("clockout_width_s");
    c.offset_lsb = num("offset_lsb"); c.span_error_percent = num("span_error_percent");
    c.noise_rms_lsb = num("noise_rms_lsb"); c.quadratic_v_inv = num("quadratic_v_inv"); c.cubic_v_inv2 = num("cubic_v_inv2");
    const auto seed_value = num("seed");
    if (seed_value < 0 || seed_value > 4294967295.0 || std::floor(seed_value) != seed_value) throw std::runtime_error("Invalid seed");
    c.seed = static_cast<std::uint64_t>(seed_value);
    c.vcmp_center_v = num("vcmp_center_v"); c.vcmp_gain_v_per_v = num("vcmp_gain_v_per_v");
    c.ctl_center_v = num("ctl_center_v"); c.ctl_aperture_s_per_v = num("ctl_aperture_s_per_v");
    const auto& a = value("transition_error_lsb");
    if (a.kind != s28_json::Value::Array || a.array.size() != 63) throw std::runtime_error("63 internal transitions required");
    for (unsigned i = 0; i < 63; ++i) c.transition_error_lsb[i] = a.array[i].as_number();
    if (value("power").kind != s28_json::Value::Null) c.power_w = num("power");
    if (value("area").kind != s28_json::Value::Null) c.area_mm2 = num("area");
    if (std::abs(num("latency") - 6*c.decision_time_s) > 1e-18) throw std::runtime_error("latency must equal six decision intervals");
    c.validate(); return c;
}
void Config::validate() const {
    auto finite = [](double v) { if (!std::isfinite(v)) throw std::runtime_error("Nonfinite configuration"); };
    for (double v : {sample_rate_hz,timestep_s,reference_v,common_mode_v,supply_v,pad_ring_v,decision_time_s,
         clockout_width_s,offset_lsb,span_error_percent,noise_rms_lsb,quadratic_v_inv,cubic_v_inv2,
         vcmp_center_v,vcmp_gain_v_per_v,ctl_center_v,ctl_aperture_s_per_v}) finite(v);
    if (bits != 6 || sample_rate_hz <= 0 || sample_rate_hz > 100e6 || timestep_s <= 0 ||
        reference_v <= 0 || supply_v <= 0 || pad_ring_v <= 0 || decision_time_s <= 0 || noise_rms_lsb < 0 ||
        span_error_percent <= -100 || clockout_width_s <= 0) throw std::runtime_error("Invalid model configuration");
    auto grid = [&](double t) { return std::abs(t/timestep_s - std::round(t/timestep_s)) < 1e-7; };
    if (!grid(decision_time_s) || !grid(clockout_width_s) || !grid(0.5/sample_rate_hz))
        throw std::runtime_error("Clock and delay must be integer multiples of timestep");
    if (6*decision_time_s + clockout_width_s >= 1/sample_rate_hz) throw std::runtime_error("Conversion does not fit sample period");
    if ((!std::isnan(power_w) && (!std::isfinite(power_w) || power_w < 0)) ||
        (!std::isnan(area_mm2) && (!std::isfinite(area_mm2) || area_mm2 <= 0))) throw std::runtime_error("Invalid metadata");
    double last = -1e100;
    for (unsigned i = 0; i < 63; ++i) {
        finite(transition_error_lsb[i]);
        double t = (-1.0 + (i+1)/32.0)*(1+span_error_percent/100) + transition_error_lsb[i]/32.0;
        if (t <= last) throw std::runtime_error("Nonmonotonic transition array");
        last = t;
    }
    // Reject a polynomial that can reverse the transfer over the specified range.
    for (unsigned i = 0; i <= 200; ++i) {
        double x = reference_v*(-1.0+i/100.0);
        if (1+2*quadratic_v_inv*x+3*cubic_v_inv2*x*x <= 0) throw std::runtime_error("Nonmonotonic input polynomial");
    }
}
S28::S28(sc_core::sc_module_name name, const Config& c) : sca_tdf::sca_module(name), cfg_(c), rng_(c.seed) { cfg_.validate(); }
void S28::set_attributes() { set_timestep(cfg_.timestep_s, sc_core::SC_SEC); }
void S28::initialize() { }
double S28::uniform_open() { return (static_cast<double>(rng_() >> 11)+0.5)/9007199254740992.0; }
double S28::gaussian() { return std::sqrt(-2*std::log(uniform_open()))*std::cos(6.2831853071795864769*uniform_open()); }
double S28::threshold(unsigned code) const {
    return (-reference_ + code*(2*reference_/64))*(1+cfg_.span_error_percent/100)
        + offset_v_ + cfg_.transition_error_lsb[code-1]*(2*reference_/64);
}
void S28::begin_sample(double now) {
    if (busy_) { ++overruns_; return; }
    for(double voltage : {VRP.read(),AVDD.read(),AVSS.read(),DVDD.read(),DVSS.read(),VDD.read(),VSS.read(),
        CLKVDD.read(),CLKVSS.read(),VDDD.read(),VSSD.read(),SAVDD.read(),SAVSS.read(),VSS2.read(),
        SAVDD1.read(),SAVSS1.read(),VSS1.read(),DVDDIO.read(),DVSSIO.read(),INVDD.read(),INVSS.read()})
        if(!std::isfinite(voltage))SC_REPORT_FATAL("S28", "Nonfinite supply or ground");
    reference_ = VRP.read() - AVSS.read();
    if (!(reference_ > 0) || AVDD.read() <= AVSS.read() || DVDD.read() <= DVSS.read() || VDD.read() <= VSS.read() ||
        CLKVDD.read() <= CLKVSS.read() || VDDD.read() <= VSSD.read() ||
        SAVDD.read() <= SAVSS.read() || SAVDD1.read() <= SAVSS1.read() ||
        DVDDIO.read() <= DVSSIO.read() || INVDD.read() <= INVSS.read())
        SC_REPORT_FATAL("S28", "Invalid reference or supply");
    sampled_ = IP.read() - IN.read();
    if (!std::isfinite(sampled_) || !std::isfinite(VCMP.read()) || !std::isfinite(CTL.read()))
        SC_REPORT_FATAL("S28", "Nonfinite analog input");
    corrected_ = sampled_ + cfg_.quadratic_v_inv*sampled_*sampled_ + cfg_.cubic_v_inv2*sampled_*sampled_*sampled_;
    if (cfg_.noise_rms_lsb > 0) corrected_ += cfg_.noise_rms_lsb*(2*reference_/64)*gaussian();
    offset_v_ = cfg_.offset_lsb*(2*reference_/64) + cfg_.vcmp_gain_v_per_v*(VCMP.read()-cfg_.vcmp_center_v);
    active_sample_time_ = now; sar_ = 0; bit_ = 5; busy_ = true; ++accepted_;
    due_ = now + cfg_.decision_time_s;
}
void S28::processing() {
    const double now = get_time().to_seconds(), eps = cfg_.timestep_s*1e-5;
    const bool high = CLK_IP.read() > CLK_IN.read();
    // Falling differential edge ends sampling; six asynchronous SAR decisions follow.
    if (!high && clock_high_) {
        const double aperture = cfg_.ctl_aperture_s_per_v*(CTL.read()-cfg_.ctl_center_v);
        if (aperture < 0 || aperture >= 0.5/cfg_.sample_rate_hz) SC_REPORT_FATAL("S28", "CTL aperture outside model range");
        if (next_sample_due_ >= 0) ++overruns_;
        else next_sample_due_ = now + aperture;
    }
    clock_high_ = high;
    if (next_sample_due_ >= 0 && now+eps >= next_sample_due_) { next_sample_due_ = -1; begin_sample(now); }
    if (busy_ && now+eps >= due_) {
        const unsigned trial = sar_ | (1u << bit_);
        if (corrected_ >= threshold(trial)) sar_ = trial;
        if (--bit_ < 0) {
            busy_ = false; held_code_ = sar_; ++seq_;
            held_sample_time_ = active_sample_time_; held_sampled_ = sampled_; held_output_time_ = now;
            pulse_end_ = now + cfg_.clockout_width_s;
        } else due_ += cfg_.decision_time_s;
    }
    const double lo = VSS.read(), hi = VDD.read();
    VCM.write(cfg_.common_mode_v + AVSS.read());
    D1OUT.write(held_code_&1u ? hi:lo); D2OUT.write(held_code_&2u ? hi:lo);
    D3OUT.write(held_code_&4u ? hi:lo); D4OUT.write(held_code_&8u ? hi:lo);
    D5OUT.write(held_code_&16u ? hi:lo); D6OUT.write(held_code_&32u ? hi:lo);
    CLKOUT1.write(now+eps < pulse_end_ ? hi:lo);
    code_out.write(held_code_); sequence_out.write(seq_);
    sample_time_out.write(held_sample_time_); sampled_diff_out.write(held_sampled_); output_time_out.write(held_output_time_);
}
} // namespace s28
