// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#include "tps53647.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <limits>

double TPS53647::p(const char* key) const {
    return cfg_.at(key).at("value").get<double>();
}

double TPS53647::clip(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

TPS53647::TPS53647(sc_core::sc_module_name name, const std::string& config_file)
    : sca_tdf::sca_module(name) {
    // 与 LDO 参考交付同风格：纯 std::ifstream(std::string) 打开配置，
    // 不依赖 std::filesystem，Ubuntu 18.04 / GCC 7.5 下 C++17 可直接编译。
    std::ifstream file(config_file);
    if (!file) {
        throw std::runtime_error("Cannot open TPS53647 config: " + config_file);
    }
    file >> cfg_;

    for (const char* key : {"dt_s", "fsw_hz", "t_ss_s", "t_edge_s", "tau_ab_s", "r_csp_ohm", "imon_int_a", "tsen_slope_v_per_c"}) {
        if (!std::isfinite(p(key)) || p(key) <= 0.0) {
            throw std::runtime_error(std::string("Invalid positive parameter: ") + key);
        }
    }
    for (const char* key : {"vin_uvlo_v", "v5_uvlo_v", "vin_max_v", "en_threshold_v",
                            "reset_threshold_v", "vref_v", "v3r3_v", "vout_min_v",
                            "vout_max_v", "duty_max", "ocl_phase_a", "ocp_total_a",
                            "vout_target_v", "r_csp_ohm", "r_imon_ohm", "imon_int_a",
                            "vcomp_max_v", "v_ovp_track_v", "v_ovp_fixed_v", "v_uvp_off_v",
                            "t_hiccup_s", "v_tsen_off_v", "tsen_slope_v_per_c",
                            "v_tsen_otp_v", "v_tsen_hot_v", "k_ab", "ab_en", "k_fb",
                            "k_i", "iq_a", "iq_shutdown_a"}) {
        if (!std::isfinite(p(key)) || p(key) < 0.0) {
            throw std::runtime_error(std::string("Invalid nonnegative parameter: ") + key);
        }
    }
    if (p("vout_max_v") < p("vout_min_v") || p("duty_max") > 1.0 ||
        p("v_tsen_otp_v") < p("v_tsen_hot_v")) {
        throw std::runtime_error("Invalid TPS53647 limit relationship");
    }
}

void TPS53647::set_attributes() {
    set_timestep(p("dt_s"), sc_core::SC_SEC);
}

std::string TPS53647::get_module_name() const {
    return cfg_.at("name").at("value").get<std::string>();
}

double TPS53647::get_version() const {
    return cfg_.at("version").at("value").get<double>();
}

std::optional<double> TPS53647::get_area() const {
    const auto& value = cfg_.at("area_mm2").at("value");
    return value.is_null() ? std::nullopt : std::optional<double>(value.get<double>());
}

double TPS53647::get_latency() const {
    return std::numeric_limits<double>::quiet_NaN();
}

void TPS53647::write_zero_outputs() {
    comp.write(0.0);
    imon.write(0.0);
    isum.write(0.0);
    vref.write(0.0);
    v3r3.write(0.0);
    vr_rdy.write(0.0);
    vr_fault.write(0.0);
    vr_hot.write(0.0);
    pmb_alert.write(0.0);
    skip_nvm.write(0.0);
    pwm1.write(0.0);
    pwm2.write(0.0);
    pwm3.write(0.0);
    pwm4.write(0.0);
}

void TPS53647::processing() {
    const double now = get_time().to_seconds();
    const double dt = p("dt_s");
    const double vin_v = vin.read();
    const double v5_v = v5.read();
    const double vout_v = vsp.read() - vsn.read();
    const double enable_v = enable.read();
    const double reset_v = reset.read();
    const double tsen_v = tsen.read();

    if (!std::isfinite(vin_v) || !std::isfinite(v5_v) || !std::isfinite(vout_v) ||
        !std::isfinite(enable_v) || !std::isfinite(reset_v) || !std::isfinite(tsen_v)) {
        throw std::runtime_error("TPS53647 inputs must be finite");
    }

    const double target = clip(p("vout_target_v"), p("vout_min_v"), p("vout_max_v"));
    const bool supply_ok = vin_v >= p("vin_uvlo_v") && vin_v <= p("vin_max_v") &&
                           v5_v >= p("v5_uvlo_v");
    const bool run_ok = supply_ok && enable_v >= p("en_threshold_v") &&
                        reset_v >= p("reset_threshold_v");
    const double v3r3_value = supply_ok ? p("v3r3_v") : 0.0;
    const double vref_value = supply_ok ? p("vref_v") : 0.0;

    const std::array<double, 4> current{{
        clip(csp1.read() / p("r_csp_ohm"), 0.0, 1.0e6),
        clip(csp2.read() / p("r_csp_ohm"), 0.0, 1.0e6),
        clip(csp3.read() / p("r_csp_ohm"), 0.0, 1.0e6),
        clip(csp4.read() / p("r_csp_ohm"), 0.0, 1.0e6)}};
    const double total_current = current[0] + current[1] + current[2] + current[3];
    const bool total_oc = total_current >= p("ocp_total_a");
    const double tsen_temp_c = (tsen_v - p("v_tsen_off_v")) / p("tsen_slope_v_per_c");
    const bool ot_fault = tsen_v >= p("v_tsen_otp_v");
    const bool ot_hot = tsen_v >= p("v_tsen_hot_v");

    if (first_) {
        state_ = State::Off;
        previous_time_s_ = now;
        first_ = false;
    }
    const double elapsed = std::max(0.0, now - start_time_s_);
    const double ss_ratio = state_ == State::Run ? clip(elapsed / p("t_ss_s"), 0.0, 1.0) : 0.0;
    const bool ovp_tracked = vout_v > target + p("v_ovp_track_v");
    const bool ovp_fixed = vout_v > p("v_ovp_fixed_v");
    const bool uvp_now = vout_v < target - p("v_uvp_off_v") && ss_ratio >= 1.0;

    if (!run_ok) {
        state_ = State::Off;
        latch_from_ot_ = false;
        integrator_ = 0.0;
    } else {
        switch (state_) {
        case State::Off:
            state_ = State::Run;
            start_time_s_ = now;
            integrator_ = 0.0;
            break;
        case State::Run:
            if (ovp_tracked || ovp_fixed || ot_fault) {
                state_ = State::Latch;
                latch_from_ot_ = ot_fault;
            } else if (uvp_now || total_oc) {
                state_ = State::Hiccup;
                hiccup_start_s_ = now;
            }
            break;
        case State::Latch:
            if (latch_from_ot_ && !ot_fault &&
                tsen_v < p("v_tsen_otp_v") - 15.0 * p("tsen_slope_v_per_c")) {
                state_ = State::Run;
                start_time_s_ = now;
                latch_from_ot_ = false;
            }
            break;
        case State::Hiccup:
            if (ovp_tracked || ovp_fixed || ot_fault) {
                state_ = State::Latch;
                latch_from_ot_ = ot_fault;
            } else if (now - hiccup_start_s_ >= p("t_hiccup_s")) {
                state_ = State::Run;
                start_time_s_ = now;
                integrator_ = 0.0;
            }
            break;
        }
    }

    const bool pwm_ok = state_ == State::Run;
    const bool fault_latched = state_ == State::Latch;
    const bool hiccup_active = state_ == State::Hiccup;

    if (now > previous_time_s_) {
        const double alpha = dt / (p("tau_ab_s") + dt);
        for (std::size_t i = 0; i < current.size(); ++i) {
            filtered_current_[i] += alpha * (current[i] - filtered_current_[i]);
        }
        filtered_average_current_ = (filtered_current_[0] + filtered_current_[1] +
                                     filtered_current_[2] + filtered_current_[3]) * 0.25;
        if (pwm_ok) {
            integrator_ = clip(integrator_ + p("k_i") * dt * (target * ss_ratio - vout_v),
                               -0.2, 0.2);
        } else {
            integrator_ = 0.0;
        }
        previous_time_s_ = now;
    }

    double duty = 0.0;
    if (pwm_ok && vin_v > 0.0) {
        const double duty_ff = target / vin_v;
        const double duty_fb = duty_ff + p("k_fb") * (target * ss_ratio - vout_v) + integrator_;
        duty = clip(ss_ratio * duty_fb, 0.0, p("duty_max"));
    }

    std::array<double, 4> phase_duty{{duty, duty, duty, duty}};
    if (p("ab_en") > 0.5) {
        for (std::size_t i = 0; i < phase_duty.size(); ++i) {
            phase_duty[i] = clip(duty + p("k_ab") * (filtered_average_current_ - filtered_current_[i]),
                                 0.0, p("duty_max"));
        }
    }

    const double period = 1.0 / p("fsw_hz");
    double phase = std::fmod(std::max(0.0, now), period);
    if (phase < 0.0) phase += period;
    std::array<double, 4> phase_time{{phase, phase - 0.25 * period,
                                       phase - 0.50 * period, phase - 0.75 * period}};
    for (double& value : phase_time) {
        if (value < 0.0) value += period;
    }

    std::array<double, 4> pwm{{0.0, 0.0, 0.0, 0.0}};
    for (std::size_t i = 0; i < pwm.size(); ++i) {
        const bool phase_oc = current[i] >= p("ocl_phase_a");
        pwm[i] = pwm_ok && !phase_oc && phase_time[i] < phase_duty[i] * period ? v3r3_value : 0.0;
    }

    const bool ready = pwm_ok && ss_ratio >= 1.0 && !fault_latched && !hiccup_active &&
                       vout_v > target - p("v_uvp_off_v") && vout_v < target + p("v_ovp_track_v");
    const double imon_value = total_current * p("r_csp_ohm") * p("r_imon_ohm") / p("imon_int_a");
    const double comp_value = clip(vref_value + 2.0 * (target * ss_ratio - vout_v), 0.0,
                                  p("vcomp_max_v"));
    last_power_w_ = supply_ok ? v5_v * (run_ok?p("iq_a"):p("iq_shutdown_a")) : 0.0;

    comp.write(comp_value);
    imon.write(imon_value);
    isum.write(0.0);       // ISUM is not driven by the supplied Verilog-A model.
    vref.write(vref_value);
    v3r3.write(v3r3_value);
    vr_rdy.write(ready ? v3r3_value : 0.0);
    vr_fault.write((fault_latched || ot_fault) ? 0.0 : v3r3_value);
    vr_hot.write(ot_hot ? 0.0 : v3r3_value);
    pmb_alert.write(0.0);   // PMBus protocol is outside this analog behavior model.
    skip_nvm.write(0.0);   // NVM configuration is represented by JSON parameters.
    pwm1.write(pwm[0]);
    pwm2.write(pwm[1]);
    pwm3.write(pwm[2]);
    pwm4.write(pwm[3]);

    (void)ocl_r; (void)pmb_clk; (void)pmb_dio; (void)addr_trise; (void)slew_mode;
    (void)o_usr; (void)vboot; (void)f_imax; (void)nc7; (void)nc8; (void)nc33;
    (void)nc34; (void)gnd17; (void)gnd20; (void)gnd22; (void)tsen_temp_c;
}
