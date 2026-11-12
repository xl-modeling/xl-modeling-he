// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#pragma once

#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <array>

// SystemC-AMS TDF behavioral model derived from the supplied TPS53647
// Verilog-A controller. Voltages are represented as double-valued TDF signals.
class TPS53647 : public sca_tdf::sca_module {
public:
    // Inputs from the external power stage and configuration pins.
    sca_tdf::sca_in<double> ocl_r{"ocl_r"};
    sca_tdf::sca_in<double> csp1{"csp1"}, csp2{"csp2"}, csp3{"csp3"}, csp4{"csp4"};
    sca_tdf::sca_in<double> vsp{"vsp"}, vsn{"vsn"};
    sca_tdf::sca_in<double> v5{"v5"}, vin{"vin"}, reset{"reset"}, enable{"enable"};
    sca_tdf::sca_in<double> pmb_clk{"pmb_clk"}, pmb_dio{"pmb_dio"};
    sca_tdf::sca_in<double> addr_trise{"addr_trise"}, slew_mode{"slew_mode"}, o_usr{"o_usr"};
    sca_tdf::sca_in<double> vboot{"vboot"}, f_imax{"f_imax"}, tsen{"tsen"};
    sca_tdf::sca_in<double> nc7{"nc7"}, nc8{"nc8"}, nc33{"nc33"}, nc34{"nc34"};
    sca_tdf::sca_in<double> gnd17{"gnd17"}, gnd20{"gnd20"}, gnd22{"gnd22"};

    // Outputs corresponding to the Verilog-A electrical outputs.
    sca_tdf::sca_out<double> comp{"comp"}, imon{"imon"}, isum{"isum"};
    sca_tdf::sca_out<double> vref{"vref"}, v3r3{"v3r3"};
    sca_tdf::sca_out<double> vr_rdy{"vr_rdy"}, vr_fault{"vr_fault"}, vr_hot{"vr_hot"};
    sca_tdf::sca_out<double> pmb_alert{"pmb_alert"}, skip_nvm{"skip_nvm"};
    sca_tdf::sca_out<double> pwm1{"pwm1"}, pwm2{"pwm2"}, pwm3{"pwm3"}, pwm4{"pwm4"};

    TPS53647(sc_core::sc_module_name, const std::string& config_file);

    void set_attributes() override;
    void processing() override;

    std::string get_module_name() const;
    double get_version() const;
    std::optional<double> get_area() const;
    double get_latency() const; // Unknown physical latency, NaN.
    double get_timestep_s() const {return p("dt_s");}
    // Current interface is a level-only approximation, not an ELN open drain.
    bool status_is_level_approximation() const {return true;}
    double get_power_dissipation() const { return last_power_w_; }
    const nlohmann::json& configuration() const { return cfg_; }

private:
    enum class State { Off, Run, Latch, Hiccup };

    nlohmann::json cfg_;
    double p(const char*) const;
    static double clip(double x, double lo, double hi);
    void write_zero_outputs();

    State state_ = State::Off;
    bool latch_from_ot_ = false;
    bool first_ = true;
    double start_time_s_ = 0.0;
    double hiccup_start_s_ = 0.0;
    double previous_time_s_ = 0.0;
    double integrator_ = 0.0;
    std::array<double, 4> filtered_current_{{0.0, 0.0, 0.0, 0.0}};
    double filtered_average_current_ = 0.0;
    double last_power_w_ = 0.0;
};
