#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

// Sources: reference/lm1117_core_verified.va; lm1117_psrr.va; TI SNOS412Q.
// Signal-flow model: voltages referenced to GND; positive load current is sinking.
class LM1117_3V3 : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"}, iload{"iload"}, rload{"rload"}, ripple{"ripple"};
    sca_tdf::sca_out<double> vout{"vout"}, iin{"iin"}, ignd{"ignd"}, power{"power"};
    LM1117_3V3(sc_core::sc_module_name, const std::string& config_file);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    std::optional<double> get_area() const; // nullopt: die area not published
    std::optional<double> get_latency() const; // physical timing unavailable until independently characterized
    double get_timestep_s() const; // numerical sampling period
    double get_power_dissipation() const { return last_power_; }
    const nlohmann::json& configuration() const { return cfg_; }
private:
    nlohmann::json cfg_;
    double p(const char*) const;
    double vc_=0, reg_=0, lp_i_=0, lp_v_=0, previous_ripple_=0, last_power_=0;
    bool first_=true;
};
