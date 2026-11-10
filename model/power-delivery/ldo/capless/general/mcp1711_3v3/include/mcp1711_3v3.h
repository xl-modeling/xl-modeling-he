#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <experimental/optional>
#include <random>
#include <string>
#include <unordered_map>

// Microchip DS20005415D pp.3-5, 19-22; supplied MCP1711 quick VA v1.
// TDF signal flow: voltages relative to GND, current positive toward load.
// External output capacitance is solved INSIDE this module, not in a second load.
class MCP1711_3V3 : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"}, shdn{"shdn"}, iload{"iload"}, rload{"rload"}, temperature{"temperature"}, ripple{"ripple"};
    sca_tdf::sca_out<double> vout{"vout"}, iin{"iin"}, ignd{"ignd"}, ishdn{"ishdn"}, ipass{"ipass"}, idischarge{"idischarge"}, power{"power"}, noise{"noise"}, unmet_load{"unmet_load"};
    sca_tdf::sca_out<int> state{"state"};
    MCP1711_3V3(sc_core::sc_module_name name, const std::string& config_file);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    std::string get_category() const;
    double get_timestep_s() const;
    std::experimental::optional<double> get_latency() const; // unknown in source, NOT numerical timestep
    std::experimental::optional<double> get_area() const;    // unknown bare die area
    double get_power_dissipation() const { return last_power_; }
    double get_noise_density() const;
    const nlohmann::json& configuration() const { return cfg_; }
private:
    nlohmann::json cfg_;
    std::unordered_map<std::string,double> nums_;
    double p(const char* key) const;
    bool first_=true, armed_=false;
    double capacitor_v_=0, startup_=0, last_power_=0, ripple_lp_=0;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0,1};
};
