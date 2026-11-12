#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>
#include <random>
#include <unordered_map>
// Sources: official TI datasheet and archived VA model listed in JSON source_documents.
// TDF signal-flow abstraction, not electrical pin connectivity. SI units unless stated.
class TLV71333_3V3 : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"}, en{"en"}, iload{"iload"}, rload{"rload"}, temperature{"temperature"}, ripple{"ripple"};
    sca_tdf::sca_out<double> vout{"vout"}, iin{"iin"}, ignd{"ignd"}, ien{"ien"}, power{"power"}, noise{"noise"};
    sca_tdf::sca_out<int> state{"state"};
    TLV71333_3V3(sc_core::sc_module_name name, const std::string& config_file);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    double get_latency() const; // datasheet characteristic startup time, not numerical dt
    double get_timestep_s() const;
    std::optional<double> get_area() const;
    double get_power_dissipation() const { return last_power_; }
    double get_noise_density() const;
    const nlohmann::json& configuration() const { return cfg_; }
private:
    nlohmann::json cfg_;
    std::unordered_map<std::string,double> numeric_;
    double p(const char* key) const;
    double ss_=0, vc_=0, err_=0, vin_lp_=0, ripple_lp_=0, ripple_dc_=0, previous_ripple_=0, previous_ac_=0, last_power_=0;
    bool first_=true, en_on_=false, uv_on_=false, thermal_ok_=true, armed_=false;
    std::vector<double> phases_;
    std::mt19937 noise_rng_;
    std::normal_distribution<double> normal_{0.0,1.0};
};
