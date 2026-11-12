#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <experimental/optional>
#include <random>
#include <string>
#include <unordered_map>

// TI TPS731 SBVS034P Rev.P, pp.4-5, 16-21; supplied TPS73133 VA blocks.
// Signal-flow model. All voltages are referenced to GND. COUT/ESR are internal
// numerical load components; do not solve the same capacitor a second time.
class TPS73133_3V3 : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"},en{"en"},iload{"iload"},rload{"rload"},temperature{"temperature"},ripple{"ripple"},vexternal{"vexternal"};
    sca_tdf::sca_out<double> vout{"vout"},vcore{"vcore"},iin{"iin"},ignd{"ignd"},ien{"ien"},ipass{"ipass"},ireverse{"ireverse"},iexternal{"iexternal"},power{"power"},noise{"noise"},psrr_voltage{"psrr_voltage"},unmet_load{"unmet_load"};
    sca_tdf::sca_out<int> state{"state"};
    TPS73133_3V3(sc_core::sc_module_name name,const std::string& config_file);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    std::string get_category() const;
    double get_timestep_s() const;
    double get_latency() const; // 600us datasheet characteristic at 3V, not a 3.3V guarantee
    std::experimental::optional<double> get_area() const;
    double get_noise_rms() const;
    double get_noise_density() const;
    double get_power_dissipation()const{return last_power_;}
    const nlohmann::json& configuration()const{return cfg_;}
private:
    nlohmann::json cfg_;
    std::unordered_map<std::string,double> nums_;
    double p(const char*)const;
    bool first_=true,thermal_ok_=true,va_=false,noise_on_=false,psrr_on_=true,separate_=false;
    int cnr_mode_=1;
    double vc_=0,ss_=0,last_power_=0,vin_dc_=0,previous_vin_=0,psrr_lp_=0,previous_psrr_=0;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0,1};
};
