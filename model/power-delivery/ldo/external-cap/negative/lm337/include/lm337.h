#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <complex>
#include <memory>
#include <string>

#ifndef XL_LDO_NULLABLE_NUMBER_DEFINED
#define XL_LDO_NULLABLE_NUMBER_DEFINED
namespace xl_ldo { struct NullableNumber { bool available=false; double value=0; }; }
#endif

// Standalone C++14 TDF model. COUT/ESR are solved internally.
// Key data sources: [1] 01_TI_LM337_Rev_current.pdf; [2] LM337_quick_function_v2.va. Datasheet ratings and supplied VA behavior/fit parameters are distinguished by the parameter-level source fields in json/lm337.json.
class LM337 : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"}, iload{"iload"}, rload{"rload"}, control{"control"}, ripple{"ripple"};
    sca_tdf::sca_out<double> vout{"vout"}, iin{"iin"}, ignd{"ignd"}, power{"power"};
    LM337(sc_core::sc_module_name name, const std::string& config_file);
    ~LM337();
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    xl_ldo::NullableNumber get_area() const;
    xl_ldo::NullableNumber get_latency() const;
    double get_timestep_s() const;
    double get_sample_period() const;
    double get_power_dissipation() const;
    const nlohmann::json& configuration() const;
    double unserved_load_current() const;
    int protection_state() const;
    double last_pass_current() const;
    double last_current_limit() const;
    double last_kcl_residual() const;
    double stored_capacitor_voltage() const;
    std::complex<double> psrr_transfer(double hz,double load) const;
    double integrated_noise_rms(double load,double low,double high) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
