#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <memory>
#include <optional>
#include <string>
// Sources: TI SNVS009H (March 2016), sec6.5 pp5-6 and sec9.2 startup Schottky; supplied lm2665_boost_sw.va, SHA256 b96226d8c608510dd56cf65a5c27b0e244ecd2311904e278390c2f9daa31a5ba.
// Signal-flow application model; all voltages relative to common GND.
// iload is positive consumed current magnitude, including the negative rail.
class LM2665_SW : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"}, control{"control"}, temperature{"temperature"}, iload{"iload"}, rload{"rload"};
    sca_tdf::sca_out<double> vout{"vout"}, iin{"iin"}, power{"power"}, external_loss{"external_loss"}, numerical_loss{"numerical_loss"}, stored_energy{"stored_energy"}, energy_residual{"energy_residual"}, fsw{"fsw"}, phi1{"phi1"}, phi2{"phi2"}, cfly_v{"cfly_v"}, skip_state{"skip_state"};
    LM2665_SW(sc_core::sc_module_name, const std::string& config_file);
    ~LM2665_SW();
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    std::string get_category() const;
    std::optional<double> get_area() const;
    std::optional<double> get_latency() const;
    double get_timestep_s() const;
    double get_power_dissipation() const;
    const nlohmann::json& configuration() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
