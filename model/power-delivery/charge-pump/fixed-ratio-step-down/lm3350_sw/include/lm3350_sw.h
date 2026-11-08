#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <memory>
#include <optional>
#include <string>
// Sources: National LM3350, 2/3 Step-Down Electrical Characteristics printed p4/PDF p5; supplied lm3350_sw.va, SHA256 67b5058aa64545b4ded7678f26affae27d8a4bef4ddbc186e620545f6a7d3e33.
// Signal-flow application model; all voltages relative to common GND.
// iload is positive consumed current magnitude, including the negative rail.
class LM3350_SW : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"}, control{"control"}, temperature{"temperature"}, iload{"iload"}, rload{"rload"};
    sca_tdf::sca_out<double> vout{"vout"}, iin{"iin"}, power{"power"}, external_loss{"external_loss"}, numerical_loss{"numerical_loss"}, stored_energy{"stored_energy"}, energy_residual{"energy_residual"}, fsw{"fsw"}, phi1{"phi1"}, phi2{"phi2"}, cfly_v{"cfly_v"}, skip_state{"skip_state"}, cfly2_v{"cfly2_v"};
    LM3350_SW(sc_core::sc_module_name, const std::string& config_file);
    ~LM3350_SW();
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
