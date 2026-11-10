#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <memory>
#include <optional>
#include <string>
// Sources: Maxim MAX865 datasheet, Electrical Characteristics p2 and Typical Operating Circuit; supplied max865_sw.va, SHA256 fda824aa3444d746203a7b5b7201652494d9aa94ec9b3c66b55e77b28e8a0c94.
// Signal-flow application model; all voltages relative to common GND.
// iload is positive consumed current magnitude, including the negative rail.
class MAX865_SW : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"}, control{"control"}, temperature{"temperature"}, iload{"iload"}, rload{"rload"}, iload_neg{"iload_neg"}, rload_neg{"rload_neg"};
    sca_tdf::sca_out<double> vout{"vout"}, iin{"iin"}, power{"power"}, external_loss{"external_loss"}, numerical_loss{"numerical_loss"}, stored_energy{"stored_energy"}, energy_residual{"energy_residual"}, fsw{"fsw"}, phi1{"phi1"}, phi2{"phi2"}, cfly_v{"cfly_v"}, skip_state{"skip_state"}, cfly2_v{"cfly2_v"}, vneg{"vneg"};
    MAX865_SW(sc_core::sc_module_name, const std::string& config_file);
    ~MAX865_SW();
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
