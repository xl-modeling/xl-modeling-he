#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <memory>
#include <optional>
#include <string>
// Sources: TI SLVS888D (April 2026), sec5.5 p5 and sec7.2; supplied tps60150_sw.va, SHA256 2c4d07f1fb4a7e006e90029728f16294825da4b9af65cc4aaa16539081e0dcbd.
// Signal-flow application model; all voltages relative to common GND.
// iload is positive consumed current magnitude, including the negative rail.
class TPS60150_SW : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"}, control{"control"}, temperature{"temperature"}, iload{"iload"}, rload{"rload"};
    sca_tdf::sca_out<double> vout{"vout"}, iin{"iin"}, power{"power"}, external_loss{"external_loss"}, numerical_loss{"numerical_loss"}, stored_energy{"stored_energy"}, energy_residual{"energy_residual"}, fsw{"fsw"}, phi1{"phi1"}, phi2{"phi2"}, cfly_v{"cfly_v"}, skip_state{"skip_state"};
    TPS60150_SW(sc_core::sc_module_name, const std::string& config_file);
    ~TPS60150_SW();
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
