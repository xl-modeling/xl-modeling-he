#pragma once
#include <systemc-ams>
#include <array>
#include <cstdint>
#include <limits>
#include <random>
#include <string>

namespace s28 {
// Source: s28 datasheet V1, pp.2,5-7,9-15. Timing numbers are model assumptions.
struct Config {
    std::string name = "s28", version = "0.1", category = "general-purpose-analog-adc-sar";
    unsigned bits = 6;
    double sample_rate_hz = 100e6, timestep_s = 0.25e-9;
    double reference_v = 1.0, common_mode_v = 0.5, supply_v = 1.0;
    double pad_ring_v = 2.5;
    double decision_time_s = 0.5e-9, clockout_width_s = 0.5e-9;
    double offset_lsb = 0, span_error_percent = 0;
    double noise_rms_lsb = 0, quadratic_v_inv = 0, cubic_v_inv2 = 0;
    double vcmp_center_v = 0.5, vcmp_gain_v_per_v = 0, ctl_center_v = 0.5;
    double ctl_aperture_s_per_v = 0;
    std::uint64_t seed = 28;
    std::array<double, 63> transition_error_lsb{};
    double power_w = std::numeric_limits<double>::quiet_NaN();
    double area_mm2 = std::numeric_limits<double>::quiet_NaN();
    static Config load(const std::string& path);
    void validate() const;
    double lsb() const { return 2 * reference_v / 64; }
};

class S28 : public sca_tdf::sca_module {
public:
    // Repeated physical PADs are consolidated into named logical nets.
    sca_tdf::sca_in<double> IP{"IP"}, IN{"IN"}, CLK_IP{"CLK_IP"}, CLK_IN{"CLK_IN"};
    sca_tdf::sca_in<double> VRP{"VRP"}, CTL{"CTL"}, VCMP{"VCMP"};
    sca_tdf::sca_in<double> AVDD{"AVDD"}, DVDD{"DVDD"}, VDD{"VDD"}, AVSS{"AVSS"};
    sca_tdf::sca_in<double> VSS{"VSS"}, DVSS{"DVSS"}, CLKVDD{"CLKVDD"}, CLKVSS{"CLKVSS"};
    sca_tdf::sca_in<double> VSS2{"VSS2"}, SAVDD{"SAVDD"}, SAVSS{"SAVSS"}, VSSD{"VSSD"}, VDDD{"VDDD"};
    sca_tdf::sca_in<double> DVDDIO{"DVDDIO"}, DVSSIO{"DVSSIO"}, INVSS{"INVSS"}, INVDD{"INVDD"};
    sca_tdf::sca_in<double> VSS1{"VSS1"}, SAVDD1{"SAVDD1"}, SAVSS1{"SAVSS1"};
    sca_tdf::sca_out<double> VCM{"VCM"};
    sca_tdf::sca_out<double> D1OUT{"D1OUT"}, D2OUT{"D2OUT"}, D3OUT{"D3OUT"};
    sca_tdf::sca_out<double> D4OUT{"D4OUT"}, D5OUT{"D5OUT"}, D6OUT{"D6OUT"}, CLKOUT1{"CLKOUT1"};
    sca_tdf::sca_out<unsigned> code_out{"code_out"}, sequence_out{"sequence_out"};
    sca_tdf::sca_out<double> sample_time_out{"sample_time_out"}, output_time_out{"output_time_out"};
    sca_tdf::sca_out<double> sampled_diff_out{"sampled_diff_out"};
    explicit S28(sc_core::sc_module_name name, const Config& config);
    const Config& config() const { return cfg_; }
    std::string get_name() const { return cfg_.name; }
    std::string get_version() const { return cfg_.version; }
    double get_latency() const { return 6 * cfg_.decision_time_s; }
    double get_power() const { return cfg_.power_w; }
    double get_area() const { return cfg_.area_mm2; }
    unsigned rejected_clock_edges() const { return overruns_; }
    unsigned accepted_samples() const { return accepted_; }
    void set_attributes() override;
    void initialize() override;
    void processing() override;
private:
    Config cfg_;
    std::mt19937_64 rng_;
    bool clock_high_ = false, busy_ = false;
    int bit_ = 5;
    unsigned sar_ = 0, held_code_ = 0, seq_ = 0, overruns_ = 0, accepted_ = 0;
    double sampled_ = 0, corrected_ = 0, reference_ = 1, offset_v_ = 0;
    double active_sample_time_ = 0, held_sample_time_ = 0, held_sampled_ = 0;
    double due_ = 0, pulse_end_ = -1, held_output_time_ = 0, next_sample_due_ = -1;
    double uniform_open();
    double gaussian();
    void begin_sample(double now);
    double threshold(unsigned code) const;
};
} // namespace s28
