#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <array>
#include <optional>
#include <random>
#include <string>

// E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\LDO\External-Cap_LDO\供电芯粒_LDO_External-Cap_LDO_Source-Sink_LDO\TPS51200_core_v1.va
// SHA256=32f61693998b072384f099ac9a73e6970ba67213b3d7b63c8740d311417f8b3f
// E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\LDO\External-Cap_LDO\供电芯粒_LDO_External-Cap_LDO_Source-Sink_LDO\01_TI_TPS51200_RevE.pdf
// SHA256=c3ad54837f5fb8620c06af33105e1aaa3403b127ab0125c299cb690bb914154e
// TI SLUS812E (2025-09), pp.5-6 electrical tables, pp.11-12 modes, p.14 GM/COUT.
// VA source: parameter declarations and all analog branches; historical report is not silicon evidence.
// Signal-flow TDF implementation. GND and PGND share the 0 V reference.
// Positive iload/iref leaves the corresponding output; sink current goes to ground.
// External C/ESR/trace/load network is solved internally, not a bidirectional ELN pin.
namespace tps51200 {
using json=nlohmann::json;
enum In {VIN,VLDOIN,REFIN,EN,ILOAD,IREF,RLOAD,TEMP_C,FORCE_VO,FORCE_REF,PG_PULLUP,NIN};
enum Out {VO,VOSNS,REFOUT,PGOOD,IIN,ILDOIN,ISOURCE,ISINK,IREF_SOURCE,IREF_SINK,
          POWER,POWER_SOURCE,POWER_SINK,POWER_REF,POWER_BIAS,POWER_DISCHARGE,
          POWER_PG,PIN,PLOAD,PSTORAGE,PEXTERNAL,RESIDUAL,NUM_DAMPING,
          UV_OK,REF_OK,EN_OK,HOT,PG_OK,NOISE,VO_OBSERVED,NOUT};
extern const std::array<const char*,NIN> input_names;
extern const std::array<const char*,NOUT> output_names;
class Config {
public:
    explicit Config(const std::string& path);
    const json& data() const {return data_;}
    double number(const char* key) const {return data_.at(key).at("value").get<double>();}
    bool flag(const char* key) const {return data_.at(key).at("value").get<bool>();}
private:
    json data_;
};
class Core {
public:
    explicit Core(Config config);
    std::array<double,NOUT> step(double t, const std::array<double,NIN>& input);
    const Config& config() const {return cfg_;}
    double power() const {return last_[POWER];}
    const std::array<double,NOUT>& snapshot() const {return last_;}
private:
    Config cfg_;
    double vc_=0,vr_=0,gon_=0,gr_=0,gpg_=1,last_t_=-1,next_pg_=0;
    double good_since_=-1,bad_since_=-1;
    bool initialized_=false,uv_=false,ref_=false,en_=false,hot_=false,pg_=false,window_=false;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0,1};
    std::array<double,NOUT> last_{};
};
class TPS51200 : public sca_tdf::sca_module {
public:
    sc_core::sc_vector<sca_tdf::sca_in<double>> inputs;
    sc_core::sc_vector<sca_tdf::sca_out<double>> outputs;
    TPS51200(sc_core::sc_module_name name,const std::string& config_file);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    std::string get_version() const;
    std::string get_category() const;
    double get_timestep_s() const;
    std::optional<double> get_area() const;
    std::optional<double> get_latency() const; // independent physical timing, not sample step
    double get_pg_delay_s() const;
    double get_power_dissipation() const;
    json get_power_breakdown() const;
    json get_noise_info() const;
    const json& configuration() const {return core_.config().data();}
private:
    Core core_;
};
}
