#pragma once
// Source provenance (delivery revision 1.0.1, 2026-10-29):
// UCC27624.va | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_栅极驱动_低边驱动_UCC27624\UCC27624.va
// SHA256 1bd18e820a0f83c566804e239109048a2110e3a3e373b45af07b77115816020e
// UCC27624_datasheet.pdf | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_栅极驱动_低边驱动_UCC27624\UCC27624_datasheet.pdf
// SHA256 09173fc9e2c7314f5bd6ecf95db5798b2244828d5a25cf5e5b363df3de14aeb3

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <nlohmann/json.hpp>
#include <random>
#include <stdexcept>
#include <string>
#include <systemc-ams>
#include <vector>

namespace ucc27624 {
using json = nlohmann::json;
// Accounting convention follows the supplied LDO power port/getter pattern.
// Equations are derived from the actual chip branches, not LDO electrical values.
namespace model_power {
struct Breakdown {
    double quiescent=0,drive=0,input=0,clamp=0,bootstrap=0,discharge=0,body=0,ct=0;
    double terminal_in=0,terminal_out=0,ct_source=0,ct_storage=0;
    double raw=0,total=0;
    void finish() {
        raw=quiescent+drive+input+clamp+bootstrap+discharge+body+ct;
        total=std::max(0.,raw); // Same nonnegative reporting convention as LDO.
        if(!std::isfinite(raw+terminal_in+terminal_out+ct_source+ct_storage))
            throw std::runtime_error("Nonfinite power accounting");
    }
    static std::vector<std::string> names() { return {
        "power_W","power_quiescent_W","power_drive_W","power_input_W",
        "power_clamp_W","power_bootstrap_W","power_discharge_W","power_body_W",
        "power_ct_W","power_terminal_in_W","power_terminal_out_W",
        "power_ct_source_W","power_ct_storage_W"}; }
    std::vector<double> values() const {return {total,quiescent,drive,input,clamp,
        bootstrap,discharge,body,ct,terminal_in,terminal_out,ct_source,ct_storage};}
    std::vector<double> append(std::vector<double> x) const {
        auto v=values(); x.insert(x.end(),v.begin(),v.end());return x;
    }
    nlohmann::json json() const {
        nlohmann::json j;auto n=names();auto v=values();
        for(size_t k=0;k<n.size();++k)j[n[k]]=v[k];
        j["raw_W"]=raw;
        j["balance_residual_W"]=terminal_in+ct_source-terminal_out-ct_storage-raw;
        return j;
    }
};
inline double gate_drive(double supply,double out,double source,double sink) {
    return (supply-out)*source+out*sink;
}
inline double gate_clamp(double supply,double out,double high,double low) {
    return (out-supply)*high-out*low;
}
}
// Public information/noise interfaces follow the archived LDO example.
// C++11 replacement for its C++17 optional<double>; null never means zero area.
namespace model_info {
using json=nlohmann::json;
class OptionalArea {
    bool known_;double value_;
public:
    explicit OptionalArea(const json& x):known_(!x.is_null()),value_(known_?x.get<double>():0.) {
        if(known_&&(!std::isfinite(value_)||value_<=0))throw std::runtime_error("Known die area must be positive mm2");
    }
    bool has_value()const{return known_;}
    explicit operator bool()const{return known_;}
    double value()const{if(!known_)throw std::logic_error("Die area is unknown");return value_;}
    double value_or(double fallback)const{return known_?value_:fallback;}
    json as_json()const{return known_?json(value_):json(nullptr);}
};
class Queries {
    json cfg_;bool enabled_;std::string mode_;double rms_,low_,high_;unsigned tones_,seed_;
    std::vector<std::vector<double>> phases_;
    std::vector<std::mt19937> rng_;
    std::vector<std::normal_distribution<double>> normal_;
    double number(const char* key)const{return cfg_.at(key).at("value").get<double>();}
public:
    explicit Queries(const json& cfg):cfg_(cfg),enabled_(cfg.at("noise_enable").at("value").get<bool>()),
      mode_(cfg.at("noise_model").at("value").get<std::string>()),rms_(number("noise_rms_v")),
      low_(number("noise_low_hz")),high_(number("noise_high_hz")),tones_(0),seed_(0) {
        OptionalArea area(cfg.at("area_mm2").at("value"));(void)area;
        const auto n=cfg.at("noise_tones").at("value"),seed=cfg.at("noise_seed").at("value");
        if(!n.is_number_integer()||n.get<int>()<8||n.get<int>()>1024)throw std::runtime_error("noise_tones must be integer 8..1024");
        if(!seed.is_number_integer()||seed.get<double>()<0||seed.get<double>()>4294967295.)throw std::runtime_error("noise_seed outside uint32");
        tones_=n.get<unsigned>();seed_=seed.get<unsigned>();
        if(!std::isfinite(rms_)||rms_<0||!std::isfinite(low_)||low_<0||!std::isfinite(high_)||high_<=low_)throw std::runtime_error("Invalid noise RMS or band");
        if(mode_!="multisine"&&mode_!="gaussian_white")throw std::runtime_error("Unknown noise model");
    }
    double get_latency()const{return number(cfg_.count("t_ramp_start")?"t_ramp_start":"t_launch_on");}
    json get_latency_info()const {
        const bool sw=cfg_.count("t_ramp_start");
        json j={{"unit","s"},{"kind","configured_event_delay"},{"rising_s",get_latency()},
          {"falling_s",number(sw?"t_off_delay":"t_launch_off")},
          {"definition",sw?"ON threshold to CT ramp-start event; not VOUT completion":"logic/UVLO request to output-strength event; not loaded voltage crossing"}};
        if(cfg_.count("t_skew_b"))j["channel_b_extra_s"]=number("t_skew_b");
        if(cfg_.count("t_comm_extra"))j["commutation_extra_s"]=number("t_comm_extra");
        if(cfg_.count("t_enable")){j["enable_s"]=number("t_enable");j["disable_s"]=number("t_disable");}
        return j;
    }
    OptionalArea get_area()const{return OptionalArea(cfg_.at("area_mm2").at("value"));}
    double get_noise_density()const{return rms_/std::sqrt(high_-low_);}
    json get_noise_info()const{return {{"enabled",enabled_},{"model",mode_},{"density_V_per_sqrt_Hz",get_noise_density()},
      {"configured_band_rms_V",rms_},{"low_Hz",low_},{"high_Hz",high_},{"calibrated_to_chip",false},
      {"scope","optional ideal output-observation perturbation; excluded from electrical solver and power; zero default is disabled, not measured zero noise"},
      {"band_definition",mode_=="multisine"?"random-phase discrete tones in configured band; density is equivalent flat ASD, not continuous PSD":"iid samples with flat one-sided PSD to Nyquist; configured RMS refers only to specified band"}};}
    void validate_timestep(double dt)const {
        if(!std::isfinite(dt)||dt<=0)throw std::runtime_error("Timestep must be finite and positive");
        if(enabled_&&dt*high_>.05*(1+1e-12))throw std::runtime_error("Noise sampling requires dt <= 1/(20*fhigh)");
    }
    double sample(double t,double dt,unsigned stream=0,bool active=true) {
        validate_timestep(dt);
        if(!std::isfinite(t)||t<0)throw std::runtime_error("Invalid noise time");
        if(!enabled_||rms_==0||!active)return 0;
        while(phases_.size()<=stream){
            std::mt19937 rng(seed_+static_cast<std::uint32_t>(phases_.size())*0x9e3779b9u);
            std::vector<double> phases;
            for(unsigned i=0;i<tones_;++i)phases.push_back(6.2831853071795864769*double(rng())/4294967296.);
            phases_.push_back(phases);rng_.emplace_back(seed_+static_cast<std::uint32_t>(rng_.size())*0x9e3779b9u);normal_.emplace_back(0.,1.);
        }
        if(mode_=="gaussian_white")return normal_[stream](rng_[stream])*get_noise_density()/std::sqrt(2*dt);
        double v=0,df=(high_-low_)/tones_;
        for(unsigned k=0;k<tones_;++k)v+=std::sin(6.2831853071795864769*(low_+(k+.5)*df)*t+phases_[stream][k]);
        return v*rms_*std::sqrt(2./tones_);
    }
};
}

// Validate metadata and declared types before constructing the electrical core.
inline void validate_contract(const json& j) {
 const std::vector<std::string> allowed={"area_mm2","area_query","boost_off_fraction","category","dimensions","function_info","i_eps","i_sink_pk","i_source_pk","i_zero_eps","iq_disabled","iq_out_high","iq_out_low","iq_uvlo","latency_query","limitations","name","noise_enable","noise_high_hz","noise_low_hz","noise_model","noise_query","noise_rms_v","noise_seed","noise_tones","package_info","ports","power_components","power_model","power_query","power_revision","power_scope","r_clamp","r_nboost","ren","rin","roh_dc","rol_dc","t_launch_off","t_launch_on","t_skew_b","t_snk_off_strength","t_snk_on_strength","t_src_off_strength","t_src_on_strength","timestep_query","tj_max_c","tj_min_c","v_clamp","v_eps","vdd_oper_max","vdd_oper_min","ven_h","ven_l","version","vin_h","vin_l","vuvlo_off","vuvlo_on"},required={"area_mm2","area_query","boost_off_fraction","category","dimensions","function_info","i_eps","i_sink_pk","i_source_pk","i_zero_eps","iq_disabled","iq_out_high","iq_out_low","iq_uvlo","latency_query","limitations","name","noise_enable","noise_high_hz","noise_low_hz","noise_model","noise_query","noise_rms_v","noise_seed","noise_tones","package_info","ports","power_components","power_model","power_query","power_revision","power_scope","r_clamp","r_nboost","ren","rin","roh_dc","rol_dc","t_launch_off","t_launch_on","t_skew_b","t_snk_off_strength","t_snk_on_strength","t_src_off_strength","t_src_on_strength","timestep_query","tj_max_c","tj_min_c","v_clamp","v_eps","vdd_oper_max","vdd_oper_min","ven_h","ven_l","version","vin_h","vin_l","vuvlo_off","vuvlo_on"};
 if(!j.is_object())throw std::invalid_argument("Configuration must be an object");
 for(const auto& key:required)if(!j.count(key))throw std::invalid_argument("Missing parameter: "+key);
 for(auto it=j.begin();it!=j.end();++it){
  if(std::find(allowed.begin(),allowed.end(),it.key())==allowed.end())throw std::invalid_argument("Unknown parameter: "+it.key());
  const auto& z=it.value();for(const char* key:{"type","value","unit","editable"})if(!z.count(key))throw std::invalid_argument("Missing typed schema: "+it.key());
  if(!z["editable"].is_boolean()||!z["unit"].is_string())throw std::invalid_argument("Invalid metadata: "+it.key());
  const std::string type=z["type"].get<std::string>();const auto& v=z["value"];
  const bool valid=(type=="string"||type=="enum")?v.is_string():type=="bool"?v.is_boolean():type=="int"?v.is_number_integer():(type=="float"||type=="double")?v.is_number():type=="object"?v.is_object():type=="array"?v.is_array():(type=="nullable_float"||type=="nullable_double"||type=="number_or_null")?(v.is_null()||v.is_number()):false;
  if(!valid)throw std::invalid_argument("Type mismatch: "+it.key());
  if(v.is_number()&&!std::isfinite(v.get<double>()))throw std::invalid_argument("Nonfinite parameter: "+it.key());
 }
 if(j.at("name").at("value")!="ucc27624")throw std::invalid_argument("Wrong model identity");
 const auto& dim=j.at("dimensions").at("value");if(!dim.at("width").is_number_integer()||!dim.at("height").is_number_integer()||dim.at("width").get<int>()<1||dim.at("height").get<int>()<1)throw std::invalid_argument("Invalid icon dimensions");
 const auto& ps=j.at("ports").at("value");if(ps.size()!=25)throw std::invalid_argument("Wrong port count");
 const std::vector<std::string> pins={"vdd","ina","inb","ena","enb","outa","outb","isa","isb","ika","ikb","ivdd","iq","iina","iinb","iena","ienb","uvlo_ok","drivea","driveb","power","noise_a_V","noise_b_V","outa_observed_V","outb_observed_V"};
 for(size_t i=0;i<ps.size();++i){const auto& z=ps[i];if(z.at("name")!=pins[i]||z.at("io_type")!=(i<5?"input":"output")||z.at("x").get<int>()<0||z.at("x").get<int>()>dim.at("width").get<int>()||z.at("y").get<int>()<0||z.at("y").get<int>()>dim.at("height").get<int>())throw std::invalid_argument("Invalid port metadata");}
}

class Config {
    json data_;
public:
    explicit Config(const std::string& path);
    explicit Config(const json& data);
    void validate() const;
    double get(const std::string& key) const;
    std::string name() const;
    std::string version() const;
    const json& parameters() const { return data_; }
};
// External fixture, deliberately separate from silicon parameter JSON.
struct OutputLoad {
    double capacitance = 1.8e-9;
    double resistance = 1e12;
    double current = 0; // positive: current drawn from output to ground
};
struct Ramp {
    double initial = 0, target = 0, start = 0, duration = 0;
    double value(double time) const;
    void set(double time, double next, double span);
};
struct Edge { double time; bool high; };
struct Channel {
    bool input = false, enable = false, requested = false, boost = false;
    double voltage = 0;
    Ramp source, sink;
    std::deque<Edge> events;
    Channel() { sink.initial = sink.target = 1; }
};
struct Observation {
    double outa=0,outb=0,isa=0,isb=0,ika=0,ikb=0,ivdd=0,iq=0,iina=0,iinb=0,iena=0,ienb=0;
    bool uvlo=false,drivea=false,driveb=false;
    model_power::Breakdown power;
    double noise_a=0,noise_b=0,outa_observed=0,outb_observed=0;
};
class DriverCore {
    Config config_;
    std::array<Channel,2> channels_;
    bool vdd_ok_ = false;
    model_power::Breakdown power_;
    model_info::Queries queries_;double last_dt_=0;
    double previous_time_ = -1;
public:
    explicit DriverCore(const Config& config) : config_(config),queries_(config.parameters()) {}
    Observation step(double time, double dt, double vdd, const std::array<double,2>& input,
                     const std::array<double,2>& enable, const std::array<OutputLoad,2>& load);
    const Config& config() const { return config_; }
    double get_latency()const{return queries_.get_latency();}
    json get_latency_info()const{return queries_.get_latency_info();}
    double get_timestep_s()const{return last_dt_;}
    model_info::OptionalArea get_area()const{return queries_.get_area();}
    double get_noise_density()const{return queries_.get_noise_density();}
    json get_noise_info()const{return queries_.get_noise_info();}
    double get_power_dissipation()const{return power_.total;}
    const model_power::Breakdown& get_power_breakdown()const{return power_;}
};
class Ucc27624Tdf : public sca_tdf::sca_module {
    DriverCore core_;
    double dt_;
    std::array<OutputLoad,2> load_;
public:
    sca_tdf::sca_in<double> vdd,ina,inb,ena,enb;
    sca_tdf::sca_out<double> outa,outb,isa,isb,ika,ikb,ivdd,iq,iina,iinb,iena,ienb;
    sca_tdf::sca_out<double> power;
    sca_tdf::sca_out<double> noise_a_V,noise_b_V,outa_observed_V,outb_observed_V;
    sca_tdf::sca_out<bool> uvlo_ok,drivea,driveb;
    Ucc27624Tdf(sc_core::sc_module_name name,const Config& config,double dt,const std::array<OutputLoad,2>& load);
    void set_attributes();
    void processing();
    std::string get_module_name() const { return core_.config().name(); }
    std::string get_version() const { return core_.config().version(); }
    std::string get_category()const{return core_.config().parameters().at("category").at("value").get<std::string>();}
    double get_parameter(const std::string& name) const { return core_.config().get(name); }
    const json& get_parameters() const { return core_.config().parameters(); }
    json get_metadata(const std::string& name) const { return get_parameters().at(name); }
    double get_latency()const{return core_.get_latency();}
    json get_latency_info()const{return core_.get_latency_info();}
    double get_timestep_s()const{return dt_;}
    model_info::OptionalArea get_area()const{return core_.get_area();}
    double get_noise_density()const{return core_.get_noise_density();}
    json get_noise_info()const{return core_.get_noise_info();}
    double get_power_dissipation()const{return core_.get_power_dissipation();}
    const model_power::Breakdown& get_power_breakdown()const{return core_.get_power_breakdown();}
};
}
