#pragma once
// Source provenance (delivery revision 1.0.1, 2026-10-29):
// TPS22975.va | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_功率开关_负载开关_TPS22975\TPS22975.va
// SHA256 c37f99b0a0c7852efe068c8cd9292ecefda40143839befbe2ce0d01b8590ae25
// TPS22975_datasheet.pdf | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_功率开关_负载开关_TPS22975\TPS22975_datasheet.pdf
// SHA256 e9544d6c90f1f0973f0cd7eac78954d45b6582f2c6f26dac2e57d018ce743af2

#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
namespace tps22975 {
using json=nlohmann::json;
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
 const std::vector<std::string> allowed={"area_mm2","area_query","body_is","body_nvt","category","dimensions","function_info","i_zero_eps","ict_charge","imax_rating","ion_leak","iq_2p5v","iq_5v","isd_vbias_typ","isd_vin_typ","latency_query","limitations","name","noise_enable","noise_high_hz","noise_low_hz","noise_model","noise_query","noise_rms_v","noise_seed","noise_tones","package_info","ports","power_components","power_model","power_query","power_revision","power_scope","rct_clamp","rct_discharge","ron_2p5v","ron_5v","rpd","t_off_delay","t_ramp_start","ta_max_c","ta_min_c","timestep_query","tsd_c","tsd_return_c","v_eps","variant","vbias_max","vbias_min","vct_full","version","vin_min","von_fall","von_rise"},required={"area_mm2","area_query","body_is","body_nvt","category","dimensions","function_info","i_zero_eps","ict_charge","imax_rating","ion_leak","iq_2p5v","iq_5v","isd_vbias_typ","isd_vin_typ","latency_query","limitations","name","noise_enable","noise_high_hz","noise_low_hz","noise_model","noise_query","noise_rms_v","noise_seed","noise_tones","package_info","ports","power_components","power_model","power_query","power_revision","power_scope","rct_clamp","rct_discharge","ron_2p5v","ron_5v","rpd","t_off_delay","t_ramp_start","ta_max_c","ta_min_c","timestep_query","tsd_c","tsd_return_c","v_eps","variant","vbias_max","vbias_min","vct_full","version","vin_min","von_fall","von_rise"};
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
 if(j.at("name").at("value")!="tps22975")throw std::invalid_argument("Wrong model identity");
 const auto& dim=j.at("dimensions").at("value");if(!dim.at("width").is_number_integer()||!dim.at("height").is_number_integer()||dim.at("width").get<int>()<1||dim.at("height").get<int>()<1)throw std::invalid_argument("Invalid icon dimensions");
 const auto& ps=j.at("ports").at("value");if(ps.size()!=30)throw std::invalid_argument("Wrong port count");
 const std::vector<std::string> pins={"vin_V","on_V","vbias_V","gnd_V","vout_rel_V","vout_V","ct_V","pass_A","qod_A","body_A","ivin_A","ivbias_A","ion_A","ron_ohm","enabled","noise_V","output_observed_V","power_W","power_quiescent_W","power_drive_W","power_input_W","power_clamp_W","power_bootstrap_W","power_discharge_W","power_body_W","power_ct_W","power_terminal_in_W","power_terminal_out_W","power_ct_source_W","power_ct_storage_W"};
 for(size_t i=0;i<ps.size();++i){const auto& z=ps[i];if(z.at("name")!=pins[i]||z.at("io_type")!=(i<4?"input":"output")||z.at("x").get<int>()<0||z.at("x").get<int>()>dim.at("width").get<int>()||z.at("y").get<int>()<0||z.at("y").get<int>()>dim.at("height").get<int>())throw std::invalid_argument("Invalid port metadata");}
}

class Config {
    json raw_;
    std::map<std::string,double> numbers_;
public:
    explicit Config(const std::string& file){std::ifstream f(file);if(!f)throw std::runtime_error("Cannot read config "+file);f>>raw_;validate();}
    explicit Config(const json& j):raw_(j){validate();}
    void validate(){
        validate_contract(raw_);
        for(auto it=raw_.begin();it!=raw_.end();++it){
            auto v=it.value();for(const char* k:{"type","value","unit","editable"})if(!v.count(k))throw std::runtime_error("Missing schema field");
            if(v["value"].is_number()){
                double x=v["value"].get<double>();if(!std::isfinite(x))throw std::runtime_error("Nonfinite config");
                if(it.key()[0]=='r' && x<=0)throw std::runtime_error("Resistance must be positive");
                if(it.key().find("t_")==0 && x<0)throw std::runtime_error("Time must be nonnegative");
                numbers_[it.key()]=x;
            }
        }
        model_info::Queries query_validation(raw_);(void)query_validation;
        if(name().empty()||version().empty())throw std::runtime_error("Missing identity");
        for(const auto& pair:std::vector<std::pair<std::string,std::string>>{{"vin_rise","vin_fall"},{"ven_rise","ven_fall"},{"von_rise","von_fall"},{"vcc_uvlo_rise","vcc_uvlo_fall"},{"vbs_uvlo_rise","vbs_uvlo_fall"},{"vdd_uvlo_rise","vdd_uvlo_fall"},{"hb_uvlo_rise","hb_uvlo_fall"}}){
            if(numbers_.count(pair.first)&&get(pair.first)<=get(pair.second))throw std::runtime_error("Hysteresis thresholds reversed");
        }
        for(const char* k:{"i_source_pk","i_sink_pk","v_eps","i_limit_eps","i_zero_eps","ict_charge","vct_full","iboot_nvt","body_nvt"})if(numbers_.count(k)&&get(k)<=0)throw std::runtime_error("Nonpositive model control");
    }
    double get(const std::string& k)const{return numbers_.at(k);}
    std::string name()const{return raw_.at("name").at("value").get<std::string>();}
    std::string version()const{return raw_.at("version").at("value").get<std::string>();}
    const json& parameters()const{return raw_;}
};
inline double positive(double x,double e){return .5*(x+std::sqrt(x*x+e*e));}
inline double smoothmin(double a,double b,double e){return .5*(a+b-std::sqrt((a-b)*(a-b)+e*e));}
inline void hyst(bool& q,double v,double hi,double lo){if(v>=hi)q=true;else if(v<=lo)q=false;}
inline double limexp(double x){return x<40?std::exp(x):std::exp(40)*(1+x-40);}
struct Load {
    double c=1e-12,r=1e12,current=0,rup=1e30,vup=0,force=0;bool forced=false;
    Load()=default;
    explicit Load(const json& x){c=x.value("capacitance_F",c);r=x.value("resistance_ohm",r);current=x.value("current_A",0.);rup=x.value("pullup_ohm",rup);vup=x.value("pullup_voltage_V",0.);forced=x.count("force_voltage_V");if(forced)force=x["force_voltage_V"];
        if(!std::isfinite(c)||!std::isfinite(r)||!std::isfinite(rup)||!std::isfinite(current)||!std::isfinite(vup)||!std::isfinite(force)||c<=0||r<=0||rup<=0)throw std::runtime_error("Invalid external load");}
    double draw(double v)const{return v/r+current+(v-vup)/rup;}
};
struct Ramp {
    double initial=0,target=0,start=0,duration=0;
    double value(double t)const{return t<start?initial:(duration<=0||t>=start+duration)?target:initial+(target-initial)*(t-start)/duration;}
    void set(double t,double next,double span){initial=value(t);target=next;start=t;duration=span;}
    void dc(double value){initial=target=value;}
};
struct Command {
    struct Event {double time;bool high;};
    bool requested=false;Ramp src,sink;std::deque<Event> queue;
    Command(){sink.dc(1);}
    void dc(bool high){requested=high;src.dc(high?1:0);sink.dc(high?0:1);}
    void update(double t,bool high,double ton,double toff,double sr,double sf,double kr,double kf){
        if(high!=requested){queue.push_back({t+(high?ton:toff),high});requested=high;}
        std::stable_sort(queue.begin(),queue.end(),[](const Event&a,const Event&b){return a.time<b.time;});
        while(!queue.empty()&&queue.front().time<=t+1e-16){auto e=queue.front();queue.pop_front();src.set(e.time,e.high?1:0,e.high?sr:sf);sink.set(e.time,e.high?0:1,e.high?kf:kr);}
    }
};
struct GateParameters {double roh,rol,source,sink,ve,ie,ze,clamp,rc;};
struct Gate {
    double v=0,is=0,ik=0,high_clamp=0,low_clamp=0;
    void step(double dt,double supply,double src,double sink,const GateParameters& p,const Load& load){
        const auto branch=[&](double v,double& a,double& b,double& ch,double& cl){a=positive(smoothmin(src*positive(supply-v,p.ve)/p.roh,p.source,p.ie),p.ze);b=positive(smoothmin(sink*positive(v,p.ve)/p.rol,p.sink,p.ie),p.ze);ch=positive(v-supply-p.clamp,p.ve)/p.rc;cl=positive(-v-p.clamp,p.ve)/p.rc;};
        const auto residual=[&](double x){double a,b,ch,cl;branch(x,a,b,ch,cl);return load.c/dt*(x-v)+load.draw(x)-a+b+ch-cl;};
        if(load.forced)v=load.force;
        else{double lo=std::min(0.,v)-20,hi=std::max(supply,v)+20;if(residual(lo)>0||residual(hi)<0)throw std::runtime_error("Gate solver cannot bracket root");
            for(int n=0;n<42;++n){double mid=(lo+hi)/2;if(residual(mid)>0)hi=mid;else lo=mid;}v=(lo+hi)/2;}
        branch(v,is,ik,high_clamp,low_clamp);if(!std::isfinite(v+is+ik))throw std::runtime_error("Nonfinite gate state");
    }
};
class Core {
protected:
    Config config_;json fixture_;model_power::Breakdown power_;model_info::Queries queries_;double last_dt_=0,previous_time_=-1;
public:
    std::vector<std::string> input_names,output_names;
    Core(const Config& c,const json& f):config_(c),fixture_(f),queries_(c.parameters()){}
    virtual ~Core(){}
    void validate_step(double t,double dt,const std::vector<double>& x){
        queries_.validate_timestep(dt);
        if(!std::isfinite(t)||t<0||t<previous_time_||x.size()!=input_names.size())throw std::invalid_argument("Invalid step time or input size");
        for(double v:x)if(!std::isfinite(v))throw std::invalid_argument("Nonfinite pin input");
        previous_time_=t;
    }
    virtual std::vector<double> step(double t,double dt,const std::vector<double>& inputs)=0;
    const Config& config()const{return config_;}
    double get_power_dissipation()const{return power_.total;}
    const model_power::Breakdown& get_power_breakdown()const{return power_;}
    double get_latency()const{return queries_.get_latency();}
    json get_latency_info()const{return queries_.get_latency_info();}
    double get_timestep_s()const{return last_dt_;}
    model_info::OptionalArea get_area()const{return queries_.get_area();}
    double get_noise_density()const{return queries_.get_noise_density();}
    json get_noise_info()const{return queries_.get_noise_info();}
    std::vector<double> observe(std::vector<double> v,double t,double dt,const std::vector<bool>& active){
        last_dt_=dt;std::vector<double> noise;
        for(unsigned k=0;k<active.size();++k)noise.push_back(queries_.sample(t,dt,k,active[k]));
        for(double x:noise)v.push_back(x);
        for(unsigned k=0;k<noise.size();++k)v.push_back(v[k==0?0:2]+noise[k]);
        return power_.append(v);
    }
    void add_power_outputs(){auto n=model_power::Breakdown::names();output_names.insert(output_names.end(),n.begin(),n.end());}
};
std::unique_ptr<Core> make_core(const Config&,const json&);
// Public TDF voltage/current interface. Named accessors avoid pin-index binding errors.
class ChipTdf:public sca_tdf::sca_module {
    std::unique_ptr<Core> core_;double dt_;std::vector<double> values_;
public:
    sc_core::sc_vector<sca_tdf::sca_in<double>> inputs;
    sc_core::sc_vector<sca_tdf::sca_out<double>> outputs;
    ChipTdf(sc_core::sc_module_name n,std::unique_ptr<Core> c,double dt):sca_tdf::sca_module(n),core_(std::move(c)),dt_(dt),values_(core_->input_names.size()),inputs("voltage_inputs",core_->input_names.size()),outputs("observations",core_->output_names.size()){}
    void set_attributes(){model_info::Queries(core_->config().parameters()).validate_timestep(dt_);set_timestep(dt_,sc_core::SC_SEC);}
    void processing(){for(size_t k=0;k<values_.size();++k){values_[k]=inputs[k].read();if(!std::isfinite(values_[k]))throw std::runtime_error("Nonfinite input");}auto x=core_->step(get_time().to_seconds(),dt_,values_);if(x.size()!=outputs.size())throw std::runtime_error("Output interface size mismatch");for(size_t k=0;k<x.size();++k){if(!std::isfinite(x[k]))throw std::runtime_error("Nonfinite output");outputs[k].write(x[k]);}}
    sca_tdf::sca_in<double>& input(const std::string& name){auto p=std::find(core_->input_names.begin(),core_->input_names.end(),name);if(p==core_->input_names.end())throw std::runtime_error("Unknown pin "+name);return inputs[p-core_->input_names.begin()];}
    sca_tdf::sca_out<double>& output(const std::string& name){auto p=std::find(core_->output_names.begin(),core_->output_names.end(),name);if(p==core_->output_names.end())throw std::runtime_error("Unknown pin "+name);return outputs[p-core_->output_names.begin()];}
    std::string get_module_name()const{return core_->config().name();}
    std::string get_version()const{return core_->config().version();}
    std::string get_category()const{return core_->config().parameters().at("category").at("value").get<std::string>();}
    double get_parameter(const std::string& name)const{return core_->config().get(name);}
    const json& get_parameters()const{return core_->config().parameters();}
    json get_metadata(const std::string& name)const{return get_parameters().at(name);}
    double get_latency()const{return core_->get_latency();}
    json get_latency_info()const{return core_->get_latency_info();}
    double get_timestep_s()const{return dt_;}
    model_info::OptionalArea get_area()const{return core_->get_area();}
    double get_noise_density()const{return core_->get_noise_density();}
    json get_noise_info()const{return core_->get_noise_info();}
    double get_power_dissipation()const{return core_->get_power_dissipation();}
    const model_power::Breakdown& get_power_breakdown()const{return core_->get_power_breakdown();}
};
}
