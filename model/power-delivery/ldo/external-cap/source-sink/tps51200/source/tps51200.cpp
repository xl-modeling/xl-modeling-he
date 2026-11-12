#include "tps51200.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>
#include <vector>

// E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\LDO\External-Cap_LDO\供电芯粒_LDO_External-Cap_LDO_Source-Sink_LDO\TPS51200_core_v1.va
// SHA256=32f61693998b072384f099ac9a73e6970ba67213b3d7b63c8740d311417f8b3f
// E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\LDO\External-Cap_LDO\供电芯粒_LDO_External-Cap_LDO_Source-Sink_LDO\01_TI_TPS51200_RevE.pdf
// SHA256=c3ad54837f5fb8620c06af33105e1aaa3403b127ab0125c299cb690bb914154e
// TI SLUS812E (2025-09), pp.5-6 electrical tables, pp.11-12 modes, p.14 GM/COUT.
// VA source: parameter declarations and all analog branches; historical report is not silicon evidence.
// VA -> SC adaptations: backward Euler for external capacitors, linear finite
// gate ramps instead of Spectre transition(), sampled thresholds with <=dt delay.
// GM constant and RHEAD/EN thresholds are inherited assumptions, not silicon fits.
namespace tps51200 {
const std::array<const char*,NIN> input_names={"vin","vldoin","refin","en","iload","iref","rload","temp_c","force_vo","force_ref","pg_pullup"};
const std::array<const char*,NOUT> output_names={"vo","vosns","refout","pgood","iin","ildoin","isource","isink","iref_source","iref_sink","power","power_source","power_sink","power_ref","power_bias","power_discharge","power_pg","pin","pload","pstorage","pexternal","residual","numerical_damping","uv_ok","ref_ok","en_ok","hot","pg_ok","noise","vo_observed"};
namespace {
double pos(double x,double e){return x>=0?0.5*(x+std::hypot(x,e)):0.5*e*(e/(std::hypot(x,e)-x));}
double smooth_min(double a,double b,double e){return 0.5*(a+b-std::hypot(a-b,e));}
double limited(double a,double b,double e){return std::max(0.,smooth_min(a,b,e));}
double ramp(double a,bool b,double fraction){return a+std::clamp((b?1.:0.)-a,-fraction,fraction);}
void latch(bool& state,double x,double hi,double lo){if(x>=hi)state=true;else if(x<=lo)state=false;}
template<class F> double root(F f,double center){
    double width=4,lo=center-width,hi=center+width;
    for(int i=0;i<24 && !(f(lo)<=0 && f(hi)>=0);++i){width*=2;lo=center-width;hi=center+width;}
    if(!(std::isfinite(f(lo))&&std::isfinite(f(hi))&&f(lo)<=0&&f(hi)>=0))throw std::runtime_error("node solve not bracketed");
    for(int i=0;i<70;++i){double mid=0.5*(lo+hi);if(f(mid)>0)hi=mid;else lo=mid;}
    return 0.5*(lo+hi);
}
}
Config::Config(const std::string& path){
    std::ifstream f(path);if(!f)throw std::runtime_error("cannot open config: "+path);f>>data_;
    if(!data_.is_object())throw std::invalid_argument("config must be an object");
    const std::set<std::string> metadata={"name","version","category","dimensions","ports","area_mm2","physical_latency_s","sources","limitations"};
    const std::set<std::string> numeric={"gm","ilim_source","ilim_sink","rhead_source","rhead_sink","vo_offset","ref_offset","gm_ref","iref_limit","vin_on","vin_off","ref_on","ref_off","en_on","en_off","tsd_c","trecover_c","iq_on","iq_s3","iq_s5","ildoin_on","ildoin_off","rdis","rdis_ref","pg_delay_s","pg_bad_delay_s","pg_tick_s","pg_ron","edge_s","epsilon_A","pg_enter_ratio","pg_exit_ratio","fold_ratio","fold_width","g_leak_S","g_input_S","cout_F","cref_F","esr_ohm","rtrace_ohm","rpg_ohm","dt_s","initial_vo_V","initial_ref_V","noise_rms_V","noise_seed"};
    const std::set<std::string> flags={"remote_sense","force_vo_enable","force_ref_enable","noise_enable"};
    std::set<std::string> expected=metadata;expected.insert(numeric.begin(),numeric.end());expected.insert(flags.begin(),flags.end());
    if(data_.size()!=expected.size())throw std::invalid_argument("missing or extra config fields");
    for(const auto& k:expected){
        if(!data_.contains(k))throw std::invalid_argument("missing field: "+k);
        const auto& v=data_.at(k);
        for(const char* field:{"type","value","unit","editable","group","source"})if(!v.contains(field))throw std::invalid_argument("metadata missing: "+k);
        if(!v["editable"].is_boolean()||!v["type"].is_string()||!v["source"].is_string())throw std::invalid_argument("bad metadata: "+k);
        if(numeric.count(k)){
            if(!v["value"].is_number()||!std::isfinite(v["value"].get<double>()))throw std::invalid_argument("nonfinite or nonnumeric: "+k);
            if(v["type"]!=(k=="noise_seed"?"int":"double"))throw std::invalid_argument("declared numeric type: "+k);
            if(v.contains("valid_range")){double x=v["value"].get<double>();if(x<v["valid_range"][0].get<double>()||x>v["valid_range"][1].get<double>())throw std::invalid_argument("out of range: "+k);}
        }
        if(flags.count(k)&&(!v["value"].is_boolean()||v["type"]!="bool"))throw std::invalid_argument("bad boolean: "+k);
    }
    if(data_["name"]["value"]!="tps51200")throw std::invalid_argument("wrong chip name");
    for(const char* k:{"name","version","category"})if(!data_[k]["value"].is_string()||data_[k]["type"]!="string")throw std::invalid_argument("identity type");
    for(const char* k:{"area_mm2","physical_latency_s"}){
        const auto& x=data_[k]["value"];if(data_[k]["type"]!="nullable_double"||(!x.is_null()&&(!x.is_number()||!std::isfinite(x.get<double>())||x.get<double>()<0)))throw std::invalid_argument("bad optional metric");
    }
    for(const char* k:{"gm","ilim_source","ilim_sink","rhead_source","rhead_sink","gm_ref","iref_limit","rdis","rdis_ref","pg_tick_s","pg_ron","edge_s","epsilon_A","fold_width","cout_F","cref_F","rpg_ohm","dt_s"})if(number(k)<=0)throw std::invalid_argument(std::string("positive required: ")+k);
    for(const char* k:{"esr_ohm","rtrace_ohm","iq_on","iq_s3","iq_s5","ildoin_on","ildoin_off","g_leak_S","g_input_S","noise_rms_V","pg_delay_s","pg_bad_delay_s"})if(number(k)<0)throw std::invalid_argument("negative passive parameter");
    if(number("vin_on")<=number("vin_off")||number("ref_on")<=number("ref_off")||number("en_on")<=number("en_off")||number("tsd_c")<=number("trecover_c"))throw std::invalid_argument("hysteresis thresholds reversed");
    if(number("pg_enter_ratio")<0||number("pg_enter_ratio")>=number("pg_exit_ratio")||number("pg_exit_ratio")>=1||number("fold_ratio")<=0||number("fold_ratio")>1)throw std::invalid_argument("window/fold ratio");
    if(number("cout_F")<20e-6||number("cref_F")>=.47e-6)throw std::invalid_argument("capacitors outside supported datasheet fixture");
    if(number("dt_s")>number("pg_tick_s")||number("dt_s")>number("edge_s"))throw std::invalid_argument("dt must resolve PG tick and edge");
    double seed=number("noise_seed");if(seed<0||seed>4294967295.||seed!=std::floor(seed))throw std::invalid_argument("invalid seed");
    auto d=data_["dimensions"]["value"];if(!d.is_object()||!d.contains("width")||!d.contains("height")||!d["width"].is_number_integer()||!d["height"].is_number_integer()||d["width"].get<int>()<=0||d["height"].get<int>()<=0)throw std::invalid_argument("invalid dimensions");
    auto ports=data_["ports"]["value"];if(!ports.is_array()||ports.size()!=NIN+NOUT)throw std::invalid_argument("invalid ports");
    for(int i=0;i<NIN+NOUT;++i){const auto& q=ports.at(i);const char* name=i<NIN?input_names[i]:output_names[i-NIN];
        if(q.at("name")!=name||q.at("io_type")!=(i<NIN?"input":"output")||q.at("data_type")!="double"||!q.at("unit").is_string()||!q.at("x").is_number_integer()||!q.at("y").is_number_integer())throw std::invalid_argument("port contract mismatch");
        if(q["x"].get<int>()<0||q["x"].get<int>()>d["width"].get<int>()||q["y"].get<int>()<0||q["y"].get<int>()>d["height"].get<int>())throw std::invalid_argument("port outside icon");
    }
}
Core::Core(Config cfg):cfg_(std::move(cfg)),rng_(static_cast<unsigned>(cfg_.number("noise_seed"))){vc_=cfg_.number("initial_vo_V");vr_=cfg_.number("initial_ref_V");}
std::array<double,NOUT> Core::step(double t,const std::array<double,NIN>& x){
    auto p=[this](const char* k){return cfg_.number(k);};double dt=p("dt_s"),eps=p("epsilon_A");
    if(!std::isfinite(t)||t<0||(initialized_&&std::abs(t-last_t_-dt)>std::max(1e-15,dt*1e-5)))throw std::invalid_argument("nonuniform time step");
    for(double v:x)if(!std::isfinite(v))throw std::invalid_argument("nonfinite input");
    if(x[RLOAD]<=0||x[VIN]<0||x[VLDOIN]<0||x[REFIN]<0||x[PG_PULLUP]<0||x[TEMP_C]<-273.15)throw std::invalid_argument("unsupported input domain");
    if(!initialized_){uv_=x[VIN]>=p("vin_on");ref_=x[REFIN]>=p("ref_on");en_=x[EN]>=p("en_on");hot_=x[TEMP_C]>=p("tsd_c");initialized_=true;}
    else{latch(uv_,x[VIN],p("vin_on"),p("vin_off"));latch(ref_,x[REFIN],p("ref_on"),p("ref_off"));latch(en_,x[EN],p("en_on"),p("en_off"));latch(hot_,x[TEMP_C],p("tsd_c"),p("trecover_c"));}
    bool ready=uv_&&ref_&&!hot_;gr_=ramp(gr_,ready,dt/p("edge_s"));gon_=ramp(gon_,ready&&en_,dt/p("edge_s"));
    // REFOUT KCL. GREF and its 40mA limit follow the VA, GM_REF is an assumption.
    auto rb=[&](double v){double cmd=p("gm_ref")*(x[REFIN]+p("ref_offset")-v);
        return std::array<double,3>{gr_*limited(limited(pos(cmd,eps),p("iref_limit"),eps),pos((x[VIN]-v)/p("rhead_source"),eps),eps),
            gr_*limited(limited(pos(-cmd,eps),p("iref_limit"),eps),pos(v/p("rhead_sink"),eps),eps),((1-gr_)/p("rdis_ref")+p("g_leak_S"))*v};};
    double oldr=vr_,oldc=vc_;
    auto rf=[&](double v){auto b=rb(v);return p("cref_F")*(v-oldr)/dt+x[IREF]+b[1]+b[2]-b[0];};
    vr_=cfg_.flag("force_ref_enable")?x[FORCE_REF]:root(rf,oldr);
    auto r=rb(vr_);
    double iforcer=rf(vr_); // ideal fixture current injected into REFOUT
    double gload=1/x[RLOAD]+p("g_input_S"),rt=p("rtrace_ohm");
    auto sense=[&](double v){return (v-rt*x[ILOAD])/(1+rt*gload);};
    auto vb=[&](double v){double vs=cfg_.flag("remote_sense")?sense(v):v;
        double er=std::abs(vs-vr_)/std::max(std::abs(vr_),.05);
        double fold=(1+p("fold_ratio"))/2-(1-p("fold_ratio"))/2*std::tanh((er-p("pg_exit_ratio"))/p("fold_width"));
        double cmd=p("gm")*(vr_+p("vo_offset")-vs);
        return std::array<double,3>{gon_*limited(pos(cmd,eps),limited(p("ilim_source")*fold,pos((x[VLDOIN]-v)/p("rhead_source"),eps),eps),eps),
            gon_*limited(pos(-cmd,eps),limited(p("ilim_sink")*fold,pos(v/p("rhead_sink"),eps),eps),eps),((1-gon_)/p("rdis")+p("g_leak_S"))*v};};
    double zc=dt/p("cout_F")+p("esr_ohm");
    auto vf=[&](double v){auto b=vb(v);return (v-oldc)/zc+x[ILOAD]+sense(v)*gload+b[1]+b[2]-b[0];};
    double vo=cfg_.flag("force_vo_enable")?x[FORCE_VO]:root(vf,oldc);
    auto b=vb(vo);double vs=sense(vo),ic=(vo-oldc)/zc;vc_=oldc+dt*ic/p("cout_F");double iforce=vf(vo);
    // Sampled PGOOD timer; requalification after every recovery is inherited VA assumption.
    if(!ready||!en_){pg_=false;window_=false;good_since_=bad_since_=-1;}
    if(t+dt*1e-6>=next_pg_){
        next_pg_=(std::floor((t+dt*1e-6)/p("pg_tick_s"))+1)*p("pg_tick_s");
        if(ready&&en_){double feedback=cfg_.flag("remote_sense")?vs:vo;double error=std::abs(feedback-vr_)/std::max(std::abs(vr_),.05);
            if(error<=p("pg_enter_ratio"))window_=true;
            if(error>=p("pg_exit_ratio"))window_=false;
            if(window_){bad_since_=-1;if(good_since_<0)good_since_=t;if(t-good_since_+dt*1e-6>=p("pg_delay_s"))pg_=true;}
            else{good_since_=-1;if(bad_since_<0)bad_since_=t;if(t-bad_since_+dt*1e-6>=p("pg_bad_delay_s"))pg_=false;}
        }
    }
    gpg_=ramp(gpg_,!pg_,dt/p("edge_s"));double gp=gpg_/p("pg_ron")+p("g_leak_S");
    double vpg=x[PG_PULLUP]/(1+p("rpg_ohm")*gp),ipg=gp*vpg;
    double iq=(p("iq_s5")+(p("iq_s3")-p("iq_s5"))*gr_+(p("iq_on")-p("iq_s3"))*gon_)*.5*(1+std::tanh((x[VIN]-.5)/.1));
    double ildoq=(p("ildoin_off")+(p("ildoin_on")-p("ildoin_off"))*gon_)*.5*(1+std::tanh((x[VLDOIN]-.5)/.1));
    // Explicit source/sink dissipation. External ESR/trace/pullup and C storage excluded.
    double ps=(x[VLDOIN]-vo)*b[0],pk=vo*b[1],pr=(x[VIN]-vr_)*r[0]+vr_*r[1];
    double pb=x[VIN]*iq+x[VLDOIN]*ildoq+p("g_input_S")*(x[REFIN]*x[REFIN]+x[EN]*x[EN]+vs*vs);
    double pd=vo*b[2]+vr_*r[2],ppg=vpg*ipg,power=ps+pk+pr+pb+pd+ppg;
    double pin=x[VIN]*(iq+r[0])+x[VLDOIN]*(ildoq+b[0])+x[PG_PULLUP]*ipg+p("g_input_S")*(x[REFIN]*x[REFIN]+x[EN]*x[EN])+vo*iforce+vr_*iforcer;
    double j=x[ILOAD]+vs*gload,pload=vs*(x[ILOAD]+vs/x[RLOAD])+vr_*x[IREF];
    double pstore=(p("cout_F")*(vc_*vc_-oldc*oldc)+p("cref_F")*(vr_*vr_-oldr*oldr))/(2*dt);
    double pext=ic*ic*p("esr_ohm")+j*j*rt+ipg*ipg*p("rpg_ohm");
    double damp=(p("cout_F")*(vc_-oldc)*(vc_-oldc)+p("cref_F")*(vr_-oldr)*(vr_-oldr))/(2*dt);
    double noise=cfg_.flag("noise_enable")?normal_(rng_)*p("noise_rms_V"):0.;
    last_={vo,vs,vr_,vpg,iq+r[0],ildoq+b[0],b[0],b[1],r[0],r[1],power,ps,pk,pr,pb,pd,ppg,pin,pload,pstore,pext,pin-pload-power-pstore-pext-damp,damp,double(uv_),double(ref_),double(en_),double(hot_),double(pg_),noise,vo+noise};
    for(double v:last_)if(!std::isfinite(v))throw std::runtime_error("nonfinite model output");
    last_t_=t;return last_;
}
TPS51200::TPS51200(sc_core::sc_module_name name,const std::string& path):sca_tdf::sca_module(name),inputs("inputs",NIN),outputs("outputs",NOUT),core_(Config(path)){}
void TPS51200::set_attributes(){set_timestep(get_timestep_s(),sc_core::SC_SEC);}
void TPS51200::processing(){std::array<double,NIN>x{};for(int i=0;i<NIN;++i)x[i]=inputs[i].read();auto y=core_.step(get_time().to_seconds(),x);for(int i=0;i<NOUT;++i)outputs[i].write(y[i]);}
std::string TPS51200::get_module_name()const{return configuration()["name"]["value"];}
std::string TPS51200::get_version()const{return configuration()["version"]["value"];}
std::string TPS51200::get_category()const{return configuration()["category"]["value"];}
double TPS51200::get_timestep_s()const{return core_.config().number("dt_s");}
std::optional<double> TPS51200::get_area()const{auto x=configuration()["area_mm2"]["value"];return x.is_null()?std::nullopt:std::optional<double>(x.get<double>());}
std::optional<double> TPS51200::get_latency()const{auto x=configuration()["physical_latency_s"]["value"];return x.is_null()?std::nullopt:std::optional<double>(x.get<double>());}
double TPS51200::get_pg_delay_s()const{return core_.config().number("pg_delay_s");}
double TPS51200::get_power_dissipation()const{return core_.power();}
json TPS51200::get_power_breakdown()const{json j;auto x=core_.snapshot();for(int i=POWER;i<=NUM_DAMPING;++i)j[output_names[i]]=x[i];return j;}
json TPS51200::get_noise_info()const{return {{"enabled",core_.config().flag("noise_enable")},{"sample_rms_V",core_.config().number("noise_rms_V")},{"calibrated",false},{"scope","observed voltage only; sample RMS, not physical spectrum"}};}
}
