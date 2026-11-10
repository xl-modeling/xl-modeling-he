#include "ucc27624.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

// Source provenance (delivery revision 1.0.1, 2026-10-29):
// UCC27624.va | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_栅极驱动_低边驱动_UCC27624\UCC27624.va
// SHA256 1bd18e820a0f83c566804e239109048a2110e3a3e373b45af07b77115816020e
// UCC27624_datasheet.pdf | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_栅极驱动_低边驱动_UCC27624\UCC27624_datasheet.pdf
// SHA256 09173fc9e2c7314f5bd6ecf95db5798b2244828d5a25cf5e5b363df3de14aeb3
namespace ucc27624 {
Config::Config(const std::string& path) {
    std::ifstream stream(path);
    if(!stream) throw std::runtime_error("Cannot open config: "+path);
    stream >> data_; validate();
}
Config::Config(const json& data):data_(data) { validate(); }
double Config::get(const std::string& k) const { return data_.at(k).at("value").get<double>(); }
std::string Config::name() const { return data_.at("name").at("value").get<std::string>(); }
std::string Config::version() const { return data_.at("version").at("value").get<std::string>(); }
void Config::validate() const {
    validate_contract(data_);
    model_info::Queries query_validation(data_);(void)query_validation;
    for(auto i=data_.begin(); i!=data_.end(); ++i) {
        const auto& v=i.value().at("value");
        if(v.is_number() && !std::isfinite(v.get<double>())) throw std::runtime_error("Nonfinite parameter: "+i.key());
    }
    const char* positive[]={"rin","ren","roh_dc","rol_dc","r_nboost","i_source_pk","i_sink_pk","r_clamp","v_eps","i_eps","i_zero_eps"};
    for(auto k:positive) if(get(k)<=0) throw std::runtime_error(std::string("Positive parameter required: ")+k);
    const char* nonnegative[]={"t_launch_on","t_launch_off","t_skew_b","t_src_on_strength","t_src_off_strength","t_snk_on_strength","t_snk_off_strength","iq_out_high","iq_out_low","iq_disabled","iq_uvlo","v_clamp"};
    for(auto k:nonnegative) if(get(k)<0) throw std::runtime_error(std::string("Nonnegative parameter required: ")+k);
    if(get("vuvlo_on")<=get("vuvlo_off") || get("vin_h")<=get("vin_l") || get("ven_h")<=get("ven_l")) throw std::runtime_error("Invalid hysteresis thresholds");
    if(get("boost_off_fraction")<=0 || get("boost_off_fraction")>=1) throw std::runtime_error("Invalid boost threshold");
    if(name().empty() || version().empty()) throw std::runtime_error("Missing model identity");
}
double Ramp::value(double t) const {
    if(t<start) return initial;
    if(duration<=0 || t>=start+duration) return target;
    return initial+(target-initial)*(t-start)/duration;
}
void Ramp::set(double time,double next,double span) { initial=value(time);target=next;start=time;duration=span; }
static double pos(double x,double eps) { return .5*(x+std::sqrt(x*x+eps*eps)); }
static double minimum(double a,double b,double eps) { return .5*(a+b-std::sqrt((a-b)*(a-b)+eps*eps)); }
static void hysteresis(bool& state,double v,double hi,double lo) { if(v>=hi) state=true; else if(v<=lo) state=false; }

Observation DriverCore::step(double time,double dt,double vdd,const std::array<double,2>& input,
                            const std::array<double,2>& enable,const std::array<OutputLoad,2>& loads) {
    if(!std::isfinite(time)||!std::isfinite(dt)||dt<=0||time<previous_time_||!std::isfinite(vdd)) throw std::runtime_error("Invalid time or supply");
    previous_time_=time;last_dt_=dt;
    const auto p=[this](const char* k){return config_.get(k);};
    hysteresis(vdd_ok_,vdd,p("vuvlo_on"),p("vuvlo_off"));
    Observation o; o.uvlo=vdd_ok_;
    const double ve=p("v_eps"),ie=p("i_eps"),ize=p("i_zero_eps"),sp=p("i_source_pk"),kp=p("i_sink_pk"),
        rol=p("rol_dc"),clamp=p("v_clamp"),rc=p("r_clamp");
    double source[2],sink[2],high_clamp[2],low_clamp[2];
    for(unsigned i=0;i<2;++i) {
        auto& c=channels_[i]; const auto& load=loads[i];
        if(!std::isfinite(input[i])||!std::isfinite(enable[i])||!std::isfinite(load.capacitance)||load.capacitance<=0||!std::isfinite(load.resistance)||load.resistance<=0||!std::isfinite(load.current)) throw std::runtime_error("Invalid pin voltage or load");
        hysteresis(c.input,input[i],p("vin_h"),p("vin_l"));
        hysteresis(c.enable,enable[i],p("ven_h"),p("ven_l"));
        const bool request=vdd_ok_ && c.input && c.enable;
        if(request!=c.requested) {
            c.events.push_back({time+(request?p("t_launch_on"):p("t_launch_off"))+(i?p("t_skew_b"):0),request});
            c.requested=request;
        }
        // Ordered by arrival: independent turn-on/off delays can reorder short pulses.
        std::stable_sort(c.events.begin(),c.events.end(),[](const Edge& a,const Edge& b){return a.time<b.time;});
        while(!c.events.empty() && c.events.front().time<=time+dt*1e-6) {
            const Edge e=c.events.front();c.events.pop_front();
            c.source.set(e.time,e.high?1:0,e.high?p("t_src_on_strength"):p("t_src_off_strength"));
            c.sink.set(e.time,e.high?0:1,e.high?p("t_snk_off_strength"):p("t_snk_on_strength"));
        }
        const double src=c.source.value(time),snk=c.sink.value(time);
        if(src>=.05 && c.source.target==1 && c.source.value(time-dt)<.05) c.boost=true;
        if(src<.05 || c.voltage>=p("boost_off_fraction")*vdd) c.boost=false;
        const double conductance=(1/p("roh_dc")+(c.boost?1/p("r_nboost"):0));
        const auto currents=[&](double voltage,double& is,double& ik,double& ch,double& cl) {
            is=pos(minimum(src*pos(vdd-voltage,ve)*conductance,sp,ie),ize);
            ik=pos(minimum(snk*pos(voltage,ve)/rol,kp,ie),ize);
            ch=pos(voltage-vdd-clamp,ve)/rc;
            cl=pos(-voltage-clamp,ve)/rc;
        };
        const auto residual=[&](double v) {
            double is,ik,ch,cl;currents(v,is,ik,ch,cl);
            return load.capacitance/dt*(v-c.voltage)+v/load.resistance+load.current-is+ik+ch-cl;
        };
        double lo=std::min(0.0,c.voltage)-20,hi=std::max(vdd,c.voltage)+20;
        if(residual(lo)>0 || residual(hi)<0) throw std::runtime_error("Output solver cannot bracket root");
        for(int n=0;n<44;++n) { const double mid=(lo+hi)/2; if(residual(mid)>0) hi=mid; else lo=mid; }
        c.voltage=(lo+hi)/2;
        currents(c.voltage,source[i],sink[i],high_clamp[i],low_clamp[i]);
        if(!std::isfinite(c.voltage)) throw std::runtime_error("Nonfinite output");
    }
    const auto& a=channels_[0];const auto& b=channels_[1];
    o.drivea=a.requested;o.driveb=b.requested;
    o.iq=!vdd_ok_?p("iq_uvlo"):(!a.enable&&!b.enable)?p("iq_disabled"):
        p("iq_out_low")-.5*(p("iq_out_low")-p("iq_out_high"))*(a.requested+b.requested);
    o.iina=input[0]/p("rin");o.iinb=input[1]/p("rin");
    o.iena=(vdd-enable[0])/p("ren");o.ienb=(vdd-enable[1])/p("ren");
    o.ivdd=o.iq+source[0]+source[1]-high_clamp[0]-high_clamp[1]+o.iena+o.ienb;
    o.outa=a.voltage;o.outb=b.voltage;o.isa=source[0];o.isb=source[1];o.ika=sink[0];o.ikb=sink[1];
    // EN current is positive FROM VDD TO EN; its terminal current is -iena.
    // This sign is necessary when EN is driven from an independent supply.
    power_=model_power::Breakdown();
    power_.quiescent=vdd*o.iq;
    power_.input=input[0]*o.iina+input[1]*o.iinb
        +(vdd-enable[0])*o.iena+(vdd-enable[1])*o.ienb;
    for(unsigned i=0;i<2;++i){double v=channels_[i].voltage;
        power_.drive+=model_power::gate_drive(vdd,v,source[i],sink[i]);
        power_.clamp+=model_power::gate_clamp(vdd,v,high_clamp[i],low_clamp[i]);
        power_.terminal_out+=v*(source[i]-sink[i]-high_clamp[i]+low_clamp[i]);}
    power_.terminal_in=vdd*o.ivdd+input[0]*o.iina+input[1]*o.iinb
        -enable[0]*o.iena-enable[1]*o.ienb;
    power_.finish();o.power=power_;
    o.noise_a=queries_.sample(time,dt,0,vdd_ok_&&channels_[0].enable);o.noise_b=queries_.sample(time,dt,1,vdd_ok_&&channels_[1].enable);
    o.outa_observed=o.outa+o.noise_a;o.outb_observed=o.outb+o.noise_b;
    return o;
}
Ucc27624Tdf::Ucc27624Tdf(sc_core::sc_module_name name,const Config& cfg,double dt,const std::array<OutputLoad,2>& load)
 :sca_tdf::sca_module(name),core_(cfg),dt_(dt),load_(load),vdd("vdd"),ina("ina"),inb("inb"),ena("ena"),enb("enb"),
 outa("outa"),outb("outb"),isa("isa"),isb("isb"),ika("ika"),ikb("ikb"),ivdd("ivdd"),iq("iq"),iina("iina"),iinb("iinb"),iena("iena"),ienb("ienb"),power("power"),noise_a_V("noise_a_V"),noise_b_V("noise_b_V"),outa_observed_V("outa_observed_V"),outb_observed_V("outb_observed_V"),uvlo_ok("uvlo_ok"),drivea("drivea"),driveb("driveb") {}
void Ucc27624Tdf::set_attributes() {model_info::Queries(core_.config().parameters()).validate_timestep(dt_); set_timestep(dt_,sc_core::SC_SEC); }
void Ucc27624Tdf::processing() {
    auto o=core_.step(get_time().to_seconds(),dt_,vdd.read(),{{ina.read(),inb.read()}},{{ena.read(),enb.read()}},load_);
    outa.write(o.outa);outb.write(o.outb);isa.write(o.isa);isb.write(o.isb);ika.write(o.ika);ikb.write(o.ikb);
    ivdd.write(o.ivdd);iq.write(o.iq);iina.write(o.iina);iinb.write(o.iinb);iena.write(o.iena);ienb.write(o.ienb);
    power.write(o.power.total);noise_a_V.write(o.noise_a);noise_b_V.write(o.noise_b);outa_observed_V.write(o.outa_observed);outb_observed_V.write(o.outb_observed);
    uvlo_ok.write(o.uvlo);drivea.write(o.drivea);driveb.write(o.driveb);
}
}
