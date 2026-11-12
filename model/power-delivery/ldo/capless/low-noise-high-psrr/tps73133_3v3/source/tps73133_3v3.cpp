// Source trace: TI SBVS034P pp.4-5,16-21; supplied TPS73133 VA core and PSRR/noise.
// 'datasheet' corrects ground current and the two load-regulation intervals.
// 'va_static' reproduces the supplied forward DC baseline, without its unphysical
// active sinking above regulation. All model assumptions are identified in JSON.
#include "tps73133_3v3.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
namespace {
constexpr double pi=3.14159265358979323846;
double gate(double v,double m,double w){return .5*(1+std::tanh((v-m)/w));}
double lowpass(double y,double prev,double input,double f,double dt){
    const double x=2*pi*f*dt,a=std::exp(-x),b=-std::expm1(-x)/x;
    return a*y+(1-b)*input+(b-a)*prev; // exact first-order hold
}
}
double TPS73133_3V3::p(const char* k)const{return nums_.at(k);}
TPS73133_3V3::TPS73133_3V3(sc_core::sc_module_name name,const std::string& path):sca_tdf::sca_module(name){
    std::ifstream f(path);if(!f)throw std::runtime_error("Cannot read configuration: "+path);f>>cfg_;
    for(auto it=cfg_.begin();it!=cfg_.end();++it)if(it.value().is_object()&&it.value().contains("value")&&it.value()["value"].is_number()){
        const double v=it.value()["value"].get<double>();if(!std::isfinite(v))throw std::runtime_error("Nonfinite parameter: "+it.key());nums_[it.key()]=v;
    }
    for(const char* k:{"dt_s","vnom_v","rated_a","rout_low_ohm","rout_high_ohm","rout_va_ohm","rdo_ohm","dropout_zo_ohm","icl_a","isc_a","fold_width_v","ren_off_ohm","rfeedback_ohm","startup_reference_s","startup_no_cnr_tau_s","en_width_v","uv_width_v","line_pole_hz","psrr_fc_hz","psrr_h_inf","noise_high_hz"})
        if(p(k)<=0)throw std::runtime_error(std::string("Positive parameter required: ")+k);
    for(const char* k:{"cout_f","esr_ohm","iq_10ma_a","iq_150ma_a","iq_off_a","ien_on_a","reverse_a","gleak_s","vdo_va_v","noise_low_hz","noise_per_v_no_cnr","noise_per_v_cnr"})
        if(p(k)<0)throw std::runtime_error(std::string("Nonnegative parameter required: ")+k);
    if(p("dt_s")<1e-10||p("dt_s")>1e-5||p("noise_low_hz")>=p("noise_high_hz")||p("rated_a")<=.01||p("rated_a")>p("icl_a")||p("isc_a")>p("icl_a")||p("tsd_reset_c")>=p("tsd_trip_c")||p("iq_150ma_a")<p("iq_10ma_a"))throw std::runtime_error("Invalid timestep, band, current, or thermal limits");
    if(std::abs(p("initial_offset_fraction"))>.005||std::abs(p("temperature_offset_fraction"))>.005)throw std::runtime_error("Accuracy corner outside configured envelope");
    const std::string profile=cfg_.at("calibration_profile").at("value"),initial=cfg_.at("initial_state").at("value"),src=cfg_.at("psrr_source").at("value");
    if(profile!="datasheet"&&profile!="va_static")throw std::runtime_error("Invalid calibration_profile");
    if(initial!="dc"&&initial!="uncharged")throw std::runtime_error("Invalid initial_state");
    if(src!="vin"&&src!="separate")throw std::runtime_error("Invalid psrr_source");
    cnr_mode_=cfg_.at("cnr_mode").at("value").get<int>();if(cnr_mode_!=0&&cnr_mode_!=1)throw std::runtime_error("cnr_mode must be 0 or 1");
    va_=profile=="va_static";separate_=src=="separate";noise_on_=cfg_.at("noise_enable").at("value").get<bool>();psrr_on_=cfg_.at("psrr_enable").at("value").get<bool>();
    if(noise_on_&&p("dt_s")>1/(20*p("noise_high_hz")))throw std::runtime_error("Noise timestep must sample highest band at least 20 times");
    rng_.seed(cfg_.at("noise_seed").at("value").get<unsigned>());
}
void TPS73133_3V3::set_attributes(){set_timestep(p("dt_s"),sc_core::SC_SEC);}
std::string TPS73133_3V3::get_module_name()const{return cfg_.at("name").at("value");}
double TPS73133_3V3::get_version()const{return p("version");}
std::string TPS73133_3V3::get_category()const{return cfg_.at("category").at("value");}
double TPS73133_3V3::get_timestep_s()const{return p("dt_s");}
double TPS73133_3V3::get_latency()const{return p("startup_reference_s");}
std::experimental::optional<double> TPS73133_3V3::get_area()const{return std::experimental::nullopt;}
double TPS73133_3V3::get_noise_rms()const{return p("vnom_v")*p(cnr_mode_?"noise_per_v_cnr":"noise_per_v_no_cnr");}
double TPS73133_3V3::get_noise_density()const{return get_noise_rms()/std::sqrt(p("noise_high_hz")-p("noise_low_hz"));}
void TPS73133_3V3::processing(){
    const double vi=vin.read(),ve=en.read(),load=iload.read(),rl=rload.read(),temp=temperature.read(),rip=ripple.read(),forced=vexternal.read();
    if(!std::isfinite(vi)||vi<0||!std::isfinite(ve)||ve<0||!std::isfinite(load)||load<0||!std::isfinite(rl)||rl<=0||!std::isfinite(temp)||!std::isfinite(rip)||!std::isfinite(forced)||forced< -1|| (forced<0&&forced!=-1))throw std::runtime_error("Invalid finite/nonnegative input, positive RLOAD or vexternal=-1/positive required");
    const double dt=p("dt_s"),c=p("cout_f"),esr=p("esr_ohm");
    if(temp>=p("tsd_trip_c"))thermal_ok_=false;else if(temp<=p("tsd_reset_c"))thermal_ok_=true;
    const double eg=ve<=.5?0:(ve>=1.7?1:gate(ve,p("en_mid_v"),p("en_width_v")));
    const double supply=gate(vi,p("uv_mid_v"),p("uv_width_v")); // VA low-input suppression, not a guaranteed UVLO threshold
    const double command=thermal_ok_?eg*supply:0;
    const bool dc_init=first_&&cfg_.at("initial_state").at("value")=="dc";
    if(first_){ss_=dc_init?command:0;vin_dc_=previous_vin_=vi;psrr_lp_=previous_psrr_=separate_?0:vi;}
    if(command<=0)ss_=0;
    else if(!first_){const double tau=cnr_mode_?p("startup_reference_s")/std::log(50.):p("startup_no_cnr_tau_s");ss_=command+(ss_-command)*std::exp(-dt/tau);}
    const bool slow_line=psrr_on_&&!separate_;
    const double line_vi=slow_line?vin_dc_:vi;
    const double nominal=p("vnom_v")*(1+p("initial_offset_fraction")+p("temperature_offset_fraction"))+p("line_gain_v_per_v")*(line_vi-p("vin_ref_v"));
    if(!first_){vin_dc_=lowpass(vin_dc_,previous_vin_,vi,p("line_pole_hz"),dt);}
    previous_vin_=vi;
    double target=ss_*nominal;
    if(va_){const double head=vi-p("vdo_va_v");target=ss_*.5*(nominal+head-std::sqrt((nominal-head)*(nominal-head)+1e-6));}
    const double g=va_?p("gleak_s"):1/p("rfeedback_ohm");
    auto limit=[&](double v){return p("isc_a")+(p("icl_a")-p("isc_a"))*gate(v,p("fold_mid_v"),p("fold_width_v"));};
    auto pass=[&](double v){
        double req;
        if(va_)req=limit(v)*std::tanh(std::max(0.,target-v)/(limit(v)*p("rout_va_ohm")));
        else {const double slope=v>=target?p("rout_low_ohm"):p("rout_high_ohm");req=std::max(0.,ss_*.01+(target-v)/slope+g*v);}
        // Fit both VDO=30mV at150mA and dVDO/dI=0.25ohm at that point.
        const double head=va_?1e9:p("rated_a")*std::pow(std::max(0.,vi-v)/(p("rated_a")*p("rdo_ohm")),p("rdo_ohm")/p("dropout_zo_ohm"));
        return command*std::max(0.,std::min({req,head,limit(v)}));
    };
    auto reverse=[&](double v){return eg==0?std::min(1.,std::max(0.,(v-vi)/.001))*p("reverse_a"):0.;};
    auto residual=[&](double v){return pass(v)-g*v-reverse(v)-load-v/rl-((c>0&&!dc_init)?(v-vc_)/(dt/c+esr):0.);};
    double lo=0,hi=std::max({vi,nominal+.1,vc_,0.});
    if(forced>=0)lo=hi=forced;
    else if(c==0||dc_init){
        double prev=hi,fp=residual(prev);bool found=false;
        for(int j=1;j<=160;++j){double v=hi*(1-j/160.),fv=residual(v);if(fv>=0&&fp<=0){lo=v;hi=prev;found=true;break;}prev=v;fp=fv;}
        if(!found)lo=hi=0;
    }else if(residual(0)<=0)hi=0;
    for(int j=0;j<48;++j){const double mid=(lo+hi)/2;if(residual(mid)>0)lo=mid;else hi=mid;}
    const double vo=(lo+hi)/2,ip=pass(vo),irev=reverse(vo),bleed=g*vo;
    const double cap_current=c>0&&!dc_init&&forced<0?(vo-vc_)/(dt/c+esr):0;
    if(c>0)vc_=dc_init||forced>=0?vo:std::max(0.,vo-esr*cap_current);else vc_=vo;
    const double external_load=load+vo/rl;
    const double iq=va_?p("iq_10ma_a"):p("iq_10ma_a")+(p("iq_150ma_a")-p("iq_10ma_a"))*std::max(0.,std::min(1.,(external_load-.01)/.14));
    const double bias=irev>0?0:command*std::max(0.,iq-bleed)+(1-command)*p("iq_off_a");
    const double ie=eg*p("ien_on_a")+(1-eg)*ve/p("ren_off_ohm");
    double perturb=0;
    if(psrr_on_){const double x=separate_?rip:vi;if(!first_)psrr_lp_=lowpass(psrr_lp_,previous_psrr_,x,p("psrr_fc_hz"),dt);previous_psrr_=x;perturb=command*p("psrr_h_inf")*(x-psrr_lp_);}
    double vn=0;if(noise_on_)vn=command*normal_(rng_)*get_noise_density()/std::sqrt(2*dt);
    if(forced>=0){perturb=0;vn=0;}
    const double iext=forced>=0?external_load+bleed+irev-ip:0;
    const double deficit=vo<1e-9&&forced<0?std::max(0.,external_load+irev-ip):0;
    last_power_=std::max(0.,(vi-vo)*ip+vi*bias+(vo-vi)*irev+vo*bleed+ve*ie);
    int code=command<.001?0:(ss_<.99*command?1:(ip>=.98*command*limit(vo)?3:(vo<.98*nominal?4:2)));
    if(deficit>1e-9)code=5;
    if(temp< -40||temp>125||vi>5.5)code=6;
    if(irev>0)code=8;
    if(!thermal_ok_)code=7;
    vout.write(vo+perturb+vn);vcore.write(vo);iin.write(ip+bias-irev);ignd.write(bias+bleed);ien.write(ie);ipass.write(ip);ireverse.write(irev);iexternal.write(iext);power.write(last_power_);noise.write(vn);psrr_voltage.write(perturb);unmet_load.write(deficit);state.write(code);first_=false;
}
