// Evidence: DS20005415D electrical tables pp.3-5; SHDN pp.19,21;
// VA baseline MCP1711_3V3_quick_function_v1.txt supplied by the user.
// Estimated/unknown parameters are individually identified in JSON.
#include "mcp1711_3v3.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace {
double smooth(double x,double mid,double width) { return .5*(1+std::tanh((x-mid)/width)); }
constexpr double pi=3.14159265358979323846;
}
double MCP1711_3V3::p(const char* k) const { return nums_.at(k); }
MCP1711_3V3::MCP1711_3V3(sc_core::sc_module_name name,const std::string& path):sca_tdf::sca_module(name) {
    std::ifstream f(path);
    if(!f) throw std::runtime_error("Cannot open configuration: "+path);
    f>>cfg_;
    for(auto it=cfg_.begin();it!=cfg_.end();++it) {
        if(it.value().is_object() && it.value().contains("value") && it.value()["value"].is_number()) {
            double v=it.value()["value"].get<double>();
            if(!std::isfinite(v)) throw std::runtime_error("Nonfinite parameter: "+it.key());
            nums_[it.key()]=v;
        }
    }
    for(const char* k:{"dt_s","vnom_v","rated_a","rout_ohm","rdo_ohm","icl_a","isc_a","rdis_ohm","shdn_width_v","uv_width_v","fold_width_v","startup_tau_s","psrr_fc_hz","noise_high_hz"})
        if(p(k)<=0) throw std::runtime_error(std::string("Expected positive parameter: ")+k);
    for(const char* k:{"cout_f","esr_ohm","iq_on_a","iq_off_a","ignd_rated_a","gleak_s","noise_rms_v","noise_low_hz","h_low","h_high"})
        if(p(k)<0) throw std::runtime_error(std::string("Expected nonnegative parameter: ")+k);
    if(p("isc_a")>p("icl_a") || p("rated_a")>p("icl_a")) throw std::runtime_error("Invalid current limits");
    if(p("ignd_rated_a")<p("iq_on_a")) throw std::runtime_error("Ground current must not decrease in this profile");
    if(p("noise_high_hz")<=p("noise_low_hz")) throw std::runtime_error("Invalid noise band");
    if(p("dt_s")>1e-5 || p("dt_s")<1e-10) throw std::runtime_error("dt_s outside supported numerical range");
    if(cfg_.at("noise_enable").at("value").get<bool>() && p("dt_s")>1/(20*p("noise_high_hz"))) throw std::runtime_error("Noise requires at least 20 samples per highest-band cycle");
    if(cfg_.at("psrr_enable").at("value").get<bool>() && p("dt_s")>1/(20*p("psrr_fc_hz"))) throw std::runtime_error("PSRR timestep too large");
    const std::string init=cfg_.at("initial_state").at("value");
    if(init!="dc" && init!="uncharged") throw std::runtime_error("initial_state must be dc or uncharged");
    const std::string drop=cfg_.at("dropout_profile").at("value");
    if(drop!="datasheet" && drop!="va_compat") throw std::runtime_error("Invalid dropout profile");
    if(p("shdn_low_v")>=p("shdn_high_v") || p("fold_arm_ratio")<=0 || p("fold_arm_ratio")>1) throw std::runtime_error("Invalid logic/arming parameters");
    if(p("temp_min_c")>=p("temp_max_c")) throw std::runtime_error("Invalid temperature range");
    if(std::abs(p("tempco_per_c"))>50e-6 || std::abs(p("vout_offset_fraction"))>.01) throw std::runtime_error("Accuracy-corner settings exceed configured datasheet envelope");
    rng_.seed(cfg_.at("noise_seed").at("value").get<unsigned>());
}
void MCP1711_3V3::set_attributes(){set_timestep(p("dt_s"),sc_core::SC_SEC);}
std::string MCP1711_3V3::get_module_name()const{return cfg_.at("name").at("value");}
double MCP1711_3V3::get_version()const{return p("version");}
std::string MCP1711_3V3::get_category()const{return cfg_.at("category").at("value");}
double MCP1711_3V3::get_timestep_s()const{return p("dt_s");}
std::experimental::optional<double> MCP1711_3V3::get_latency()const{return std::experimental::nullopt;}
std::experimental::optional<double> MCP1711_3V3::get_area()const{return std::experimental::nullopt;}
double MCP1711_3V3::get_noise_density()const{return p("noise_rms_v")/std::sqrt(p("noise_high_hz")-p("noise_low_hz"));}

void MCP1711_3V3::processing(){
    const double vi=vin.read(), ve=shdn.read(), load=iload.read(), rl=rload.read(), t=temperature.read(), rip=ripple.read();
    if(!std::isfinite(vi)||vi<0||!std::isfinite(ve)||ve<0||!std::isfinite(load)||load<0||!std::isfinite(rl)||rl<=0||!std::isfinite(t)||!std::isfinite(rip))
        throw std::runtime_error("Inputs require finite nonnegative VIN/SHDN/ILOAD, positive finite RLOAD, finite temperature/ripple");
    const double dt=p("dt_s"), c=p("cout_f"), esr=p("esr_ohm");
    // Guaranteed low/high regions, VA smooth approximation inside unspecified region.
    const double en=ve<=p("shdn_low_v")?0:(ve>=p("shdn_high_v")?1:smooth(ve,p("shdn_mid_v"),p("shdn_width_v")));
    // Low-input suppression is a behavioral guard, NOT a specified UVLO circuit.
    const double supply=smooth(vi,p("uv_mid_v"),p("uv_width_v"));
    const double command=en*supply;
    const bool dc_init=first_ && cfg_.at("initial_state").at("value")=="dc";
    if(first_) startup_=dc_init?command:0;
    else startup_=command+(startup_-command)*std::exp(-dt/p("startup_tau_s"));
    // Shutdown pass switch acts immediately; startup time constant is an explicit assumption.
    const double drive=std::min(command,startup_);
    if(command<.1) armed_=false;
    const double vreg=p("vnom_v")*(1+p("vout_offset_fraction")+p("tempco_per_c")*(t-25)+p("line_reg_per_v")*(vi-p("vin_ref_v")));
    auto limit=[&](double v){return armed_?p("isc_a")+(p("icl_a")-p("isc_a"))*smooth(v,p("fold_mid_v"),p("fold_width_v")):p("icl_a");};
    const bool va_drop=cfg_.at("dropout_profile").at("value")=="va_compat";
    auto head=[&](double v){
        const double dv=std::max(0.,vi-v);
        if(va_drop) return dv/p("rdo_ohm");
        // Inverse piecewise VDO(I), through 0.17 V/50 mA and 0.50 V/150 mA.
        // Both table points use the manufacturer's 2%-below-reference criterion.
        return dv<=.17?dv*(.05/.17):.05+(dv-.17)*(.10/.33);
    };
    auto pass=[&](double v){return drive*std::max(0.,std::min({std::max(0.,vreg-v)/p("rout_ohm"),head(v),limit(v)}));};
    auto discharge=[&](double v){return (1-command)*v/p("rdis_ohm");};
    auto residual=[&](double v){return pass(v)-discharge(v)-p("gleak_s")*v-load-v/rl-((c>0&&!dc_init)?(v-capacitor_v_)/(dt/c+esr):0.);};
    double lo=0, hi=std::max({vi,vreg,capacitor_v_,0.});
    // Highest stable positive root avoids spurious low-voltage DC foldback roots.
    if(c==0 || dc_init){
        double prev=hi, fp=residual(prev);bool found=false;
        for(int j=1;j<=160;++j){const double v=hi*(1-j/160.0),fv=residual(v);if(fv>=0&&fp<=0){lo=v;hi=prev;found=true;break;}prev=v;fp=fv;}
        if(!found) hi=lo=0;
    }else if(residual(0)<=0) hi=0;
    for(int j=0;j<50;++j){double mid=(lo+hi)/2;if(residual(mid)>0)lo=mid;else hi=mid;}
    const double vo=(lo+hi)/2, ip=pass(vo), id=discharge(vo);
    if(c>0) capacitor_v_=dc_init?vo:std::max(0.,vo-esr*(vo-capacitor_v_)/(dt/c+esr));
    else capacitor_v_=vo;
    if(command>.9 && startup_>.99*command && vo>=p("fold_arm_ratio")*p("vnom_v")) armed_=true;
    // Table IQ uses IOUT=1mA by its default condition; the 45uA endpoint is VA-only.
    const double load_fraction=std::max(0.,std::min((ip-.001)/(p("rated_a")-.001),1.));
    const double iq=drive*(p("iq_on_a")+(p("ignd_rated_a")-p("iq_on_a"))*load_fraction)+(1-drive)*p("iq_off_a");
    const double ie=en*p("ishdn_on_a")+(1-en)*ve/1e7;
    double perturb=0;
    if(cfg_.at("psrr_enable").at("value").get<bool>()){
        const double a=std::exp(-2*pi*p("psrr_fc_hz")*dt);
        ripple_lp_=a*ripple_lp_+(1-a)*rip;
        perturb=drive*(p("h_high")*rip+(p("h_low")-p("h_high"))*ripple_lp_);
    }
    double vn=0;
    if(cfg_.at("noise_enable").at("value").get<bool>()) vn=drive*normal_(rng_)*get_noise_density()/std::sqrt(2*dt);
    // Internal chip losses include active discharge, output leakage and SHDN input.
    // External capacitor/ESR, load and ideal small-signal sources are excluded.
    last_power_=std::max(0.,(vi-vo)*ip+vi*iq+vo*id+vo*vo*p("gleak_s")+ve*ie);
    const double deficit=vo<1e-9?std::max(0.,load-ip):0;
    int code=command<.001?0:(startup_<.99*command?1:(ip>.98*drive*limit(vo)?3:(vo<.98*vreg?4:2)));
    if(deficit>1e-9) code=5;
    if(t<p("temp_min_c") || t>p("temp_max_c") || vi>6 || ve>6) code=6;
    vout.write(vo+perturb+vn);iin.write(ip+iq);ignd.write(iq);ishdn.write(ie);ipass.write(ip);idischarge.write(id);power.write(last_power_);noise.write(vn);unmet_load.write(deficit);state.write(code);
    first_=false;
}
