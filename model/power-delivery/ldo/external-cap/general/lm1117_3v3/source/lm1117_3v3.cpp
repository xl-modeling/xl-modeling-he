// Key source references: TI LM1117 SNOS412Q Rev.Q; lm1117_core_verified.va and lm1117_psrr.va; full originals in verification annex models/lm1117_3v3/reference and doc.
#include "lm1117_3v3.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

double LM1117_3V3::p(const char* k) const { return cfg_.at(k).at("value").get<double>(); }
LM1117_3V3::LM1117_3V3(sc_core::sc_module_name n,const std::string& path):sca_tdf::sca_module(n) {
    std::ifstream f(path);
    if(!f) throw std::runtime_error("Cannot open config: "+path);
    f >> cfg_;
    for(const char* k:{"dt_s","cout_f","rout_ohm","rdo_ohm","icl_a","treg_s","tload_s","fp_hz","ftran_hz","iq_slope_v"})
        if(!std::isfinite(p(k)) || p(k)<=0) throw std::runtime_error(std::string("Invalid positive parameter: ")+k);
    for(const char* k:{"vnom_v","vin_ref_v","line_gain","iq_a","vdo_offset_v","rload_dyn_ohm","gmin_s","esr_ohm","h_low","h_inf","iq_on_v"})
        if(!std::isfinite(p(k)) || p(k)<0) throw std::runtime_error(std::string("Invalid nonnegative parameter: ")+k);
    if(p("h_low")<=0 || p("h_inf")<p("h_low")) throw std::runtime_error("Invalid PSRR gains");
    std::string mode=cfg_.at("mode").at("value");
    if(mode!="transient" && mode!="ripple") throw std::runtime_error("mode must be transient or ripple");
    std::string init=cfg_.at("initial_state").at("value");
    if(init!="dc" && init!="uncharged") throw std::runtime_error("Invalid initial_state");
}
void LM1117_3V3::set_attributes(){set_timestep(p("dt_s"),sc_core::SC_SEC);}
std::string LM1117_3V3::get_module_name() const{return cfg_.at("name").at("value");}
double LM1117_3V3::get_version() const{return p("version");}
std::optional<double> LM1117_3V3::get_area() const{
    const auto& v=cfg_.at("area_mm2").at("value");
    return v.is_null()?std::nullopt:std::optional<double>(v.get<double>());
}
std::optional<double> LM1117_3V3::get_latency() const{
    const auto& v=cfg_.at("latency_s").at("value");
    return v.is_null()?std::nullopt:std::optional<double>(v.get<double>());
}
double LM1117_3V3::get_timestep_s() const{return p("dt_s");}
void LM1117_3V3::processing(){
    const double vi=vin.read(), load=iload.read(), rl=rload.read(), rip=ripple.read();
    if(!std::isfinite(vi)||vi<0||!std::isfinite(load)||load<0||!std::isfinite(rl)||rl<=0||!std::isfinite(rip))
        throw std::runtime_error("Inputs require finite VIN>=0, ILOAD>=0, RLOAD>0, ripple finite");
    const bool ac=cfg_.at("mode").at("value")=="ripple";
    const double dt=p("dt_s"), c=p("cout_f"), esr=p("esr_ohm"), g=p("gmin_s");
    const double target=std::max(0.,p("vnom_v")+p("line_gain")*(vi-p("vin_ref_v")));
    if(first_){
        reg_=target; lp_v_=ac?0:vi;
        if(cfg_.at("initial_state").at("value")=="dc"){
            double lo=0,hi=std::max(vi,target);
            for(int k=0;k<70;k++){
                double v=(lo+hi)/2;
                double ip=std::max(0.,std::min({(reg_-v)/p("rout_ohm"),(vi-v-p("vdo_offset_v"))/p("rdo_ohm"),p("icl_a")}));
                if(ip>load+v/rl+g*v)lo=v;else hi=v;
            }
            vc_=(lo+hi)/2; lp_i_=load+vc_/rl+g*vc_;
        }
        first_=false;
    }
    // Exact first-order update for held command. Engineering time constant from VA.
    reg_=target+(reg_-target)*std::exp(-dt/p("treg_s"));
    const double freq=ac?p("fp_hz")*p("h_inf")/p("h_low"):p("ftran_hz");
    const double a=std::exp(-2*std::acos(-1.)*freq*dt);
    const double input=ac?rip:vi;
    // Evaluate feedthrough at current sample, then advance low-pass state.
    double series;
    if(ac){
        // Exact first-order-hold integration between sampled ripple values.
        // Avoids a one-sample delay corrupting low-frequency PSRR cancellation.
        const double x=2*std::acos(-1.)*freq*dt;
        const double b=-std::expm1(-x)/x;
        lp_v_=a*lp_v_+(1-b)*input+(b-a)*previous_ripple_;
        previous_ripple_=input;
        series=p("h_inf")*input+(p("h_low")-p("h_inf"))*lp_v_;
    }else{
        series=p("h_inf")*(input-lp_v_);
        lp_v_=input+(lp_v_-input)*a;
    }
    const double ai=std::exp(-dt/p("tload_s"));
    const double z=dt/c+esr;
    auto voltage=[&](double ip){return std::max(0.,(vc_+z*(ip-load+g*series))/(1+z*(1/rl+g)));};
    auto residual=[&](double ip){
        double core=voltage(ip)-series;
        double dyn=ac?0:p("rload_dyn_ohm")*ai*(ip-lp_i_);
        double demand=std::max(0.,std::min({(reg_-dyn-core)/p("rout_ohm"),(vi-core-p("vdo_offset_v"))/p("rdo_ohm"),p("icl_a")}));
        return ip-demand;
    };
    double lo=0,hi=p("icl_a");
    for(int k=0;k<60;k++){double mid=(lo+hi)/2;if(residual(mid)>0)hi=mid;else lo=mid;}
    double ip=(lo+hi)/2, vo=voltage(ip);
    double ic=ip-load-vo/rl-g*(vo-series);
    vc_=std::max(0.,vo-esr*ic);
    lp_i_=ip+(lp_i_-ip)*ai;
    double iq=p("iq_a")*.5*(1+std::tanh((vi-p("iq_on_v"))/p("iq_slope_v")));
    // Chip dissipation excludes capacitor stored energy and ESR loss.
    last_power_=std::max(0.,(vi-(vo-series))*ip+vi*iq);
    vout.write(vo); iin.write(ip+iq); ignd.write(iq); power.write(last_power_);
}
