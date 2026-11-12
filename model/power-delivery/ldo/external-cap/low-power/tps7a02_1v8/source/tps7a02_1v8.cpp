// Key source references: TI TPS7A02 SBVS277C Rev.C sections 6.5-6.6; tps7a02_1v8_core_v2.txt and TPS7A02_psrr_v1.txt; originals in verification annex models/tps7a02_1v8/reference.
// References: TI TPS7A02 SBVS277C Rev.C, sections 6.5-6.6; tps7a02_1v8_core_v2; TPS7A02_psrr_v1.
#include "tps7a02_1v8.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <random>
namespace {
constexpr bool external_cap = true;
constexpr double pi = 3.14159265358979323846;
double gate(double x, double mid, double width) { return .5*(1+std::tanh((x-mid)/width)); }
double foh(double old, double previous, double input, double frequency, double dt) {
    const double x=2*pi*frequency*dt, a=std::exp(-x), b=-std::expm1(-x)/x;
    return a*old+(1-b)*input+(b-a)*previous;
}
}
double TPS7A02_1V8::p(const char* key) const { return numeric_.at(key); }
TPS7A02_1V8::TPS7A02_1V8(sc_core::sc_module_name name,const std::string& path):sca_tdf::sca_module(name) {
    std::ifstream f(path);
    if(!f) throw std::runtime_error("Cannot read configuration: "+path);
    f>>cfg_;
    for(auto it=cfg_.begin();it!=cfg_.end();++it) { if(it.value().is_object() && it.value().contains("value") && it.value()["value"].is_number()) numeric_[it.key()]=it.value()["value"].get<double>(); }
    for(const char* k:{"dt_s","vnom_v","rated_a","icl_a","isc_a","rout_ohm","tss_s","tdisable_s","tline_s","treg_s","ren_ohm","fold_slope_v","psrr_fc_hz","noise_high_hz","latency_s"}) {
        if(!std::isfinite(p(k)) || p(k)<=0) throw std::runtime_error(std::string("Expected positive parameter: ")+k);
    }
    for(const char* k:{"cout_f","esr_ohm","iq_on_a","iq_off_a","noise_rms_v","noise_low_hz","gleak_s","rdis_ohm","rov_ohm","dc_remove_hz"}) {
        if(!std::isfinite(p(k)) || p(k)<0) throw std::runtime_error(std::string("Expected nonnegative parameter: ")+k);
    }
    for(auto it=cfg_.begin();it!=cfg_.end();++it) {
        if(it.value().is_object() && it.value().contains("value") && it.value()["value"].is_number() && !std::isfinite(it.value()["value"].get<double>())) throw std::runtime_error("Nonfinite configuration parameter");
    }
    if(external_cap && (p("cout_f")<=0 || p("rdo_ohm")<=0 || p("rdis_ohm")<=0 || p("rov_ohm")<=0)) throw std::runtime_error("External-cap model requires positive COUT/RDO/RDIS/ROV");
    if(!external_cap && (p("ven_fall_v")<=0 || p("uvlo_fall_v")<=0 || p("tsd_on_c")<=0)) throw std::runtime_error("Smooth transition widths must be positive");
    if(p("noise_high_hz")<=p("noise_low_hz") || p("h_low")<=0 || p("h_high")<p("h_low") || p("isc_a")>p("icl_a")) throw std::runtime_error("Invalid noise/PSRR/current-limit range");
    if(external_cap && (p("ven_rise_v")<=p("ven_fall_v") || p("uvlo_rise_v")<=p("uvlo_fall_v") || p("tsd_off_c")<=p("tsd_on_c"))) throw std::runtime_error("Invalid hysteresis thresholds");
    const std::string mode=cfg_.at("mode").at("value"), init=cfg_.at("initial_state").at("value");
    if((mode!="transient" && mode!="ripple") || (init!="dc" && init!="uncharged")) throw std::runtime_error("Invalid mode or initial_state");
    const int n=cfg_.at("noise_tones").at("value").get<int>();
    if(n<8 || n>1024) throw std::runtime_error("noise_tones must be 8..1024");
    if(cfg_.at("noise_enable").at("value").get<bool>() && p("dt_s")*p("noise_high_hz")>.05) throw std::runtime_error("Noise sampling requires dt <= 1/(20*fhigh)");
    const std::string noise_model=cfg_.at("noise_model").at("value");
    if(noise_model!="gaussian_white" && noise_model!="multisine") throw std::runtime_error("Unknown noise_model");
    noise_rng_.seed(cfg_.at("noise_seed").at("value").get<unsigned>());
    std::mt19937 rng(cfg_.at("noise_seed").at("value").get<unsigned>());
    for(int i=0;i<n;++i) phases_.push_back(2*pi*double(rng())/double(std::mt19937::max()));
}
void TPS7A02_1V8::set_attributes(){ set_timestep(p("dt_s"),sc_core::SC_SEC); }
std::string TPS7A02_1V8::get_module_name() const { return cfg_.at("name").at("value"); }
double TPS7A02_1V8::get_version() const { return p("version"); }
double TPS7A02_1V8::get_latency() const { return p("latency_s"); }
double TPS7A02_1V8::get_timestep_s() const { return p("dt_s"); }
std::optional<double> TPS7A02_1V8::get_area() const { const auto& v=cfg_.at("area_mm2").at("value"); return v.is_null()?std::nullopt:std::optional<double>(v.get<double>()); }
double TPS7A02_1V8::get_noise_density() const { return p("noise_rms_v")/std::sqrt(p("noise_high_hz")-p("noise_low_hz")); }
void TPS7A02_1V8::processing(){
    const double param_dt_s=p("dt_s");
    const double param_cout_f=p("cout_f");
    const double param_esr_ohm=p("esr_ohm");
    const double param_ven_rise_v=p("ven_rise_v");
    const double param_ven_fall_v=p("ven_fall_v");
    const double param_uvlo_rise_v=p("uvlo_rise_v");
    const double param_uvlo_fall_v=p("uvlo_fall_v");
    const double param_tsd_off_c=p("tsd_off_c");
    const double param_tsd_on_c=p("tsd_on_c");
    const double param_tss_s=p("tss_s");
    const double param_tdisable_s=p("tdisable_s");
    const double param_vnom_v=p("vnom_v");
    const double param_line_gain=p("line_gain");
    const double param_vin_ref_v=p("vin_ref_v");
    const double param_line_trans_gain=p("line_trans_gain");
    const double param_tline_s=p("tline_s");
    const double param_vdo_v=p("vdo_v");
    const double param_treg_s=p("treg_s");
    const double param_isc_a=p("isc_a");
    const double param_icl_a=p("icl_a");
    const double param_fold_mid_v=p("fold_mid_v");
    const double param_fold_slope_v=p("fold_slope_v");
    const double param_rout_ohm=p("rout_ohm");
    const double param_rdo_ohm=p("rdo_ohm");
    const double param_gleak_s=p("gleak_s");
    const double param_rov_ohm=p("rov_ohm");
    const double param_rdis_ohm=p("rdis_ohm");
    const double param_fold_arm_fraction=p("fold_arm_fraction");
    const double param_iq1_a=p("iq1_a");
    const double param_iq100_a=p("iq100_a");
    const double param_iq200_a=p("iq200_a");
    const double param_iq_on_a=p("iq_on_a");
    const double param_iq_off_a=p("iq_off_a");
    const double param_dc_remove_hz=p("dc_remove_hz");
    const double param_psrr_fc_hz=p("psrr_fc_hz");
    const double param_h_high=p("h_high");
    const double param_h_low=p("h_low");
    const double param_noise_high_hz=p("noise_high_hz");
    const double param_noise_low_hz=p("noise_low_hz");
    const double param_noise_rms_v=p("noise_rms_v");
    const double param_ren_ohm=p("ren_ohm");
    const double param_ien_a=p("ien_a");

    const double vi=vin.read(), ve=en.read(), load=iload.read(), rl=rload.read(), temp=temperature.read(), rip=ripple.read();
    if(!std::isfinite(vi)||!std::isfinite(ve)||!std::isfinite(load)||!std::isfinite(rl)||!std::isfinite(temp)||!std::isfinite(rip)||vi<0||ve<0||load<0||rl<=0) throw std::runtime_error("Invalid input: finite VIN/EN/ILOAD>=0, RLOAD>0, temperature/ripple finite required");
    const double dt=param_dt_s, c=param_cout_f, esr=param_esr_ohm;
    const bool ac=cfg_.at("mode").at("value")=="ripple";
    double command;
    if(external_cap){
        if(first_) { en_on_=ve>=.5*(param_ven_rise_v+param_ven_fall_v); uv_on_=vi>=.5*(param_uvlo_rise_v+param_uvlo_fall_v); thermal_ok_=temp<param_tsd_off_c; }
        if(ve>=param_ven_rise_v) en_on_=true;
        else if(ve<=param_ven_fall_v) en_on_=false;
        if(vi>=param_uvlo_rise_v) uv_on_=true;
        else if(vi<=param_uvlo_fall_v) uv_on_=false;
        if(temp>=param_tsd_off_c) thermal_ok_=false;
        else if(temp<=param_tsd_on_c) thermal_ok_=true;
        command=(en_on_&&uv_on_&&thermal_ok_)?1.:0.;
        if(command==0) armed_=false;
    }else{ command=gate(ve,param_ven_rise_v,param_ven_fall_v); }
    const bool dc_init=first_ && cfg_.at("initial_state").at("value")=="dc";
    if(first_) { ss_=dc_init?command:0.; vin_lp_=vi; }
    // The first sample represents t=0; subsequent samples advance states by dt.
    if(!first_) ss_=command+(ss_-command)*std::exp(-dt/(command>ss_?param_tss_s:param_tdisable_s));
    double drive=external_cap?command:ss_*gate(vi,param_uvlo_rise_v,param_uvlo_fall_v)*(1-gate(temp,param_tsd_off_c,param_tsd_on_c));
    const double line=param_vnom_v+param_line_gain*(vi-param_vin_ref_v);
    const double line_dyn=(external_cap&&!ac)?param_line_trans_gain*(vi-vin_lp_):0.;
    if(!first_) vin_lp_=vi+(vin_lp_-vi)*std::exp(-dt/param_tline_s);
    const double target=external_cap?ss_*(line+line_dyn):drive*std::min(line,std::max(0.,vi-param_vdo_v));
    const double k=dt/param_treg_s;
    auto limit=[&](double v){ return (!external_cap||armed_)?param_isc_a+(param_icl_a-param_isc_a)*gate(v,param_fold_mid_v,param_fold_slope_v):param_icl_a; };
    auto pass=[&](double v){
        const double lim=limit(v);
        if(external_cap){
            const double er=dc_init?target-v:(err_+k*(target-v))/(1+k);
            const double req=std::max(0.,er)/param_rout_ohm;
            return drive*std::max(0.,std::min({req,lim,std::max(0.,(vi-v)/param_rdo_ohm)}));
        }
        return drive*lim*std::tanh(std::max(0.,target-v)/(lim*param_rout_ohm));
    };
    auto sink=[&](double v){return param_gleak_s*v+(external_cap?std::max(0.,v-target)/param_rov_ohm+(1-command)*v/param_rdis_ohm:0.);};
    auto residual=[&](double v){return pass(v)-sink(v)-load-v/rl-((c>0&&!dc_init)?(v-vc_)/(dt/c+esr):0.);};
    double lo=0,hi=std::max({vi,param_vnom_v*1.5,vc_,target});
    // Algebraic capless/foldback branch: select the highest stable positive root.
    if(c==0 || dc_init){
        double previous=hi, fp=residual(previous);bool found=false;
        for(int j=1;j<=128;++j){double v=hi*(1-double(j)/128),fv=residual(v);if(fv>=0&&fp<=0){lo=v;hi=previous;found=true;break;}previous=v;fp=fv;}
        if(!found){lo=0;hi=0;}
    }else if(residual(0)<=0){hi=0;}
    for(int j=0;j<48;++j){double mid=.5*(lo+hi);if(residual(mid)>0)lo=mid;else hi=mid;}
    const double vo=.5*(lo+hi), ip=pass(vo);
    if(external_cap) err_=dc_init?target-vo:(err_+k*(target-vo))/(1+k);
    if(c>0) vc_=dc_init?vo:std::max(0.,vo-esr*(vo-vc_)/(dt/c+esr));
    else vc_=vo;
    if(external_cap&&command>0&&vo>=param_fold_arm_fraction*param_vnom_v) armed_=true;
    double iq;
    if(external_cap){
        double extra;
        if(ip<=.001)extra=param_iq1_a*ip/.001;
        else if(ip<=.1)extra=param_iq1_a+(param_iq100_a-param_iq1_a)*(ip-.001)/.099;
        else extra=param_iq100_a+(param_iq200_a-param_iq100_a)*(ip-.1)/.1;
        iq=command*(param_iq_on_a+extra)+(1-command)*param_iq_off_a;
    }else iq=drive*param_iq_on_a+(1-drive)*param_iq_off_a;
    double perturb=0.;
    if(ac || !external_cap){
        const double signal=ac?rip:vi;
        if(first_ && !ac) { ripple_dc_=vi; previous_ripple_=vi; }
        double input=signal;
        if(param_dc_remove_hz>0){ripple_dc_=foh(ripple_dc_,previous_ripple_,signal,param_dc_remove_hz,dt);input=signal-ripple_dc_;}
        ripple_lp_=foh(ripple_lp_,previous_ac_,input,param_psrr_fc_hz,dt);
        perturb=param_h_high*input+(param_h_low-param_h_high)*ripple_lp_;
        previous_ripple_=signal;previous_ac_=input;
    }
    double vn=0.;
    if(cfg_.at("noise_enable").at("value").get<bool>()){
        if(cfg_.at("noise_model").at("value")=="gaussian_white") {
            // One-sided flat PSD: sigma^2 = density^2 * Nyquist bandwidth.
            // The configured RMS refers to [flow,fhigh], not total sample variance.
            vn=normal_(noise_rng_)*get_noise_density()/std::sqrt(2*dt)*drive;
        }else{
        const double df=(param_noise_high_hz-param_noise_low_hz)/phases_.size();
        const double t=get_time().to_seconds();
        for(size_t j=0;j<phases_.size();++j)vn+=std::sin(2*pi*(param_noise_low_hz+(j+.5)*df)*t+phases_[j]);
        vn*=param_noise_rms_v*std::sqrt(2./phases_.size())*drive;
        }
    }
    // Power excludes the external load/capacitor and the ideal ripple/noise sources.
    last_power_=std::max(0.,(vi-vo)*ip+vi*iq);
    const double ie=external_cap?ve/param_ren_ohm:command*param_ien_a+(1-command)*ve/param_ren_ohm;
    int code=drive<.001?0:(ss_<.99?1:(ip>=.98*limit(vo)?3:(vo<.95*line?4:2)));
    vout.write(vo+drive*perturb+vn);iin.write(ip+iq);ignd.write(iq);ien.write(ie);power.write(last_power_);noise.write(vn);state.write(code);
    first_=false;
}
