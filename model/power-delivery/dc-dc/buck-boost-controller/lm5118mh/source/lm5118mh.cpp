// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#include "lm5118mh.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
// Migration of supplied Rev6 equations, not an independent transistor model.
// Linear edge transitions replace Verilog-A transition interpolation.
namespace lm5118mh {
namespace {
double clamp(double x,double a,double b) {return std::max(a,std::min(b,x));}
void latch(bool& s,double x,double rise,double fall) {if(x>=rise)s=true;else if(x<=fall)s=false;}
double slew(double x,bool high,double dt,double rise,double fall) {return clamp(x+(high?dt/rise:-dt/fall),0,1);}
}
nlohmann::json load_config(const std::string& path) {
    std::ifstream f(path); if(!f)throw std::runtime_error("Cannot open LM5118MH configuration: "+path);
    nlohmann::json j; f>>j;
    if(j.at("name").at("value")!="LM5118MH")throw std::invalid_argument("Wrong model name");
    return j;
}
Parameters::Parameters(const nlohmann::json& j) {
    VREF=j.at("VREF").at("value").get<double>(); if(!std::isfinite(VREF))throw std::invalid_argument("Nonfinite VREF");
    VCC_REG=j.at("VCC_REG").at("value").get<double>(); if(!std::isfinite(VCC_REG))throw std::invalid_argument("Nonfinite VCC_REG");
    VCC_ILIM=j.at("VCC_ILIM").at("value").get<double>(); if(!std::isfinite(VCC_ILIM))throw std::invalid_argument("Nonfinite VCC_ILIM");
    UVLO_RISE=j.at("UVLO_RISE").at("value").get<double>(); if(!std::isfinite(UVLO_RISE))throw std::invalid_argument("Nonfinite UVLO_RISE");
    UVLO_FALL=j.at("UVLO_FALL").at("value").get<double>(); if(!std::isfinite(UVLO_FALL))throw std::invalid_argument("Nonfinite UVLO_FALL");
    EN_RISE=j.at("EN_RISE").at("value").get<double>(); if(!std::isfinite(EN_RISE))throw std::invalid_argument("Nonfinite EN_RISE");
    EN_FALL=j.at("EN_FALL").at("value").get<double>(); if(!std::isfinite(EN_FALL))throw std::invalid_argument("Nonfinite EN_FALL");
    VCC_RISE=j.at("VCC_RISE").at("value").get<double>(); if(!std::isfinite(VCC_RISE))throw std::invalid_argument("Nonfinite VCC_RISE");
    VCC_FALL=j.at("VCC_FALL").at("value").get<double>(); if(!std::isfinite(VCC_FALL))throw std::invalid_argument("Nonfinite VCC_FALL");
    VCCX_RISE=j.at("VCCX_RISE").at("value").get<double>(); if(!std::isfinite(VCCX_RISE))throw std::invalid_argument("Nonfinite VCCX_RISE");
    VCCX_FALL=j.at("VCCX_FALL").at("value").get<double>(); if(!std::isfinite(VCCX_FALL))throw std::invalid_argument("Nonfinite VCCX_FALL");
    ISS=j.at("ISS").at("value").get<double>(); if(!std::isfinite(ISS))throw std::invalid_argument("Nonfinite ISS");
    IQ_RUN=j.at("IQ_RUN").at("value").get<double>(); if(!std::isfinite(IQ_RUN))throw std::invalid_argument("Nonfinite IQ_RUN");
    IQ_OFF=j.at("IQ_OFF").at("value").get<double>(); if(!std::isfinite(IQ_OFF))throw std::invalid_argument("Nonfinite IQ_OFF");
    TON_MIN=j.at("TON_MIN").at("value").get<double>(); if(!std::isfinite(TON_MIN))throw std::invalid_argument("Nonfinite TON_MIN");
    TOFF_MIN=j.at("TOFF_MIN").at("value").get<double>(); if(!std::isfinite(TOFF_MIN))throw std::invalid_argument("Nonfinite TOFF_MIN");
    T_RISE_HO=j.at("T_RISE_HO").at("value").get<double>(); if(!std::isfinite(T_RISE_HO))throw std::invalid_argument("Nonfinite T_RISE_HO");
    T_FALL_HO=j.at("T_FALL_HO").at("value").get<double>(); if(!std::isfinite(T_FALL_HO))throw std::invalid_argument("Nonfinite T_FALL_HO");
    T_RISE_LO=j.at("T_RISE_LO").at("value").get<double>(); if(!std::isfinite(T_RISE_LO))throw std::invalid_argument("Nonfinite T_RISE_LO");
    T_FALL_LO=j.at("T_FALL_LO").at("value").get<double>(); if(!std::isfinite(T_FALL_LO))throw std::invalid_argument("Nonfinite T_FALL_LO");
    VCS_BUCK=j.at("VCS_BUCK").at("value").get<double>(); if(!std::isfinite(VCS_BUCK))throw std::invalid_argument("Nonfinite VCS_BUCK");
    VCS_BB=j.at("VCS_BB").at("value").get<double>(); if(!std::isfinite(VCS_BB))throw std::invalid_argument("Nonfinite VCS_BB");
    GM_ERR=j.at("GM_ERR").at("value").get<double>(); if(!std::isfinite(GM_ERR))throw std::invalid_argument("Nonfinite GM_ERR");
    ICOMP_MAX=j.at("ICOMP_MAX").at("value").get<double>(); if(!std::isfinite(ICOMP_MAX))throw std::invalid_argument("Nonfinite ICOMP_MAX");
    R_DRIVER=j.at("R_DRIVER").at("value").get<double>(); if(!std::isfinite(R_DRIVER))throw std::invalid_argument("Nonfinite R_DRIVER");
    R_VCC=j.at("R_VCC").at("value").get<double>(); if(!std::isfinite(R_VCC))throw std::invalid_argument("Nonfinite R_VCC");
    R_SS_DIS=j.at("R_SS_DIS").at("value").get<double>(); if(!std::isfinite(R_SS_DIS))throw std::invalid_argument("Nonfinite R_SS_DIS");
    R_UVLO_HIC=j.at("R_UVLO_HIC").at("value").get<double>(); if(!std::isfinite(R_UVLO_HIC))throw std::invalid_argument("Nonfinite R_UVLO_HIC");
    TJ_C=j.at("TJ_C").at("value").get<double>(); if(!std::isfinite(TJ_C))throw std::invalid_argument("Nonfinite TJ_C");
    FSW_HZ=j.at("FSW_HZ").at("value").get<double>(); if(!std::isfinite(FSW_HZ))throw std::invalid_argument("Nonfinite FSW_HZ");
    USE_RT_PIN=j.at("USE_RT_PIN").at("value").get<double>(); if(!std::isfinite(USE_RT_PIN))throw std::invalid_argument("Nonfinite USE_RT_PIN");
    COMP_LOW=j.at("COMP_LOW").at("value").get<double>(); if(!std::isfinite(COMP_LOW))throw std::invalid_argument("Nonfinite COMP_LOW");
    COMP_HIGH=j.at("COMP_HIGH").at("value").get<double>(); if(!std::isfinite(COMP_HIGH))throw std::invalid_argument("Nonfinite COMP_HIGH");
    R_COMP_CLAMP=j.at("R_COMP_CLAMP").at("value").get<double>(); if(!std::isfinite(R_COMP_CLAMP))throw std::invalid_argument("Nonfinite R_COMP_CLAMP");
    STEPS_PER_CYCLE=j.at("STEPS_PER_CYCLE").at("value").get<double>(); if(!std::isfinite(STEPS_PER_CYCLE))throw std::invalid_argument("Nonfinite STEPS_PER_CYCLE");
    CS_PWM_GAIN=j.at("CS_PWM_GAIN").at("value").get<double>(); if(!std::isfinite(CS_PWM_GAIN))throw std::invalid_argument("Nonfinite CS_PWM_GAIN");
    PWM_OFFSET=j.at("PWM_OFFSET").at("value").get<double>(); if(!std::isfinite(PWM_OFFSET))throw std::invalid_argument("Nonfinite PWM_OFFSET");
    R_RAMP_RESET=j.at("R_RAMP_RESET").at("value").get<double>(); if(!std::isfinite(R_RAMP_RESET))throw std::invalid_argument("Nonfinite R_RAMP_RESET");
    T_RAMP_SWITCH=j.at("T_RAMP_SWITCH").at("value").get<double>(); if(!std::isfinite(T_RAMP_SWITCH))throw std::invalid_argument("Nonfinite T_RAMP_SWITCH");
    DT=j.at("DT").at("value").get<double>(); if(!std::isfinite(DT))throw std::invalid_argument("Nonfinite DT");

    if(DT<=0 || DT>2e-9 || FSW_HZ<50e3 || FSW_HZ>500e3 ||
       TON_MIN<=0 || TOFF_MIN<=0 || TON_MIN+TOFF_MIN>=1./500e3 ||
       T_RISE_HO<=0 || T_FALL_HO<=0 || T_RISE_LO<=0 || T_FALL_LO<=0 ||
       T_RAMP_SWITCH<=0 || R_DRIVER<=0 || R_VCC<=0 || R_SS_DIS<=0 ||
       R_UVLO_HIC<=0 || R_COMP_CLAMP<=0 || R_RAMP_RESET<=0 ||
       COMP_HIGH<=COMP_LOW || VREF<=0 || VCC_REG<=0 || VCC_ILIM<=0 ||
       ICOMP_MAX<=0 || GM_ERR<=0 || ISS<0 || IQ_RUN<0 || IQ_OFF<0 ||
       CS_PWM_GAIN<=0 || VCS_BUCK<=0 || VCS_BB<VCS_BUCK ||
       EN_RISE<=EN_FALL || UVLO_RISE<=UVLO_FALL || VCC_RISE<=VCC_FALL ||
       VCCX_RISE<=VCCX_FALL || STEPS_PER_CYCLE<32 || std::floor(STEPS_PER_CYCLE)!=STEPS_PER_CYCLE ||
       (USE_RT_PIN!=0 && USE_RT_PIN!=1))throw std::invalid_argument("Invalid LM5118MH parameters");
}
Output Engine::sample(double t,const Inputs& in) {
    if(!std::isfinite(in.vin))throw std::invalid_argument("Nonfinite vin");
    if(!std::isfinite(in.uvlo))throw std::invalid_argument("Nonfinite uvlo");
    if(!std::isfinite(in.rt_current))throw std::invalid_argument("Nonfinite rt_current");
    if(!std::isfinite(in.en))throw std::invalid_argument("Nonfinite en");
    if(!std::isfinite(in.ramp))throw std::invalid_argument("Nonfinite ramp");
    if(!std::isfinite(in.ss))throw std::invalid_argument("Nonfinite ss");
    if(!std::isfinite(in.fb))throw std::invalid_argument("Nonfinite fb");
    if(!std::isfinite(in.comp))throw std::invalid_argument("Nonfinite comp");
    if(!std::isfinite(in.vout))throw std::invalid_argument("Nonfinite vout");
    if(!std::isfinite(in.sync))throw std::invalid_argument("Nonfinite sync");
    if(!std::isfinite(in.cs))throw std::invalid_argument("Nonfinite cs");
    if(!std::isfinite(in.csg))throw std::invalid_argument("Nonfinite csg");
    if(!std::isfinite(in.vcc))throw std::invalid_argument("Nonfinite vcc");
    if(!std::isfinite(in.vccx))throw std::invalid_argument("Nonfinite vccx");
    if(!std::isfinite(in.hb_hs))throw std::invalid_argument("Nonfinite hb_hs");

    if(!std::isfinite(t) || t<0 || (initialized && (t<=last_time || t-last_time>p.DT*1.001)))
        throw std::invalid_argument("Samples must advance by at most DT");
    const bool first=!initialized;
    const double dt=first?0:t-last_time;
    Output o;
    double f=p.FSW_HZ;
    if(p.USE_RT_PIN!=0 && std::abs(in.rt_current)>100e-9)
        f=6.4e9/(p.VREF/std::abs(in.rt_current)+3020.);
    f=clamp(f,50e3,500e3);
    double oldphase=phase;
    phase+=dt*f;
    bool wrap=phase>=1.;
    if(wrap)phase-=1.;
    latch(en_ok,in.en,p.EN_RISE,p.EN_FALL);
    latch(uvlo_ok,in.uvlo,p.UVLO_RISE,p.UVLO_FALL);
    latch(vcc_ok,in.vcc,p.VCC_RISE,p.VCC_FALL);
    latch(vccx_ok,in.vccx,p.VCCX_RISE,p.VCCX_FALL);
    latch(vin_ok,in.vin,5.,3.);
    if(hiccup && prev_uvlo>=.1 && in.uvlo<.1){hiccup=false;count=0;limited=false;}
    bool enabled=en_ok && uvlo_ok && vcc_ok && vin_ok && p.TJ_C<165 && !hiccup;
    const double dmax=std::max(p.TON_MIN*f,1.-f*p.TOFF_MIN),dmin=p.TON_MIN*f;
    // Preserve the original hardcoded duty proxy offset, separately from PWM_OFFSET.
    const double cmd=clamp((in.comp-.2)/1.5,0,dmax);
    const double ratio=in.vin>.1?(in.fb>.05?in.vout*p.VREF/in.fb/in.vin:cmd):10.;
    o.blend=clamp((ratio-.75)/.16,0,1);
    o.cs_threshold=p.VCS_BUCK+o.blend*(p.VCS_BB-p.VCS_BUCK);
    const double sense=in.csg-in.cs;
    const double sample_phase=1.-.25*f*p.TOFF_MIN;
    if(!wrap && oldphase<sample_phase && phase>=sample_phase)valley=enabled?std::max(sense,0.):0.;
    if(wrap){
        count=enabled && limited?count+1:0;
        limited=enabled && sense>=o.cs_threshold;
        done=!enabled;
        if(!enabled)valley=0;
        if(count>=256){hiccup=true;enabled=false;}
    }
    // Level-sensitive safety inhibition also handles an initially excessive CS.
    if(enabled && sense>=o.cs_threshold)limited=true;
    const double compare=p.PWM_OFFSET+p.CS_PWM_GAIN*valley+std::max(in.ramp,0.)-in.comp;
    if(phase>=dmax || (phase>=dmin &&
       ((oldphase<dmin && compare>=0) || (prev_compare<0 && compare>=0))))done=true;
    bool allowed=enabled && !limited && !hiccup && sense<o.cs_threshold;
    // Pure boost holds the input-side switch on and PWM-controls the
    // output-side low switch. Applying the same off latch to both switches
    // imposed an artificial input disconnection and limited low-VIN gain.
    // Linear blend retains the original transition region, with independent
    // high-side window after full boost entry (target/VIN >= 0.91).
    o.pwm_ho=allowed && phase<dmax && (o.blend>=1.0 || !done);
    o.pwm_lo=allowed && !done && o.blend>0 && phase<o.blend*dmax;
    gh=slew(gh,o.pwm_ho,dt,p.T_RISE_HO,p.T_FALL_HO);
    gl=slew(gl,o.pwm_lo,dt,p.T_RISE_LO,p.T_FALL_LO);
    rw=slew(rw,enabled && phase<dmax,dt,p.T_RAMP_SWITCH,p.T_RAMP_SWITCH);
    double vlo=std::max(in.vcc-.25,0.);
    o.ho=gh*std::min(std::max(in.hb_hs-.25,0.),vlo);o.lo=gl*vlo;
    double target=vccx_ok?in.vccx:(in.vin<10?std::max(in.vin-.25,0.):p.VCC_REG);
    o.i_vcc=p.TJ_C>=165?in.vcc/1e6:(in.vcc<target?-std::min((target-in.vcc)/p.R_VCC,p.VCC_ILIM):1e-9*in.vcc);
    o.i_uvlo=-5e-6+(hiccup?in.uvlo/p.R_UVLO_HIC:0);
    o.i_ss=enabled?-p.ISS+std::max(in.ss-in.fb-.15,0.)/10.:in.ss/p.R_SS_DIS;
    double ic=clamp(p.GM_ERR*(clamp(in.ss,0,p.VREF)-in.fb),-p.ICOMP_MAX,p.ICOMP_MAX);
    if((in.comp>=p.COMP_HIGH && ic>0)||(in.comp<=p.COMP_LOW && ic<0))ic=0;
    o.i_comp=enabled?-ic:in.comp/1e3;
    if(in.comp>p.COMP_HIGH)o.i_comp+=(in.comp-p.COMP_HIGH)/p.R_COMP_CLAMP;
    if(in.comp<p.COMP_LOW)o.i_comp+=(in.comp-p.COMP_LOW)/p.R_COMP_CLAMP;
    double ir=5e-6*std::max(o.blend<.5?in.vin-in.vout:in.vin,0.)+50e-6;
    o.i_ramp=-rw*ir+(1.-rw)*in.ramp/p.R_RAMP_RESET;
    o.i_vin=en_ok?p.IQ_RUN:p.IQ_OFF;
    o.i_fb=20e-9;o.i_vout=5.1e-6*std::max(in.vout,0.);
    o.i_cs=o.i_csg=45e-6;o.i_hb=205e-6;
    o.i_en=1e-12*in.en;o.i_sync=1e-12*in.sync;
    o.rt_voltage=p.VREF;o.frequency=f;o.phase=phase;o.valley_cs=valley;
    o.power_bias=in.vin*o.i_vin;o.active=enabled;o.limited=limited;o.hiccup=hiccup;
    o.consecutive_limit=count;
    prev_uvlo=in.uvlo;prev_compare=compare;last_time=t;initialized=true;
    return o;
}
LM5118MH::LM5118MH(sc_core::sc_module_name n,const std::string& path):sca_tdf::sca_module(n),config(load_config(path)),engine(config){}
void LM5118MH::set_attributes(){
    if(sc_core::sc_get_time_resolution().to_seconds()>1e-12)throw std::runtime_error("Use time resolution <=1 ps");
    set_timestep(engine.parameters().DT,sc_core::SC_SEC);
}
void LM5118MH::processing(){ Inputs in;
    in.vin=vin.read();
    in.uvlo=uvlo.read();
    in.rt_current=rt_current.read();
    in.en=en.read();
    in.ramp=ramp.read();
    in.ss=ss.read();
    in.fb=fb.read();
    in.comp=comp.read();
    in.vout=vout.read();
    in.sync=sync.read();
    in.cs=cs.read();
    in.csg=csg.read();
    in.vcc=vcc.read();
    in.vccx=vccx.read();
    in.hb_hs=hb_hs.read();
    const auto o=engine.sample(get_time().to_seconds(),in);
    ho.write(o.ho);
    lo.write(o.lo);
    i_vin.write(o.i_vin);
    i_vcc.write(o.i_vcc);
    i_uvlo.write(o.i_uvlo);
    i_ss.write(o.i_ss);
    i_comp.write(o.i_comp);
    i_ramp.write(o.i_ramp);
    i_fb.write(o.i_fb);
    i_vout.write(o.i_vout);
    i_cs.write(o.i_cs);
    i_csg.write(o.i_csg);
    i_hb.write(o.i_hb);
    i_en.write(o.i_en);
    i_sync.write(o.i_sync);
    rt_voltage.write(o.rt_voltage);
    frequency.write(o.frequency);
    phase.write(o.phase);
    blend.write(o.blend);
    cs_threshold.write(o.cs_threshold);
    valley_cs.write(o.valley_cs);
    power_bias.write(o.power_bias);
    active.write(o.active);
    limited.write(o.limited);
    hiccup.write(o.hiccup);
    pwm_ho.write(o.pwm_ho);
    pwm_lo.write(o.pwm_lo);
    last_power=o.power_bias;
}
std::string LM5118MH::get_module_name() const{return config.at("name").at("value").get<std::string>();}
double LM5118MH::get_version() const{return config.at("version").at("value").get<double>();}
std::string LM5118MH::get_category() const{return config.at("category").at("value").get<std::string>();}
} // namespace lm5118mh
