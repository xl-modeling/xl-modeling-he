// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
// Source: supplied TPS61235P_v3 Verilog-A baseline; TI SLVSCK4A.
// Application-level PWM behavior; no silicon/transistor accuracy sign-off.
#include "tps61235p.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>

namespace {
double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
// 与 VA 的 0.5*(x+sqrt(x*x+eps*eps)) 等价，负半轴避免相消。
double positive(double x, double eps) {
    const double h = std::hypot(x, eps);
    return x >= 0 ? 0.5*(x+h) : 0.5*eps*eps/(h-x);
}
double slope(double x, double eps) { return 0.5*(1+x/std::hypot(x, eps)); }
double physical_value(const nlohmann::json& v) {
    if (v.is_null()) return std::numeric_limits<double>::quiet_NaN();
    const double x=v.get<double>();
    if (!std::isfinite(x)||x<0) throw std::invalid_argument("invalid physical metadata");
    return x;
}
bool same_physical(double x,double y) { return (std::isnan(x)&&std::isnan(y))||x==y; }
}

TPS61235P::configs TPS61235P::get_configs_from_json(const char* json_path) {
    CFGS_BEGIN(c)
        c.module_name = meta.at("name");
        c.version = meta.at("version");
        c.latency_s = physical_value(configuration.at("latency_s"));
        c.power_w = physical_value(configuration.at("power_w"));
        c.area_mm2 = physical_value(configuration.at("area_mm2"));
        c.parameter = parameter;
        c.timestep = sc_core::sc_time(configuration.at("timestep_s").get<double>(), sc_core::SC_SEC);
        c.softstart = sc_core::sc_time(parameter.at("softstart").at("value").get<double>(), sc_core::SC_SEC);
        c.inductance = configuration.at("external_inductance_h");
        c.capacitance = configuration.at("external_capacitance_f");
        c.initial_vout = configuration.at("initial_vout_v");
        c.initial_il = configuration.at("initial_inductor_current_a");
        if (c.module_name != j.at("name").at("value").get<std::string>() ||
            c.version != j.at("version").at("value").get<double>() ||
            !same_physical(c.latency_s,physical_value(j.at("latency").at("value"))) ||
            !same_physical(c.power_w,physical_value(j.at("power").at("value"))) ||
            !same_physical(c.area_mm2,physical_value(j.at("area").at("value"))))
            throw std::invalid_argument("inconsistent delivery/toolkit metadata");
    CFGS_END(c)
}

double TPS61235P::p(const char* key) const { return cfg_.parameter.at(key).at("value").get<double>(); }

TPS61235P::TPS61235P(sc_core::sc_module_name nm, const configs& cfg)
    : sca_tdf::sca_module(nm), PGND("PGND"), VIN("VIN"), CC("CC"), AGND("AGND"),
      FB("FB"), EN("EN"), INACT("INACT"), load_resistance("load_resistance"),
      SW("SW"), VOUT("VOUT"), input_current("input_current"), inductor_current("inductor_current"), cfg_(cfg) {
    par_.vout_nominal = p("vout_nominal");
    par_.fsw = p("fsw");
    par_.softstart = p("softstart");
    par_.ven_high = p("ven_high");
    par_.ven_low = p("ven_low");
    par_.vuvlo_rise = p("vuvlo_rise");
    par_.vuvlo_hyst = p("vuvlo_hyst");
    par_.ron_low = p("ron_low");
    par_.ron_high = p("ron_high");
    par_.rdiode = p("rdiode");
    par_.vdiode = p("vdiode");
    par_.roff = p("roff");
    par_.valley_limit = p("valley_limit");
    par_.duty_min = p("duty_min");
    par_.duty_max = p("duty_max");
    par_.kp = p("kp");
    par_.ki = p("ki");
    par_.iq_vin = p("iq_vin");
    par_.iq_vout = p("iq_vout");
    par_.isd_vin = p("isd_vin");
    par_.precharge_i0 = p("precharge_i0");
    par_.precharge_i2 = p("precharge_i2");
    par_.precharge_i3 = p("precharge_i3");
    par_.precharge_drop = p("precharge_drop");
    par_.precharge_taper = p("precharge_taper");
    par_.pwm_start_margin = p("pwm_start_margin");
    par_.precharge_assist_margin = p("precharge_assist_margin");
    par_.vovp = p("vovp");
    par_.vovp_hyst = p("vovp_hyst");
    par_.r_sw_hold = p("r_sw_hold");
    par_.switch_smooth = p("switch_smooth");
    const char* positive_keys[] = {"vout_nominal","fsw","softstart","ron_low","ron_high","rdiode","roff",
        "valley_limit","precharge_taper","vovp","r_sw_hold","switch_smooth"};
    for (auto k : positive_keys) if (!std::isfinite(p(k)) || p(k)<=0) throw std::invalid_argument(std::string("invalid positive parameter: ")+k);
    for (auto it=cfg_.parameter.begin();it!=cfg_.parameter.end();++it)
        if (!std::isfinite(it.value().at("value").get<double>())) throw std::invalid_argument("non-finite parameter");
    const char* nonnegative_keys[] = {"vuvlo_hyst","vdiode","kp","ki","iq_vin","iq_vout","isd_vin",
        "precharge_i0","precharge_i2","precharge_i3","precharge_drop","pwm_start_margin","precharge_assist_margin","vovp_hyst"};
    for (auto k : nonnegative_keys) if (p(k)<0) throw std::invalid_argument(std::string("negative parameter: ")+k);
    if (!(cfg_.inductance>0 && cfg_.capacitance>0 && std::isfinite(cfg_.inductance) && std::isfinite(cfg_.capacitance)) ||
        cfg_.timestep == sc_core::SC_ZERO_TIME || cfg_.softstart == sc_core::SC_ZERO_TIME ||
        !(par_.ven_high>par_.ven_low) || !(par_.vuvlo_rise>par_.vuvlo_hyst) ||
        !(par_.vovp>par_.vovp_hyst) || !(0<=par_.duty_min && par_.duty_min<par_.duty_max && par_.duty_max<=1) ||
        !std::isfinite(cfg_.initial_vout) || cfg_.initial_vout<0 || !std::isfinite(cfg_.initial_il))
        throw std::invalid_argument("invalid circuit/timing/hysteresis configuration");
    period_ = sc_core::sc_time(1/par_.fsw, sc_core::SC_SEC);
    const double count = period_.to_seconds()/cfg_.timestep.to_seconds();
    samples_per_period_ = static_cast<unsigned long>(std::llround(count));
    if (samples_per_period_<100 || std::fabs(count-samples_per_period_)>1e-8)
        throw std::invalid_argument("timestep must divide switching period exactly, at least 100 samples/period");
    vo_=cfg_.initial_vout; il_=cfg_.initial_il;
}

void TPS61235P::set_attributes() {
    // 开关功率级需要 ns 级积分，避免通用 vsource 的 10 ps 步长带来过量计算。
    set_timestep(cfg_.timestep);
    VOUT.set_delay(1);  // FB<-VOUT 的因果闭环；一采样的数值延迟，不是物理传播延迟。
}
void TPS61235P::initialize() { VOUT.initialize(cfg_.initial_vout); }
void TPS61235P::reset_control() {
    startup_state_=enabled_?1:0; integrator_=ref_=start_v_=progress_=0;
    start_time_=sc_core::SC_ZERO_TIME; duty_=par_.duty_min;
}

void TPS61235P::update_control(double vin, double fb) {
    if (!enabled_) { reset_control(); return; }
    if (startup_state_==1) {
        duty_=par_.duty_min; integrator_=ref_=progress_=0;
        if (vo_>=vin-par_.pwm_start_margin) {
            startup_state_=2; ref_=clamp(vo_,0,par_.vout_nominal); start_v_=ref_; start_time_=get_time();
        }
        return;
    }
    progress_=clamp((get_time()-start_time_).to_seconds()/cfg_.softstart.to_seconds(),0,1);
    ref_=start_v_+(par_.vout_nominal-start_v_)*progress_;
    const double error=ref_-fb;
    if (ref_>vin+par_.pwm_start_margin && !ovp_) integrator_+=par_.ki*error/par_.fsw;
    else integrator_=0;
    integrator_=clamp(integrator_,-0.25,0.25);
    double ff=ref_>0.1?1-vin/ref_:par_.duty_min;
    ff=std::max(0.0,ff);
    duty_=clamp(ff+par_.kp*error+integrator_,progress_<1?0:par_.duty_min,par_.duty_max);
}

void TPS61235P::solve_circuit(double vin, double resistance, bool low, bool high, bool active, bool precharge) {
    const double dt=cfg_.timestep.to_seconds(), a=dt/cfg_.inductance, b=cfg_.capacitance/dt;
    const double old_vo=vo_, old_il=il_, eps=par_.switch_smooth;
    double s=sw_, v=vo_, pre=0, hold=0;
    bool converged=false;
    for (int iter=0;iter<60;++iter) {
        const double lo=(low?positive(s,eps)/par_.ron_low:s/par_.roff);
        const double dlo=low?slope(s,eps)/par_.ron_low:1/par_.roff;
        const double x=s-v;
        double hi=high?positive(x,eps)/par_.ron_high:x/par_.roff;
        double dhi=high?slope(x,eps)/par_.ron_high:1/par_.roff;
        if (active && !high) { hi+=positive(x-par_.vdiode,1e-4)/par_.rdiode; dhi+=slope(x-par_.vdiode,1e-4)/par_.rdiode; }
        const double ld=(active && !low)?positive(-s-par_.vdiode,1e-4)/par_.rdiode:0;
        const double dld=(active && !low)?-slope(-s-par_.vdiode,1e-4)/par_.rdiode:0;
        const double gh=active?1/par_.roff:1/par_.r_sw_hold;
        hold=gh*(s-vin);
        double limit, dlimit;
        if (v<=0) { limit=par_.precharge_i0; dlimit=0; }
        else if (v<2) { dlimit=(par_.precharge_i2-par_.precharge_i0)/2; limit=par_.precharge_i0+dlimit*v; }
        else if (v<3) { dlimit=par_.precharge_i3-par_.precharge_i2; limit=par_.precharge_i2+dlimit*(v-2); }
        else { limit=par_.precharge_i3; dlimit=0; }
        const double head=vin-par_.precharge_drop-v;
        const double factor=clamp(head/par_.precharge_taper,0,1);
        const double df=(head>0 && head<par_.precharge_taper)?-1/par_.precharge_taper:0;
        pre=precharge?limit*factor:0;
        const double dp=precharge?dlimit*factor+limit*df:0;
        const double iq=enabled_?par_.iq_vout:v/par_.roff;
        const double f1=lo+hi+hold-ld-old_il-a*(vin-s);
        const double f2=b*(v-old_vo)+v/resistance+iq-hi-pre;
        const double norm=std::max(std::fabs(f1),std::fabs(f2));
        if (norm<1e-8) { max_residual_=std::max(max_residual_,norm); converged=true; break; }
        const double j11=dlo+dhi+gh-dld+a, j12=-dhi, j21=-dhi;
        const double j22=b+1/resistance+(enabled_?0:1/par_.roff)+dhi-dp;
        const double det=j11*j22-j12*j21;
        if (!(std::isfinite(det) && std::fabs(det)>1e-20)) break;
        const double ds=(-f1*j22+j12*f2)/det, dv=(-j11*f2+j21*f1)/det;
        const double damping=std::min(1.0,2.0/std::max(2.0,std::max(std::fabs(ds),std::fabs(dv))));
        s+=damping*ds; v+=damping*dv;
    }
    if (!converged || !std::isfinite(s) || !std::isfinite(v)) throw std::runtime_error("power-stage Newton solver did not converge");
    sw_=s; vo_=v; il_=old_il+a*(vin-s); pre_=pre;
    iin_=il_-hold+pre+(enabled_?par_.iq_vin:par_.isd_vin);
    // Dissipative branch sum; do not count BE integration error as chip loss.
    const double ilo=low?positive(s,eps)/par_.ron_low:s/par_.roff;
    double ihi=high?positive(s-v,eps)/par_.ron_high:(s-v)/par_.roff;
    if(active&&!high)ihi+=positive(s-v-par_.vdiode,1e-4)/par_.rdiode;
    const double ild=active&&!low?positive(-s-par_.vdiode,1e-4)/par_.rdiode:0;
    last_power_w_=std::max(0.,ilo*s+ihi*(s-v)-ild*s+hold*(s-vin)+
        pre*std::max(vin-v,0.)+vin*(enabled_?par_.iq_vin:par_.isd_vin)+
        v*(enabled_?par_.iq_vout:v/par_.roff));
}

void TPS61235P::processing() {
    const double ground=AGND.read(), vin=VIN.read()-ground, en=EN.read()-ground, fb=FB.read()-ground;
    const double resistance=load_resistance.read();
    for (double value : {ground,PGND.read(),vin,en,fb,CC.read(),INACT.read(),resistance})
        if (!std::isfinite(value)) throw std::invalid_argument("non-finite port value");
    if (std::fabs(PGND.read()-ground)>1e-9 || resistance<=0)
        throw std::invalid_argument("AGND/PGND must be tied; load_resistance must be positive");
    if (first_) {
        enabled_=en>=par_.ven_high && vin>=par_.vuvlo_rise; ovp_=vo_>=par_.vovp; reset_control(); first_=false;
    } else {
        const bool rise_en=prev_en_<par_.ven_high && en>=par_.ven_high;
        const bool rise_vin=prev_vin_<par_.vuvlo_rise && vin>=par_.vuvlo_rise;
        if ((rise_en && vin>=par_.vuvlo_rise) || (rise_vin && en>=par_.ven_high)) {
            enabled_=true; ovp_=vo_>=par_.vovp; reset_control();
        }
        if ((prev_en_>par_.ven_low && en<=par_.ven_low) ||
            (prev_vin_>par_.vuvlo_rise-par_.vuvlo_hyst && vin<=par_.vuvlo_rise-par_.vuvlo_hyst)) {
            enabled_=false; reset_control();
        }
    }
    if (vo_>=par_.vovp) ovp_=true;
    else if (vo_<=par_.vovp-par_.vovp_hyst) ovp_=false;
    prev_en_=en; prev_vin_=vin;
    // 原 VA 的 timer 从一个周期后开始。
    if (sample_>0 && sample_%samples_per_period_==0) update_control(vin,fb);
    const bool active=enabled_ && startup_state_==2 && !ovp_;
    const double phase=double(sample_%samples_per_period_)/samples_per_period_;
    const bool low=active && phase<duty_, high=active && !low;
    if (low && !last_low_) ++pwm_rises_;
    last_low_=low;
    const bool precharge=enabled_ && !ovp_ && (startup_state_==1 ||
        (startup_state_==2 && ref_<vin+par_.precharge_assist_margin && vo_<vin));
    solve_circuit(vin,resistance,low,high,active,precharge);
    // TDF 引脚为单向信号，不向外部网络回注 FB/CC/INACT/EN 的高阻泄漏电流。
    SW.write(sw_+ground); VOUT.write(vo_+ground); input_current.write(iin_); inductor_current.write(il_);
    ++sample_;
}

void TPS61235P::trace_me(sca_util::sca_trace_file* tf) {
    sca_util::sca_trace(tf,VIN,"VIN"); sca_util::sca_trace(tf,EN,"EN");
    sca_util::sca_trace(tf,VOUT,"VOUT"); sca_util::sca_trace(tf,SW,"SW");
    sca_util::sca_trace(tf,input_current,"input_current"); sca_util::sca_trace(tf,inductor_current,"inductor_current");
}
void TPS61235P::dump_performance() {
    std::cout<<get_module_name()<<" version="<<get_version()<<" latency="<<get_latency()
             <<" power_W="<<get_power_dissipation()<<" area_mm2="<<get_area()<<'\n';
}
