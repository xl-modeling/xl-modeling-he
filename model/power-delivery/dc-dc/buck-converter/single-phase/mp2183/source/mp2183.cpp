// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#include "mp2183.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace mp2183 {
namespace {
double bound(double x,double a,double b) { return std::max(a,std::min(x,b)); }
double value(const nlohmann::json& j,const char* k) {
    double v=j.at(k).at("value").get<double>();
    if (!std::isfinite(v)) throw std::invalid_argument(std::string(k)+" must be finite");
    return v;
}
void positive(double v,const char* k) {
    if (!(v>0)) throw std::invalid_argument(std::string(k)+" must be positive");
}
// Linear conductance edge, interval-average for backward Euler. This approximates
// VA transition(); it is not a transistor switching-energy characterization.
double edge(double& actual,double target,double dt,double duration) {
    double distance=std::abs(target-actual);
    if (!distance) return actual;
    double reach=distance*duration;
    double end=actual+(target>actual?1:-1)*std::min(distance,dt/duration);
    double average=dt>=reach ? target+(actual-target)*reach/(2*dt) : (actual+end)/2;
    actual=end;
    return average;
}
}
nlohmann::json load_config(const std::string& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot open configuration: "+path);
    nlohmann::json j; stream>>j;
    if (j.at("name").at("value")!="mp2183" || j.at("version").at("value")!=0.1)
        throw std::invalid_argument("Unsupported model name/version");
    return j;
}
Parameters::Parameters(const nlohmann::json& j) {
#define REAL(k) k=value(j,#k)
    REAL(VREF); REAL(FSW); REAL(RON_H); REAL(RON_L); REAL(IQ); REAL(ISHDN);
    REAL(IFB); REAL(ISS); REAL(UVLO_ON); REAL(UVLO_HYS); REAL(EN_ON); REAL(EN_OFF);
    REAL(ILIM_PEAK); REAL(ILIM_VALLEY); REAL(IZCD); REAL(PG_ON); REAL(PG_OFF);
    REAL(OVP_ON); REAL(OVP_OFF); REAL(OVP_REVERSE); REAL(VIN_OVP); REAL(VIN_OVP_HYS);
    REAL(RDIS); REAL(REN); REAL(RPG); REAL(TSHUT); REAL(THYST); REAL(ROFF);
    REAL(CSW); REAL(VBD); REAL(RBD); REAL(KRAMP); REAL(KI); REAL(TRIM_MAX);
    REAL(AAM_PEAK); REAL(RON_TC); REAL(L); REAL(DCR); REAL(COUT); REAL(ESR);
    REAL(CSS); REAL(R_TOP); REAL(R_BOT); REAL(R_PG);
    REAL(INITIAL_VOUT); REAL(INITIAL_IL); REAL(INITIAL_SS);
#undef REAL
#define TIME(k) { double v=value(j,#k); positive(v,#k); k=sc_time(v,sc_core::SC_SEC); }
    TIME(DT); TIME(TEDGE); TIME(TDEAD); TIME(EN_DELAY_ON); TIME(EN_DELAY_OFF);
    TIME(TMIN_ON); TIME(TMIN_OFF); TIME(PG_DELAY); TIME(OVP_DELAY); TIME(OVP_RETRY);
    TIME(TAVG); TIME(HICCUP); TIME(TSC_LIMIT);
#undef TIME
#define POS(k) positive(k,#k)
    POS(VREF); POS(FSW); POS(RON_H); POS(RON_L); POS(ISS); POS(RDIS); POS(REN);
    POS(RPG); POS(ROFF); POS(CSW); POS(RBD); POS(L); POS(COUT); POS(CSS);
    POS(R_TOP); POS(R_BOT); POS(R_PG); POS(ILIM_PEAK); POS(ILIM_VALLEY);
#undef POS
    if (DCR<0 || ESR<0 || IQ<0 || ISHDN<0 || IFB<0 || UVLO_HYS<0 || THYST<0 ||
        EN_OFF>=EN_ON || PG_OFF>=PG_ON || OVP_OFF>=OVP_ON || UVLO_ON<=UVLO_HYS ||
        ILIM_PEAK<=ILIM_VALLEY || IZCD<0 || VIN_OVP_HYS<0 || TRIM_MAX<0 ||
        VBD<0 || AAM_PEAK<0 || INITIAL_VOUT<0 || INITIAL_SS<0)
        throw std::invalid_argument("Inconsistent or negative circuit/protection parameter");
    if (DT>sc_time(5,sc_core::SC_NS) || DT>std::min(TDEAD,std::min(TMIN_ON,TMIN_OFF)))
        throw std::invalid_argument("DT must be <=5 ns and <= dead/minimum switch times");
    if (DT==sc_core::SC_ZERO_TIME || TEDGE==sc_core::SC_ZERO_TIME)
        throw std::invalid_argument("Time parameters below SystemC resolution");
}
void Controller::sample(sc_time now,const ControlInput& in) {
    if (in.vin>=p.UVLO_ON) s.uv=true;
    else if (in.vin<=p.UVLO_ON-p.UVLO_HYS) s.uv=false;
    if (!s.en && in.en>=p.EN_ON) { s.en=true;ten=now; }
    if (s.en && in.en<=p.EN_OFF) { s.en=false;ten=now; }
    if (s.en && now-ten>=p.EN_DELAY_ON) s.runen=true;
    if (!s.en && now-ten>=p.EN_DELAY_OFF) s.runen=false;
    if (in.temperature>=p.TSHUT) s.hot=true;
    else if (in.temperature<=p.TSHUT-p.THYST) s.hot=false;
    if (in.fb>=p.OVP_ON*p.VREF) {
        if (!ov_timer) { tov=now;ov_timer=true; }
        if (now-tov>=p.OVP_DELAY) s.ovp=true;
    } else ov_timer=false;
    if (in.fb<=p.OVP_OFF*p.VREF) s.ovp=false;
    if (s.ovp && in.vin>=p.VIN_OVP) s.vinovp=true;
    if (in.vin<=p.VIN_OVP-p.VIN_OVP_HYS) s.vinovp=false;
    s.active=s.uv && s.runen && !s.hot && !s.vinovp && now>=retry;
    double refss=bound(in.ss,0,p.VREF);
    if (!s.active) {
        s.high=s.low=s.pending=false; thoff=tloff=now;
        s.iavg=s.trim=0; s.sleep=false; short_timer=false;
    } else {
        s.iavg+=bound(p.DT/p.TAVG,0,1)*(in.il-s.iavg);
        if (refss>0.95*p.VREF && in.fb<0.25*p.VREF && s.iavg>0.8*p.ILIM_VALLEY) {
            if (!short_timer) { tsc=now;short_timer=true; }
        } else short_timer=false;
        if ((in.fb<refss && s.trim<p.TRIM_MAX) || (in.fb>refss && s.trim>-p.TRIM_MAX))
            s.trim=bound(s.trim+p.KI*p.DT.to_seconds()*(refss-in.fb),-p.TRIM_MAX,p.TRIM_MAX);
        double cmp=in.fb+p.KRAMP*(in.il-s.iavg);
        if (s.ovp) {
            if (s.high) { s.high=false;thoff=now; }
            s.pending=false;
            if (s.low && in.il<=-p.OVP_REVERSE) {
                s.low=false;tloff=now;treverse=now+p.OVP_RETRY;
            }
            if (!s.low && now-thoff>=p.TDEAD && now>=treverse) s.low=true;
        } else if ((s.high && in.il>=p.ILIM_PEAK) ||
                   (short_timer && now-tsc>=p.TSC_LIMIT)) {
            s.high=s.low=s.pending=s.active=false;
            thoff=tloff=now;retry=now+p.HICCUP;
        } else {
            if (s.high && now>=tend &&
                (now>=tend+sc_time(1/p.FSW,sc_core::SC_SEC) || in.il>=p.AAM_PEAK || !s.sleep)) {
                if (!(in.out>0.97*in.vin && in.fb<refss)) { s.high=false;thoff=now; }
            }
            if (!s.high) {
                if (s.low && in.il<=p.IZCD) { s.low=false;tloff=now; }
                if (!s.pending && now-thoff>=p.TMIN_OFF && in.il<p.ILIM_VALLEY &&
                    cmp<refss+s.trim && refss>1e-4) {
                    s.sleep=(now-thoff>sc_time(3.5,sc_core::SC_US));
                    s.low=false;tloff=now;s.pending=true;
                }
                if (s.pending && now-tloff>=p.TDEAD) {
                    double ton=std::max(p.TMIN_ON.to_seconds(),in.out/std::max(in.vin,0.1)/p.FSW);
                    if (s.sleep) ton*=1.4;
                    s.high=true;s.low=s.pending=false;tend=now+sc_time(ton,sc_core::SC_SEC);
                }
                if (!s.high && !s.pending && !s.low && in.il>p.IZCD && now-thoff>=p.TDEAD)
                    s.low=true;
            }
        }
    }
    if (!s.active || in.fb<p.PG_OFF*p.VREF) {
        if (s.pg_target) { s.pg_target=false;tpg=now; }
    } else if (in.fb>p.PG_ON*p.VREF) {
        if (!s.pg_target) { s.pg_target=true;tpg=now; }
    }
    if (now-tpg>=p.PG_DELAY) s.pg_high=s.pg_target;
}
Engine::Engine(const nlohmann::json& j) : p(j),control(p),il(p.INITIAL_IL),
    vc(p.INITIAL_VOUT),ss(p.INITIAL_SS),out(p.INITIAL_VOUT) {}
Outputs Engine::advance(double h,const Inputs& in) {
    Outputs o;
    double a=edge(gh,target_h,h,p.TEDGE.to_seconds());
    double b=edge(gl,target_l,h,p.TEDGE.to_seconds());
    double run=edge(grun,target_run,h,p.TEDGE.to_seconds());
    double pg_gate=edge(gpg,target_pg,h,p.TEDGE.to_seconds());
    double rh=p.RON_H*std::max(0.1,1+p.RON_TC*(in.temperature-25));
    double rl=p.RON_L*std::max(0.1,1+p.RON_TC*(in.temperature-25));
    double gH=a/rh+(1-a)/p.ROFF,gL=b/rl+(1-b)/p.ROFF;
    double gd=(1-run)/p.RDIS+run/p.ROFF;
    double gr=1/in.rload+1/(p.R_TOP+p.R_BOT)+gd;
    // Divider: FB = VOUT*RB/(RT+RB) - IFB*(RT||RB).
    // Its constant current drawn from OUT is IFB*RB/(RT+RB).
    double load=in.iload+p.IFB*p.R_BOT/(p.R_TOP+p.R_BOT);
    double z=p.ESR+h/p.COUT,den=1+z*gr;
    double vo0=(vc-z*load)/den,voi=z/den;
    double li=1+h/p.L*(p.DCR+voi);
    double i0=(il-h/p.L*vo0)/li,isw=(h/p.L)/li;
    double cap=p.CSW/h;
    double next_sw=0;
    bool solved=false;
    // Enumerate the three exact piecewise-linear body-diode regions.
    for (int region=0;region<3;++region) {
        double diode=(region?1/p.RBD:0);
        double diode_source=region==1?-p.VBD:(in.vin+p.VBD);
        double v=(cap*sw+gH*in.vin+diode*diode_source-i0)/(cap+gH+gL+diode+isw);
        if ((region==0 && v>=-p.VBD-1e-9 && v<=in.vin+p.VBD+1e-9) ||
            (region==1 && v<=-p.VBD+1e-9) || (region==2 && v>=in.vin+p.VBD-1e-9)) {
            next_sw=v;solved=true;break;
        }
    }
    if (!solved) throw std::runtime_error("Body diode solve failed");
    sw=next_sw;il=i0+isw*sw;out=vo0+voi*il;
    double ic=il-gr*out-load;
    vc+=h/p.COUT*ic;
    // SS pin includes the VA's 1pF internal capacitor, discharge and clamp.
    double css=p.CSS+1e-12,ssg=(1-run)/100;
    double next_ss=(css/h*ss+run*p.ISS)/(css/h+ssg);
    if (next_ss>p.VREF+0.05)
        next_ss=(css/h*ss+run*p.ISS+(p.VREF+0.05)/1000)/(css/h+ssg+1.0/1000);
    ss=next_ss;
    double dhi=std::max(sw-in.vin-p.VBD,0.0)/p.RBD;
    double dlo=std::max(-sw-p.VBD,0.0)/p.RBD;
    double iq=run*p.IQ+(1-run)*p.ISHDN;
    double pgconduct=(1-pg_gate)/p.RPG+pg_gate/p.ROFF;
    o.pg=in.pg_pullup/(1+p.R_PG*pgconduct);o.ipg=o.pg*pgconduct;
    o.iin=gH*(in.vin-sw)-dhi+iq;
    o.ien=in.en/p.REN;
    o.power=gH*std::pow(in.vin-sw,2)+gL*sw*sw+
        dhi*(sw-in.vin)+dlo*(-sw)+in.vin*iq+gd*out*out+
        in.en*o.ien+o.pg*o.ipg;
    o.external_loss=p.DCR*il*il+p.ESR*ic*ic;
    o.energy=0.5*(p.L*il*il+p.COUT*vc*vc+p.CSW*sw*sw+css*ss*ss);
    return o;
}
Outputs Engine::sample(sc_time now,const Inputs& in) {
    const double fields[]={in.vin,in.en,in.iload,in.rload,in.temperature,in.pg_pullup};
    for (double v:fields) if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite TDF input");
    if (in.vin<0 || in.en<0 || in.rload<=0 || in.pg_pullup<0)
        throw std::invalid_argument("VIN/EN/PG pullup must be nonnegative; RLOAD must be positive");
    if ((!started && now!=sc_core::SC_ZERO_TIME) || (started && now-previous!=p.DT))
        throw std::invalid_argument("Engine must start at t=0 on the configured DT grid");
    Outputs o;
    if (started) o=advance(p.DT.to_seconds(),in);
    else { o.pg=in.pg_pullup/(1+p.R_PG/p.RPG);o.ien=in.en/p.REN; }
    double feedback=out*p.R_BOT/(p.R_TOP+p.R_BOT)-p.IFB*p.R_TOP*p.R_BOT/(p.R_TOP+p.R_BOT);
    control.sample(now,{in.vin,in.en,feedback,il,out,ss,in.temperature});
    const auto& s=control.state();
    target_h=s.high?1:0;target_l=s.low?1:0;target_run=s.active?1:0;target_pg=s.pg_high?1:0;
    o.vout=out;o.sw=sw;o.il=il;o.ss=ss;o.fb=feedback;
    o.high=s.high;o.low=s.low;o.active=s.active;
    o.state=s.hot?5:(s.vinovp?7:(!s.uv?6:(!s.runen?0:(!s.active?4:
        (s.ovp?3:(ss<p.VREF?1:2))))));
    previous=now;started=true;
    if (!std::isfinite(out) || !std::isfinite(il)) throw std::runtime_error("Nonfinite buck state");
    return o;
}
MP2183::MP2183(sc_core::sc_module_name name,const std::string& path)
    : sca_tdf::sca_module(name),cfg(load_config(path)),engine(cfg) {}
void MP2183::set_attributes() { set_timestep(engine.parameters().DT); }
void MP2183::processing() {
    Inputs input;input.vin=vin.read();input.en=en.read();input.iload=iload.read();
    input.rload=rload.read();input.temperature=temperature.read();input.pg_pullup=pg_pullup.read();
    Outputs o=engine.sample(get_time(),input);
#define WRITE(k) k.write(o.k)
    WRITE(vout);WRITE(sw);WRITE(il);WRITE(ss);WRITE(fb);WRITE(pg);WRITE(ipg);WRITE(iin);
    WRITE(ien);WRITE(power);WRITE(external_loss);WRITE(energy);WRITE(high);WRITE(low);
    WRITE(active);WRITE(state);
#undef WRITE
    last_power=o.power;
}
std::string MP2183::get_module_name() const { return cfg.at("name").at("value").get<std::string>(); }
double MP2183::get_version() const { return cfg.at("version").at("value").get<double>(); }
std::string MP2183::get_category() const { return cfg.at("category").at("value").get<std::string>(); }
sc_time MP2183::get_latency() const { return engine.parameters().EN_DELAY_ON; }
sc_time MP2183::get_timestep() const { return engine.parameters().DT; }
} // namespace mp2183
