// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#include "lt8627sp.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <fstream>
#include <stdexcept>

namespace lt8627sp {
using namespace sc_core;
namespace {
double clip(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
double real(const nlohmann::json& j,const char* key){
    double v=j.at(key).at("value").get<double>();
    if(!std::isfinite(v))throw std::invalid_argument(std::string(key)+" must be finite");
    return v;
}
bool flag(const nlohmann::json& j,const char* key){
    double v=real(j,key);if(v!=0&&v!=1)throw std::invalid_argument(std::string(key)+" must be 0 or 1");
    return v!=0;
}
double positive(double x,double smooth){
    double r=std::hypot(x,smooth);
    return x>=0?.5*(x+r):.5*smooth*smooth/(r-x);
}
double edge(double& actual,bool command,double h,double duration){
    double target=command?1:0,distance=std::abs(target-actual);
    if(distance==0)return actual;
    double reach=distance*duration;
    double end=actual+(command?1:-1)*std::min(distance,h/duration);
    double mean=h>=reach?target+(actual-target)*reach/(2*h):(actual+end)/2;
    actual=end;return mean;
}
}
nlohmann::json load_config(const std::string& path){
    std::ifstream f(path);if(!f)throw std::runtime_error("Cannot open configuration: "+path);
    nlohmann::json j;f>>j;
    if(j.at("name").at("value")!="lt8627sp"||j.at("version").at("value")!=.1)
        throw std::invalid_argument("Unsupported model identity/version");
    return j;
}
Parameters::Parameters(const nlohmann::json& j){
#define REAL(k) k=real(j,#k)
    REAL(FSW);REAL(LOUT);REAL(LDCR);REAL(RDS_TOP);REAL(RDS_BOT);REAL(ROFF);REAL(RSW_LEAK);REAL(CSW);
    REAL(DIODE_VF);REAL(DIODE_RD);REAL(DIODE_SMOOTH);REAL(ISET);REAL(IQ_ON);REAL(IQ_OFF);
    REAL(ILIM_TOP);REAL(ILIM_BOT);REAL(IOUT_MAX);REAL(VEN_RISE);REAL(VEN_FALL);REAL(VIN_MIN);
    REAL(VINTVCC);REAL(VHEAD);REAL(KP);REAL(PG_RON);REAL(SET_RPD);REAL(VOUT_MAX);
    REAL(NOISE_DUTY);REAL(VNOISE_WHITE);REAL(VNOISE_LF_1HZ);REAL(VNOISE_LF_EXP);
    REAL(COUT);REAL(ESR);REAL(RSET);REAL(CSET);REAL(PGFB_RATIO);REAL(PG_PULLUP_V);REAL(PG_PULLUP_R);REAL(INITIAL_VOUT);
#undef REAL
    // Gray-box residual fits are kept separate from data-sheet/VA parameters.
    ESR_RESIDUAL_FIT=j.value("ESR_RESIDUAL_FIT",nlohmann::json{{"value",0.}}).at("value").get<double>();
    SWITCH_CAP_FIT=j.value("SWITCH_CAP_FIT",nlohmann::json{{"value",0.}}).at("value").get<double>();
    if(!std::isfinite(ESR_RESIDUAL_FIT)||ESR_RESIDUAL_FIT<0||
       !std::isfinite(SWITCH_CAP_FIT)||SWITCH_CAP_FIT<0)
        throw std::invalid_argument("Residual fit parameters must be finite and nonnegative");
#define TIME(k) {double v=real(j,#k);if(v<=0)throw std::invalid_argument(#k " must be positive");k=sc_time(v,SC_SEC);}
    TIME(TREF);TIME(DEADTIME);TIME(TEDGE);TIME(DT);
#undef TIME
    ENABLE_CLKOUT=flag(j,"ENABLE_CLKOUT");ENABLE_NOISE=flag(j,"ENABLE_NOISE");NOISE_CHAR_MODE=flag(j,"NOISE_CHAR_MODE");
    for(double v:{FSW,LOUT,RDS_TOP,RDS_BOT,ROFF,RSW_LEAK,CSW,DIODE_RD,DIODE_SMOOTH,ISET,ILIM_TOP,ILIM_BOT,IOUT_MAX,VIN_MIN,VINTVCC,PG_RON,SET_RPD,VOUT_MAX,COUT,RSET,CSET,PG_PULLUP_R})
        if(v<=0)throw std::invalid_argument("Nonpositive electrical parameter");
    if(FSW<300e3||FSW>4e6||LDCR<0||ESR<0||DIODE_VF<0||IQ_ON<0||IQ_OFF<0||
       VEN_FALL<0||VEN_RISE<=VEN_FALL||VHEAD<0||KP<0||PGFB_RATIO<=0||PGFB_RATIO>1||PG_PULLUP_V<0||
       INITIAL_VOUT<0||VNOISE_WHITE<0||VNOISE_LF_1HZ<0||VNOISE_LF_EXP<0||VNOISE_LF_EXP>3)
        throw std::invalid_argument("Invalid electrical/noise parameter");
    period=sc_time(1/FSW,SC_SEC);
    double df=clip(DEADTIME.to_seconds()*FSW,0,.05);
    if(DT>sc_time(2,SC_NS)||TEDGE<sc_time(1,SC_PS)||DEADTIME*2>=period||
       NOISE_DUTY<.005||NOISE_DUTY>1-2*df-.005)
        throw std::invalid_argument("Invalid PWM timing, DT>2ns or fixed duty outside legal window");
    if(sc_get_time_resolution()>sc_time(1,SC_PS))throw std::invalid_argument("Set SystemC resolution <=1ps");
}
Engine::Engine(const nlohmann::json& j):p(j),vc(p.INITIAL_VOUT),vo(p.INITIAL_VOUT){}
void Engine::update_enable(const Inputs& in){
    if(in.en_uvlo>=p.VEN_RISE)enabled=true;
    else if(in.en_uvlo<=p.VEN_FALL)enabled=false;
}
void Engine::tick(sc_time now,const Inputs& in){
    cycle=now;next=now+p.period;
    double df=clip(p.DEADTIME.to_seconds()*p.FSW,0,.05);
    double feed=in.pvin>.1?ref/in.pvin:0;
    duty=p.NOISE_CHAR_MODE?p.NOISE_DUTY:clip(feed+p.KP*(ref-vo-in.sense_offset),.005,1-2*df-.005);
    top_start=sc_time(df/p.FSW,SC_SEC);
    top_stop=top_start+sc_time(duty/p.FSW,SC_SEC);
    bot_start=top_stop+top_start;
}
void Engine::commands(sc_time now,const Inputs& in){
    if(p.NOISE_CHAR_MODE||in.sync_mode>1.5)pulse=true;
    else if(vo+in.sense_offset<ref-.002)pulse=true;
    else if(vo+in.sense_offset>ref+.002)pulse=false;
    bool ready=enabled&&in.pvin>=p.VIN_MIN&&in.svin>=p.VIN_MIN;
    sc_time pos=now-cycle;
    double ripple=in.pvin>.1?std::abs(ref*(1-ref/in.pvin)/(p.LOUT*p.FSW)):0;
    double limit=std::min(p.ILIM_TOP,p.IOUT_MAX+.5*ripple);
    bool window=ready&&pulse&&ref>.001&&pos>=top_start&&pos<top_stop;
    limited=window&&il>=limit;high=window&&!limited;
    low=ready&&pos>=bot_start&&pos<p.period&&
        (in.sync_mode>1.5||(il>0&&vo+in.sense_offset<ref+.002&&il<p.ILIM_BOT));
}
sc_time Engine::next_event(sc_time now,sc_time end)const{
    sc_time stop=end;
    for(sc_time t:{next,cycle+top_start,cycle+top_stop,cycle+bot_start})if(t>now&&t<stop)stop=t;
    return stop;
}
Output Engine::advance(double h,const Inputs& in){
    bool ready=enabled&&in.pvin>=p.VIN_MIN&&in.svin>=p.VIN_MIN;
    if(in.set_override>=0)setv=in.set_override;
    else{
        double resistance=ready?p.RSET:1/(1/p.RSET+1/p.SET_RPD);
        double target=ready?p.ISET*p.RSET:0;
        setv=target+(setv-target)*std::exp(-h/(resistance*p.CSET));
    }
    double target=ready?clip(setv,0,p.VOUT_MAX):0;
    target=std::max(0.,std::min(target,in.svin-p.VHEAD));
    ref=target+(ref-target)*std::exp(-h/(p.TREF.to_seconds()/2.2));
    double hg=edge(gt,high,h,p.TEDGE.to_seconds()),lg=edge(gb,low,h,p.TEDGE.to_seconds());
    double gH=hg/p.RDS_TOP+(1-hg)/p.ROFF;
    double gL=lg/p.RDS_BOT+(1-lg)/p.ROFF+1/p.RSW_LEAK;
    double z=p.ESR+p.ESR_RESIDUAL_FIT+h/p.COUT,gload=1/in.rload,guess=sw,newil=il,newout=vo,newsw=sw;
    bool converged=false;
    for(unsigned iteration=0;iteration<40;++iteration){
        double x=guess-in.pvin-p.DIODE_VF,y=-guess-p.DIODE_VF;
        double current=(positive(x,p.DIODE_SMOOTH)-positive(y,p.DIODE_SMOOTH))/p.DIODE_RD;
        double gd=.5*(2+x/std::hypot(x,p.DIODE_SMOOTH)+y/std::hypot(y,p.DIODE_SMOOTH))/p.DIODE_RD;
        double a=p.CSW/h+gH+gL+gd,b=p.CSW/h*sw+gH*in.pvin-current+gd*guess;
        double den=1+h/p.LOUT*(p.LDCR+1/a);
        double alpha=(il+h/p.LOUT*b/a)/den,beta=(h/p.LOUT)/den;
        newout=(vc+z*(alpha-in.iload-160e-9))/(1+z*(gload+beta));
        newil=alpha-beta*newout;newsw=(b-newil)/a;
        if(std::abs(newsw-guess)<1e-9){converged=true;break;}
        guess=newsw;
    }
    if(!converged)throw std::runtime_error("Power-stage Newton solver did not converge");
    sw=newsw;il=newil;vo=newout;
    double ic=il-in.iload-gload*vo-160e-9;vc+=h/p.COUT*ic;
    double dhi=positive(sw-in.pvin-p.DIODE_VF,p.DIODE_SMOOTH)/p.DIODE_RD;
    double dlo=positive(-sw-p.DIODE_VF,p.DIODE_SMOOTH)/p.DIODE_RD;
    Output o;o.vout=vo;o.sw=sw;o.il=il;
    o.ipvin=gH*(in.pvin-sw)-dhi;
    o.isvin=enabled&&in.svin>=p.VIN_MIN?p.IQ_ON:p.IQ_OFF;
    o.power=gH*std::pow(in.pvin-sw,2)+gL*sw*sw+dhi*(sw-in.pvin)-dlo*sw+in.svin*o.isvin;
    // Switching-energy residual is VIN^2*Cfit per switching period. It is
    // fitted to a historical application point, not measured die capacitance.
    const double residual=ready&&pulse?p.SWITCH_CAP_FIT*in.pvin*in.pvin*p.FSW:0.;
    o.ipvin+=in.pvin>0?residual/in.pvin:0.;o.power+=residual;
    o.external_loss=p.LDCR*il*il+(p.ESR+p.ESR_RESIDUAL_FIT)*ic*ic;
    o.energy=.5*(p.LOUT*il*il+p.CSW*sw*sw+p.COUT*vc*vc+p.CSET*setv*setv);
    return o;
}
Output Engine::sample(sc_time now,const Inputs& in){
    for(double v:{in.pvin,in.svin,in.en_uvlo,in.sync_mode,in.set_override,in.sense_offset,in.iload,in.rload})
        if(!std::isfinite(v))throw std::invalid_argument("Nonfinite input");
    if(in.pvin<0||in.svin<0||in.en_uvlo<0||in.rload<=0)throw std::invalid_argument("Supplies/EN>=0 and RLOAD>0 required");
    if((!started&&now!=SC_ZERO_TIME)||(started&&now-previous!=p.DT))throw std::invalid_argument("Samples must start at zero on configured DT grid");
    update_enable(in);Output o;o.vout=vo;o.sw=sw;o.il=il;
    if(!started)tick(now,in);
    else{
        double pin=0,loss=0,ext=0;sc_time cursor=previous;
        while(cursor<now){
            if(cursor==next)tick(cursor,in);
            commands(cursor,in);sc_time end=next_event(cursor,now);double h=(end-cursor).to_seconds();
            o=advance(h,in);pin+=o.ipvin*h;loss+=o.power*h;ext+=o.external_loss*h;cursor=end;
        }
        o.ipvin=pin/p.DT.to_seconds();o.power=loss/p.DT.to_seconds();o.external_loss=ext/p.DT.to_seconds();
        if(now==next)tick(now,in);
    }
    commands(now,in);
    o.active=enabled&&in.pvin>=p.VIN_MIN&&in.svin>=p.VIN_MIN;
    o.isvin=enabled&&in.svin>=p.VIN_MIN?p.IQ_ON:p.IQ_OFF;
    o.limited=limited;o.gh=high?1:0;o.gl=low?1:0;o.duty=duty;o.set_voltage=setv;o.vref=ref;
    o.intvcc=enabled&&in.svin>=p.VIN_MIN?p.VINTVCC:0;
    double pgfb=vo*p.PGFB_RATIO;
    bool good=enabled&&in.svin>=2.8&&pgfb>=.4625&&pgfb<=.5375;
    o.pg=good?std::max(0.,p.PG_PULLUP_V-40e-9*p.PG_PULLUP_R):p.PG_PULLUP_V*p.PG_RON/(p.PG_PULLUP_R+p.PG_RON);
    o.clkout=p.ENABLE_CLKOUT&&(now-cycle)<p.period*.5?p.VINTVCC:0;
    double ripple=in.pvin>.1?std::abs(ref*(1-ref/in.pvin)/(p.LOUT*p.FSW)):0;
    o.ilim=std::min(p.ILIM_TOP,p.IOUT_MAX+.5*ripple);
    o.state=!enabled?0:(!o.active?1:(limited?4:(ref<.99*std::min(clip(setv,0,p.VOUT_MAX),std::max(0.,in.svin-p.VHEAD))?2:3)));
    for(double v:{vo,sw,il,o.ipvin,o.power})if(!std::isfinite(v))throw std::runtime_error("Nonfinite circuit state");
    previous=now;started=true;return o;
}
double Engine::noise_density(double f,double load)const{
    if(!p.NOISE_CHAR_MODE)throw std::logic_error("Noise query requires NOISE_CHAR_MODE=1; closed-loop spectrum uncharacterized");
    if(!std::isfinite(f)||!std::isfinite(load)||f<=0||f>=p.FSW/10||load<=0)
        throw std::invalid_argument("Noise query: 0<f<FSW/10 and finite RLOAD>0 required");
    if(!p.ENABLE_NOISE)return 0;
    const double pi=std::acos(-1.);std::complex<double> s(0,2*pi*f);
    std::complex<double> cap=p.ESR+p.ESR_RESIDUAL_FIT+1./(s*p.COUT),z=1./(1./load+1./cap);
    double resistance=p.LDCR+p.NOISE_DUTY*p.RDS_TOP+(1-p.NOISE_DUTY)*p.RDS_BOT;
    double source=std::hypot(p.VNOISE_WHITE,p.VNOISE_LF_1HZ/std::pow(f,p.VNOISE_LF_EXP/2));
    return source*std::abs(z/(z+resistance+s*p.LOUT));
}
double Engine::noise_rms(double low_hz,double high_hz,double load)const{
    if(!std::isfinite(low_hz)||!std::isfinite(high_hz)||low_hz<=0||high_hz<=low_hz)
        throw std::invalid_argument("Invalid noise integration band");
    noise_density(low_hz,load);noise_density(high_hz,load);
    const unsigned n=4096;double a=std::log(low_hz),step=std::log(high_hz/low_hz)/n,sum=0;
    for(unsigned k=0;k<=n;++k){double f=std::exp(a+k*step),v=noise_density(f,load);sum+=(k==0||k==n?1:(k%2?4:2))*v*v*f;}
    return std::sqrt(sum*step/3);
}
LT8627SP::LT8627SP(sc_module_name name,const std::string& path):sca_tdf::sca_module(name),config(load_config(path)),engine(config){}
void LT8627SP::set_attributes(){set_timestep(engine.parameters().DT);}
void LT8627SP::processing(){
    Inputs in;
#define READ(k) in.k=k.read()
    READ(pvin);READ(svin);READ(en_uvlo);READ(sync_mode);READ(set_override);READ(sense_offset);READ(iload);READ(rload);
#undef READ
    Output o=engine.sample(get_time(),in);
#define WRITE(k) k.write(o.k)
    WRITE(vout);WRITE(sw);WRITE(il);WRITE(duty);WRITE(gh);WRITE(gl);WRITE(ipvin);WRITE(isvin);WRITE(power);WRITE(external_loss);WRITE(energy);
    WRITE(set_voltage);WRITE(vref);WRITE(intvcc);WRITE(pg);WRITE(clkout);WRITE(ilim);WRITE(active);WRITE(limited);WRITE(state);
#undef WRITE
    last_power=o.power;
}
std::string LT8627SP::get_module_name()const{return config.at("name").at("value").get<std::string>();}
double LT8627SP::get_version()const{return config.at("version").at("value").get<double>();}
std::string LT8627SP::get_category()const{return config.at("category").at("value").get<std::string>();}
} // namespace lt8627sp
