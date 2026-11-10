// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#include "tps546a24a.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <limits>

namespace tps546a24a {
namespace {
double bound(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
double value(const nlohmann::json& j,const char* key){
    double v=j.at(key).at("value").get<double>();
    if(!std::isfinite(v))throw std::invalid_argument(std::string(key)+" must be finite");
    return v;
}
unsigned device_count(const nlohmann::json& j){
    double n=value(j,"device_count");
    if(n<1||n>4||std::floor(n)!=n)throw std::invalid_argument("TPS546A24A supports only 1..4 devices");
    return static_cast<unsigned>(n);
}
double edge(double& actual,double target,double h,double duration){
    double distance=std::abs(target-actual);
    if(distance==0)return actual;
    double reach=distance*duration;
    double end=actual+(target>actual?1:-1)*std::min(distance,h/duration);
    double mean=h>=reach?target+(actual-target)*reach/(2*h):(actual+end)/2;
    actual=end;return mean;
}
// Stable positive-part smoothing, identical algebra to the supplied power VA.
double diode(double x,double smooth){
    double r=std::hypot(x,smooth);
    return x>=0?.5*(x+r):.5*smooth*smooth/(r-x);
}
}
nlohmann::json load_config(const std::string& path){
    std::ifstream f(path);if(!f)throw std::runtime_error("Cannot open configuration: "+path);
    nlohmann::json j;f>>j;
    if(j.at("name").at("value")!="tps546a24a"||j.at("version").at("value")!=.1)
        throw std::invalid_argument("Unsupported model identity");
    return j;
}
Parameters::Parameters(const nlohmann::json& j){
    devices=device_count(j);
#define REAL(k) k=value(j,#k)
    REAL(VOUT_SET);REAL(FSW);REAL(VEN_ON);REAL(VEN_OFF);REAL(VIN_ON);REAL(VIN_OFF);
    REAL(SENSE_SCALE);REAL(KP);REAL(KI);REAL(RI);REAL(IREF_MAX);REAL(IREF_MIN);REAL(DMAX);
    REAL(RHS);REAL(RLS);REAL(RSWPD);REAL(GOFF);REAL(VDIODE);REAL(RDIODE);REAL(VSMOOTH);
    REAL(CSW);REAL(COUT);REAL(ESR);REAL(INITIAL_VOUT);
#undef REAL
#define TIME(k) {double v=value(j,#k);if(v<0)throw std::invalid_argument(#k " negative");k=sc_time(v,sc_core::SC_SEC);}
    TIME(TSS);TIME(TDEAD);TIME(TEDGE);TIME(TON_NUM);TIME(DT);
#undef TIME
    for(const char* key:{"L","DCR"}){
        auto a=j.at(key).at("value").get<std::vector<double>>();
        if(a.size()!=4)throw std::invalid_argument(std::string(key)+" must contain four device slots");
        for(double v:a)if(!std::isfinite(v)||(std::string(key)=="L"?v<=0:v<0))
            throw std::invalid_argument(std::string(key)+" invalid");
        if(std::string(key)=="L")L=a;else DCR=a;
    }
    for(double v:{VOUT_SET,FSW,SENSE_SCALE,KI,RI,IREF_MAX,RHS,RLS,RSWPD,GOFF,RDIODE,VSMOOTH,CSW,COUT})
        if(v<=0)throw std::invalid_argument("Nonpositive required electrical parameter");
    if(VOUT_SET<.25||VOUT_SET>5.5||FSW<225e3||FSW>1.5e6||VEN_OFF>=VEN_ON||VIN_OFF>=VIN_ON||
       IREF_MIN>=0||DMAX<=0||DMAX>=1||KP<0||ESR<0||VDIODE<0||INITIAL_VOUT<0)
        throw std::invalid_argument("Invalid TPS546A24A operating/configuration parameter");
    period=sc_time(1/FSW,sc_core::SC_SEC);
    if(DT==sc_core::SC_ZERO_TIME||DT>sc_time(5,sc_core::SC_NS)||TSS==sc_core::SC_ZERO_TIME||
       TDEAD==sc_core::SC_ZERO_TIME||TEDGE==sc_core::SC_ZERO_TIME||TDEAD*2>=period)
        throw std::invalid_argument("Invalid timing or DT >5ns");
    if(sc_core::sc_get_time_resolution()>sc_time(1,sc_core::SC_PS))
        throw std::invalid_argument("Use SystemC time resolution <=1ps");
}
Engine::Engine(const nlohmann::json& j):p(j),phase(p.devices),vc(p.INITIAL_VOUT),out(p.INITIAL_VOUT){
    for(unsigned k=0;k<p.devices;++k)phase[k].next=p.period+p.period*(double(k)/p.devices);
}
void Engine::tick(unsigned index,sc_time t,const Inputs& in){
    auto& s=phase[index];
    // First local cycle integrates from t=0; VA-equivalent first master interval=T.
    double interval=s.cycle==sc_core::SC_ZERO_TIME?t.to_seconds():p.period.to_seconds();
    s.iavg=s.integral_i/interval;s.vavg=s.integral_v/interval;
    s.integral_i=s.integral_v=0;s.cycle=t;s.next=t+p.period;
    if(in.en>=p.VEN_ON)s.en=true;else if(in.en<=p.VEN_OFF)s.en=false;
    if(in.vin>=p.VIN_ON)s.uv=true;else if(in.vin<=p.VIN_OFF)s.uv=false;
    if(index==0){
        bool enable=s.en&&s.uv;
        if(!enable){master_running=false;ix=shared_iref=vref=0;}
        else{
            if(!master_running){start=t;master_running=true;ix=0;}
            vref=p.VOUT_SET*bound((t-start)/p.TSS,0,1);
            double error=vref-s.vavg,raw=p.KP*error+ix;
            double hi=p.IREF_MAX*p.devices,lo=p.IREF_MIN*p.devices;
            if(!((raw>=hi&&error>0)||(raw<=lo&&error<0)))
                ix=bound(ix+p.KI*error/p.FSW,lo,hi);
            shared_iref=bound((p.KP*error+ix)/p.devices,p.IREF_MIN,p.IREF_MAX);
        }
    }
    s.running=s.en&&s.uv&&master_running;
    if(!s.running){s.duty=0;s.high_end=sc_core::SC_ZERO_TIME;return;}
    // Device-specific VSHARE abstraction: common per-device current command,
    // independent local cycle-average current loops. N=1 is the supplied VA PI.
    s.duty=bound((s.vavg+p.RI*(shared_iref-s.iavg))/std::max(in.vin,1.0),0,p.DMAX);
    if(s.duty/p.FSW<p.TON_NUM.to_seconds())s.duty=0;
    s.high_end=s.duty>0?std::min(sc_time(p.DMAX/p.FSW,sc_core::SC_SEC),
        p.TDEAD+sc_time(s.duty/p.FSW,sc_core::SC_SEC)):sc_core::SC_ZERO_TIME;
}
void Engine::commands(sc_time now){
    for(auto& s:phase){
        sc_time position=now-s.cycle;
        s.high=s.running&&s.high_end>sc_core::SC_ZERO_TIME&&position>=p.TDEAD&&position<s.high_end;
        s.low=s.running&&position>=s.high_end+p.TDEAD&&position<p.period;
    }
}
sc_time Engine::next_event(sc_time now,sc_time end)const{
    sc_time result=end;
    for(const auto& s:phase){
        for(sc_time t:{s.next,s.cycle+p.TDEAD,s.cycle+s.high_end,s.cycle+s.high_end+p.TDEAD})
            if(t>now&&t<result)result=t;
    }
    return result;
}
Output Engine::advance(double h,const Inputs& in){
    const unsigned n=p.devices;
    double hgate[4]={},lgate[4]={},aa[4]={},bb[4]={},alpha[4]={},beta[4]={};
    double guesses[4]={},newil[4]={},newsw[4]={};
    double z=p.ESR+h/p.COUT,gload=1/in.rload,newout=out;
    for(unsigned k=0;k<n;++k){
        hgate[k]=edge(phase[k].gh,phase[k].high?1:0,h,p.TEDGE.to_seconds());
        lgate[k]=edge(phase[k].gl,phase[k].low?1:0,h,p.TEDGE.to_seconds());
        guesses[k]=phase[k].sw;
    }
    bool converged=false;
    for(unsigned iteration=0;iteration<30;++iteration){
        double suma=0,sumb=0;
        for(unsigned k=0;k<n;++k){
            double x=guesses[k]-in.vin-p.VDIODE,y=-guesses[k]-p.VDIODE;
            double current=(diode(x,p.VSMOOTH)-diode(y,p.VSMOOTH))/p.RDIODE;
            double g=.5*(2+x/std::hypot(x,p.VSMOOTH)+y/std::hypot(y,p.VSMOOTH))/p.RDIODE;
            double highg=p.GOFF+hgate[k]/p.RHS,lowg=p.GOFF+lgate[k]/p.RLS+1/p.RSWPD;
            aa[k]=p.CSW/h+highg+lowg+g;
            bb[k]=p.CSW/h*phase[k].sw+highg*in.vin-current+g*guesses[k];
            double den=1+h/p.L[k]*(p.DCR[k]+1/aa[k]);
            alpha[k]=(phase[k].il+h/p.L[k]*bb[k]/aa[k])/den;
            beta[k]=(h/p.L[k])/den;suma+=alpha[k];sumb+=beta[k];
        }
        newout=(vc+z*(suma-in.iload))/(1+z*(gload+sumb));
        double change=0;
        for(unsigned k=0;k<n;++k){
            newil[k]=alpha[k]-beta[k]*newout;newsw[k]=(bb[k]-newil[k])/aa[k];
            change=std::max(change,std::abs(newsw[k]-guesses[k]));guesses[k]=newsw[k];
        }
        if(change<1e-9){converged=true;break;}
    }
    if(!converged)throw std::runtime_error("Smooth diode network did not converge");
    Output o;o.vout=newout;
    for(unsigned k=0;k<n;++k){
        auto& s=phase[k];s.integral_i+=h*(s.il+newil[k])/2;s.integral_v+=h*(out+newout)/2;
        s.il=newil[k];s.sw=newsw[k];o.total_current+=s.il;
        double dhi=diode(s.sw-in.vin-p.VDIODE,p.VSMOOTH)/p.RDIODE;
        double dlo=diode(-s.sw-p.VDIODE,p.VSMOOTH)/p.RDIODE;
        double gH=p.GOFF+hgate[k]/p.RHS,gL=p.GOFF+lgate[k]/p.RLS+1/p.RSWPD;
        o.iin+=gH*(in.vin-s.sw)-dhi;
        o.power+=gH*std::pow(in.vin-s.sw,2)+gL*s.sw*s.sw+dhi*(s.sw-in.vin)-dlo*s.sw;
        o.external_loss+=p.DCR[k]*s.il*s.il;
        o.energy+=.5*(p.L[k]*s.il*s.il+p.CSW*s.sw*s.sw);
    }
    double ic=o.total_current-in.iload-gload*newout;
    vc+=h/p.COUT*ic;out=newout;
    o.external_loss+=p.ESR*ic*ic;o.energy+=.5*p.COUT*vc*vc;
    return o;
}
Output Engine::sample(sc_time now,const Inputs& in){
    for(double v:{in.vin,in.en,in.iload,in.rload})if(!std::isfinite(v))throw std::invalid_argument("Nonfinite input");
    if(in.vin<0||in.en<0||in.rload<=0)throw std::invalid_argument("VIN/EN>=0, RLOAD>0 required");
    if((!started&&now!=sc_core::SC_ZERO_TIME)||(started&&now-previous!=p.DT))
        throw std::invalid_argument("Sample on the configured DT grid starting at zero");
    Output result;result.vout=out;
    if(started){
        sc_time cursor=previous;
        double isum=0,loss=0,external=0;
        while(cursor<now){
            for(unsigned k=0;k<p.devices;++k)if(cursor==phase[k].next)tick(k,cursor,in);
            commands(cursor);sc_time end=next_event(cursor,now);double h=(end-cursor).to_seconds();
            auto o=advance(h,in);isum+=o.iin*h;loss+=o.power*h;external+=o.external_loss*h;
            result=o;cursor=end;
        }
        result.iin=isum/p.DT.to_seconds();result.power=loss/p.DT.to_seconds();result.external_loss=external/p.DT.to_seconds();
    }
    for(unsigned k=0;k<p.devices;++k)if(now==phase[k].next)tick(k,now,in);
    commands(now);result.vref=vref;result.iref=shared_iref;result.active=master_running;
    result.state=!phase[0].en?0:(!phase[0].uv?3:(!master_running?0:(vref<p.VOUT_SET?1:2)));
    if(!std::isfinite(out)||!std::isfinite(result.iin))throw std::runtime_error("Nonfinite simulation state");
    previous=now;started=true;return result;
}
TPS546A24A::TPS546A24A(sc_core::sc_module_name name,const std::string& path):TPS546A24A(name,load_config(path),false){}
TPS546A24AStack::TPS546A24AStack(sc_core::sc_module_name name,const std::string& path):TPS546A24A(name,load_config(path),true){}
TPS546A24A::TPS546A24A(sc_core::sc_module_name name,const nlohmann::json& j,bool stacked)
    :sca_tdf::sca_module(name),sw("sw",device_count(j)),
     il("il",sw.size()),isns("isns",sw.size()),duty("duty",sw.size()),gh("gh",sw.size()),gl("gl",sw.size()),cfg(j),engine(j){
    if((!stacked&&engine.parameters().devices!=1)||(stacked&&engine.parameters().devices<2))
        throw std::invalid_argument("Use TPS546A24A for one device; TPS546A24AStack for 2..4 devices");
}
void TPS546A24A::set_attributes(){set_timestep(engine.parameters().DT);}
void TPS546A24A::processing(){
    Inputs in;in.vin=vin.read();in.en=en.read();in.iload=iload.read();in.rload=rload.read();
    Output o=engine.sample(get_time(),in);
#define WRITE(k) k.write(o.k)
    WRITE(vout);WRITE(iin);WRITE(power);WRITE(external_loss);WRITE(energy);WRITE(vref);WRITE(iref);WRITE(total_current);WRITE(active);WRITE(state);
#undef WRITE
    for(unsigned k=0;k<sw.size();++k){const auto& s=engine.phases()[k];sw[k].write(s.sw);il[k].write(s.il);isns[k].write(engine.parameters().SENSE_SCALE*s.il);duty[k].write(s.duty);gh[k].write(s.high?1:0);gl[k].write(s.low?1:0);}
    last_power=o.power;
}
std::string TPS546A24A::get_module_name()const{return cfg.at("name").at("value").get<std::string>();}
double TPS546A24A::get_version()const{return cfg.at("version").at("value").get<double>();}
std::string TPS546A24A::get_category()const{return cfg.at("category").at("value").get<std::string>();}
} // namespace tps546a24a
