// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
// Source: supplied historical MCP1623 rev0.5; Microchip DS40001420E.
// Application-level behavior; no silicon/transistor accuracy sign-off.
#include "mcp1623.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <limits>
static double clip(double x,double a,double b){return std::max(a,std::min(x,b));}
MCP1623::configs MCP1623::get_configs_from_json(const char* json_path){CFGS_BEGIN(x) x.parameter=parameter;x.timestep_s=configuration.at("timestep_s");x.module_name=meta.at("name").get<std::string>();x.version=meta.at("version").get<double>();x.category=meta.at("category").get<std::string>();if(x.module_name!=j.at("name").at("value").get<std::string>()||x.version!=j.at("version").at("value").get<double>()||x.category!=j.at("category").at("value").get<std::string>())throw std::invalid_argument("inconsistent metadata");CFGS_END(x)}
double MCP1623::p(const char* k)const{return c.parameter.at(k).at("value");}
MCP1623::MCP1623(sc_core::sc_module_name n,const configs& cfg):sca_tdf::sca_module(n),c(cfg){
 for(auto it=c.parameter.begin();it!=c.parameter.end();++it)if(!std::isfinite(it.value().at("value").get<double>()))throw std::invalid_argument("nonfinite parameter");
 for(const char* k:{"FSW","VREF","ILIM","VSTART","RN","RP","KP","KI","L_SLOPE","TBLANK","TEDGE","TSS_RAMP","IPRE","VRUN_FLOOR","CSW","R_DAMP","R_NEG_CLAMP","KAW"})if(p(k)<=0)throw std::invalid_argument(k);
 for(const char* k:{"IQ_PWM","IQ_OFF","P_SWITCH"})if(p(k)<0)throw std::invalid_argument(k);
 if(p("FSW")<370e3||p("FSW")>630e3||p("VREF")<1.120||p("VREF")>1.299||p("ILIM")<.300||p("ILIM")>.600||p("VSTART")<.5||p("VSTART")>.8||p("DMAX")<=.70||p("DMAX")>=.99||p("TBLANK")>=150e-9||p("TEDGE")>=10e-9||p("VRUN_FLOOR")>=.35)throw std::invalid_argument("source parameter range");
 if(!std::isfinite(c.timestep_s)||c.timestep_s<=0||c.timestep_s>5e-9||c.timestep_s>p("TBLANK")/10)throw std::invalid_argument("timestep must resolve blanking");
 next=1/p("FSW");
}
void MCP1623::set_attributes(){set_timestep(c.timestep_s,sc_core::SC_SEC);}
void MCP1623::processing(){
 double g=SGND.read(),vi=VIN.read()-g,vo=VOUTS.read()-g,en=EN.read()-g,fb=VFB.read()-g,il=inductor_current.read();
 for(double x:{g,vi,vo,en,fb,il,SW.read(),PGND.read(),EP.read(),VOUTP.read(),rectifier_current.read()})if(!std::isfinite(x))SC_REPORT_FATAL("MCP1623","nonfinite input");
 if(std::abs(PGND.read()-g)>1e-6||std::abs(EP.read()-g)>1e-6)SC_REPORT_FATAL("MCP1623","PGND SGND EP must share reference");
 if(std::abs(VOUTP.read()-VOUTS.read())>1e-6)SC_REPORT_FATAL("MCP1623","VOUTP VOUTS must be tied");
 double dt=c.timestep_s,t=get_time().to_seconds(),period=1/p("FSW");
 delivered+=dt/(50e-6+dt)*(rectifier_current.read()-delivered);
 if(en<=.20*vi){active=false;mode=0;gate=false;damp=false;cycle_on=false;}
 bool edge=t+dt*1e-5>=next;
 if(edge){
  cycle=next;next+=period;gate=false;damp=false;cycle_on=false;
  if(en<=.20*vi||vi<p("VRUN_FLOOR")){active=false;mode=0;}
  else if(!active&&en>=.90*vi&&vi>=p("VSTART")){active=true;start=t;mode=1;}
  if(active){if(vo<std::min(.90*vi,1.50))mode=1;else if(vi<1.60&&vo<1.60)mode=2;else mode=3;}
 }
 double ramp=clip((t-start)/p("TSS_RAMP"),0,1),age=std::max(t-cycle,0.);
 limit=mode==2?.5*p("ILIM"):p("ILIM")*(.5+.5*ramp);
 slope=.5*std::max(vo-vi,.10)/p("L_SLOPE");
 double scale=std::sqrt(3.3/std::max(std::max(vi,vo),1.)),iav=.5*limit;
 double dest=clip((vo-vi+iav*p("RP")*scale)/std::max(vo+iav*(p("RP")-p("RN"))*scale,.1),0,p("DMAX"));
 double ceiling=limit+slope*dest*period;
 reference=p("VREF")*(1+p("LINE_FRAC")*(vi-1.5)-p("LOAD_FRAC")*(std::max(delivered-p("IQ_PWM"),0.)-.025));
 double err=reference-fb,raw=integrator+p("KP")*err;
 command=clip(raw,0,ceiling);
 if(edge&&active){if(mode==2||(mode==3&&raw>0)){gate=true;cycle_on=true;}else if(mode==3)damp=true;}
 // 用每拍比较替代VA的cross事件；100 ns消隐与最大占空比仍独立生效。
 if(gate&&age>=p("TBLANK")&&(il>=limit||(mode==3&&il+slope*age>=command)))gate=false;
 if((mode==2&&age>=.70*period)||age>=p("DMAX")*period)gate=false;
 if(active&&mode>=2&&!gate&&il<=1e-6)damp=true;
 if(mode==3)integrator+=dt*(p("KI")*err+p("KAW")*(command-raw));
 else integrator/=1+dt/10e-6;
 double mix=.5*(1+std::tanh((vo-vi-.02)/.02));
 double ibin=(active?p("IQ_PWM")*(1-mix):p("IQ_OFF"))*std::tanh(std::max(vi,0.)/.05);
 double ibout=(active?p("IQ_PWM")*mix:0)*std::tanh(std::max(vo,0.)/.05);
 double neg=.5*(vo-std::sqrt(vo*vo+1e-8));
 ibout+=(active&&cycle_on?p("P_SWITCH")/std::max(vo,.5):0)+neg/p("R_NEG_CLAMP");
 low_conductance.write((gate?1/(p("RN")*scale):0)+1e-10);
 damping_conductance.write(damp?1/p("R_DAMP"):0);
 rectifier_enabled.write(active?1:0);precharge_enabled.write(mode==1?1:0);
 input_bias_current.write(ibin);output_sink_current.write(ibout);
 feedback_current.write(10e-12*std::tanh(fb/.05));enable_current.write(1e-9*en);
  // Approximate dissipative branches, using externally solved SW/current.
  // RP*scale is the same source-VA effective rectifier resistance. No die
  // accuracy is implied; external inductor DCR is deliberately excluded.
  const double sw=SW.read()-g,ir=rectifier_current.read();
  last_power_w_=std::max(0.,vi*ibin+vo*ibout)+
    sw*sw*((gate?1/(p("RN")*scale):0)+1e-10)+
    (damp?sw*sw/p("R_DAMP"):0)+(active?ir*ir*p("RP")*scale:0)+
    std::abs(fb*10e-12*std::tanh(fb/.05))+1e-9*en*en;
}
void MCP1623::trace_me(sca_util::sca_trace_file* f){sca_util::sca_trace(f,low_conductance,"low_conductance");sca_util::sca_trace(f,output_sink_current,"output_sink_current");sca_util::sca_trace(f,VOUTS,"VOUT");}

// Unknown die area/physical latency remain NaN. Runtime branch loss is available but uncalibrated.
double MCP1623::get_latency() const { return std::numeric_limits<double>::quiet_NaN(); }
double MCP1623::get_area() const { return std::numeric_limits<double>::quiet_NaN(); }
double MCP1623::get_power_dissipation() const {return last_power_w_;}
