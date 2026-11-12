// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
// Source: supplied historical TPS62746 rev0.3; TI SLVSD27B.
// Application-level behavior; no silicon/transistor accuracy sign-off.
#include "tps62746.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <limits>
static double clip(double x,double a,double b){return std::max(a,std::min(x,b));}
TPS62746::configs TPS62746::get_configs_from_json(const char* json_path){CFGS_BEGIN(x) x.parameter=parameter;x.timestep_s=configuration.at("timestep_s");x.module_name=meta.at("name").get<std::string>();x.version=meta.at("version").get<double>();x.category=meta.at("category").get<std::string>();if(x.module_name!=j.at("name").at("value").get<std::string>()||x.version!=j.at("version").at("value").get<double>()||x.category!=j.at("category").at("value").get<std::string>())throw std::invalid_argument("inconsistent metadata");CFGS_END(x)}
double TPS62746::p(const char* k)const{return c.parameter.at(k).at("value");}
TPS62746::TPS62746(sc_core::sc_module_name n,const configs& cfg):sca_tdf::sca_module(n),c(cfg){
 if(!std::isfinite(c.timestep_s)||c.timestep_s<=0||c.timestep_s>p("tick"))throw std::invalid_argument("timestep outside (0,tick]");
 for(auto i=c.parameter.begin();i!=c.parameter.end();++i)if(!std::isfinite(i.value().at("value").get<double>()))throw std::invalid_argument("nonfinite parameter");
 for(const char* k:{"fsw","l_nom","tick","tr","soft_time","r_hi","r_lo","i_limit","i_soft","kp","ki","pfm_band"})if(p(k)<=0)throw std::invalid_argument(k);
 for(const char* k:{"start_delay","iq_sleep","iq_active","i_shutdown","vos_bias","e_switch"})if(p(k)<0)throw std::invalid_argument(k);
 if(p("uv_fall")>=p("uv_rise")||1/p("fsw")<=275e-9)throw std::invalid_argument("threshold or period");
}
void TPS62746::set_attributes(){set_timestep(c.timestep_s,sc_core::SC_SEC);}
void TPS62746::processing(){
 double g=GND.read(),vi=VIN.read()-g,vo=VOS.read()-g,il=inductor_current.read(),rl=vinsw_load.read();
 for(double x:{g,vi,vo,il,rl,VSEL.read(),EN.read(),CTRL.read()})if(!std::isfinite(x))SC_REPORT_FATAL("TPS62746","nonfinite input");
 if(rl<=0)SC_REPORT_FATAL("TPS62746","vinsw_load must be positive");
 double dt=c.timestep_s,t=get_time().to_seconds(),vn=VSEL.read()-g>.75?1.8:1.2;
 if(vi>=p("uv_rise"))uv=true;
 if(vi<=p("uv_fall"))uv=false;
 enabled=uv&&EN.read()-g>.75;
 if(enabled&&!old)te=t;
 old=enabled;double age=t-te-p("start_delay"),qloss=0;reference=0;
 if(!enabled||age<0){state=0;mode100=false;burst=false;integ=0;avg=0;}
 else{
  avg+=dt/(20e-6+dt)*(il-avg);
  reference=vn*(1+p("v_error")+p("line_slope")*(vi-3.6)-p("load_slope")*(std::max(avg,0.)-.010))*clip(age/p("soft_time"),0,1);
  double err=reference-vo,lim=age<p("soft_time")?p("i_soft"):p("i_limit");
  double ton=clip(vn/std::max(vi,.1)/p("fsw"),225e-9,1/p("fsw")-50e-9);
  double ripple=std::max(0.,(vi-vo)*ton/p("l_nom")),mx=std::max(0.,lim-.5*std::min(ripple,lim)),raw=integ+p("kp")*err;
  if(!((raw>=mx&&err>0)||(raw<=0&&err<0)))integ=clip(integ+p("ki")*err*dt,0,p("i_limit"));
  double demand=clip(integ+p("kp")*err,0,mx),valley=std::max(0.,demand-ripple/2);
  if(vi-vn<.200)mode100=true;
  if(vi-vn>.250)mode100=false;
  if(vo<reference-p("pfm_band")/2)burst=true;
  if(vo>reference+p("pfm_band")/2)burst=false;
  if(state==1){if(il>=lim||(!mode100&&t-edge>=hold)){state=2;edge=t;}}
  else{
   if(state==2&&il<=0)state=0;
   if(t-edge>=50e-9&&il<lim&&((mode100&&il<lim*.95)||(!mode100&&((valley>0&&il<=valley)||(state==0&&burst)||(state==2&&burst&&raw>=mx&&il<.90*lim))))){state=1;edge=t;hold=ton;qloss=p("e_switch")/dt/std::max(vi,.1);}
  }
 }
 // 开关节点包含两条体二极管，分段线性求解KCL。
 double gh=state==1?1/p("r_hi")+1e-12:1e-12,gl=state==2?1/p("r_lo")+1e-12:1e-12;
 high_conductance.write(gh);low_conductance.write(gl);
 double sw=(gh*vi-il)/(gh+gl);
 if(sw<-.5)sw=(gh*vi-il-5)/(gh+gl+10);
 else if(sw>vi+.5)sw=(gh*vi-il+10*(vi+.5))/(gh+gl+10);
 // 单调求解150 ohm / 5 mA限流的VINSW负载线，CTRL独立于EN。
 bool ctrl=CTRL.read()-g>.75;double lo=std::min(0.,vi),hi=std::max(0.,vi);
 for(int i=0;i<55;++i){double x=(lo+hi)/2;double current=ctrl?.005*std::tanh((vi-x)/.75):1e-12*(vi-x);if(current>x/rl+1e-12*x)lo=x;else hi=x;}
 double vs=(lo+hi)/2,iv=ctrl?.005*std::tanh((vi-vs)/.75):1e-12*(vi-vs);
 SW.write(sw+g);VINSW.write(vs+g);
 input_current.write(gh*(vi-sw)-std::max(sw-vi-.5,0.)*10+(enabled?(state==0?p("iq_sleep"):p("iq_active")):p("i_shutdown"))+qloss+iv);
 const double ibias=enabled?(state==0?p("iq_sleep"):p("iq_active")):p("i_shutdown");
  const double ivos=enabled?p("vos_bias")*std::max(vo,0.)/1.8:vo/30.;
  vos_current.write(ivos);
  // Explicit channel/diode, bias, configured switching energy, VINSW and
  // discharge losses. External L/C/DCR belong to the attached test network.
  last_power_w_=gh*(vi-sw)*(vi-sw)+gl*sw*sw+
    std::max(-sw-.5,0.)*10*(-sw)+std::max(sw-vi-.5,0.)*10*(sw-vi)+
    std::max(vi,0.)*(ibias+qloss)+std::max((vi-vs)*iv,0.)+std::max(vo*ivos,0.);
}
void TPS62746::trace_me(sca_util::sca_trace_file* f){sca_util::sca_trace(f,SW,"SW");sca_util::sca_trace(f,VINSW,"VINSW");sca_util::sca_trace(f,input_current,"input_current");}

// Unknown die area/physical latency remain NaN. Runtime branch loss is available but uncalibrated.
double TPS62746::get_latency() const { return std::numeric_limits<double>::quiet_NaN(); }
double TPS62746::get_area() const { return std::numeric_limits<double>::quiet_NaN(); }
double TPS62746::get_power_dissipation() const {return last_power_w_;}
