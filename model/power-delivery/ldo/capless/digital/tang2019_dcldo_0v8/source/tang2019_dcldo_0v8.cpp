// Key sources: Tang et al. IEEE TIE 2019, DOI 10.1109/TIE.2018.2840492; supplied VA v2.
// Digital cell sizing and shift delay remain engineering assumptions; see JSON source_documents.
#include "tang2019_dcldo_0v8.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
using nlohmann::json;
double Tang2019_DCLDO::n(const std::string& k)const{return c_.at(k).at("value").get<double>();}
std::string Tang2019_DCLDO::s(const std::string& k)const{return c_.at(k).at("value").get<std::string>();}
bool Tang2019_DCLDO::b(const std::string& k)const{return c_.at(k).at("value").get<bool>();}
Tang2019_DCLDO::Tang2019_DCLDO(sc_core::sc_module_name nm,const std::string& path):sca_tdf::sca_module(nm){
 std::ifstream f(path);if(!f)throw std::runtime_error("Cannot open configuration");f>>c_;
 // Every scalar is typed and finite; unknown measurements are explicit nulls.
 for(auto it=c_.begin();it!=c_.end();++it){
  if(it.key()=="dimensions") {
   const auto& q=it.value();if(!q.is_object()||!q.contains("x")||!q.contains("y")||!q["x"].is_number()||!q["y"].is_number()||q["x"].get<double>()<=0||q["y"].get<double>()<=0)throw std::runtime_error("Invalid icon dimensions");continue;
  }
  if(it.key()=="ports") {
   const auto& q=it.value();if(!q.is_array())throw std::runtime_error("Invalid ports metadata");
   for(const auto& pin:q)if(!pin.is_object()||!pin.contains("name")||!pin["name"].is_string()||!pin.contains("io_type")||!pin.contains("data_type")||!pin.contains("unit")||!pin.contains("x")||!pin.contains("y"))throw std::runtime_error("Invalid port metadata");
   continue;
  }
  const auto& q=it.value();if(!q.is_object()||!q.contains("type")||!q.contains("value")||!q.contains("editable")||!q.contains("unit"))throw std::runtime_error("Missing metadata: "+it.key());
  const auto& v=q.at("value");std::string t=q.at("type");bool ok=(t=="float"&&v.is_number())||(t=="int"&&v.is_number_integer())||(t=="string"&&v.is_string())||(t=="bool"&&v.is_boolean())||(t=="array"&&v.is_array())||(t=="unknown"&&v.is_null())||(t=="nullable_float"&&(v.is_null()||v.is_number()));
  if(!ok || (v.is_number()&&!std::isfinite(v.get<double>())))throw std::runtime_error("Invalid typed value: "+it.key());
 }
 dt_=n("dt_s");for(auto k:{"dt_s","shift_delay_s","cell_min_a","cell_max_a","cell_slope_v","vnom_v","vdo_v","vin_nom_v","rout_va_ohm","icl_num_a","tset_va_s","vsmooth_v","psrr_fz_hz","psrr_fp_hz","active_area_mm2"})if(n(k)<=0)throw std::runtime_error(std::string("Expected positive ")+k);
 if(n("cout_f")<0||n("gmin_s")<0||n("iq_min_a")<0||n("iq_max_a")<n("iq_min_a")||n("load_min_a")<=0||n("load_max_a")<=n("load_min_a")||n("cell_max_a")<=n("cell_min_a")||n("switches")<2||n("switches")>10000||n("initial_code")<0||n("initial_code")>n("switches")||!(n("vlow_v")<n("vnom_v")&&n("vnom_v")<n("vhigh_v"))||n("psrr_h0")<0)throw std::runtime_error("Invalid physical or controller parameter");
 if(s("profile")!="window_digital"&&s("profile")!="va_v2")throw std::runtime_error("Invalid profile");
 if(s("initial_state")!="dc"&&s("initial_state")!="uncharged")throw std::runtime_error("Invalid initial state");
 if(s("analysis")!="transient"&&s("analysis")!="small_signal")throw std::runtime_error("Invalid analysis");
 if(s("psrr_source")!="vin"&&s("psrr_source")!="ripple")throw std::runtime_error("Invalid PSRR source");
 if(s("profile")=="window_digital"&&(dt_>n("shift_delay_s")||dt_<1e-12))throw std::runtime_error("Digital timestep must be >=1ps and <=shift delay");
 k_=static_cast<int>(n("initial_code"));
}
std::string Tang2019_DCLDO::get_module_name()const{return s("name");}
double Tang2019_DCLDO::get_version()const{return n("version");}
std::string Tang2019_DCLDO::get_category()const{return s("category");}
double Tang2019_DCLDO::get_timestep_s()const{return dt_;}
double Tang2019_DCLDO::get_latency()const{return s("profile")=="window_digital"?n("shift_delay_s"):n("tset_va_s")/4;}
double Tang2019_DCLDO::get_active_area_mm2()const{return n("active_area_mm2");}
void Tang2019_DCLDO::set_attributes(){set_timestep(dt_,sc_core::SC_SEC);}
double Tang2019_DCLDO::target(double vi)const{
 if(s("analysis")=="small_signal")return n("vnom_v");
 double a=n("vnom_v"),h=vi-n("vdo_v"),e=n("vsmooth_v");
 return .5*(a+h-std::hypot(a-h,e));
}
double Tang2019_DCLDO::capacity(int k)const{
 if(k==0)return 0.;
 // Assumed monotonic cumulative sizing; paper specifies N=28 but no widths.
 return n("cell_min_a")*std::pow(n("cell_max_a")/n("cell_min_a"),double(k-1)/(n("switches")-1));
}
double Tang2019_DCLDO::pass(double vo,double vi,int k)const{
 if(s("profile")=="va_v2")return n("icl_num_a")*std::tanh((target(vi)-vo)/(n("icl_num_a")*n("rout_va_ohm")));
 double head=std::max(0.,std::min(1.,(vi-vo)/n("vdo_v")));
 return capacity(k)*std::exp(std::max(-50.,std::min(50.,(n("vnom_v")-vo)/n("cell_slope_v"))))*head;
}
double Tang2019_DCLDO::solve(double vi,double load,double rl,int k,double old,double alpha)const{
 // Monotonic KCL root: pass = Iload + V/Rload + Gmin*V + C*dV/dt.
 const bool va=s("profile")=="va_v2";
 const double ic=n("icl_num_a"),den=ic*n("rout_va_ohm"),tar=target(vi),cp=capacity(k),slope=n("cell_slope_v"),nom=n("vnom_v"),vdo=n("vdo_v"),g=n("gmin_s");
 double lo=0,hi=std::max(vi,n("vnom_v"))+2.;
 for(int j=0;j<62;j++){
  double mid=(lo+hi)/2;
  double current=va?ic*std::tanh((tar-mid)/den):cp*std::exp(std::max(-50.,std::min(50.,(nom-mid)/slope)))*std::max(0.,std::min(1.,(vi-mid)/vdo));
  double err=current-load-mid/rl-g*mid-alpha*(mid-old);
  if(err>0)lo=mid;else hi=mid;
 }
 return (lo+hi)/2;
}
void Tang2019_DCLDO::processing(){
 double vi=vin.read(),load=iload.read(),rl=rload.read(),ri=ripple.read();
 if(!std::isfinite(vi)||!std::isfinite(load)||!std::isfinite(rl)||!std::isfinite(ri)||vi<0||load<0||rl<=0)throw std::runtime_error("Invalid stimulus; require finite VIN>=0 ILOAD>=0 RLOAD>0");
 double bias=s("analysis")=="small_signal"?n("vin_nom_v"):vi;
 double x=s("psrr_source")=="vin"?vi-n("vin_nom_v"):ri;
 bool init=first_;int ac=0;
 if(init){
  lp_=x;xprev_=x;
  if(s("initial_state")=="dc"){
   if(s("profile")=="window_digital"){
    double best=1e100;
    for(int k=0;k<=n("switches");k++){double z=solve(bias,load,rl,k,0,0),e=std::abs(z-n("vnom_v"));if(e<best){best=e;k_=k;v_=z;}}
   }else v_=solve(bias,load,rl,0,0,0);
  }
  first_=false;
 }else{
  if(s("profile")=="window_digital"){
   timer_+=dt_;
   if(timer_+1e-18>=n("shift_delay_s")){
    timer_=std::fmod(timer_+1e-18,n("shift_delay_s"));
    if(v_<n("vlow_v")&&k_<n("switches")){++k_;ac=1;}
    else if(v_>n("vhigh_v")&&k_>0){--k_;ac=-1;}
   }
  }
  double cap=n("cout_f")+(s("profile")=="va_v2"?n("tset_va_s")/(4*n("rout_va_ohm")):0.);
  v_=solve(bias,load,rl,k_,v_,cap/dt_);
  // Exact low-pass state for a linearly interpolated input over this sample.
  // This avoids a frequency-dependent ZOH feedthrough error near the pole.
  double a=2*3.14159265358979323846*n("psrr_fp_hz")*dt_,q=-std::expm1(-a);
  lp_=(1-q)*lp_+q*xprev_+(1-q/a)*(x-xprev_);xprev_=x;
 }
 double h0=n("psrr_h0"),hinf=h0*n("psrr_fp_hz")/n("psrr_fz_hz");
 double ps=b("psrr_enable")?hinf*x+(h0-hinf)*lp_:0.,vo=v_+ps,ip=pass(v_,bias,k_);
 double frac=std::max(0.,std::min(1.,(ip-n("load_min_a"))/(n("load_max_a")-n("load_min_a"))));
 double ground=s("profile")=="va_v2"?n("iq_min_a")+(n("iq_max_a")-n("iq_min_a"))*.5*(1+std::tanh((ip-.5*n("load_max_a"))/(.18*n("load_max_a")))):n("iq_min_a")+frac*(n("iq_max_a")-n("iq_min_a"));
 double ii=ip+ground,actual=load+v_/rl,unmet=std::max(0.,actual+n("gmin_s")*v_-ip);
 // unmet is reported only if pinned at zero, not capacitor current during a step.
 if(v_>1e-9)unmet=0;
 int st=load<n("load_min_a")||load>n("load_max_a")?2:0;
 if(unmet>1e-9)st=3;
 if(s("profile")=="window_digital"&&(k_==0||k_==n("switches"))&&(v_<n("vlow_v")||v_>n("vhigh_v")))st=1;
 last_power_=(vi-v_)*ip+vi*ground;
 vout.write(vo);vcore.write(v_);iin.write(ii);iq.write(ground);ipass.write(ip);power.write(last_power_);psrr_voltage.write(ps);code.write(k_);action.write(ac);state.write(st);unmet_load.write(unmet);
 eta_current.write(ii>0?actual/ii:0.);eta_power.write(vi*ii>0?vo*actual/(vi*ii):0.);
}
