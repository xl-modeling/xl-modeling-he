#include "lm3350_sw.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <vector>
// Sources: National LM3350, 2/3 Step-Down Electrical Characteristics printed p4/PDF p5; lm3350_sw.va.
// VA migration: same switch graph and fitted controls; backward-Euler RC
// companions, event-aligned piecewise-linear gates, zero-order-held TDF inputs.
// Capacitors/ESR and LM2665 startup diode are the APPLICATION network, not die.
// No forced output voltage, load-dependent ideal source or injected ripple.
namespace {
constexpr int K=3, N=5, OUT=4, NEG=-1;
constexpr int GND=-1, VIN=-2;
double clamp(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
struct Branch {int a,b;double g,source;bool external;};
struct Cap {int a,b;double c,r,v=0;};
using Vec=std::array<double,6>;
using Mat=std::array<std::array<double,6>,6>;
Vec solve(Mat a,Vec b){
 for(int k=0;k<N;++k){int pivot=k;for(int i=k+1;i<N;++i)if(std::abs(a[i][k])>std::abs(a[pivot][k]))pivot=i;
  if(std::abs(a[pivot][k])<1e-22)throw std::runtime_error("singular charge-pump network");
  std::swap(a[k],a[pivot]);std::swap(b[k],b[pivot]);
  for(int i=k+1;i<N;++i){double q=a[i][k]/a[k][k];for(int j=k+1;j<N;++j)a[i][j]-=q*a[k][j];b[i]-=q*b[k];}}
 Vec x{};for(int i=N-1;i>=0;--i){double s=b[i];for(int j=i+1;j<N;++j)s-=a[i][j]*x[j];x[i]=s/a[i][i];}return x;
}
}
struct LM3350_SW::Impl {
 nlohmann::json cfg;Vec x{};std::vector<Cap> caps;double phase=0,filtered=0,soft=0;
 double vin=0,control=0,temp=25,load=0,rload=1e12,loadneg=0,rloadneg=1e12;
 double freq=0,gatea=0,gateb=0,last_power=0,ein=0,pex=0,pnum=0,err=0,iq=0,en=1;
 bool skip=false;double p(const char* key)const{return cfg.at(key).at("value").get<double>();}
 explicit Impl(const std::string& file){
  std::ifstream in(file);if(!in)throw std::invalid_argument("cannot open JSON: "+file);in>>cfg;
  const std::vector<std::string> expected={"name","version","category","initial_state","dt_s","max_internal_step_s","fsw","ron","iq","tr_edge","dead","freq_scale","r_scale","cfly_f","cout_f","cfly_esr_ohm","cout_esr_ohm","cfly2_f","cfly2_esr_ohm","gmin_s","pin_cap_f","vin_min_v","vin_max_v","vin_absolute_max_v","temperature_min_c","temperature_max_c","area_mm2","latency_s","dimensions","ports"};
  for(auto it=cfg.begin();it!=cfg.end();++it)if(std::find(expected.begin(),expected.end(),it.key())==expected.end())throw std::invalid_argument("unknown JSON key: "+it.key());
  for(const auto& key:expected){if(!cfg.contains(key))throw std::invalid_argument("missing JSON key: "+key);
   if(key=="dimensions"||key=="ports")continue;
   const auto& z=cfg.at(key);if(!z.contains("type")||!z.contains("value")||!z.contains("editable"))throw std::invalid_argument("typed JSON required: "+key);
   if(z["type"]=="float"||z["type"]=="int"){if(!z["value"].is_number()||!std::isfinite(z["value"].get<double>()))throw std::invalid_argument("finite number required: "+key);}
  }
  if(cfg["name"]["value"]!="lm3350_sw"||cfg["category"]["value"]!="Charge_Pump-固定比例降压")throw std::invalid_argument("identity mismatch");
  for(const auto& key:std::vector<std::string>{"dt_s","max_internal_step_s","fsw","ron","tr_edge","freq_scale","pin_cap_f"})if(p(key.c_str())<=0)throw std::invalid_argument("positive parameter required: "+key);
  for(const auto& key:std::vector<std::string>{"iq","dead","cfly_f","cout_f","cfly_esr_ohm","cout_esr_ohm","cfly2_f","cfly2_esr_ohm","pin_cap_f"})if(p(key.c_str())<0)throw std::invalid_argument("nonnegative parameter required: "+key);
  if(p("dt_s")>1e-4||p("max_internal_step_s")>p("dt_s")||p("temperature_min_c")>=p("temperature_max_c"))throw std::invalid_argument("invalid numerical/environment range");
  if(p(K==0?"dead_fraction":"dead")>=0.1)throw std::invalid_argument("dead fraction >= 0.1");
  if(cfg["initial_state"]["value"]!="uncharged")throw std::invalid_argument("only uncharged initial state is supported");
  if(!cfg["ports"].is_array()||cfg["ports"].size()!=18||!cfg["dimensions"].is_object())throw std::invalid_argument("invalid port/icon metadata");
  caps.push_back({0,1,p("cfly_f"),p("cfly_esr_ohm")});caps.push_back({2,3,p("cfly2_f"),p("cfly2_esr_ohm")});caps.push_back({OUT,GND,p("cout_f"),p("cout_esr_ohm")});for(int i=0;i<N;++i)if(i!=OUT&&i!=NEG)caps.push_back({i,GND,p("pin_cap_f"),0});
 }
 double voltage(int n,const Vec& v)const{return n==VIN?vin:(n==GND?0:v[n]);}
 double energy()const{double e=0;for(const auto& c:caps)e+=0.5*c.c*c.v*c.v;return e;}
 void parameters(double& r,double& rn){
  const double t=temp-25;double il=std::max(0.0,filtered);en=1;
  if constexpr(K==0){ // LM2665 VA rev2.1, SNVS009H pp5-6 Figs2,3,8,15.
   double fs,rs,is;
   if(vin<=2.5){fs=120e3;rs=14;is=230e-6;}else if(vin<=3.5){fs=120e3+(vin-2.5)*18e3;rs=14-(vin-2.5)*2;is=230e-6+(vin-2.5)*120e-6;}
   else if(vin<=4.5){fs=138e3+(vin-3.5)*14e3;rs=12-(vin-3.5)*1.3;is=350e-6+(vin-3.5)*190e-6;}
   else if(vin<=5.5){fs=152e3+(vin-4.5)*13e3;rs=10.7-(vin-4.5)*1.2;is=540e-6+(vin-4.5)*290e-6;}else{fs=165e3;rs=9.5;is=830e-6;}
   double tr=temp-p("temp_ref_c"),dt=temp-55;
   freq=std::max(1.0,p("fosc_5v")*(fs/158.5e3)*p("freq_corner")*std::max(0.1,1+p("fosc_tempco")*tr))/2;
   r=p("ron_5v")*(rs/10.1)*p("ron_corner")*std::max(0.1,1+p("ron_tempco")*tr);
   double it=670e-6+0.0166e-6*dt*dt+1.08e-6*std::exp(std::min(20.0,(temp-50)/8));dt=temp-10;
   double sd=(0.29e-6+0.000044e-6*dt*dt+0.0000000024e-6*std::pow(dt,4))/0.3000215e-6;
   double enabled=.5*(1-std::tanh((control-p("sd_ratio")*vin)/p("sd_smooth_v")));
   en=enabled*.5*(1+std::tanh((x[OUT]-p("osc_supply_min"))/p("supply_smooth_v")));
   double ie=clamp(il,0,.1),extra=14+9*std::pow(clamp((5.5-vin)/3,0,1),2);
   iq=enabled*p("iq_5v")*(is/685e-6)*(it/684.98745e-6)*p("iq_corner")+(1-enabled)*p("iq_shutdown")*sd*p("iq_corner");
   iq+=p("loss_enable")*p("loss_scale")*en*(extra*ie*ie*std::max(0.0,vin)/(vin*vin+.01)+p("light_loss_gain")*clamp(vin-2.5,0,1)*ie*std::exp(-ie/.01)*std::max(0.0,vin)/std::max(.1,std::abs(vin)));
  }else if constexpr(K==1){freq=p("fsw")*p("freq_scale")*(1-.001*t)*(.9+.1*std::min(1.0,vin/5));r=p("ron_pos")*p("r_scale")*std::pow(5/std::max(vin,1.0),.35)*(1+.003*t);rn=p("ron_neg")*r/p("ron_pos");iq=p("iq")*std::pow(std::max(vin,.1)/5,1.3)*(1+.002*t);
  }else if constexpr(K==2){ // SLVS324C Fig7-19 hand-digitized fit from provided VA.
   double z=il*1000,f5=z<5?50+z:z<10?55+1.4*(z-5):z<20?62+3.3*(z-10):z<70?95+2.2*(z-20):std::min(250.0,205+.666667*(z-70));
   double f33=z<40?50+4.3*z:std::min(250.0,222+.45*(z-40)),f18=z<10?50+15*z:std::min(250.0,200+1.5*(z-10));
   double blend=vin>=3.3?clamp((vin-3.3)/1.7,0,1):clamp((vin-1.8)/1.5,0,1),fn=vin>=3.3?f33*(1-blend)+f5*blend:f18*(1-blend)+f33*blend;
   freq=(p("fmin")+(p("fmax")-p("fmin"))*(fn-50)/200)*p("freq_scale")*(1-.001*t);
   r=p("ron")*p("r_scale")*std::pow(5/std::max(vin,1.0),.45)*(1+.0025*t);iq=p("iq")*std::pow(std::max(vin,.1)/5,.8)*(1+.0015*t);
  }else if constexpr(K==3){en=.5*(1+std::tanh((control-1.7)/.025));freq=p("fsw")*p("freq_scale")*(1-.0015*t);r=p("ron")*p("r_scale")*std::pow(5/std::max(vin,1.0),.30)*(1+.002*t);iq=en*p("iq")*std::pow(std::max(vin,.1)/5,.6)*(1+.001*t)+(1-en)*.25e-6;
  }else{ // TPS60150 VA controller approximation; datasheet SLVS888D p5.
   en=.5*(1+std::tanh((control-.85)/.035))*.5*(1+std::tanh((vin-2)/.03));
   if(en<.01||x[OUT]<5.02||il>.009)skip=false;else if(x[OUT]>5.10&&il<.008)skip=true;
   freq=p("fsw")*p("freq_scale")*(1-.0005*t);r=p("ron")*p("r_scale")*(1+.002*t);iq=en*(p("iq_skip")+(p("iq_normal")-p("iq_skip"))*.5*(1+std::tanh((il-.008)/.001)))+(1-en)*1e-6;
  }
  if(!(freq>0&&r>0&&iq>=0))throw std::runtime_error("parameters outside modeled domain");
 }
 void tick(double duration){
  double remain=duration;ein=last_power=pex=pnum=err=0;
  while(remain>duration*1e-10){double r=1,rn=1;parameters(r,rn);double dead=p(K==0?"dead_fraction":"dead"),edge=p("tr_edge")*freq;
   if(edge>=dead)throw std::runtime_error("gate edge must be shorter than dead interval");
   double h=std::min({remain,p("max_internal_step_s"),1/(128*freq)});
   // Align solver steps with ramp boundaries; edge subdivision preserves nonoverlap.
   for(double b:{dead,dead+edge,.5-dead,.5-dead+edge,.5+dead,.5+dead+edge,1-dead,1-dead+edge,1.0})if(b-phase>1e-12)h=std::min(h,(b-phase)/freq);
   auto pulse=[&](double start,double stop,double q){return clamp((q-start)/edge,0,1)*clamp((stop+edge-q)/edge,0,1);};
   double q=phase+freq*h*.5,a=pulse(dead,.5-dead,q),b=pulse(.5+dead,1-dead,q);
   if((q>dead&&q<dead+edge)||(q>.5-dead&&q<.5-dead+edge)||(q>.5+dead&&q<.5+dead+edge)||(q>1-dead&&q<1-dead+edge)){h=std::min(h,p("tr_edge")/4);q=phase+freq*h*.5;a=pulse(dead,.5-dead,q);b=pulse(.5+dead,1-dead,q);}
   double run=en;if constexpr(K==4)run=(en>.5&&!skip)?1:0;
   gatea=a*run;gateb=b*run;double off=K==0?1/p("roff"):1e-12;
   double ga=off+gatea*(1/r-(K==0?off:0)),gb=off+gateb*(1/r-(K==0?off:0));
   std::vector<Branch> br;auto branch=[&](int u,int v,double g,bool ext=false,double src=0){br.push_back({u,v,g,src,ext});};
   if constexpr(K==2){branch(VIN,0,ga);branch(1,GND,ga);branch(0,GND,gb);branch(1,OUT,gb);if(x[OUT]>.3)branch(OUT,GND,.5,false,.3);}
   else if constexpr(K==3){branch(VIN,0,ga);branch(VIN,2,ga);branch(1,OUT,ga);branch(3,OUT,ga);branch(OUT,0,gb);branch(1,2,gb);branch(3,GND,gb);}
   else if constexpr(K==4){double ref=(5+p("line_coeff")*(vin-3.6)+.12*(1-.5*(1+std::tanh((std::max(0.0,filtered)-.008)/.001))))*clamp(soft,0,1);
    double drive=clamp(p("loop_gain")*(ref-x[OUT]),0,1),cf=clamp((2.5-(x[0]-x[1]))/.025,0,1);
    branch(VIN,0,1e-12+gatea*cf/r);branch(1,GND,ga);branch(VIN,1,1e-12+gateb*drive/r);branch(0,OUT,gb);
   }else{branch(VIN,0,ga);branch(1,GND,ga);branch(VIN,1,gb);branch(0,OUT,gb);
    if constexpr(K==0){if(p("startup_diode_enabled")>0&&vin-x[OUT]>p("diode_vf_v"))branch(VIN,OUT,1/p("diode_ron_ohm"),true,p("diode_vf_v"));}
    if constexpr(K==1){double an=1e-12+gatea/rn,bn=1e-12+gateb/rn;branch(OUT,2,bn);branch(3,GND,bn);branch(2,GND,an);branch(3,NEG,an);}
   }
   for(int i=0;i<N;++i)branch(i,GND,p(K==0?"gmin_pin":"gmin_s"));
   Mat mat{};Vec rhs{};
   auto stamp=[&](int u,int v,double g,double ie){
    if(u>=0){mat[u][u]+=g;rhs[u]+=ie;if(v>=0)mat[u][v]-=g;else rhs[u]+=g*voltage(v,x);}
    if(v>=0){mat[v][v]+=g;rhs[v]-=ie;if(u>=0)mat[v][u]-=g;else rhs[v]+=g*voltage(u,x);}
   };
   for(const auto& z:br)stamp(z.a,z.b,z.g,z.g*z.source);
   for(const auto& c:caps)if(c.c>0){double g=c.c/(h+c.r*c.c);stamp(c.a,c.b,g,g*c.v);}
   stamp(OUT,GND,1/rload,0);rhs[OUT]+=(K==2?load:-load);
   if constexpr(K==1){stamp(NEG,GND,1/rloadneg,0);rhs[NEG]+=loadneg;}
   Vec next=solve(mat,rhs);for(int i=0;i<N;++i)if(!std::isfinite(next[i])||std::abs(next[i])>1000)throw std::runtime_error("divergent input/load condition");
   double input=iq,loss=vin*iq,ext=0,num=0,e0=energy(),transfer=0;
   for(const auto& z:br){double dv=voltage(z.a,next)-voltage(z.b,next),i=z.g*(dv-z.source);if(z.a==VIN)input+=i;if(z.b==VIN)input-=i;
    if(z.external)ext+=dv*i;else loss+=dv*i;
    if(K==2&&z.a==1&&z.b==OUT)transfer=-i;else if(K!=2&&z.a==0&&z.b==OUT)transfer=i;
   }
   for(auto& c:caps)if(c.c>0){double i=c.c/(h+c.r*c.c)*(voltage(c.a,next)-voltage(c.b,next)-c.v),delta=h*i/c.c;c.v+=delta;ext+=c.r*i*i;num+=.5*c.c*delta*delta/h;}
   double pout=next[OUT]*next[OUT]/rload+(K==2?-next[OUT]:next[OUT])*load;
   if constexpr(K==1)pout+=next[NEG]*next[NEG]/rloadneg-next[NEG]*loadneg;
   double residual=vin*input-pout-loss-ext-num-(energy()-e0)/h;
   ein+=input*h;last_power+=loss*h;pex+=ext*h;pnum+=num*h;err+=residual*h;
   double tau=K==0?p("loss_filter_tau"):(K==2?40e-6:10e-6);filtered+=(transfer-filtered)*(-std::expm1(-h/tau));
   if constexpr(K==4)soft+=(en-soft)*(-std::expm1(-h/p("tau_start")));
   x=next;phase+=freq*h;if(phase>=1-1e-12)phase=0;remain-=h;
  }
  ein/=duration;last_power/=duration;pex/=duration;pnum/=duration;err/=duration;
 }
};
LM3350_SW::LM3350_SW(sc_core::sc_module_name nm,const std::string& path):sca_tdf::sca_module(nm),impl_(new Impl(path)){}
LM3350_SW::~LM3350_SW()=default;
void LM3350_SW::set_attributes(){set_timestep(impl_->p("dt_s"),sc_core::SC_SEC);}
void LM3350_SW::processing(){auto& s=*impl_;s.vin=vin.read();s.control=control.read();s.temp=temperature.read();s.load=iload.read();s.rload=rload.read();
 for(double z:{s.vin,s.control,s.temp,s.load,s.rload,s.loadneg,s.rloadneg})if(!std::isfinite(z))SC_REPORT_FATAL("charge_pump","nonfinite input");
 if(s.vin<0||s.vin>s.p("vin_absolute_max_v")||s.temp<s.p("temperature_min_c")||s.temp>s.p("temperature_max_c")||s.load<0||s.rload<=0||s.loadneg<0||s.rloadneg<=0)SC_REPORT_FATAL("charge_pump","input outside supported domain");
 s.tick(s.p("dt_s"));vout.write(s.x[OUT]);iin.write(s.ein);power.write(s.last_power);external_loss.write(s.pex);numerical_loss.write(s.pnum);stored_energy.write(s.energy());energy_residual.write(s.err);fsw.write(s.freq);phi1.write(s.gatea);phi2.write(s.gateb);cfly_v.write(s.caps[0].v);skip_state.write(s.skip?1:0);cfly2_v.write(s.caps[1].v);
}
std::string LM3350_SW::get_module_name()const{return impl_->cfg["name"]["value"];}
double LM3350_SW::get_version()const{return impl_->p("version");}
std::string LM3350_SW::get_category()const{return impl_->cfg["category"]["value"];}
std::optional<double> LM3350_SW::get_area()const{return std::nullopt;}
std::optional<double> LM3350_SW::get_latency()const{return std::nullopt;}
double LM3350_SW::get_timestep_s()const{return impl_->p("dt_s");}
double LM3350_SW::get_power_dissipation()const{return impl_->last_power;}
const nlohmann::json& LM3350_SW::configuration()const{return impl_->cfg;}
