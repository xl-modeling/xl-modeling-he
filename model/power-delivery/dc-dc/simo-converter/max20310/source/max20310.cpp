// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
// Sources and limitations: see max20310.h and JSON source hashes.
#include "max20310.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
namespace max20310 {
namespace {
double clip(double x,double a,double b){return std::max(a,std::min(x,b));}
double pos(double x){return std::max(0.0,x);}
void require(bool ok,const char* msg){if(!ok)throw std::invalid_argument(msg);}
void solve(double a[5][5],double b[5]){
 for(int k=0;k<5;++k){int m=k;for(int i=k+1;i<5;++i)if(std::abs(a[i][k])>std::abs(a[m][k]))m=i;
  if(std::abs(a[m][k])<1e-18)throw std::runtime_error("MAX20310 singular circuit");
  for(int j=k;j<5;++j)std::swap(a[k][j],a[m][j]);std::swap(b[k],b[m]);
  for(int i=k+1;i<5;++i){double q=a[i][k]/a[k][k];for(int j=k+1;j<5;++j)a[i][j]-=q*a[k][j];b[i]-=q*b[k];}
 }
 for(int i=4;i>=0;--i){for(int j=i+1;j<5;++j)b[i]-=a[i][j]*b[j];b[i]/=a[i][i];}
}
}
void Parameters::validate() const {
 const double positive[]={PIN_LIMIT,IIN_LIMIT,UV_START,UV_STOP,ILDO_MAX,RDROP,RLOADSW,RDIS,RCHG,ROUTSW,ROFF,RDUMP,CLX,VDIODE,RDIODE,HYST,TCTRL,TEDGE,TDEAD,TIDLE,TCHG_MAX,TXFER_MAX,IZCD,TLIMIT,TSOFT,GM_LDO,TAU_LDO,DT,LOUT,CBB1,CBB2,CLDO1,CLDO2};
 for(double x:positive)require(std::isfinite(x)&&x>0,"positive finite parameter required");
 const double flags[]={BB1_EN,BB2_EN,LDO1_EN,LDO2_EN,LDO1_SWITCH,LDO2_SWITCH,DISCHARGE};
 for(double x:flags)require(x==0||x==1,"flag must be 0 or 1");
 for(double x:{VBB1,VBB2})require(std::isfinite(x)&&x>=.9&&x<=4.05,"BB setpoint out of range");
 for(double x:{VLDO1,VLDO2})require(std::isfinite(x)&&x>=.5&&x<=3.65,"LDO setpoint out of range");
 for(double x:{VBB1,VBB2,VLDO1,VLDO2})require(std::abs(x/.05-std::round(x/.05))<1e-8,"setpoints require 50mV grid");
 require(IPEAK>=.3&&IPEAK<=.6&&std::abs(IPEAK/.1-std::round(IPEAK/.1))<1e-8,"IPEAK must be .3/.4/.5/.6A");
 for(double x:{START_DELAY,IQ_ON,IQ_OFF,QSW,DCR})require(std::isfinite(x)&&x>=0,"nonnegative finite parameter required");
 require(UV_STOP<UV_START,"UVLO threshold order");
 require(TDEAD+1e-18>=2*TEDGE+TCTRL,"dead time too short");
 require(DT<=TEDGE&&DT<=2e-9,"DT must not exceed TEDGE or 2ns");
 require(std::abs(TCTRL/DT-std::round(TCTRL/DT))<1e-8,"TCTRL must be integral DT multiple");
}
Engine::Engine(const Parameters& p):p_(p){p_.validate();control_ticks_=static_cast<std::uint64_t>(std::llround(p_.TCTRL/p_.DT));}
void Engine::control(double t,const Inputs& in){
 if(in.temperature>=150)thermal_=true;else if(in.temperature<=129)thermal_=false;
 if(!running_&&in.en_model>.7&&in.vbat>=p_.UV_START){running_=true;tboot_=t;}
 if(in.en_model<.3||in.vbat<p_.UV_STOP)running_=false;
 ss_=clip((t-tboot_-p_.START_DELAY)/p_.TSOFT,0,1);
 en1_=running_&&p_.BB1_EN&&t>=tboot_+p_.START_DELAY;en2_=running_&&p_.BB2_EN&&t>=tboot_+p_.START_DELAY;
 el1_=en1_&&p_.LDO1_EN&&!thermal_;el2_=en2_&&p_.LDO2_EN&&!thermal_;
 ipk_=.08+(p_.IPEAK-.08)*ss_;
 // I(sense) in VA excludes CLX displacement current; sense is LX->sw.
 // CLX belongs to sw, thus I(sense)=external inductor current in this topology.
 switch(state_){
 case 0:if(t>=tready_&&(en1_||en2_)&&ss_>0&&pmon_<p_.PIN_LIMIT&&imon_<p_.IIN_LIMIT){
  double e1=en1_?(p_.VBB1*ss_-v_[1]-.5*p_.HYST)/p_.VBB1:-1;
  double e2=en2_?(p_.VBB2*ss_-v_[2]-.5*p_.HYST)/p_.VBB2:-1;
  if(e1>0||e2>0){selected_=e1>=e2?1:2;state_=4;tstate_=t;}}
  break;
 case 4:if(!running_){state_=0;tready_=t+p_.TIDLE;}else if(t-tstate_+1e-18>=p_.TDEAD){state_=1;tstate_=t;tgate_=t;}break;
 case 1:if(il_>=ipk_||t-tstate_+1e-18>=p_.TCHG_MAX||!running_){state_=2;tstate_=t;}break;
 case 2:if(t-tstate_+1e-18>=p_.TDEAD){state_=3;tstate_=t;}break;
 case 3:if(il_<=p_.IZCD||t-tstate_+1e-18>=p_.TXFER_MAX){state_=5;tstate_=t;}break;
 case 5:if(t-tstate_+1e-18>=p_.TDEAD){state_=0;tready_=t+p_.TIDLE;}break;
 default:state_=0;
 }
 std::array<double,4> next{{state_==1?1.0:0.0,state_==3&&selected_==1?1.0:0.0,state_==3&&selected_==2?1.0:0.0,state_==0?1.0:0.0}};
 for(int k=0;k<4;++k)if(next[k]!=target_[k]){start_[k]=gates_[k];target_[k]=next[k];change_[k]=t;}
}
void Engine::circuit(double t,const Inputs& in){
 const double dt=p_.DT,bat=-in.vbat,alpha=dt/p_.TAU_LDO;
 std::array<double,4> g;
 for(int k=0;k<4;++k){double v=start_[k]+(target_[k]-start_[k])*clip((t-change_[k])/p_.TEDGE,0,1);g[k]=.5*(v+gates_[k]);gates_[k]=v;}
 const double charge=g[0]/p_.RCHG+1/p_.ROFF,dump=g[3]/p_.RDUMP+1/p_.ROFF;
 const double c[5]={p_.CLX,p_.CBB1,p_.CBB2,p_.CLDO1,p_.CLDO2};
 const double rl[4]={in.rbb1,in.rbb2,in.rldo1,in.rldo2};
 const double loads[4]={in.ibb1_load,in.ibb2_load,in.ildo1_load,in.ildo2_load};
 const bool ens[4]={en1_,en2_,el1_,el2_};
 const bool steer=state_==1||state_==2||state_==3||state_==5;
 const double lden=p_.LOUT/dt+p_.DCR,ilbase=p_.LOUT/dt*il_;
 auto eval=[&](const std::array<double,5>& x,double f[5],double j[5][5],double* pass,double* lc){
  for(int a=0;a<5;++a){f[a]=c[a]/dt*(x[a]-v_[a]);for(int b=0;b<5;++b)j[a][b]=0;j[a][a]=c[a]/dt;}
  f[0]-=(ilbase-x[0])/lden;j[0][0]+=1/lden;
  f[0]+=charge*(x[0]-bat)+dump*x[0];j[0][0]+=charge+dump;
  auto diode=[&](int a,int b,double vv,double gd){if(vv<=0)return;double i=gd*vv;f[a]+=i;j[a][a]+=gd;if(b>=0){f[b]-=i;j[a][b]-=gd;j[b][a]-=gd;j[b][b]+=gd;}};
  for(int k=1;k<=2;++k){double v=x[0]-x[k];double off=v/p_.ROFF;f[0]+=off;f[k]-=off;j[0][0]+=1/p_.ROFF;j[k][k]+=1/p_.ROFF;j[0][k]-=1/p_.ROFF;j[k][0]-=1/p_.ROFF;
   diode(0,k,v,g[k]/p_.ROUTSW);if(steer&&selected_==k)diode(0,k,v-p_.VDIODE,1/p_.RDIODE);
  }
  diode(0,-1,x[0]-6,.5);
  if(bat-x[0]>p_.VDIODE){f[0]-=(bat-x[0]-p_.VDIODE)/p_.RDIODE;j[0][0]+=1/p_.RDIODE;}
  for(int k=1;k<5;++k){double gd=1/rl[k-1]+1/p_.ROFF+((p_.DISCHARGE&&!ens[k-1])?1/p_.RDIS:0);f[k]+=gd*x[k]+loads[k-1];j[k][k]+=gd;}
  for(int k=0;k<2;++k){int b=k+1,l=k+3;double old=k==0?lc1_:lc2_,ref=(k==0?p_.VLDO1:p_.VLDO2)*ss_;bool enabled=k==0?el1_:el2_,sw=k==0?p_.LDO1_SWITCH:p_.LDO2_SWITCH;
   double cmd=enabled?clip(p_.GM_LDO*(ref-x[l]),0,p_.ILDO_MAX):0;
   double dc=enabled&&cmd>0&&cmd<p_.ILDO_MAX?-p_.GM_LDO:0;
   lc[k]=(old+alpha*cmd)/(1+alpha);double dlc=alpha*dc/(1+alpha);
   double r=sw?p_.RLOADSW:p_.RDROP,head=clip((x[b]-x[l])/r,0,p_.ILDO_MAX),dh=head>0&&head<p_.ILDO_MAX?1/r:0;
   double v=0,db=0,dl=0;
   if(enabled){if(sw||head<=lc[k]){v=head;db=dh;dl=-dh;}else{v=clip(lc[k],0,p_.ILDO_MAX);dl=v>0&&v<p_.ILDO_MAX?dlc:0;}}
   pass[k]=v;f[b]+=v;f[l]-=v;j[b][b]+=db;j[b][l]+=dl;j[l][b]-=db;j[l][l]-=dl;
  }
 };
 std::array<double,5>x=v_;double f[5],j[5][5],pass[2],lc[2];bool converged=false;
 for(int it=0;it<35;++it){eval(x,f,j,pass,lc);double norm=0;for(double a:f)norm=std::max(norm,std::abs(a));if(norm<1e-9){converged=true;break;}
  double dx[5];for(int k=0;k<5;++k)dx[k]=-f[k];solve(j,dx);
  double step=1;std::array<double,5>trial=x;
  for(int ls=0;ls<18;++ls){for(int k=0;k<5;++k)trial[k]=x[k]+step*dx[k];double nf[5],nj[5][5],np[2],nl[2];eval(trial,nf,nj,np,nl);double nn=0;for(double a:nf)nn=std::max(nn,std::abs(a));if(nn<norm||step<1e-5)break;step*=.5;}x=trial;
 }
 if(!converged){eval(x,f,j,pass,lc);double norm=0;for(double a:f)norm=std::max(norm,std::abs(a));if(norm>1e-7)throw std::runtime_error("MAX20310 circuit residual at t="+std::to_string(t));}
 eval(x,f,j,pass,lc);v_=x;il_=(ilbase-x[0])/lden;lc1_=lc[0];lc2_=lc[1];
 double iq=running_?p_.IQ_ON:p_.IQ_OFF,gate=(tgate_>=0&&t>=tgate_&&t<tgate_+p_.TCTRL)?p_.QSW/p_.TCTRL:0;
 double ichg=charge*(x[0]-bat),rev=pos(bat-x[0]-p_.VDIODE)/p_.RDIODE;
 o_.iin=ichg-rev+(iq+gate)*clip(in.vbat/.1,0,1);
 // VA limit monitors omit reverse diode and low-voltage IQ attenuation.
 double a=std::exp(-dt/p_.TLIMIT);pmon_=a*pmon_+(1-a)*pos(in.vbat*(ichg+iq+gate));imon_=a*imon_+(1-a)*pos(ichg+iq+gate);
 double loss=charge*std::pow(x[0]-bat,2)+dump*x[0]*x[0]+pos(x[0]-6)*.5*x[0]+rev*(bat-x[0]);
 for(int k=1;k<=2;++k){double dv=x[0]-x[k];loss+=(g[k]*pos(dv)/p_.ROUTSW+dv/p_.ROFF+(steer&&selected_==k?pos(dv-p_.VDIODE)/p_.RDIODE:0))*dv;}
 for(int k=1;k<5;++k)loss+=x[k]*x[k]*(1/p_.ROFF+((p_.DISCHARGE&&!ens[k-1])?1/p_.RDIS:0));
 loss+=pass[0]*(x[1]-x[3])+pass[1]*(x[2]-x[4])+in.vbat*(iq+gate)*clip(in.vbat/.1,0,1)+in.en_model*in.en_model/1e12;
 o_.power=loss;o_.external_loss=p_.DCR*il_*il_;o_.energy=.5*p_.LOUT*il_*il_;for(int k=0;k<5;++k)o_.energy+=.5*c[k]*x[k]*x[k];
 o_.lx=x[0];o_.bb1=x[1];o_.bb2=x[2];o_.ldo1=x[3];o_.ldo2=x[4];o_.il=il_;o_.pass1=pass[0];o_.pass2=pass[1];
 o_.gate_charge=gates_[0];o_.gate_bb1=gates_[1];o_.gate_bb2=gates_[2];o_.gate_dump=gates_[3];
}
Output Engine::step(double t,const Inputs& in){
 require(std::isfinite(t)&&std::abs(t-tick_*p_.DT)<p_.DT*.01,"nonsequential time grid");
 for(double x:{in.vbat,in.en_model,in.temperature,in.rbb1,in.rbb2,in.rldo1,in.rldo2,in.ibb1_load,in.ibb2_load,in.ildo1_load,in.ildo2_load})require(std::isfinite(x),"nonfinite input");
 require(in.rbb1>0&&in.rbb2>0&&in.rldo1>0&&in.rldo2>0,"load resistance must be positive");
 if(tick_>0)circuit(t,in);
 // Peak comparator is evaluated at every circuit sample, not only TCTRL.
 if(tick_%control_ticks_==0 || (state_==1 && il_>=ipk_))control(t,in);
 o_.running=running_;o_.thermal_trip=thermal_;o_.state=state_;o_.selected=selected_;o_.softstart=ss_;o_.peak_limit=ipk_;o_.pin_filtered=pmon_;o_.iin_filtered=imon_;
 ++tick_;return o_;
}
Parameters load_config(const std::string& path,json& c){
 std::ifstream f(path);if(!f)throw std::runtime_error("cannot open config: "+path);f>>c;Parameters p;
 p.VBB1=c.at("parameters").at("VBB1").at("value").get<double>();
 p.VBB2=c.at("parameters").at("VBB2").at("value").get<double>();
 p.VLDO1=c.at("parameters").at("VLDO1").at("value").get<double>();
 p.VLDO2=c.at("parameters").at("VLDO2").at("value").get<double>();
 p.BB1_EN=c.at("parameters").at("BB1_EN").at("value").get<double>();
 p.BB2_EN=c.at("parameters").at("BB2_EN").at("value").get<double>();
 p.LDO1_EN=c.at("parameters").at("LDO1_EN").at("value").get<double>();
 p.LDO2_EN=c.at("parameters").at("LDO2_EN").at("value").get<double>();
 p.LDO1_SWITCH=c.at("parameters").at("LDO1_SWITCH").at("value").get<double>();
 p.LDO2_SWITCH=c.at("parameters").at("LDO2_SWITCH").at("value").get<double>();
 p.IPEAK=c.at("parameters").at("IPEAK").at("value").get<double>();
 p.PIN_LIMIT=c.at("parameters").at("PIN_LIMIT").at("value").get<double>();
 p.IIN_LIMIT=c.at("parameters").at("IIN_LIMIT").at("value").get<double>();
 p.UV_START=c.at("parameters").at("UV_START").at("value").get<double>();
 p.UV_STOP=c.at("parameters").at("UV_STOP").at("value").get<double>();
 p.ILDO_MAX=c.at("parameters").at("ILDO_MAX").at("value").get<double>();
 p.RDROP=c.at("parameters").at("RDROP").at("value").get<double>();
 p.RLOADSW=c.at("parameters").at("RLOADSW").at("value").get<double>();
 p.DISCHARGE=c.at("parameters").at("DISCHARGE").at("value").get<double>();
 p.RDIS=c.at("parameters").at("RDIS").at("value").get<double>();
 p.RCHG=c.at("parameters").at("RCHG").at("value").get<double>();
 p.ROUTSW=c.at("parameters").at("ROUTSW").at("value").get<double>();
 p.ROFF=c.at("parameters").at("ROFF").at("value").get<double>();
 p.RDUMP=c.at("parameters").at("RDUMP").at("value").get<double>();
 p.CLX=c.at("parameters").at("CLX").at("value").get<double>();
 p.VDIODE=c.at("parameters").at("VDIODE").at("value").get<double>();
 p.RDIODE=c.at("parameters").at("RDIODE").at("value").get<double>();
 p.HYST=c.at("parameters").at("HYST").at("value").get<double>();
 p.TCTRL=c.at("parameters").at("TCTRL").at("value").get<double>();
 p.TEDGE=c.at("parameters").at("TEDGE").at("value").get<double>();
 p.TDEAD=c.at("parameters").at("TDEAD").at("value").get<double>();
 p.TIDLE=c.at("parameters").at("TIDLE").at("value").get<double>();
 p.TCHG_MAX=c.at("parameters").at("TCHG_MAX").at("value").get<double>();
 p.TXFER_MAX=c.at("parameters").at("TXFER_MAX").at("value").get<double>();
 p.IZCD=c.at("parameters").at("IZCD").at("value").get<double>();
 p.TLIMIT=c.at("parameters").at("TLIMIT").at("value").get<double>();
 p.START_DELAY=c.at("parameters").at("START_DELAY").at("value").get<double>();
 p.TSOFT=c.at("parameters").at("TSOFT").at("value").get<double>();
 p.GM_LDO=c.at("parameters").at("GM_LDO").at("value").get<double>();
 p.TAU_LDO=c.at("parameters").at("TAU_LDO").at("value").get<double>();
 p.IQ_ON=c.at("parameters").at("IQ_ON").at("value").get<double>();
 p.IQ_OFF=c.at("parameters").at("IQ_OFF").at("value").get<double>();
 p.QSW=c.at("parameters").at("QSW").at("value").get<double>();
 p.DT=c.at("parameters").at("DT").at("value").get<double>();
 p.LOUT=c.at("parameters").at("LOUT").at("value").get<double>();
 p.DCR=c.at("parameters").at("DCR").at("value").get<double>();
 p.CBB1=c.at("parameters").at("CBB1").at("value").get<double>();
 p.CBB2=c.at("parameters").at("CBB2").at("value").get<double>();
 p.CLDO1=c.at("parameters").at("CLDO1").at("value").get<double>();
 p.CLDO2=c.at("parameters").at("CLDO2").at("value").get<double>();
 p.validate();return p;
}
MAX20310::MAX20310(sc_core::sc_module_name name,const std::string& path):sca_tdf::sca_module(name),p_(load_config(path,config_)),engine_(p_){}
std::string MAX20310::get_module_name()const{return config_.at("name").at("value").get<std::string>();}
double MAX20310::get_version()const{return config_.at("version").at("value").get<double>();}
std::string MAX20310::get_category()const{return config_.at("category").at("value").get<std::string>();}
json MAX20310::get_area()const{return config_.at("area").at("value");}
json MAX20310::get_latency()const{return config_.at("latency").at("value");}
json MAX20310::get_cost()const{return config_.at("cost").at("value");}
void MAX20310::set_attributes(){if(sc_core::sc_get_time_resolution().to_seconds()>1e-12)throw std::runtime_error("set SystemC time resolution <=1ps");set_timestep(p_.DT,sc_core::SC_SEC);}
void MAX20310::processing(){Inputs in;
 in.vbat=vbat.read();
 in.en_model=en_model.read();
 in.temperature=temperature.read();
 in.rbb1=rbb1.read();
 in.rbb2=rbb2.read();
 in.rldo1=rldo1.read();
 in.rldo2=rldo2.read();
 in.ibb1_load=ibb1_load.read();
 in.ibb2_load=ibb2_load.read();
 in.ildo1_load=ildo1_load.read();
 in.ildo2_load=ildo2_load.read();
 last_=engine_.step(get_time().to_seconds(),in);
 bb1.write(last_.bb1);
 bb2.write(last_.bb2);
 ldo1.write(last_.ldo1);
 ldo2.write(last_.ldo2);
 lx.write(last_.lx);
 il.write(last_.il);
 iin.write(last_.iin);
 power.write(last_.power);
 external_loss.write(last_.external_loss);
 energy.write(last_.energy);
 pin_filtered.write(last_.pin_filtered);
 iin_filtered.write(last_.iin_filtered);
 softstart.write(last_.softstart);
 peak_limit.write(last_.peak_limit);
 gate_charge.write(last_.gate_charge);
 gate_bb1.write(last_.gate_bb1);
 gate_bb2.write(last_.gate_bb2);
 gate_dump.write(last_.gate_dump);
 pass1.write(last_.pass1);
 pass2.write(last_.pass2);
 running.write(last_.running);
 thermal_trip.write(last_.thermal_trip);
 state.write(last_.state);
 selected.write(last_.selected);
}
} // namespace max20310
