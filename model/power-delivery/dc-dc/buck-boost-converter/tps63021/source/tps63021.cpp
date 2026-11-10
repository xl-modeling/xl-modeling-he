// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#include "tps63021.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
namespace tps63021 {
namespace {
double clip(double x,double l,double u){return std::max(l,std::min(u,x));}
struct Branch {double i,d;};
// Peak current is controlled by the cycle latch, not by making every MOS
// branch an independent ideal current source. The latter has no finite
// compliance at I=ILIMIT and trapped the low-input startup at saturation.
Branch channel(double v,double g,double){return {g*v,g};}
Branch diode(double v,double vf,double rd,double eps){double x=v-vf,z=std::hypot(x,eps);return {(.5*(x+z)-.5*(-vf+std::hypot(vf,eps)))/rd,.5*(1+x/z)/rd};}
void linear_solve(double m[4][5],double* x){
 for(int i=0;i<4;++i){int k=i;for(int j=i+1;j<4;++j)if(std::abs(m[j][i])>std::abs(m[k][i]))k=j;
  if(std::abs(m[k][i])<1e-20)throw std::runtime_error("Singular power-stage Jacobian");
  for(int j=i;j<5;++j)std::swap(m[i][j],m[k][j]);
  double piv=m[i][i];for(int j=i;j<5;++j)m[i][j]/=piv;
  for(int row=0;row<4;++row)if(row!=i){double f=m[row][i];for(int col=i;col<5;++col)m[row][col]-=f*m[i][col];}}
 for(int i=0;i<4;++i)x[i]=m[i][4];
}
}
nlohmann::json load_config(const std::string& path){std::ifstream f(path);if(!f)throw std::runtime_error("Cannot open TPS63021 JSON: "+path);nlohmann::json j;f>>j;if(j.at("name").at("value")!="TPS63021")throw std::invalid_argument("Wrong model name");return j;}
Parameters::Parameters(const nlohmann::json& j){
    VOUT_NOM=j.at("VOUT_NOM").at("value").get<double>();if(!std::isfinite(VOUT_NOM))throw std::invalid_argument("Nonfinite VOUT_NOM");
    FSW_NOM_HZ=j.at("FSW_NOM_HZ").at("value").get<double>();if(!std::isfinite(FSW_NOM_HZ))throw std::invalid_argument("Nonfinite FSW_NOM_HZ");
    FSW_MIN_HZ=j.at("FSW_MIN_HZ").at("value").get<double>();if(!std::isfinite(FSW_MIN_HZ))throw std::invalid_argument("Nonfinite FSW_MIN_HZ");
    FSW_MAX_HZ=j.at("FSW_MAX_HZ").at("value").get<double>();if(!std::isfinite(FSW_MAX_HZ))throw std::invalid_argument("Nonfinite FSW_MAX_HZ");
    VIN_OPER_MIN=j.at("VIN_OPER_MIN").at("value").get<double>();if(!std::isfinite(VIN_OPER_MIN))throw std::invalid_argument("Nonfinite VIN_OPER_MIN");
    VIN_OPER_MAX=j.at("VIN_OPER_MAX").at("value").get<double>();if(!std::isfinite(VIN_OPER_MAX))throw std::invalid_argument("Nonfinite VIN_OPER_MAX");
    UVLO_FALL=j.at("UVLO_FALL").at("value").get<double>();if(!std::isfinite(UVLO_FALL))throw std::invalid_argument("Nonfinite UVLO_FALL");
    UVLO_HYST=j.at("UVLO_HYST").at("value").get<double>();if(!std::isfinite(UVLO_HYST))throw std::invalid_argument("Nonfinite UVLO_HYST");
    EN_VIL=j.at("EN_VIL").at("value").get<double>();if(!std::isfinite(EN_VIL))throw std::invalid_argument("Nonfinite EN_VIL");
    EN_VIH=j.at("EN_VIH").at("value").get<double>();if(!std::isfinite(EN_VIH))throw std::invalid_argument("Nonfinite EN_VIH");
    ILIMIT_NOM_A=j.at("ILIMIT_NOM_A").at("value").get<double>();if(!std::isfinite(ILIMIT_NOM_A))throw std::invalid_argument("Nonfinite ILIMIT_NOM_A");
    ILIMIT_START_A=j.at("ILIMIT_START_A").at("value").get<double>();if(!std::isfinite(ILIMIT_START_A))throw std::invalid_argument("Nonfinite ILIMIT_START_A");
    ILIMIT_LOWVIN_A=j.at("ILIMIT_LOWVIN_A").at("value").get<double>();if(!std::isfinite(ILIMIT_LOWVIN_A))throw std::invalid_argument("Nonfinite ILIMIT_LOWVIN_A");
    RSW_ON_OHM=j.at("RSW_ON_OHM").at("value").get<double>();if(!std::isfinite(RSW_ON_OHM))throw std::invalid_argument("Nonfinite RSW_ON_OHM");
    RSW_OFF_OHM=j.at("RSW_OFF_OHM").at("value").get<double>();if(!std::isfinite(RSW_OFF_OHM))throw std::invalid_argument("Nonfinite RSW_OFF_OHM");
    IQ_VINA_A=j.at("IQ_VINA_A").at("value").get<double>();if(!std::isfinite(IQ_VINA_A))throw std::invalid_argument("Nonfinite IQ_VINA_A");
    IQ_VOUT_A=j.at("IQ_VOUT_A").at("value").get<double>();if(!std::isfinite(IQ_VOUT_A))throw std::invalid_argument("Nonfinite IQ_VOUT_A");
    ISHUTDOWN_A=j.at("ISHUTDOWN_A").at("value").get<double>();if(!std::isfinite(ISHUTDOWN_A))throw std::invalid_argument("Nonfinite ISHUTDOWN_A");
    FB_RIN_OHM=j.at("FB_RIN_OHM").at("value").get<double>();if(!std::isfinite(FB_RIN_OHM))throw std::invalid_argument("Nonfinite FB_RIN_OHM");
    OVP_V=j.at("OVP_V").at("value").get<double>();if(!std::isfinite(OVP_V))throw std::invalid_argument("Nonfinite OVP_V");
    TSD_C=j.at("TSD_C").at("value").get<double>();if(!std::isfinite(TSD_C))throw std::invalid_argument("Nonfinite TSD_C");
    TSD_HYST_C=j.at("TSD_HYST_C").at("value").get<double>();if(!std::isfinite(TSD_HYST_C))throw std::invalid_argument("Nonfinite TSD_HYST_C");
    SOFTSTART_S=j.at("SOFTSTART_S").at("value").get<double>();if(!std::isfinite(SOFTSTART_S))throw std::invalid_argument("Nonfinite SOFTSTART_S");
    LOOP_BW_HZ=j.at("LOOP_BW_HZ").at("value").get<double>();if(!std::isfinite(LOOP_BW_HZ))throw std::invalid_argument("Nonfinite LOOP_BW_HZ");
    KP_DUTY_PER_V=j.at("KP_DUTY_PER_V").at("value").get<double>();if(!std::isfinite(KP_DUTY_PER_V))throw std::invalid_argument("Nonfinite KP_DUTY_PER_V");
    KI_DUTY_PER_V_S=j.at("KI_DUTY_PER_V_S").at("value").get<double>();if(!std::isfinite(KI_DUTY_PER_V_S))throw std::invalid_argument("Nonfinite KI_DUTY_PER_V_S");
    INT_DUTY_MAX=j.at("INT_DUTY_MAX").at("value").get<double>();if(!std::isfinite(INT_DUTY_MAX))throw std::invalid_argument("Nonfinite INT_DUTY_MAX");
    INT_RESET_S=j.at("INT_RESET_S").at("value").get<double>();if(!std::isfinite(INT_RESET_S))throw std::invalid_argument("Nonfinite INT_RESET_S");
    DAMP_FILTER_HZ=j.at("DAMP_FILTER_HZ").at("value").get<double>();if(!std::isfinite(DAMP_FILTER_HZ))throw std::invalid_argument("Nonfinite DAMP_FILTER_HZ");
    ACTIVE_DAMP_GAIN=j.at("ACTIVE_DAMP_GAIN").at("value").get<double>();if(!std::isfinite(ACTIVE_DAMP_GAIN))throw std::invalid_argument("Nonfinite ACTIVE_DAMP_GAIN");
    MODE_WINDOW_V=j.at("MODE_WINDOW_V").at("value").get<double>();if(!std::isfinite(MODE_WINDOW_V))throw std::invalid_argument("Nonfinite MODE_WINDOW_V");
    DUTY_MIN=j.at("DUTY_MIN").at("value").get<double>();if(!std::isfinite(DUTY_MIN))throw std::invalid_argument("Nonfinite DUTY_MIN");
    DUTY_MAX=j.at("DUTY_MAX").at("value").get<double>();if(!std::isfinite(DUTY_MAX))throw std::invalid_argument("Nonfinite DUTY_MAX");
    DEADTIME_S=j.at("DEADTIME_S").at("value").get<double>();if(!std::isfinite(DEADTIME_S))throw std::invalid_argument("Nonfinite DEADTIME_S");
    EDGE_S=j.at("EDGE_S").at("value").get<double>();if(!std::isfinite(EDGE_S))throw std::invalid_argument("Nonfinite EDGE_S");
    STEPS_PER_CYCLE=j.at("STEPS_PER_CYCLE").at("value").get<double>();if(!std::isfinite(STEPS_PER_CYCLE))throw std::invalid_argument("Nonfinite STEPS_PER_CYCLE");
    PSM_LOW_RATIO=j.at("PSM_LOW_RATIO").at("value").get<double>();if(!std::isfinite(PSM_LOW_RATIO))throw std::invalid_argument("Nonfinite PSM_LOW_RATIO");
    PSM_HIGH_RATIO=j.at("PSM_HIGH_RATIO").at("value").get<double>();if(!std::isfinite(PSM_HIGH_RATIO))throw std::invalid_argument("Nonfinite PSM_HIGH_RATIO");
    SYNC_TIMEOUT_S=j.at("SYNC_TIMEOUT_S").at("value").get<double>();if(!std::isfinite(SYNC_TIMEOUT_S))throw std::invalid_argument("Nonfinite SYNC_TIMEOUT_S");
    P_SWITCH_PWM_W=j.at("P_SWITCH_PWM_W").at("value").get<double>();if(!std::isfinite(P_SWITCH_PWM_W))throw std::invalid_argument("Nonfinite P_SWITCH_PWM_W");
    P_SWITCH_PSM_W=j.at("P_SWITCH_PSM_W").at("value").get<double>();if(!std::isfinite(P_SWITCH_PSM_W))throw std::invalid_argument("Nonfinite P_SWITCH_PSM_W");
    PG_RLOW_OHM=j.at("PG_RLOW_OHM").at("value").get<double>();if(!std::isfinite(PG_RLOW_OHM))throw std::invalid_argument("Nonfinite PG_RLOW_OHM");
    PG_ROFF_OHM=j.at("PG_ROFF_OHM").at("value").get<double>();if(!std::isfinite(PG_ROFF_OHM))throw std::invalid_argument("Nonfinite PG_ROFF_OHM");
    PG_GOOD_RATIO=j.at("PG_GOOD_RATIO").at("value").get<double>();if(!std::isfinite(PG_GOOD_RATIO))throw std::invalid_argument("Nonfinite PG_GOOD_RATIO");
    PG_BAD_RATIO=j.at("PG_BAD_RATIO").at("value").get<double>();if(!std::isfinite(PG_BAD_RATIO))throw std::invalid_argument("Nonfinite PG_BAD_RATIO");
    BODY_VF_V=j.at("BODY_VF_V").at("value").get<double>();if(!std::isfinite(BODY_VF_V))throw std::invalid_argument("Nonfinite BODY_VF_V");
    BODY_RD_OHM=j.at("BODY_RD_OHM").at("value").get<double>();if(!std::isfinite(BODY_RD_OHM))throw std::invalid_argument("Nonfinite BODY_RD_OHM");
    SMOOTH_V=j.at("SMOOTH_V").at("value").get<double>();if(!std::isfinite(SMOOTH_V))throw std::invalid_argument("Nonfinite SMOOTH_V");
    DT=j.at("DT").at("value").get<double>();if(!std::isfinite(DT))throw std::invalid_argument("Nonfinite DT");
    LOUT=j.at("LOUT").at("value").get<double>();if(!std::isfinite(LOUT))throw std::invalid_argument("Nonfinite LOUT");
    LDCR=j.at("LDCR").at("value").get<double>();if(!std::isfinite(LDCR))throw std::invalid_argument("Nonfinite LDCR");
    COUT=j.at("COUT").at("value").get<double>();if(!std::isfinite(COUT))throw std::invalid_argument("Nonfinite COUT");
    ESR=j.at("ESR").at("value").get<double>();if(!std::isfinite(ESR))throw std::invalid_argument("Nonfinite ESR");
    PG_PULLUP_V=j.at("PG_PULLUP_V").at("value").get<double>();if(!std::isfinite(PG_PULLUP_V))throw std::invalid_argument("Nonfinite PG_PULLUP_V");
    PG_PULLUP_R=j.at("PG_PULLUP_R").at("value").get<double>();if(!std::isfinite(PG_PULLUP_R))throw std::invalid_argument("Nonfinite PG_PULLUP_R");
    INITIAL_VOUT=j.at("INITIAL_VOUT").at("value").get<double>();if(!std::isfinite(INITIAL_VOUT))throw std::invalid_argument("Nonfinite INITIAL_VOUT");

    if(DT<=0||DT>2e-9||LOUT<=0||COUT<=0||LDCR<0||ESR<0||PG_PULLUP_R<=0||PG_PULLUP_V<0||INITIAL_VOUT<0||
       VOUT_NOM<=0||FSW_MIN_HZ<=0||FSW_NOM_HZ<FSW_MIN_HZ||FSW_NOM_HZ>FSW_MAX_HZ||FSW_MAX_HZ>10e6||
       VIN_OPER_MIN<=0||VIN_OPER_MIN>=2.3||VIN_OPER_MAX<=VIN_OPER_MIN||UVLO_FALL<=0||UVLO_HYST<0||EN_VIH<=EN_VIL||
       ILIMIT_NOM_A<=0||ILIMIT_START_A<=0||ILIMIT_LOWVIN_A<=0||ILIMIT_NOM_A<ILIMIT_START_A||ILIMIT_NOM_A<ILIMIT_LOWVIN_A||
       RSW_ON_OHM<=0||RSW_OFF_OHM<=RSW_ON_OHM||IQ_VINA_A<0||IQ_VOUT_A<0||ISHUTDOWN_A<0||FB_RIN_OHM<=0||
       OVP_V<=0||TSD_HYST_C<0||SOFTSTART_S<=0||LOOP_BW_HZ<=0||KP_DUTY_PER_V<0||KI_DUTY_PER_V_S<0||
       INT_DUTY_MAX<=0||INT_DUTY_MAX>1||INT_RESET_S<=0||DAMP_FILTER_HZ<=0||ACTIVE_DAMP_GAIN<0||MODE_WINDOW_V<0||
       DUTY_MIN<0||DUTY_MAX>1||DUTY_MAX<=DUTY_MIN||DEADTIME_S<0||EDGE_S<=0||STEPS_PER_CYCLE<16||STEPS_PER_CYCLE>512||
       std::floor(STEPS_PER_CYCLE)!=STEPS_PER_CYCLE||PSM_LOW_RATIO<=1||PSM_HIGH_RATIO<=PSM_LOW_RATIO||SYNC_TIMEOUT_S<=0||
       P_SWITCH_PWM_W<0||P_SWITCH_PSM_W<0||PG_RLOW_OHM<=0||PG_ROFF_OHM<=PG_RLOW_OHM||PG_BAD_RATIO<=0||
       PG_GOOD_RATIO>=1||PG_BAD_RATIO>=PG_GOOD_RATIO||BODY_VF_V<0||BODY_RD_OHM<=0||SMOOTH_V<=0)
        throw std::invalid_argument("Invalid TPS63021 parameters");
}
Engine::Engine(const nlohmann::json& j):p(j),fsync(p.FSW_NOM_HZ),vc(p.INITIAL_VOUT),vo(p.INITIAL_VOUT),damp(p.INITIAL_VOUT),pg_g(1./p.PG_RLOW_OHM){}
Output Engine::advance(double h,const Inputs& in,const double* g,double limit,bool run,bool power_save){
 const double cap_g=1./(p.ESR+h/p.COUT),lh=h/p.LOUT,fb_g=run&&in.fb_override<0?1./p.FB_RIN_OHM:0;
 const double qo=run?p.IQ_VOUT_A:0;
 auto eval=[&](const double* x,double mat[4][5])->double{
  auto ah=channel(in.vin-x[0],g[0],limit),al=channel(x[0],g[1],limit);
  auto bh=channel(x[2]-x[1],g[2],limit),bl=channel(x[1],g[3],limit);
  auto da=diode(x[0]-in.vin,p.BODY_VF_V,p.BODY_RD_OHM,p.SMOOTH_V),dl=diode(-x[0],p.BODY_VF_V,p.BODY_RD_OHM,p.SMOOTH_V);
  auto db=diode(x[1]-x[2],p.BODY_VF_V,p.BODY_RD_OHM,p.SMOOTH_V),dr=diode(-x[1],p.BODY_VF_V,p.BODY_RD_OHM,p.SMOOTH_V);
  auto ov=diode(x[2],p.OVP_V,.02,p.SMOOTH_V);
  double f[4]={-ah.i+al.i+da.i-dl.i+x[3],-bh.i+bl.i+db.i-dr.i-x[3],bh.i-db.i+x[2]*(1./in.rload+fb_g)+in.iload+qo+ov.i+cap_g*(x[2]-vc),x[3]-il-lh*(x[0]-x[1]-p.LDCR*x[3])};
  double j[4][4]={{ah.d+al.d+da.d+dl.d,0,0,1},{0,bh.d+bl.d+db.d+dr.d,-bh.d-db.d,-1},{0,-bh.d-db.d,bh.d+db.d+1./in.rload+fb_g+ov.d+cap_g,0},{-lh,lh,0,1+lh*p.LDCR}};
  double norm=0;for(int i=0;i<4;++i){norm=std::max(norm,std::abs(f[i]));for(int k=0;k<4;++k)mat[i][k]=j[i][k];mat[i][4]=-f[i];}return norm;
 };
 double x[4]={a,b,vo,il};bool converged=false;
 for(int it=0;it<80;++it){double mat[4][5],dx[4];double norm=eval(x,mat);if(norm<1e-9){converged=true;break;}linear_solve(mat,dx);
  double scale=1,largest=0;for(double v:dx)largest=std::max(largest,std::abs(v));if(largest>2)scale=2/largest;
  double candidate[4],tmp[4][5];bool accept=false;
  for(int tries=0;tries<24;++tries){for(int k=0;k<4;++k)candidate[k]=x[k]+scale*dx[k];if(eval(candidate,tmp)<norm){accept=true;break;}scale*=.5;}
  if(!accept)break;for(int k=0;k<4;++k)x[k]=candidate[k];
 }
 if(!converged){double tmp[4][5];if(eval(x,tmp)>1e-7)throw std::runtime_error("TPS63021 power-stage Newton failed");}
 a=x[0];b=x[1];vo=x[2];il=x[3];double ic=cap_g*(vo-vc);vc+=h/p.COUT*ic;
 auto ah=channel(in.vin-a,g[0],limit),al=channel(a,g[1],limit),bh=channel(vo-b,g[2],limit),bl=channel(b,g[3],limit);
 double da=diode(a-in.vin,p.BODY_VF_V,p.BODY_RD_OHM,p.SMOOTH_V).i,dl=diode(-a,p.BODY_VF_V,p.BODY_RD_OHM,p.SMOOTH_V).i;
 double db=diode(b-vo,p.BODY_VF_V,p.BODY_RD_OHM,p.SMOOTH_V).i,dr=diode(-b,p.BODY_VF_V,p.BODY_RD_OHM,p.SMOOTH_V).i;
 double ov=diode(vo,p.OVP_V,.02,p.SMOOTH_V).i,psw=run&&burst?(power_save?p.P_SWITCH_PSM_W:p.P_SWITCH_PWM_W):0;
 Output o;o.vout=vo;o.l1=a;o.l2=b;o.il=il;o.i_vin=ah.i-da+psw/std::max(in.vin,.2);
 o.i_vina=run?p.IQ_VINA_A:p.ISHUTDOWN_A;o.i_en=in.en*.01e-6/std::max(in.vina,.1);o.i_ps=in.ps_sync*.01e-6/std::max(in.vina,.1);
 double fbv=in.fb_override<0?vo:in.fb_override;
 o.power=ah.i*(in.vin-a)+al.i*a+bh.i*(vo-b)+bl.i*b+da*(a-in.vin)-dl*a+db*(b-vo)-dr*b+ov*vo+psw+in.vina*o.i_vina+qo*vo+(run?fbv*fbv/p.FB_RIN_OHM:0)+in.en*o.i_en+in.ps_sync*o.i_ps;
 o.external_loss=p.LDCR*il*il+p.ESR*ic*ic;o.energy=.5*p.LOUT*il*il+.5*p.COUT*vc*vc;
 return o;
}
Output Engine::sample(double t,const Inputs& in){
 for(double v:{t,in.vin,in.vina,in.en,in.ps_sync,in.temperature,in.fb_override,in.iload,in.rload})if(!std::isfinite(v))throw std::invalid_argument("Nonfinite input");
 if(in.vin<0||in.vina<0||in.en<0||in.rload<=0||in.iload<0||t<0||(!initialized&&t!=0)||
    (initialized&&(t<=previous||std::abs(t-previous-p.DT)>p.DT*1e-4)))throw std::invalid_argument("Invalid input or time grid");
 const double h=initialized?p.DT:0,pi=std::acos(-1.);
 if(in.en>=p.EN_VIH)en_ok=true;else if(in.en<=p.EN_VIL)en_ok=false;
 if(in.vina>=p.UVLO_FALL+p.UVLO_HYST)uvlo=true;else if(in.vina<=p.UVLO_FALL)uvlo=false;
 if(in.temperature>=p.TSD_C)thermal=false;else if(in.temperature<=p.TSD_C-p.TSD_HYST_C)thermal=true;
 if(initialized&&prev_ps<.8&&in.ps_sync>=.8){if(last_edge>=0)fsync=clip(1./(t-last_edge),p.FSW_MIN_HZ,p.FSW_MAX_HZ);last_edge=t;}
 bool synced=last_edge>=0&&t-last_edge<p.SYNC_TIMEOUT_S,psm=in.ps_sync<=p.EN_VIL&&!synced;
 double f=synced?fsync:p.FSW_NOM_HZ;
 bool run=en_ok&&uvlo&&thermal&&in.vin>=p.VIN_OPER_MIN&&in.vin<=p.VIN_OPER_MAX;
 if(!run)burst=false;else if(!psm)burst=true;else if(vo<=p.VOUT_NOM*p.PSM_LOW_RATIO)burst=true;else if(vo>=p.VOUT_NOM*p.PSM_HIGH_RATIO)burst=false;
 ss=clip(ss+(run?h/p.SOFTSTART_S:-5*h/p.SOFTSTART_S),0,1);
 double refcmd=p.VOUT_NOM*(psm?p.PSM_HIGH_RATIO:1),ref=refcmd*ss,fb=in.fb_override<0?vo:in.fb_override;
 double error=run&&burst?ref-fb:0;
 err+=(error-err)*(-std::expm1(-2*pi*p.LOOP_BW_HZ*h));
 // Conditional integration prevents latent windup during startup/short circuit.
  // The integral itself is bounded; clipping only its contribution leaves memory
  // far outside the duty range and delays recovery after a fault.
  if(run&&burst&&!psm){
   const double feed=in.vin>refcmd?ref/std::max(in.vin,.2):1.-in.vin/std::max(ref,.2);
   const double raw=feed+p.KP_DUTY_PER_V*err+integ-p.ACTIVE_DAMP_GAIN*(vo-damp);
   if(!((raw>=p.DUTY_MAX&&err>0)||(raw<=p.DUTY_MIN&&err<0)))
    integ=clip(integ+h*p.KI_DUTY_PER_V_S*ss*err,-p.INT_DUTY_MAX,p.INT_DUTY_MAX);
  }else integ*=std::exp(-h/p.INT_RESET_S);
 damp+=(vo-damp)*(-std::expm1(-2*pi*p.DAMP_FILTER_HZ*h));
 double correction=p.KP_DUTY_PER_V*err+clip(integ,-p.INT_DUTY_MAX,p.INT_DUTY_MAX)-p.ACTIVE_DAMP_GAIN*(vo-damp);
 double dbuck=clip(ref/std::max(in.vin,.2)+correction,p.DUTY_MIN,p.DUTY_MAX);
 double dboost=clip(1.-in.vin/std::max(ref,.2)+correction,p.DUTY_MIN,p.DUTY_MAX);
 double cycles=std::floor(t*f),phase=t*f-cycles,dead=clip(p.DEADTIME_S*f,0,.2);
 bool buck=in.vin>refcmd+p.MODE_WINDOW_V,boost=in.vin<refcmd-p.MODE_WINDOW_V,near=!buck&&!boost;
 if(near)buck=std::fmod(cycles,2)<.5;
 double limstart=p.ILIMIT_START_A+(p.ILIMIT_NOM_A-p.ILIMIT_START_A)*clip(vo/1.2,0,1);
 double limvin=p.ILIMIT_LOWVIN_A+(p.ILIMIT_NOM_A-p.ILIMIT_LOWVIN_A)*clip((in.vin-p.VIN_OPER_MIN)/(2.3-p.VIN_OPER_MIN),0,1);
 double limit=std::min(limstart,limvin),cmd[4]={0,0,0,0},g[4];
  if(cycles!=current_limit_cycle){current_limit_cycle=cycles;current_limit_phase=1;}
  if(il>=limit)current_limit_phase=std::min(current_limit_phase,phase);
  double buck_end=std::min(dbuck,current_limit_phase),boost_end=std::min(dboost,current_limit_phase);
 // Break before make at BOTH transitions, including the period boundary.
  // VA V4 omitted the wrap dead time; preserve power-stage topology but fix
  // that controller defect rather than reproducing half-bridge shoot-through.
  if(run&&burst){if(buck){cmd[0]=phase>=dead&&phase<buck_end;cmd[1]=phase>buck_end+dead;cmd[2]=1;}
   else{cmd[0]=1;cmd[3]=phase>=dead&&phase<boost_end;cmd[2]=phase>boost_end+dead;}}
 for(int k=0;k<4;++k){double old=gates[k];gates[k]+=clip(cmd[k]-gates[k],-h/p.EDGE_S,h/p.EDGE_S);double avg=(old+gates[k])*.5;g[k]=avg/p.RSW_ON_OHM+(1-avg)/p.RSW_OFF_OHM;}
 Output o;if(h>0)o=advance(h,in,g,limit,run,psm);else{o.vout=vo;o.energy=.5*p.COUT*vc*vc;}
 if(!run||vo>=p.OVP_V)pgood=false;else if(vo>=p.PG_GOOD_RATIO*p.VOUT_NOM)pgood=true;else if(vo<=p.PG_BAD_RATIO*p.VOUT_NOM)pgood=false;
 double pg_target=pgood?1./p.PG_ROFF_OHM:1./p.PG_RLOW_OHM;
 double pg_slew=(1./p.PG_RLOW_OHM-1./p.PG_ROFF_OHM)*h/1e-6;pg_g+=clip(pg_target-pg_g,-pg_slew,pg_slew);
 o.pg=p.PG_PULLUP_V/(1+p.PG_PULLUP_R*pg_g);o.i_pg=o.pg*pg_g;o.power+=o.pg*o.i_pg;
 o.active=run;o.psm=psm;o.burst=burst;o.sync_active=synced;o.pg_good=pgood;o.thermal_ok=thermal;o.mode=!run?0:near?3:buck?1:2;
 o.ilim=limit;o.duty_buck=dbuck;o.duty_boost=dboost;o.frequency=f;o.softstart=ss;
 o.ga_h=gates[0];o.ga_l=gates[1];o.gb_h=gates[2];o.gb_l=gates[3];
 previous=t;prev_ps=in.ps_sync;initialized=true;return o;
}
TPS63021::TPS63021(sc_core::sc_module_name name,const std::string& path):sca_tdf::sca_module(name),config(load_config(path)),engine(config){}
void TPS63021::set_attributes(){if(sc_core::sc_get_time_resolution().to_seconds()>1e-12)throw std::runtime_error("Use time resolution <=1 ps");set_timestep(engine.parameters().DT,sc_core::SC_SEC);}
void TPS63021::processing(){Inputs in;
    in.vin=vin.read();
    in.vina=vina.read();
    in.en=en.read();
    in.ps_sync=ps_sync.read();
    in.temperature=temperature.read();
    in.fb_override=fb_override.read();
    in.iload=iload.read();
    in.rload=rload.read();
    auto o=engine.sample(get_time().to_seconds(),in);
    vout.write(o.vout);
    l1.write(o.l1);
    l2.write(o.l2);
    il.write(o.il);
    i_vin.write(o.i_vin);
    i_vina.write(o.i_vina);
    i_en.write(o.i_en);
    i_ps.write(o.i_ps);
    i_pg.write(o.i_pg);
    power.write(o.power);
    external_loss.write(o.external_loss);
    energy.write(o.energy);
    pg.write(o.pg);
    ilim.write(o.ilim);
    duty_buck.write(o.duty_buck);
    duty_boost.write(o.duty_boost);
    frequency.write(o.frequency);
    softstart.write(o.softstart);
    ga_h.write(o.ga_h);
    ga_l.write(o.ga_l);
    gb_h.write(o.gb_h);
    gb_l.write(o.gb_l);
    active.write(o.active);
    psm.write(o.psm);
    burst.write(o.burst);
    sync_active.write(o.sync_active);
    pg_good.write(o.pg_good);
    thermal_ok.write(o.thermal_ok);
    mode.write(o.mode);
    last_power=o.power;
}
std::string TPS63021::get_module_name()const{return config.at("name").at("value").get<std::string>();}
double TPS63021::get_version()const{return config.at("version").at("value").get<double>();}
std::string TPS63021::get_category()const{return config.at("category").at("value").get<std::string>();}
} // namespace tps63021
