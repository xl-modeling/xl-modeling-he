// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#include "tps43060.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
// 参数来源：verification/reference/tps43060.va；参考TI SLVSBP4D。
static double clip(double v,double a,double b){return std::max(a,std::min(v,b));}
TPS43060::configs TPS43060::get_configs_from_json(const char* json_path){
 CFGS_BEGIN(c)
 c.parameter=parameter;c.timestep_s=configuration.at("timestep_s").get<double>();
 c.module_name=meta.at("name").get<std::string>();c.version=meta.at("version").get<double>();
 CFGS_END(c)
}
double TPS43060::p(const char* k) const{return values_.at(k);}
std::string TPS43060::get_module_name() const{return c_.module_name;}
double TPS43060::get_version() const{return c_.version;}
double TPS43060::get_latency() const{return std::numeric_limits<double>::quiet_NaN();}
double TPS43060::get_area() const{return std::numeric_limits<double>::quiet_NaN();}
double TPS43060::get_power_dissipation() const {return last_power_w_;}
TPS43060::TPS43060(sc_core::sc_module_name n,const configs& c):sca_tdf::sca_module(n),c_(c){
 for(auto it=c.parameter.begin();it!=c.parameter.end();++it)values_[it.key()]=it.value().at("value").get<double>();
 if(!std::isfinite(c.timestep_s)||c.timestep_s<=0||c.timestep_s>5e-9)throw std::invalid_argument("timestep_s must be in (0,5 ns]");
 for(auto it=c.parameter.begin();it!=c.parameter.end();++it)if(!std::isfinite(it.value().at("value").get<double>()))throw std::invalid_argument("nonfinite parameter");
 if(p("vin_uv_fall")>=p("vin_uv_rise")||p("ven_fall")>=p("ven_rise")||p("comp_high")<=p("comp_zero_duty")||p("ton_min")<=0||p("dead_time")<0||p("sync_min")>p("sync_max"))throw std::invalid_argument("invalid thresholds/timing");
 for(const char* k:{"rt_pin_r","vref_nom","rea","comp_clamp_r","ss_discharge_r","r_leak","vcc_source_r","r_vcc_off","r_ldrv_pu_hi","r_ldrv_pd_hi","r_hdrv_pu_hi","r_hdrv_pd_hi","r_ldrv_pu_4v5","r_ldrv_pd_4v5","r_hdrv_pu_4v5","r_hdrv_pd_4v5","r_gate_no_bias"})if(p(k)<=0)throw std::invalid_argument(k);
}
void TPS43060::set_attributes(){set_timestep(c_.timestep_s,sc_core::SC_SEC);}
void TPS43060::processing(){
 double t=get_time().to_seconds(),ag=AGND.read(),vin=VIN.read()-ag,en=EN.read()-ag,fb=FB.read()-ag,ss=SS.read()-ag,comp=COMP.read()-ag,clk=RT_CLK.read()-ag,sense=ISNS_P.read()-ISNS_N.read();
 for(double v:{vin,en,fb,ss,comp,clk,sense,VCC.read(),BOOT.read(),SW.read(),PGND.read(),POWERPAD.read()})if(!std::isfinite(v))SC_REPORT_FATAL("TPS43060","nonfinite input");
 if(std::abs(PGND.read()-ag)>1e-6||std::abs(POWERPAD.read()-ag)>1e-6)SC_REPORT_FATAL("TPS43060","tie PGND POWERPAD and AGND");
 // 使能和输入欠压分别保持迟滞状态，重新使能时重新计启动延迟。
 bool was=vin_ok_&&en_ok_;
 if(first_){vin_ok_=vin>=p("vin_uv_rise");en_ok_=en>=p("ven_rise");ovp_=fb>=1.07*p("vref_nom");pglo_=fb<=.90*p("vref_nom");pghi_=fb>=1.10*p("vref_nom");}
 else{if(vin>=p("vin_uv_rise"))vin_ok_=true;else if(vin<=p("vin_uv_fall"))vin_ok_=false;if(en>=p("ven_rise"))en_ok_=true;else if(en<=p("ven_fall"))en_ok_=false;}
 bool run=vin_ok_&&en_ok_;if(run&&!was)enabled_at_=t;
 if(fb>=1.07*p("vref_nom"))ovp_=true;else if(fb<=1.05*p("vref_nom"))ovp_=false;
 if(fb<=.90*p("vref_nom"))pglo_=true;else if(fb>=.92*p("vref_nom"))pglo_=false;
 if(fb>=1.10*p("vref_nom"))pghi_=true;else if(fb<=1.08*p("vref_nom"))pghi_=false;
 // 两个有效同步边沿建立频率；失去时钟后退回RT振荡器。
 bool edge=!first_&&prev_clk_<p("vclk_high")&&clk>=p("vclk_high");
 if(edge){if(last_edge_>=0&&p("sync_enable")!=0){double f=1/(t-last_edge_);if(f>=p("sync_min")*(1-1e-9)&&f<=p("sync_max")*(1+1e-9)){sync_freq_=f;sync_=true;}}last_edge_=t;}
 if(sync_&&t-last_edge_>p("sync_timeout"))sync_=false;
 double rt=p("rt_override");if(rt<=0)rt=clk>1e-6&&clk<p("vrt_nom")-1e-6?clk*p("rt_pin_r")/(p("vrt_nom")-clk):1.15e6;
 freq_=clip(57.5e9/clip(rt,57.5e3,1.15e6),50e3,1e6);
 if(sync_)freq_=sync_freq_;
 if(p("fsw_override")>0)freq_=clip(p("fsw_override"),50e3,1e6);
 pwm_=run&&VCC.read()-PGND.read()>=p("vcc_gate_ready")&&t-enabled_at_>=p("ten_start");
 double dmax=1-std::max(p("toff_fixed")*freq_,p("toff_fraction"));
 double requested=pwm_&&!ovp_?clip((comp-p("comp_zero_duty"))*dmax/(p("comp_high")-p("comp_zero_duty")),0,dmax):0;
 // 每周期锁存脉宽和保护状态；低占空比使用脉冲密度实现。
 bool cycle=false;
 if(sync_&&p("fsw_override")<=0){cycle=edge;phase_=0;}else{phase_+=freq_*c_.timestep_s;if(phase_>=1-1e-12){phase_-=1;cycle=true;}}
 if(cycle){start_=t;period_=1/freq_;limited_=false;reverse_=sense<=-p("reverse_trip");duty_=requested;
  pulse_=requested>0;width_=requested*period_;
  if(requested>0&&requested<p("ton_min")*freq_){skip_+=requested/(p("ton_min")*freq_);pulse_=skip_>=1;if(pulse_)skip_-=1;width_=p("ton_min");}else skip_=0;
  low_end_=width_;
 }
 // 正向限流关闭低侧，反向检测关闭高侧，各自锁存到下一周期。
 double elapsed=t-start_,limit=p("vcs_zero_duty")-(p("vcs_zero_duty")-p("vcs_max_duty"))*duty_/dmax;
 if(pulse_&&!limited_&&elapsed<width_&&sense>=limit){limited_=true;low_end_=elapsed;}
 if(sense<=-p("reverse_trip"))reverse_=true;
 bool lo=pwm_&&!ovp_&&pulse_&&!limited_&&elapsed<width_;
 bool hi=pwm_&&!ovp_&&pulse_&&!reverse_&&elapsed>low_end_+p("dead_time")&&elapsed<period_-p("dead_time");
 // 输出空载驱动电压及串联电阻，外部电路负责求解带载门极电压。
 double a=clip((vin-4.5)/7.5,0,1),rlu=p("r_ldrv_pu_4v5")+a*(p("r_ldrv_pu_hi")-p("r_ldrv_pu_4v5")),rhu=p("r_hdrv_pu_4v5")+a*(p("r_hdrv_pu_hi")-p("r_hdrv_pu_4v5"));
 double rld=p("r_ldrv_pd_4v5")+a*(p("r_ldrv_pd_hi")-p("r_ldrv_pd_4v5")),rhd=p("r_hdrv_pd_4v5")+a*(p("r_hdrv_pd_hi")-p("r_hdrv_pd_4v5"));if(vin<.5)rld=rhd=p("r_gate_no_bias");
 LDRV.write(lo?VCC.read():PGND.read());HDRV.write(hi?BOOT.read():SW.read());ldrv_resistance.write(lo?rlu:rld);hdrv_resistance.write(hi?rhu:rhd);
 double ivcc=run?clip((std::min(vin,p("vcc_nom"))-(VCC.read()-PGND.read()))/p("vcc_source_r"),0,p("ivcc_max")):0;
 vcc_current.write(ivcc-(VCC.read()-PGND.read())/(run?p("r_leak"):p("r_vcc_off")));
 input_current.write((run?p("iq_enabled"):en<=p("ven_shutdown")?p("iq_shutdown"):p("iq_standby"))+ivcc);
 ss_current.write((pwm_?p("ss_current"):-ss/p("ss_discharge_r"))-(ss>p("ss_clamp")?(ss-p("ss_clamp"))/100:ss/p("r_leak")));
 double ea=clip(p("gea")*(pwm_?clip(ss,0,p("vref_nom"))-fb:-fb),-p("iea_limit"),p("iea_limit"));
 double ic=pwm_?ea-comp/p("rea"):-comp/1e3;
 if(pwm_){if(comp>p("comp_high"))ic-=(comp-p("comp_high"))/p("comp_clamp_r");else if(comp<p("comp_low"))ic-=(comp-p("comp_low"))/p("comp_clamp_r");}
 comp_current.write(ic);en_current.write(p("ien_pullup")+(en_ok_?p("ien_hyst"):0)-en/p("r_leak")-std::max(0.,en-vin-.2)/1e3);
 rt_current.write((clk-p("vrt_nom"))/p("rt_pin_r"));PGOOD_sink.write(!run||pglo_||pghi_?1:0);
 // State-dependent bias and VCC linear-regulator dissipation. External
  // MOSFET switching losses and gate-load power are not inferred from voltage.
  const double iq=run?p("iq_enabled"):en<=p("ven_shutdown")?p("iq_shutdown"):p("iq_standby");
  const double vcc_rel=std::max(VCC.read()-PGND.read(),0.);
  last_power_w_=std::max(vin,0.)*iq+std::max(vin-vcc_rel,0.)*ivcc+
                vcc_rel*vcc_rel/(run?p("r_leak"):p("r_vcc_off"));
  prev_clk_=clk;first_=false;
}
void TPS43060::trace_me(sca_util::sca_trace_file* f){sca_util::sca_trace(f,LDRV,"LDRV");sca_util::sca_trace(f,HDRV,"HDRV");sca_util::sca_trace(f,PGOOD_sink,"PGOOD_sink");sca_util::sca_trace(f,input_current,"input_current");}
