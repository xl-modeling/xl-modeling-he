#include "tps22975.h"
// Source provenance (delivery revision 1.0.1, 2026-10-29):
// TPS22975.va | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_功率开关_负载开关_TPS22975\TPS22975.va
// SHA256 c37f99b0a0c7852efe068c8cd9292ecefda40143839befbe2ce0d01b8590ae25
// TPS22975_datasheet.pdf | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_功率开关_负载开关_TPS22975\TPS22975_datasheet.pdf
// SHA256 e9544d6c90f1f0973f0cd7eac78954d45b6582f2c6f26dac2e57d018ce743af2
namespace tps22975 {
class LoadSwitch:public Core {
    bool on_=false,gate_=false,first_=true;double ct_=0,v_=0;Load load_;double capacitance_;Ramp strength_;struct Event{double t;bool on;};std::deque<Event> queue_;
public:
    LoadSwitch(const Config& c,const json& f):Core(c,f),load_(f.value("load",json::object())),capacitance_(f.value("ct_F",1e-9)){
        if(!std::isfinite(capacitance_)||capacitance_<=0)throw std::runtime_error("CT capacitance must be positive");
        input_names={"vin_V","on_V","vbias_V","gnd_V"};output_names={"vout_rel_V","vout_V","ct_V","pass_A","qod_A","body_A","ivin_A","ivbias_A","ion_A","ron_ohm","enabled"};output_names.insert(output_names.end(),{"noise_V","output_observed_V"});add_power_outputs();
    }
    std::vector<double> step(double t,double dt,const std::vector<double>& x){
        validate_step(t,dt,x);
        auto p=[&](const char* k){return config_.get(k);};double vi=x[0]-x[3],vo=x[1]-x[3],vb=x[2]-x[3];bool on=on_;hyst(on,vo,p("von_rise"),p("von_fall"));
        if(first_&&fixture_.value("initial_dc",false)){on_=gate_=on;ct_=on?p("vct_full"):0;strength_.dc(on?1:0);}
        else if(on!=on_){queue_.push_back({t+(on?p("t_ramp_start"):p("t_off_delay")),on});on_=on;}
        first_=false;std::stable_sort(queue_.begin(),queue_.end(),[](const Event&a,const Event&b){return a.t<b.t;});while(!queue_.empty()&&queue_.front().t<=t+1e-16){gate_=queue_.front().on;strength_.set(queue_.front().t,gate_?1:0,1e-9);queue_.pop_front();}
        const double strength=strength_.value(t),cdt=capacitance_/dt,charge=strength*p("ict_charge"),dis=(1-strength)/p("rct_discharge"),clamp=1/p("rct_clamp");
        double candidate=(cdt*ct_+charge)/(cdt+dis);if(candidate>p("vct_full"))candidate=(cdt*ct_+charge+p("vct_full")*clamp)/(cdt+dis+clamp);ct_=candidate;
        double alpha=std::max(0.,std::min(1.,(vb-2.5)/2.5)),ron=p("ron_2p5v")+(p("ron_5v")-p("ron_2p5v"))*alpha,iq=p("iq_2p5v")+(p("iq_5v")-p("iq_2p5v"))*alpha;
        double ramp=std::max(0.,std::min(1.,ct_/p("vct_full"))),target=ramp*vi;
        auto branch=[&](double v,double& pass,double& qod,double& body){pass=positive((target-v)/ron,p("i_zero_eps"));qod=(1-strength)*v/p("rpd");body=p("body_is")*(limexp((v-vi)/p("body_nvt"))-1);};
        auto residual=[&](double v){double a,b,c;branch(v,a,b,c);return load_.c/dt*(v-v_)+load_.draw(v)-a+b+c;};
        double low=std::min(0.,v_)-1,high=std::max(vi,v_)+1;if(residual(low)>0||residual(high)<0)throw std::runtime_error("Load switch root not bracketed");for(int n=0;n<44;++n){double m=(low+high)/2;if(residual(m)>0)high=m;else low=m;}v_=(low+high)/2;
        if(load_.forced)v_=load_.force; // Honor the same voltage clamp fixture as gate drivers.
        double pass,qod,body;branch(v_,pass,qod,body);
        power_=model_power::Breakdown();
        const double ivin=pass-body+(1-strength)*p("isd_vin_typ"),ibias=p("isd_vbias_typ")+strength*(iq-p("isd_vbias_typ")),ion=vo*p("ion_leak")/5.5;
        const double ct_dis=dis*ct_+clamp*std::max(0.,ct_-p("vct_full"));
        power_.quiescent=vi*(1-strength)*p("isd_vin_typ")+vb*ibias;
        power_.input=vo*ion;power_.drive=(vi-v_)*pass;
        power_.discharge=v_*qod;power_.body=(v_-vi)*body;
        // Original VA uses I(gnd,ct): an ideal control source, not VBIAS draw.
        // Keep the electrical model unchanged and account for that source
        // separately from chip supply power and external CT stored energy.
        power_.ct=ct_*ct_dis;power_.ct_source=ct_*charge;
        power_.ct_storage=ct_*(charge-ct_dis);
        power_.terminal_in=vi*ivin+vb*ibias+vo*ion;
        power_.terminal_out=v_*(pass-qod-body);
        power_.finish();
        return observe({v_,v_+x[3],ct_,pass,qod,body,pass-body+(1-strength)*p("isd_vin_typ"),p("isd_vbias_typ")+strength*(iq-p("isd_vbias_typ")),vo*p("ion_leak")/5.5,ron,strength},t,dt,{strength>.5});
    }
};
std::unique_ptr<Core> make_core(const Config& c,const json& f){return std::unique_ptr<Core>(new LoadSwitch(c,f));}
}
