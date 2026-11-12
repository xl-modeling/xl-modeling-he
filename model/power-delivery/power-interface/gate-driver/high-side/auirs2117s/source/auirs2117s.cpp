#include "auirs2117s.h"
// Source provenance (delivery revision 1.0.1, 2026-10-29):
// AUIRS2117S.va | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_栅极驱动_高边驱动_AUIRS2117S\AUIRS2117S.va
// SHA256 ef23aac7d883d3dad4f03bfdb5f0f968bea65e563bde0d5891983cf13169fdc2
// AUIRS2117S_2118S_datasheet.pdf | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_栅极驱动_高边驱动_AUIRS2117S\AUIRS2117S_2118S_datasheet.pdf
// SHA256 238841e34c21996b43b425890e496eef0f58f71ba427602ad38f11181524120b
namespace auirs2117s {
class Auirs:public Core {
    bool in_=false,cc_=false,bs_=false,first_=true;Command cmd_;Gate gate_;Load load_;
public:
    Auirs(const Config& c,const json& f):Core(c,f),load_(f.value("load",json::object())){
        input_names={"vcc_V","in_V","vb_V","vs_V","com_V"};
        output_names={"ho_rel_V","ho_V","source_A","sink_A","icc_A","ibs_A","iin_A","vbs_V","logic"};output_names.insert(output_names.end(),{"noise_V","output_observed_V"});add_power_outputs();
    }
    std::vector<double> step(double t,double dt,const std::vector<double>& x){
        validate_step(t,dt,x);
        auto p=[&](const char* k){return config_.get(k);};double vcc=x[0]-x[4],vin=x[1]-x[4],vbs=x[2]-x[3];
        hyst(in_,vin,p("vin_rise"),p("vin_fall"));hyst(cc_,vcc,p("vcc_uvlo_rise"),p("vcc_uvlo_fall"));hyst(bs_,vbs,p("vbs_uvlo_rise"),p("vbs_uvlo_fall"));
        bool req=in_&&cc_&&bs_;if(first_&&fixture_.value("initial_dc",false))cmd_.dc(req);first_=false;
        cmd_.update(t,req,p("t_launch_on"),p("t_launch_off"),p("t_src_on"),p("t_src_off"),p("t_snk_on"),p("t_snk_off"));
        GateParameters gp{p("r_pullup"),p("r_pulldown"),p("i_source_pk"),p("i_sink_pk"),p("v_eps"),p("i_limit_eps"),p("i_zero_eps"),p("v_clamp"),p("r_clamp")};
        gate_.step(dt,std::max(0.,vbs),cmd_.src.value(t),cmd_.sink.value(t),gp,load_);
        power_=model_power::Breakdown();
        const double iin=vin/p("rin_pulldown"),icc=p("iqcc"),ibs=p("iqbs")+p("ihv_leak")+gate_.is-gate_.high_clamp;
        power_.quiescent=vcc*icc+vbs*(p("iqbs")+p("ihv_leak"));
        power_.input=vin*iin;
        power_.drive=model_power::gate_drive(vbs,gate_.v,gate_.is,gate_.ik);
        power_.clamp=model_power::gate_clamp(vbs,gate_.v,gate_.high_clamp,gate_.low_clamp);
        power_.terminal_in=vcc*icc+vbs*ibs+vin*iin;
        power_.terminal_out=gate_.v*(gate_.is-gate_.ik-gate_.high_clamp+gate_.low_clamp);
        power_.finish();
        return observe({gate_.v,gate_.v+x[3],gate_.is,gate_.ik,p("iqcc"),p("iqbs")+p("ihv_leak")+gate_.is-gate_.high_clamp,vin/p("rin_pulldown"),vbs,req?1.:0.},t,dt,{cc_&&bs_});
    }
};
std::unique_ptr<Core> make_core(const Config& c,const json& f){return std::unique_ptr<Core>(new Auirs(c,f));}
}
