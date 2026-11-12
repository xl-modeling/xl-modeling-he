#include "ucc27282.h"
// Source provenance (delivery revision 1.0.1, 2026-10-29):
// UCC27282_datasheet.pdf | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_栅极驱动_半桥驱动_UCC27282\UCC27282_datasheet.pdf
// SHA256 bf856648b4e9ff0de0593220b4c440f9b9a10f5aec20e858fbb543ac25f92620
// UCC27282_EN.va | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_栅极驱动_半桥驱动_UCC27282\UCC27282_EN.va
// SHA256 5c871e3b661e83571318a3074fd4bb74075a0a9fb24e70cc3d1a5af2f0b0b908
// UCC27282_noEN.va | original: E:\电源芯粒汇总\电源芯粒datasheet建模\VA建模\功率接口\供电芯粒_功率接口_栅极驱动_半桥驱动_UCC27282\UCC27282_noEN.va
// SHA256 3235924daaa2cebd3220ef0fbf1760c10e49342de52e75f3a76cc678fb39fe6e
namespace ucc27282 {
class HalfBridge:public Core {
    bool hi_=false,li_=false,dd_=false,hb_=false,en_=false,en_gate_=false,first_=true,with_en_;int comm_=0;
    struct EnableEvent {double time;bool enabled;};std::deque<EnableEvent> eq_;Ramp enable_strength_;Command hc_,lc_;Gate ho_,lo_;Load hl_,ll_;
public:
    HalfBridge(const Config& c,const json& f):Core(c,f),with_en_(c.parameters().count("ven_rise")!=0),hl_(f.value("load_ho",json::object())),ll_(f.value("load_lo",json::object())){
        input_names={"vdd_V","hb_V","hs_V","hi_V","li_V","vss_V","en_V"};
        output_names={"ho_rel_V","ho_V","lo_rel_V","lo_V","ho_source_A","ho_sink_A","lo_source_A","lo_sink_A","idd_A","ihb_A","ihi_A","ili_A","ien_A","iboot_A","vbs_V","logic_ho","logic_lo","enabled"};output_names.insert(output_names.end(),{"noise_ho_V","noise_lo_V","ho_observed_V","lo_observed_V"});add_power_outputs();
    }
    std::vector<double> step(double t,double dt,const std::vector<double>& x){
        validate_step(t,dt,x);
        auto p=[&](const char* k){return config_.get(k);};double vd=x[0]-x[5],vbs=x[1]-x[2],vi=x[3]-x[5],vl=x[4]-x[5],ve=x[6]-x[5];
        bool h=hi_,l=li_;hyst(h,vi,p("vin_rise"),p("vin_fall"));hyst(l,vl,p("vin_rise"),p("vin_fall"));
        if(h!=hi_){if(h){if(li_&&(comm_==2||comm_==4))comm_=3;else if(!li_)comm_=1;else comm_=0;}else{if(comm_!=4)comm_=li_?2:0;}hi_=h;}
        if(l!=li_){if(l){if(hi_&&(comm_==1||comm_==3))comm_=4;else if(!hi_)comm_=2;else comm_=0;}else{if(comm_!=3)comm_=hi_?1:0;}li_=l;}
        hyst(dd_,vd,p("vdd_uvlo_rise"),p("vdd_uvlo_fall"));hyst(hb_,vbs,p("hb_uvlo_rise"),p("hb_uvlo_fall"));
        if(with_en_){bool e=en_;hyst(e,ve,p("ven_rise"),p("ven_fall"));if(first_&&fixture_.value("initial_dc",false)){en_=e;en_gate_=e;enable_strength_.dc(e?1:0);}else if(e!=en_){eq_.push_back({t+(e?p("t_enable"):p("t_disable")),e});en_=e;}
            std::stable_sort(eq_.begin(),eq_.end(),[](const EnableEvent&a,const EnableEvent&b){return a.time<b.time;});while(!eq_.empty()&&eq_.front().time<=t+1e-16){en_gate_=eq_.front().enabled;enable_strength_.set(eq_.front().time,en_gate_?1:0,1e-9);eq_.pop_front();}
        }else {en_gate_=true;enable_strength_.dc(1);}
        const double eg=enable_strength_.value(t);
        bool rh=dd_&&hb_&&hi_&&!li_,rl=dd_&&li_&&!hi_;if(!dd_)comm_=hi_&&!li_?1:li_&&!hi_?2:0;
        if(first_&&fixture_.value("initial_dc",false)){hc_.dc(rh);lc_.dc(rl);}first_=false;
        double extra=with_en_?0:p("t_comm_extra");
        hc_.update(t,rh,p("t_launch_on")+(comm_==3?extra:0),p("t_launch_off"),p("t_src_on"),p("t_src_off"),p("t_snk_on"),p("t_snk_off"));
        lc_.update(t,rl,p("t_launch_on")+(comm_==4?extra:0),p("t_launch_off"),p("t_src_on"),p("t_src_off"),p("t_snk_on"),p("t_snk_off"));
        GateParameters gp{p("rho_ho"),p("rlo_ho"),p("i_source_pk"),p("i_sink_pk"),p("v_eps"),p("i_limit_eps"),p("i_zero_eps"),p("v_clamp"),p("r_clamp")};
        ho_.step(dt,std::max(0.,vbs),eg*hc_.src.value(t),1-eg+eg*hc_.sink.value(t),gp,hl_);gp.roh=p("rho_lo");gp.rol=p("rlo_lo");lo_.step(dt,std::max(0.,vd),eg*lc_.src.value(t),1-eg+eg*lc_.sink.value(t),gp,ll_);
        double boot=p("iboot_is")*(limexp((x[0]-x[1])/p("iboot_nvt"))-1);
        double id=with_en_?p("idd_disabled")+eg*(p("idd_quiescent")-p("idd_disabled")):p("idd_quiescent"),ib=eg*p("ihb_quiescent");
        power_=model_power::Breakdown();
        const double ihi=vi/p("rin"),ili=vl/p("rin"),ien=with_en_?ve/p("ren"):0;
        const double idd=id+lo_.is-lo_.high_clamp+boot,ihb=ib+ho_.is-ho_.high_clamp-boot;
        power_.quiescent=vd*id+vbs*ib;
        power_.input=vi*ihi+vl*ili+ve*ien;
        power_.drive=model_power::gate_drive(vbs,ho_.v,ho_.is,ho_.ik)+model_power::gate_drive(vd,lo_.v,lo_.is,lo_.ik);
        power_.clamp=model_power::gate_clamp(vbs,ho_.v,ho_.high_clamp,ho_.low_clamp)+model_power::gate_clamp(vd,lo_.v,lo_.high_clamp,lo_.low_clamp);
        power_.bootstrap=(x[0]-x[1])*boot;
        // Floating high-side reference: correction prevents bootstrap transfer
        // from being mistaken for dissipated power at a nonzero HS potential.
        power_.terminal_in=vd*idd+vbs*ihb-(x[2]-x[5])*boot+power_.input;
        power_.terminal_out=ho_.v*(ho_.is-ho_.ik-ho_.high_clamp+ho_.low_clamp)+lo_.v*(lo_.is-lo_.ik-lo_.high_clamp+lo_.low_clamp);
        power_.finish();
        return observe({ho_.v,ho_.v+x[2],lo_.v,lo_.v+x[5],ho_.is,ho_.ik,lo_.is,lo_.ik,id+lo_.is-lo_.high_clamp+boot,ib+ho_.is-ho_.high_clamp-boot,vi/p("rin"),vl/p("rin"),with_en_?ve/p("ren"):0,boot,vbs,rh&&en_gate_?1.:0.,rl&&en_gate_?1.:0.,eg},t,dt,{dd_&&hb_&&en_gate_,dd_&&en_gate_});
    }
};
std::unique_ptr<Core> make_core(const Config& c,const json& f){return std::unique_ptr<Core>(new HalfBridge(c,f));}
}
