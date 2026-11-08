// Key data sources: [1] 01_TI_TPS7B4250-Q1_RevC.pdf; [2] TPS7B4250_Q1_quick_function_v2.va. Datasheet ratings and supplied VA behavior/fit parameters are distinguished by the parameter-level source fields in json/tps7b4250_q1.json.
#include "tps7b4250_q1.h"
#include <fstream>
#include <limits>
#include <utility>
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

namespace xl_ldo {
enum class Kind { Negative, Tracking, LowNoise };
struct PinCurrents { double in=0,out=0,control=0,ground=0,pass=0,limit=0; int flags=0; };
class Kernel {
public:
    Kind kind;
    std::map<std::string,double> values;
    bool initialized=false, uvlo=false, enabled=false, armed=false, above_arm=false, block_reverse=false;
    explicit Kernel(Kind k):kind(k) {}
    double p(const char* key) const { return values.at(key); }
    void update_state(double vi,double vo,double ctrl) {
        if(kind==Kind::Tracking) {
            if(!initialized) { uvlo=vi>=p("VUVLO_RISE"); enabled=ctrl>=p("VEN_HIGH"); }
            if(vi>=p("VUVLO_RISE")) uvlo=true; else if(vi<=p("VUVLO_FALL")) uvlo=false;
            if(ctrl>=p("VEN_HIGH")) enabled=true; else if(ctrl<=p("VEN_LOW")) enabled=false;
        } else if(kind==Kind::LowNoise) {
            double gate=(.5+.5*std::tanh((ctrl-p("VEN_MID"))/p("VEN_SLOPE")))*
                        (.5+.5*std::tanh((vi-p("VUVLO_MID"))/p("VUVLO_SLOPE")));
            bool above=vo>p("VOUT_ARM_RATIO")*p("VOUT_NOM");
            if(gate<.1) armed=false; else if(above&&!above_arm) armed=true;
            above_arm=above;
        }
        initialized=true;
    }
    PinCurrents evaluate(double vi,double vo,double ctrl) const {
        PinCurrents r;
        if(kind==Kind::Negative) {
            double head=vo-vi;
            double target=ctrl-p("VREF")+p("LINE_VPV")*(head-p("HEADROOM_REF"));
            target=std::max(p("VOUT_MIN"),std::min(p("VOUT_MAX"),target));
            double reg=std::max(target,vi+p("VDO"));
            double w=std::max(0.,std::min(1.,(head-15.)/25.));
            r.limit=p("ICL_15V")+(p("ICL_40V")-p("ICL_15V"))*w;
            double raw=(vo-reg)/p("RREG");
            double pos=.5*(raw+std::sqrt(raw*raw+p("EPS_I")*p("EPS_I")));
            r.pass=r.limit*std::tanh(pos/r.limit);
            r.in=-r.pass-p("IADJ"); r.out=r.pass+p("GMIN_OUT")*vo;
            r.control=p("IADJ"); r.ground=-p("GMIN_OUT")*vo;
        } else if(kind==Kind::Tracking) {
            double on=uvlo&&enabled?1.:0.;
            double reg=pos((on*(ctrl+p("TRACK_ERR"))-vo)/p("RREG"));
            double drop=pos((vi-vo-p("VDO_OFFSET"))/p("RDO_SLOPE"));
            r.pass=on*pos(min2(reg,min2(drop,p("ICL"))));
            double iq=p("IQ_LIGHT");
            if(r.pass>=.030) iq=p("IQ_30MA");
            else if(r.pass>.0005) iq+=(p("IQ_30MA")-iq)*(r.pass-.0005)/.0295;
            iq=on*iq+(1-on)*p("IQ_STANDBY");
            double sink=on*pos((vo-ctrl-p("TRACK_ERR"))/p("ROV_CLAMP"));
            r.in=r.pass+iq; r.out=-r.pass+sink+p("GMIN_OUT")*vo;
            r.control=on*p("IADJ"); r.ground=-iq-sink-r.control-p("GMIN_OUT")*vo;
            r.limit=on*p("ICL"); r.flags=(uvlo?1:0)+(enabled?2:0);
        } else {
            double von=.5+.5*std::tanh((ctrl-p("VEN_MID"))/p("VEN_SLOPE"));
            double vuv=.5+.5*std::tanh((vi-p("VUVLO_MID"))/p("VUVLO_SLOPE"));
            double gate=von*vuv;
            double line=p("VOUT_NOM")*(1+p("LINE_REG")*(vi-p("VIN_REF")));
            double head=vi-p("VDO_MAX");
            double target=gate*.5*(line+head-std::sqrt((line-head)*(line-head)+1e-6));
            r.limit=armed ? p("ISC_TYP")+(p("ICL_TYP")-p("ISC_TYP"))*(.5+.5*std::tanh((vo-.55*p("VOUT_NOM"))/(.12*p("VOUT_NOM")))) : p("ICL_TYP");
            r.pass=r.limit*std::tanh((target-vo)/(r.limit*p("ROUT")));
            if(block_reverse) r.pass=std::max(0.,r.pass);
            double iq=gate*p("IQ_ON")+(1-gate)*p("IQ_OFF");
            r.in=r.pass+iq; r.out=-r.pass+p("GMIN_OUT")*vo;
            r.control=(1-von)*ctrl/500000.+von*90e-9;
            r.ground=-iq-r.control-p("GMIN_OUT")*vo; r.flags=armed?1:0;
        }
        return r;
    }
private:
    static double pos(double x) { return .5*(x+std::sqrt(x*x+1e-18)); }
    static double min2(double a,double b) { return .5*(a+b-std::sqrt((a-b)*(a-b)+1e-14)); }
};
}

using xl_ldo::Kernel;
using xl_ldo::Kind;
using xl_ldo::PinCurrents;
using xl_ldo::NullableNumber;
struct TPS7B4250Q1::Impl {
    double output_v_=0,output_i_=0,output_g_=0;
    Impl(const std::string& path,Kind kind):kernel_(kind) {
        std::ifstream file(path);
        if(!file) throw std::runtime_error("Cannot open config: "+path);
        file>>cfg_;
        for(auto it=cfg_.begin();it!=cfg_.end();++it) {
            if(it.value().is_object()&&it.value().contains("value")&&it.value()["value"].is_number()) {
                double value=it.value()["value"].template get<double>();
                if(!std::isfinite(value)) throw std::invalid_argument("Non-finite parameter: "+it.key());
                kernel_.values[it.key()]=value;
            }
        }
        for(const char* k:{"dt_s","cout_f"}) if(p(k)<=0) throw std::invalid_argument(std::string(k)+" must be positive");
        if(p("esr_ohm")<0) throw std::invalid_argument("Negative ESR");
        if(kind==Kind::Negative) for(const char* k:{"RREG","ICL_15V","ICL_40V","r1_ohm","r2_ohm"}) require_positive(k);
        if(kind==Kind::Tracking) {
            for(const char* k:{"RREG","RDO_SLOPE","ROV_CLAMP","ICL"}) require_positive(k);
            if(p("VUVLO_RISE")<=p("VUVLO_FALL")||p("VEN_HIGH")<=p("VEN_LOW")) throw std::invalid_argument("Invalid hysteresis");
        }
        if(kind==Kind::LowNoise) for(const char* k:{"ROUT","VEN_SLOPE","VUVLO_SLOPE","ICL_TYP","ISC_TYP","VOUT_NOM"}) require_positive(k);
        if(kind==Kind::LowNoise) {
            for(const char* k:{"FZ1_20","FZ2_20","FP1_20","FP2_20","FZ1_300","FZ2_300","FP1_300","FP2_300","H0_20","H0_300"}) require_positive(k);
            if(p("ILOAD_HIGH")<=p("ILOAD_LOW")||p("INOISE_HIGH")<=p("INOISE_LOW")||p("NOISE_FHI")<=p("NOISE_FLO"))
                throw std::invalid_argument("Invalid interpolation or noise bandwidth");
            kernel_.block_reverse=cfg_.at("reverse_pass_policy").at("value")=="block";
        }
        initial_=cfg_.at("initial_state").at("value").template get<std::string>();
        mode_=cfg_.at("mode").at("value").template get<std::string>();
        adjust_=cfg_.at("adjust_mode").at("value").template get<std::string>();
        if(initial_!="dc"&&initial_!="uncharged") throw std::invalid_argument("initial_state must be dc or uncharged");
        if(mode_!="transient"&&mode_!="ripple") throw std::invalid_argument("Invalid mode");
        if(mode_=="ripple"&&kind!=Kind::LowNoise) throw std::invalid_argument("Ripple mode is implemented only for TPS7A20");
        if(adjust_!="fixed"&&adjust_!="feedback") throw std::invalid_argument("Invalid adjust_mode");
        if(cfg_.at("enable_time_noise").at("value").template get<bool>()) throw std::invalid_argument("Time-domain noise is not implemented; use the explicit noise-query API");
        if(kind!=Kind::Negative&&adjust_!="fixed") throw std::invalid_argument("Feedback divider is LM337-only");
        period_=sc_core::sc_time(p("dt_s"),sc_core::SC_SEC);
    }
    const nlohmann::json& configuration() const { return cfg_; }
    double get_sample_period() const { return period_.to_seconds(); }
    double unserved_load_current() const { return unserved_; }
    int protection_state() const { return last_.flags; }
    double last_pass_current() const { return last_.pass; }
    double last_current_limit() const { return last_.limit; }
    double last_kcl_residual() const { return last_residual_; }
    double stored_capacitor_voltage() const { return polarity()*vc_; }
    std::complex<double> psrr_transfer(double hz,double load) const {
        if(kernel_.kind!=Kind::LowNoise||hz<0||load<0) throw std::invalid_argument("PSRR API requires TPS7A20 and nonnegative inputs");
        std::complex<double> s(0,2*std::acos(-1.)*hz);
        auto h=[&](const char* suffix) {
            std::string t=suffix;
            double z1=kernel_.values.at("FZ1_"+t),z2=kernel_.values.at("FZ2_"+t);
            double p1=kernel_.values.at("FP1_"+t),p2=kernel_.values.at("FP2_"+t);
            return kernel_.values.at("H0_"+t)*(1.+s/(2*std::acos(-1.)*z1))*(1.+s/(2*std::acos(-1.)*z2))/
                ((1.+s/(2*std::acos(-1.)*p1))*(1.+s/(2*std::acos(-1.)*p2)));
        };
        double w=std::max(0.,std::min(1.,(load-p("ILOAD_LOW"))/(p("ILOAD_HIGH")-p("ILOAD_LOW"))));
        return (1-w)*h("20")+w*h("300");
    }
    double integrated_noise_rms(double load,double low,double high) const {
        if(kernel_.kind!=Kind::LowNoise||load<0||low<0||high<low) throw std::invalid_argument("Invalid noise query");
        double w=std::max(0.,std::min(1.,(load-p("INOISE_LOW"))/(p("INOISE_HIGH")-p("INOISE_LOW"))));
        double rms=p("VN_RMS_LOW")+w*(p("VN_RMS_HIGH")-p("VN_RMS_LOW"));
        double bandwidth=std::max(0.,std::min(high,p("NOISE_FHI"))-std::max(low,p("NOISE_FLO")));
        return rms*std::sqrt(bandwidth/(p("NOISE_FHI")-p("NOISE_FLO")));
    }
    void step(double vi,double load,double res,double ctrl,double rip) {
        if(!std::isfinite(vi)||!std::isfinite(load)||load<0||!std::isfinite(res)||res<=0||!std::isfinite(ctrl)||!std::isfinite(rip)||polarity()*vi<0)
            throw std::invalid_argument("Invalid voltage/load input or wrong VIN polarity");
        double dt=period_.to_seconds(),series=0;
        if(mode_=="ripple") {
            double actual_load=load+previous_output_/res;
            double w=std::max(0.,std::min(1.,(actual_load-p("ILOAD_LOW"))/(p("ILOAD_HIGH")-p("ILOAD_LOW"))));
            series=(1-w)*ripple_path(rip,"20",0)+w*ripple_path(rip,"300",2);
        } else if(rip!=0) throw std::invalid_argument("Set ripple=0 in transient mode; drive VIN for line transients");
        auto quantities=[&](double u) {
            double core=polarity()*u,adj=adjust_voltage(core,ctrl);
            PinCurrents r=kernel_.evaluate(vi,core,adj);
            double divider=feedback_load(core,adj);
            return std::make_pair(r,-polarity()*r.out-load-u/res-divider);
        };
        if(first_) {
            kernel_.update_state(vi,0,adjust_voltage(0,ctrl));
            if(initial_=="dc") {
                auto residual=[&](double u){return -quantities(u).second;};
                vc_=solve_voltage(residual,vi,ctrl);
                kernel_.update_state(vi,polarity()*vc_,adjust_voltage(polarity()*vc_,ctrl));
                // Keep the reached startup branch when foldback becomes armed.
                // Re-solving from zero would select the collapsed DC branch at loads above ISC.
                previous_output_=vc_;
            }
            first_=false;
        }
        kernel_.update_state(vi,polarity()*previous_output_,adjust_voltage(polarity()*previous_output_,ctrl));
        double before=vc_,z=dt/p("cout_f")+p("esr_ohm");
        auto residual=[&](double u){return u-before-z*quantities(u).second;};
        double u=solve_voltage(residual,vi,ctrl);
        auto q=quantities(u); last_=q.first;
        double ic=q.second;
        last_residual_=residual(u)/z;
        unserved_=u==0?std::max(0.,last_residual_):0;
        vc_=std::max(0.,u-p("esr_ohm")*ic);
        if(u==0&&unserved_>0) vc_=0;
        double core=polarity()*u,adj=adjust_voltage(core,ctrl);
        last_power_=vi*last_.in+core*last_.out+adj*last_.control-p("GMIN_OUT")*core*core;
        previous_output_=u;
        // Ripple mode reproduces the original VA's AC-only output-voltage path.
        // Current, capacitor and power outputs remain bias quantities in this mode.
        output_v_=polarity()*u+series; output_i_=last_.in; output_g_=-last_.ground;
    }
    nlohmann::json cfg_;
    Kernel kernel_;
    double last_power_=0;
    double p(const char* key) const { return kernel_.p(key); }
    NullableNumber nullable(const char* key) const {
        const auto& v=cfg_.at(key).at("value");
        return v.is_null()?NullableNumber{}:NullableNumber{true,v.template get<double>()};
    }
    sc_core::sc_time period_;
    std::string initial_,mode_,adjust_;
    double vc_=0,previous_output_=0,unserved_=0,last_residual_=0;
    bool first_=true;
    PinCurrents last_;
    double lp_[4]={0,0,0,0},previous_[4]={0,0,0,0};
    double polarity() const { return kernel_.kind==Kind::Negative?-1.:1.; }
    void require_positive(const char* k) const { if(p(k)<=0) throw std::invalid_argument(std::string(k)+" must be positive"); }
    double adjust_voltage(double core,double external) const {
        if(kernel_.kind!=Kind::Negative||adjust_!="feedback") return external;
        return (core/p("r1_ohm")-p("IADJ"))/(1/p("r1_ohm")+1/p("r2_ohm"));
    }
    double feedback_load(double core,double adj) const {
        return kernel_.kind==Kind::Negative&&adjust_=="feedback" ? polarity()*(core-adj)/p("r1_ohm") : 0.;
    }
    template<class F> double solve_voltage(F residual,double vi,double ctrl) const {
        if(residual(0)>=0) return 0;
        double hi=std::max({1.,std::fabs(vi),std::fabs(ctrl),vc_,previous_output_})+1;
        for(int n=0;n<20&&residual(hi)<0;++n) hi*=2;
        if(residual(hi)<0) throw std::runtime_error("Voltage root is not bracketed");
        double lo=0;
        for(int n=0;n<70;++n) { double mid=.5*(lo+hi); if(residual(mid)>0) hi=mid; else lo=mid; }
        return .5*(lo+hi);
    }
    double ripple_path(double input,const char* suffix,int start) {
        std::string t=suffix;
        double out=input*kernel_.values.at("H0_"+t);
        for(int section=1;section<=2;++section) {
            double fz=kernel_.values.at("FZ"+std::to_string(section)+"_"+t);
            double fp=kernel_.values.at("FP"+std::to_string(section)+"_"+t);
            double x=2*std::acos(-1.)*fp*period_.to_seconds(),a=std::exp(-x),b=-std::expm1(-x)/x;
            int i=start+section-1;
            lp_[i]=a*lp_[i]+(1-b)*out+(b-a)*previous_[i]; previous_[i]=out;
            double ratio=fp/fz; out=ratio*out+(1-ratio)*lp_[i];
        }
        return out;
    }
};
TPS7B4250Q1::TPS7B4250Q1(sc_core::sc_module_name name,const std::string& config_file)
    :sca_tdf::sca_module(name),impl_(std::make_unique<Impl>(config_file,Kind::Tracking)) {}
TPS7B4250Q1::~TPS7B4250Q1() = default;
void TPS7B4250Q1::set_attributes() { set_timestep(impl_->period_); }
void TPS7B4250Q1::processing() {
    impl_->step(vin.read(),iload.read(),rload.read(),control.read(),ripple.read());
    vout.write(impl_->output_v_); iin.write(impl_->output_i_); ignd.write(impl_->output_g_); power.write(impl_->last_power_);
}
std::string TPS7B4250Q1::get_module_name() const { return impl_->cfg_.at("name").at("value").get<std::string>(); }
double TPS7B4250Q1::get_version() const { return impl_->p("version"); }
NullableNumber TPS7B4250Q1::get_area() const { return impl_->nullable("area_mm2"); }
NullableNumber TPS7B4250Q1::get_latency() const { return impl_->nullable("latency_s"); }
double TPS7B4250Q1::get_timestep_s() const { return impl_->get_sample_period(); }
double TPS7B4250Q1::get_sample_period() const { return get_timestep_s(); }
double TPS7B4250Q1::get_power_dissipation() const { return impl_->last_power_; }
const nlohmann::json& TPS7B4250Q1::configuration() const { return impl_->configuration(); }
double TPS7B4250Q1::unserved_load_current() const { return impl_->unserved_load_current(); }
int TPS7B4250Q1::protection_state() const { return impl_->protection_state(); }
double TPS7B4250Q1::last_pass_current() const { return impl_->last_pass_current(); }
double TPS7B4250Q1::last_current_limit() const { return impl_->last_current_limit(); }
double TPS7B4250Q1::last_kcl_residual() const { return impl_->last_kcl_residual(); }
double TPS7B4250Q1::stored_capacitor_voltage() const { return impl_->stored_capacitor_voltage(); }
std::complex<double> TPS7B4250Q1::psrr_transfer(double hz,double load) const { return impl_->psrr_transfer(hz,load); }
double TPS7B4250Q1::integrated_noise_rms(double load,double low,double high) const { return impl_->integrated_noise_rms(load,low,high); }

// Optional complete self-test entry. Normal model builds do not define this macro.
// Build this .cpp with -DLDO_SELF_TEST_MAIN, then use:
//   ./ldo_selftest --self-test json/tps7b4250_q1.json selftest/retest
#ifdef LDO_SELF_TEST_MAIN
#include <cstdlib>
using ModelUnderTest=TPS7B4250Q1;
#include <iomanip>
#include <iostream>
#include <fstream>
#include <cmath>

struct ScenarioSource : sca_tdf::sca_module {
    sca_tdf::sca_out<double> vin{"vin"},iload{"iload"},rload{"rload"},control{"control"},ripple{"ripple"};
    nlohmann::json segments;
    double dt;
    ScenarioSource(sc_core::sc_module_name n,const nlohmann::json& s,double step):sca_module(n),segments(s),dt(step) {}
    void set_attributes() override { set_timestep(dt,sc_core::SC_SEC); }
    void processing() override {
        double t=get_time().to_seconds(),elapsed=0;
        const nlohmann::json* selected=&segments.back();
        for(const auto& s:segments) { elapsed+=s.at("duration_s").get<double>(); if(t<elapsed-1e-15) { selected=&s; break; } }
        const auto& s=*selected;
        vin.write(s.at("vin").get<double>()); iload.write(s.at("iload").get<double>());
        rload.write(s.at("rload").get<double>()); control.write(s.at("control").get<double>());
        double amplitude=s.value("ripple_amplitude",0.),hz=s.value("ripple_frequency",0.);
        ripple.write(amplitude*std::sin(2*std::acos(-1.)*hz*t));
    }
};
struct ScenarioRecorder : sca_tdf::sca_module {
    sca_tdf::sca_in<double> vin{"vin"},iload{"iload"},rload{"rload"},control{"control"},ripple{"ripple"};
    sca_tdf::sca_in<double> vout{"vout"},iin{"iin"},ignd{"ignd"},power{"power"};
    ModelUnderTest& dut;
    std::ofstream file;
    ScenarioRecorder(sc_core::sc_module_name n,ModelUnderTest& m,const char* path):sca_module(n),dut(m),file(path) {
        if(!file) throw std::runtime_error("Cannot open scenario CSV");
        file<<"time_s,vin_V,control_V,iload_A,rload_ohm,ripple_V,vout_V,iin_A,ignd_A,power_W,pass_A,limit_A,state,kcl_A,unserved_A,cap_voltage_V\n"<<std::setprecision(17);
    }
    void processing() override {
        file<<get_time().to_seconds()<<','<<vin.read()<<','<<control.read()<<','<<iload.read()<<','<<rload.read()<<','<<ripple.read()<<','
            <<vout.read()<<','<<iin.read()<<','<<ignd.read()<<','<<power.read()<<','<<dut.last_pass_current()<<','<<dut.last_current_limit()<<','
            <<dut.protection_state()<<','<<dut.last_kcl_residual()<<','<<dut.unserved_load_current()<<','<<dut.stored_capacitor_voltage()<<'\n';
    }
};
struct ScenarioSimulator : sc_core::sc_module {
    ModelUnderTest dut;
    ScenarioSource source;
    ScenarioRecorder recorder;
    sca_tdf::sca_signal<double> vi,load,res,ctrl,rip,vo,ii,ig,pw;
    ScenarioSimulator(sc_core::sc_module_name n,const char* cfg,const nlohmann::json& segments,const char* csv)
        :sc_module(n),dut("dut",cfg),source("source",segments,dut.get_sample_period()),recorder("recorder",dut,csv) {
        source.vin(vi);source.iload(load);source.rload(res);source.control(ctrl);source.ripple(rip);
        dut.vin(vi);dut.iload(load);dut.rload(res);dut.control(ctrl);dut.ripple(rip);
        dut.vout(vo);dut.iin(ii);dut.ignd(ig);dut.power(pw);
        recorder.vin(vi);recorder.iload(load);recorder.rload(res);recorder.control(ctrl);recorder.ripple(rip);
        recorder.vout(vo);recorder.iin(ii);recorder.ignd(ig);recorder.power(pw);
    }
};
int run_signal_flow_scenario(int argc,char* argv[]) {
    try {
        if(argc!=5) throw std::invalid_argument("Expected config, scenario JSON, CSV and metadata JSON");
        std::ifstream f(argv[2]);nlohmann::json scenario;f>>scenario;
        ScenarioSimulator sim("sim",argv[1],scenario.at("segments"),argv[3]);
        double duration=0;for(const auto& s:scenario.at("segments")) duration+=s.at("duration_s").get<double>();
        sc_core::sc_start(sc_core::sc_time(duration,sc_core::SC_SEC));
        auto area=sim.dut.get_area(),latency=sim.dut.get_latency();
        nlohmann::json meta={{"name",sim.dut.get_module_name()},{"version",sim.dut.get_version()},
            {"area_mm2",area.available?nlohmann::json(area.value):nlohmann::json(nullptr)},
            {"latency_s",latency.available?nlohmann::json(latency.value):nlohmann::json(nullptr)},
            {"sample_period_s",sim.dut.get_sample_period()},{"power_w",sim.dut.get_power_dissipation()},
            {"ams_release",sca_core::sca_release()},{"ams_numeric_version",sca_core::sca_version()}};
        if(scenario.contains("psrr_queries")) {
            for(const auto& q:scenario.at("psrr_queries")) {
                auto h=sim.dut.psrr_transfer(q.at("hz"),q.at("load_a"));
                meta["psrr_queries"].push_back({{"hz",q.at("hz")},{"load_a",q.at("load_a")},{"gain",std::abs(h)},{"psrr_db",-20*std::log10(std::abs(h))}});
            }
        }
        if(scenario.contains("noise_queries")) {
            for(const auto& q:scenario.at("noise_queries")) meta["noise_queries"].push_back({{"rms_v",sim.dut.integrated_noise_rms(q.at("load_a"),q.at("low_hz"),q.at("high_hz"))}});
        }
        std::ofstream out(argv[4]);if(!out) throw std::runtime_error("Cannot write metadata");out<<meta.dump(2);
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 2; }
}

namespace {
// POSIX shell quoting keeps every CLI argument and the Python source literal.
std::string ldo_shell_quote(const std::string& value) {
    std::string result="'";
    for(char c:value) {
        if(c=='\'') result+="'\\''";
        else result+=c;
    }
    return result+"'";
}
const char* ldo_selftest_python = R"LDO_TEST_PY(from pathlib import Path
import argparse
import csv
import json
import math
import subprocess
import numpy as np

parser=argparse.ArgumentParser()
parser.add_argument('--exe',required=True,type=Path)
parser.add_argument('--config',required=True,type=Path)
parser.add_argument('--results',type=Path)
args=parser.parse_args()
base=json.loads(args.config.read_text(encoding='utf8'))
base.pop('self_test_evidence',None)
if args.results is None:args.results=(args.config.parent/base['test_results_dir']['value']).resolve()
args.results.mkdir(parents=True,exist_ok=True)
chip=base['name']['value']
negative=chip=='lm337'
tracking=chip=='tps7b4250_q1'
low_noise=chip=='tps7a20_2v8'
vi=-15. if negative else 13.5 if tracking else 3.8
ctrl=0. if negative else 5. if tracking else 1.
nominal=10.0546 if negative else 5. if tracking else 2.8
load0=.01 if negative or tracking else .02
rated=1.5 if negative else .05 if tracking else .3
metrics=[]

def p(key):return float(base[key]['value'])
def segment(duration=2e-5,**kw):
    s=dict(duration_s=duration,vin=vi,control=ctrl,iload=load0,rload=1e12)
    s.update(kw);return s
def metric(name,value,reference,tolerance,unit='',scope='migration_or_numerical'):
    error=abs(float(value)-float(reference))
    passed=bool(math.isfinite(float(value)) and error<=tolerance)
    metrics.append(dict(name=name,value=float(value),reference=float(reference),absolute_error=error,tolerance=tolerance,unit=unit,passed=passed,evidence_scope=scope))
def condition(name,ok,scope='migration_or_numerical'):
    metric(name,1 if ok else 0,1,0,scope=scope)
def run(name,segments,updates=None,extra=None):
    cfg=json.loads(json.dumps(base))
    for key,value in (updates or {}).items():cfg[key]['value']=value
    folder=args.results/name;folder.mkdir(exist_ok=True)
    (folder/'config.json').write_text(json.dumps(cfg,indent=2,ensure_ascii=False),encoding='utf8')
    scenario={'segments':segments,**(extra or {})}
    (folder/'scenario.json').write_text(json.dumps(scenario,indent=2),encoding='utf8')
    command=[str(args.exe),str(folder/'config.json'),str(folder/'scenario.json'),str(folder/'wave.csv'),str(folder/'metadata.json')]
    done=subprocess.run(command,capture_output=True,text=True,timeout=90)
    (folder/'run.log').write_text(done.stdout+done.stderr,encoding='utf8')
    if done.returncode:raise RuntimeError(f'{chip}/{name} failed; see {folder}/run.log')
    data=np.atleast_1d(np.genfromtxt(folder/'wave.csv',delimiter=',',names=True))
    meta=json.loads((folder/'metadata.json').read_text())
    condition(name+'_finite',all(np.isfinite(data[field]).all() for field in data.dtype.names))
    # Voltage is an endpoint state of the held-input integration interval.
    return data,meta,cfg
def tail(data,field,n=10):return float(np.mean(data[field][-min(n,len(data)):]))
def invalid(name,updates=None,input_update=None):
    folder=args.results/name;folder.mkdir(exist_ok=True)
    cfg=json.loads(json.dumps(base))
    for key,value in (updates or {}).items():cfg[key]['value']=value
    (folder/'config.json').write_text(json.dumps(cfg),encoding='utf8')
    (folder/'scenario.json').write_text(json.dumps({'segments':[segment(**(input_update or {}))]}))
    done=subprocess.run([str(args.exe),str(folder/'config.json'),str(folder/'scenario.json'),str(folder/'wave.csv'),str(folder/'metadata.json')],capture_output=True,text=True,timeout=15)
    (folder/'run.log').write_text(done.stdout+done.stderr,encoding='utf8')
    condition(name+'_rejected',done.returncode!=0)

dc,metadata,cfg=run('dc',[segment()])
vdc=tail(dc,'vout_V')
condition('output_polarity',vdc<0 if negative else vdc>0)
metric('nominal_output',abs(vdc),nominal,.03 if negative else .005,'V','legacy_model_nominal')
condition('metadata_name',metadata['name']==chip)
condition('metadata_version',metadata['version']==p('version'))
condition('unknown_area_explicit',metadata['area_mm2'] is None)
condition('physical_latency_not_sample_period',metadata['latency_s'] is None)
metric('sample_period',metadata['sample_period_s'],p('dt_s'),1e-15,'s')
metric('metadata_power',metadata['power_w'],tail(dc,'power_W',1),1e-12,'W')
metric('dc_kcl',float(np.max(np.abs(dc['kcl_A']))),0,1e-7,'A')
condition('nonnegative_chip_dissipation',float(np.min(dc['power_W']))>=-1e-9)
if negative:
    fixed,_,_=run('fixed_adj',[segment(control=-8.75)],{'adjust_mode':'fixed'})
    expected=(-8.75-p('VREF')-p('LINE_VPV')*(vi+p('HEADROOM_REF'))+p('RREG')*p('ICL_15V')*np.arctanh(load0/p('ICL_15V')))/(1-p('LINE_VPV'))
    metric('fixed_adj_closed_form',tail(fixed,'vout_V'),expected,2e-6,'V','legacy_current_law_closed_form')
    adj=(vdc/p('r1_ohm')-p('IADJ'))/(1/p('r1_ohm')+1/p('r2_ohm'))
    expected_power=(vdc-vi)*tail(dc,'pass_A')+(adj-vi)*p('IADJ')
elif tracking:
    metric('tracking_10mA_drop',vdc,ctrl-p('RREG')*load0,2e-7,'V','legacy_current_law_closed_form')
    expected_power=vi*tail(dc,'iin_A')-vdc*load0+ctrl*p('IADJ')-p('GMIN_OUT')*vdc*vdc
else:
    expected_power=vi*tail(dc,'iin_A')-vdc*load0+ctrl*90e-9-p('GMIN_OUT')*vdc*vdc
metric('dc_terminal_power',tail(dc,'power_W'),expected_power,2e-7,'W','terminal_energy_balance')

light,_,_=run('light_load',[segment(iload=0)])
heavy,_,_=run('rated_load',[segment(iload=rated)])
condition('load_regulation_direction',abs(tail(light,'vout_V'))>=abs(tail(heavy,'vout_V')))
metric('rated_load_kcl',float(np.max(np.abs(heavy['kcl_A']))),0,1e-7,'A')
condition('rated_output_not_collapsed',abs(tail(heavy,'vout_V'))>.9*nominal)
condition('rated_pass_below_limit',bool(np.all(heavy['pass_A']<=heavy['limit_A']+1e-8)))

step_segments=[segment(1e-4,iload=load0),segment(1e-4,iload=rated*.5),segment(1e-4,iload=load0)]
step,_,_=run('load_step',step_segments)
metric('load_step_final_output',tail(step,'vout_V'),vdc,1e-4,'V')
metric('load_step_kcl',float(np.max(np.abs(step['kcl_A']))),0,1e-7,'A')
fine,_,_=run('load_step_half_dt',step_segments,{'dt_s':p('dt_s')/2})
coarse_end=step['time_s']+p('dt_s')
fine_end=fine['time_s']+p('dt_s')/2
error=float(np.max(np.abs(step['vout_V']-np.interp(coarse_end,fine_end,fine['vout_V']))))
metric('load_step_dt_halving_max_error',error,0,.005,'V','numerical_convergence')

startup,_,_=run('startup',[segment(2e-4)],{'initial_state':'uncharged'})
condition('startup_initial_below_final',abs(startup['vout_V'][0])<abs(vdc))
metric('startup_settled_voltage',tail(startup,'vout_V'),vdc,1e-4,'V')
condition('startup_no_wrong_polarity',bool(np.all(startup['vout_V']<=1e-9) if negative else np.all(startup['vout_V']>=-1e-9)))

short,_,_=run('short_and_recovery',[segment(2e-4,iload=0,rload=1000),segment(2e-4,iload=0,rload=1),segment(4e-4,iload=0,rload=1000)])
normal_window=short['time_s']<1.9e-4
short_window=(short['time_s']>3.8e-4)&(short['time_s']<4e-4)
normal=float(np.mean(short['vout_V'][normal_window][-10:]))
condition('short_reduces_output',abs(float(np.mean(short['vout_V'][short_window])))<.5*abs(normal))
metric('short_recovery_final',tail(short,'vout_V'),normal,1e-3,'V')
condition('short_pass_limited',bool(np.all(short['pass_A']<=short['limit_A']+1e-8)))
if low_noise:metric('armed_short_foldback',float(np.mean(short['pass_A'][short_window])),.160, .01,'A','legacy_VA_foldback_parameter')

esr,_,_=run('external_cap_esr',[segment(2e-4)],{'cout_f':p('cout_f')*2,'esr_ohm':.3,'initial_state':'uncharged'})
metric('external_cap_final_dc',tail(esr,'vout_V'),vdc,1e-4,'V','numerical_circuit_check_not_stability_qualification')

if tracking:
    voltages=[2.5,3.7,3.3,2.9,3.3,3.7]
    hysteresis,_,_=run('uvlo_hysteresis',[segment(3e-4,vin=v,control=2,iload=0,rload=100) for v in voltages])
    for i,expected_state in enumerate([2,3,3,2,2,3]):
        mask=(hysteresis['time_s']>(i+.8)*3e-4)&(hysteresis['time_s']<(i+1)*3e-4)
        metric(f'uvlo_sequence_{i}',float(np.median(hysteresis['state'][mask])),expected_state,0)
    dropout,_,_=run('dropout',[segment(vin=4.3)])
    metric('dropout_10mA',tail(dropout,'vout_V'),4.3-p('VDO_OFFSET')-p('RDO_SLOPE')*load0,2e-6,'V','legacy_current_law_closed_form')
if tracking or low_noise:
    off,_,_=run('disable',[segment(2e-4),segment(4e-4,control=0,iload=0,rload=100)])
    expected=p('IQ_STANDBY') if tracking else p('IQ_OFF')
    metric('disabled_input_current',tail(off,'iin_A'),expected,1e-9,'A','legacy_VA_parameter')
    condition('disable_no_unphysical_negative_power',float(np.min(off['power_W']))>=-1e-9)

if low_noise:
    queries=[{'hz':f,'load_a':load} for f in [100,1000,100000,1000000] for load in [.02,.3]]
    _,spectral,_=run('spectral_queries',[segment()],extra={'psrr_queries':queries,'noise_queries':[{'load_a':.001,'low_hz':10,'high_hz':100000},{'load_a':.3,'low_hz':10,'high_hz':100000}]})
    metric('noise_integral_1mA',spectral['noise_queries'][0]['rms_v'],10e-6,1e-12,'V_rms','legacy_white_noise_band_integral')
    metric('noise_integral_300mA',spectral['noise_queries'][1]['rms_v'],7e-6,1e-12,'V_rms','legacy_white_noise_band_integral')
    for f in [1000,100000,1000000]:
        dt=1/(f*512)
        wave,meta,_=run('ripple_'+str(f),[segment(12/f,ripple_amplitude=.1,ripple_frequency=f)],{'mode':'ripple','dt_s':dt},extra={'psrr_queries':[{'hz':f,'load_a':load0}]})
        mask=wave['time_s']>=8/f
        time=wave['time_s'][mask];out=wave['vout_V'][mask]
        design=np.column_stack([np.sin(2*np.pi*f*time),np.cos(2*np.pi*f*time),np.ones_like(time)])
        coef=np.linalg.lstsq(design,out,rcond=None)[0]
        gain=float(np.hypot(coef[0],coef[1]))/.1
        measured=-20*np.log10(gain)
        metric(f'psrr_tdf_{f}Hz',measured,meta['psrr_queries'][0]['psrr_db'],.15,'dB','time_waveform_vs_legacy_transfer_function')

invalid('invalid_capacitor',{'cout_f':0})
invalid('invalid_mode',{'mode':'unsupported'})
invalid('time_noise_not_silently_enabled',{'enable_time_noise':True})
invalid('invalid_load',input_update={'rload':0})
summary={'chip':chip,'version':p('version'),'checks':len(metrics),'passed':sum(x['passed'] for x in metrics),'failed':sum(not x['passed'] for x in metrics),
    'scope':'behavioral_migration_and_numerical_checks','independent_physical_accuracy':'pending','metrics':metrics}
(args.results/'metrics.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf8')
with (args.results/'metrics.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(metrics[0]));writer.writeheader();writer.writerows(metrics)
print(json.dumps({k:v for k,v in summary.items() if k!='metrics'},ensure_ascii=False))
for m in metrics:
    if not m['passed']:print('FAIL',m)
raise SystemExit(1 if summary['failed'] else 0)
)LDO_TEST_PY";
}
int sc_main(int argc,char* argv[]) {
    if(argc==4 && std::string(argv[1])=="--self-test") {
        std::string command="python3 -c "+ldo_shell_quote(ldo_selftest_python)+
            " --exe "+ldo_shell_quote(argv[0])+" --config "+ldo_shell_quote(argv[2])+
            " --results "+ldo_shell_quote(argv[3]);
        return std::system(command.c_str())==0 ? 0 : 1;
    }
    return run_signal_flow_scenario(argc,argv);
}
#endif
