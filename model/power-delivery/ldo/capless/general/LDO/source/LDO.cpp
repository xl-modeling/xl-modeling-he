#include "LDO.h"
#include <complex>
#include <filesystem>
#include <fstream>

namespace {
double highpass(double input,double state,double previous,double tau,double step) {
    double coefficient=step/(2*tau);
    double next=((1-coefficient)*state+coefficient*(input+previous))/(1+coefficient);
    return input-next;
}
double smooth_direction(double value,double width,bool positive) {
    double radius=std::sqrt(value*value+width*width);
    return positive?.5*(value+radius-width):.5*(value-radius+width);
}
}

LDO::configs LDO::get_configs_from_json(const char* path) {
    std::ifstream input(path);
    if(!input)throw std::invalid_argument("Missing LDO configuration");
    nlohmann::json document;
    input>>document;
    auto folder=std::filesystem::absolute(path).parent_path();
    document["resolved_table_path"]=(folder/document.at("table_file").get<std::string>()).lexically_normal().string();
    return {document,document.at("configuration").at("timestep_s").get<double>()};
}

LDO::LDO(sc_core::sc_module_name name,const configs& cfg)
    :sca_tdf::sca_module(name),model(cfg.document),dt(cfg.timestep_s) {
    if(!std::isfinite(dt)||dt<=0||dt>1e-9)throw std::invalid_argument("LDO timestep must be in (0,1 ns]");
}

void LDO::set_attributes(){set_timestep(dt,sc_core::SC_SEC);}

void LDO::processing() {
    double ground=AVSS.read();
    double supply=VDD.read()-ground,reference=VREF.read()-ground,analog=AVDD.read()-ground,bias=IBIAS.read();
    double load=load_current.read(),conductance=load_conductance.read(),external_cap=external_capacitance.read(),temp=temperature.read();
    for(double value:{ground,supply,reference,analog,bias,load,conductance,external_cap,temp}) {
        if(!std::isfinite(value))SC_REPORT_FATAL("LDO_INPUT","Nonfinite input");
    }
    if(supply<0||analog<0||bias<0||reference<0||load<0||conductance<0||external_cap<0||temp<-40||temp>85)
        SC_REPORT_FATAL("LDO_INPUT","Invalid input or temperature outside -40..85 C");
    double gate=std::clamp(std::min({analog/2.5,bias/1e-5,reference/.9,supply/model.supplies.front()}),0.0,1.0);
    double normalized_supply=gate>1e-9?supply/gate:model.startup_supply;
    double normalized_reference=gate>1e-9?reference/gate:1.0;
    normalized_supply=std::clamp(normalized_supply,model.supplies.front(),model.supplies.back());
    normalized_reference=std::clamp(normalized_reference,model.references.front(),model.references.back());
    double effective_supply=supply-model.startup_supply*gate;
    if(!initialized) {
        lowpass.fill(effective_supply);
        previous_supply=effective_supply;
        previous_load=load;
        for(auto& branch:load_states)branch.fill(load);
        startup_state=gate;
        initialized=true;
    }
    double startup_alpha=std::exp(-dt/model.startup_tau);
    startup_state=startup_alpha*startup_state+(1-startup_alpha)*gate;
    auto gains=model.ac_gains(normalized_supply,temp);
    double supply_response=0;
    for(size_t term=0;term<3;++term) {
        double tau=1/(2*std::acos(-1.)*model.ac_poles[term]);
        double response=highpass(effective_supply,lowpass[term],previous_supply,tau,dt);
        lowpass[term]=effective_supply-response;
        supply_response+=gains[term]*response;
    }
    previous_supply=effective_supply;
    if(supply_response<0)supply_response=model.negative_supply_limit*std::tanh(supply_response/model.negative_supply_limit);
    auto dynamic_drop=[&](double terminal_current) {
        double result=0;
        for(size_t branch=0;branch<2;++branch) {
            const auto& parameters=model.load_branches[branch];
            double slow=highpass(terminal_current,load_states[branch][0],previous_load,parameters.slow_tau,dt);
            double fast=highpass(terminal_current,load_states[branch][1],previous_load,parameters.fast_tau,dt);
            double band=(slow-fast)*parameters.slow_tau/(parameters.slow_tau-parameters.fast_tau);
            result+=model.branch_gain(branch,terminal_current)*smooth_direction(band,model.transition_current,branch==0);
        }
        return result;
    };
    double previous_voltage=voltage;
    auto terminal_current=[&](double output) {
        return load+conductance*output+external_cap*(output-previous_voltage)/dt;
    };
    auto command=[&](double current) {
        return startup_state*model.surface(normalized_supply,normalized_reference,current,temp,0)+gate*supply_response-dynamic_drop(current);
    };
    if(conductance==0 && external_cap==0) {
        voltage=command(load);
    } else {
        auto residual=[&](double output){return output-command(terminal_current(output));};
        double low=-1,high=2;
        for(int count=0;count<40&&residual(low)>0;++count)low*=2;
        for(int count=0;count<40&&residual(high)<0;++count)high*=2;
        if(residual(low)>0||residual(high)<0)SC_REPORT_FATAL("LDO_SOLVE","Cannot bracket output network solution");
        for(int count=0;count<55;++count) {
            double middle=(low+high)/2;
            if(residual(middle)>0)high=middle;else low=middle;
        }
        voltage=(low+high)/2;
    }
    double current=terminal_current(voltage);
    for(size_t branch=0;branch<2;++branch) {
        const auto& parameters=model.load_branches[branch];
        load_states[branch][0]=current-highpass(current,load_states[branch][0],previous_load,parameters.slow_tau,dt);
        load_states[branch][1]=current-highpass(current,load_states[branch][1],previous_load,parameters.fast_tau,dt);
    }
    previous_load=current;
    double capacitive_current=model.internal_capacitance*(voltage-previous_voltage)/dt;
    double pass=current+capacitive_current;
    VOUT.write(voltage+ground);
    pass_current.write(pass);
    vdd_current.write(pass+gate*model.surface(normalized_supply,normalized_reference,current,temp,1));
    avdd_current.write(gate*model.surface(normalized_supply,normalized_reference,current,temp,2));
    ibias_current.write(bias);
    op={supply,analog,reference,bias,load,conductance,model.internal_capacitance+external_cap,temp,voltage,current,pass-load-conductance*voltage,true};
}

void LDO::ac_processing() {
    using namespace sca_ac_analysis;
    using complex=sca_util::sca_complex;
    if(!op.valid||std::abs(op.capacitive_current)>1e-8)SC_REPORT_FATAL("LDO_AC","Settle the DC operating point before AC/noise analysis");
    complex delay=std::exp(complex(0,-sca_ac_w()*dt));
    complex ground=sca_ac(AVSS),input=sca_ac(VDD)-ground,reference=sca_ac(VREF)-ground;
    auto transfer=[&](double tau) {
        double coefficient=dt/(2*tau);
        return (1.-delay)/(1.+coefficient-(1.-coefficient)*delay);
    };
    complex supply_transfer=model.derivative(op.vdd,op.vref,op.current,op.temperature,0,0);
    auto gains=model.ac_gains(op.vdd,op.temperature);
    for(size_t term=0;term<3;++term)supply_transfer+=gains[term]*transfer(1/(2*std::acos(-1.)*model.ac_poles[term]));
    complex impedance=-model.derivative(op.vdd,op.vref,op.current,op.temperature,0,2);
    for(size_t branch=0;branch<2;++branch) {
        const auto& p=model.load_branches[branch];
        impedance+=.5*model.branch_gain(branch,op.current)*p.slow_tau/(p.slow_tau-p.fast_tau)*(transfer(p.slow_tau)-transfer(p.fast_tau));
    }
    double external_cap=op.capacitance-model.internal_capacitance;
    complex external_admittance=op.conductance+external_cap*(1.-delay)/dt;
    complex load=sca_ac(load_current)+op.voltage*sca_ac(load_conductance);
    complex drive=supply_transfer*input+model.derivative(op.vdd,op.vref,op.current,op.temperature,0,1)*reference
        +model.derivative(op.vdd,op.vref,op.current,op.temperature,0,3)*sca_ac(temperature);
    complex denominator=1.+external_admittance*impedance;
    complex output=(drive-impedance*load)/denominator;
    complex current=load+external_admittance*output;
    complex pass=current+model.internal_capacitance*(1.-delay)/dt*output;
    sca_ac(VOUT)=ground+output;
    sca_ac(pass_current)=pass;
    sca_ac(ibias_current)=sca_ac(IBIAS);
    auto rail=[&](int field) {
        return model.derivative(op.vdd,op.vref,op.current,op.temperature,field,0)*input
            +model.derivative(op.vdd,op.vref,op.current,op.temperature,field,1)*reference
            +model.derivative(op.vdd,op.vref,op.current,op.temperature,field,2)*current
            +model.derivative(op.vdd,op.vref,op.current,op.temperature,field,3)*sca_ac(temperature);
    };
    sca_ac(vdd_current)=pass+rail(1);
    sca_ac(avdd_current)=rail(2);
    if(sca_ac_noise_is_running())sca_ac_noise(VOUT)=model.noise_density(sca_ac_f())/denominator;
}
