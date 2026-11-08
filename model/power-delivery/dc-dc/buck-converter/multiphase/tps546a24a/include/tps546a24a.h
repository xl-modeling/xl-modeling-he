// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

// Sources: supplied tps546a24a_ctrl_sw_v1.va / power_sw_v1.va;
// TI SLUSE16A (Sep 2020), pp.7,10,28-29,34.
// Device-specific signal-flow model, NOT TI proprietary control or PMBus model.
// One TPS546A24A instance includes its configured external L/C test circuit.
// TPS546A24AStack represents 2..4 separate identical devices, not one multiphase IC.
namespace tps546a24a {
using sc_core::sc_time;
struct Parameters {
    unsigned devices;
    double VOUT_SET,FSW,VEN_ON,VEN_OFF,VIN_ON,VIN_OFF,SENSE_SCALE;
    double KP,KI,RI,IREF_MAX,IREF_MIN,DMAX,RHS,RLS,RSWPD,GOFF;
    double VDIODE,RDIODE,VSMOOTH,CSW,COUT,ESR,INITIAL_VOUT;
    std::vector<double> L,DCR;
    sc_time TSS,TDEAD,TEDGE,TON_NUM,DT,period;
    explicit Parameters(const nlohmann::json& j);
};
struct PhaseState {
    bool en=false,uv=false,running=false,high=false,low=false;
    double duty=0,iavg=0,vavg=0,sw=0,il=0;
    double gh=0,gl=0,integral_i=0,integral_v=0;
    sc_time cycle=sc_core::SC_ZERO_TIME,next=sc_core::SC_ZERO_TIME;
    sc_time high_end=sc_core::SC_ZERO_TIME;
};
struct Inputs { double vin=12,en=3.3,iload=0,rload=.24; };
struct Output {
    double vout=0,iin=0,power=0,external_loss=0,energy=0;
    double vref=0,iref=0,total_current=0;
    bool active=false;
    int state=0;
};
class Engine {
public:
    explicit Engine(const nlohmann::json& j);
    Output sample(sc_time now,const Inputs& in);
    const Parameters& parameters() const {return p;}
    const std::vector<PhaseState>& phases() const {return phase;}
private:
    Parameters p;
    std::vector<PhaseState> phase;
    double vc=0,out=0,ix=0,shared_iref=0,vref=0;
    bool master_running=false,started=false;
    sc_time start=sc_core::SC_ZERO_TIME,previous=sc_core::SC_ZERO_TIME;
    void tick(unsigned index,sc_time time,const Inputs& in);
    sc_time next_event(sc_time now,sc_time end) const;
    void commands(sc_time now);
    Output advance(double h,const Inputs& in);
};
nlohmann::json load_config(const std::string& path);
class TPS546A24A : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"},en{"en"},iload{"iload"},rload{"rload"};
    sca_tdf::sca_out<double> vout{"vout"},iin{"iin"},power{"power"},external_loss{"external_loss"};
    sca_tdf::sca_out<double> energy{"energy"},vref{"vref"},iref{"iref"},total_current{"total_current"};
    sca_tdf::sca_out<bool> active{"active"};
    sca_tdf::sca_out<int> state{"state"};
    sc_core::sc_vector<sca_tdf::sca_out<double>> sw,il,isns,duty,gh,gl;
    TPS546A24A(sc_core::sc_module_name name,const std::string& path);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    std::string get_category() const;
    unsigned get_device_count() const {return engine.parameters().devices;}
    sc_time get_timestep() const {return engine.parameters().DT;}
    sc_time get_soft_start_time() const {return engine.parameters().TSS;}
    const nlohmann::json& get_latency() const {return cfg.at("physical_latency_s").at("value");}
    const nlohmann::json& get_area() const {return cfg.at("area_mm2").at("value");}
    const nlohmann::json& get_cost() const {return cfg.at("cost").at("value");}
    double get_power_dissipation() const {return last_power;}
    const PhaseState& get_phase_state(unsigned index) const {return engine.phases().at(index);}
    const nlohmann::json& configuration() const {return cfg;}
protected:
    TPS546A24A(sc_core::sc_module_name name,const nlohmann::json& config,bool stacked);
private:
    nlohmann::json cfg;
    Engine engine;
    double last_power=0;
};
class TPS546A24AStack : public TPS546A24A {
public:
    TPS546A24AStack(sc_core::sc_module_name name,const std::string& path);
};
} // namespace tps546a24a
