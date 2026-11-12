// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <string>

// Sources: supplied MP2183.va (2026-10-23) and MP2183 Rev.1.0 (2019-05-09).
// TDF signal-flow buck assembly, NOT bidirectional electrical pin connectivity.
// External L/C/ESR/divider/CSS are solved here once; do not duplicate them outside.
namespace mp2183 {
using sc_core::sc_time;
struct Parameters {
    double VREF,FSW,RON_H,RON_L,IQ,ISHDN,IFB,ISS,UVLO_ON,UVLO_HYS;
    double EN_ON,EN_OFF,ILIM_PEAK,ILIM_VALLEY,IZCD,PG_ON,PG_OFF;
    double OVP_ON,OVP_OFF,OVP_REVERSE,VIN_OVP,VIN_OVP_HYS;
    double RDIS,REN,RPG,TSHUT,THYST,ROFF,CSW,VBD,RBD,KRAMP,KI;
    double TRIM_MAX,AAM_PEAK,RON_TC,L,DCR,COUT,ESR,CSS,R_TOP,R_BOT,R_PG;
    double INITIAL_VOUT,INITIAL_IL,INITIAL_SS;
    sc_time DT,TEDGE,TDEAD,EN_DELAY_ON,EN_DELAY_OFF,TMIN_ON,TMIN_OFF;
    sc_time PG_DELAY,OVP_DELAY,OVP_RETRY,TAVG,HICCUP,TSC_LIMIT;
    explicit Parameters(const nlohmann::json& j);
};
struct ControlInput { double vin,en,fb,il,out,ss,temperature; };
struct ControlState {
    bool uv=false,en=false,runen=false,hot=false,active=false;
    bool high=false,low=false,pending=false,ovp=false,vinovp=false;
    bool pg_target=false,pg_high=false,sleep=false;
    double iavg=0,trim=0;
};
class Controller {
public:
    explicit Controller(const Parameters& parameters) : p(parameters) {}
    void sample(sc_time now, const ControlInput& in);
    const ControlState& state() const { return s; }
private:
    const Parameters& p;
    ControlState s;
    sc_time ten=sc_core::SC_ZERO_TIME,tpg=sc_core::SC_ZERO_TIME;
    sc_time tov=sc_core::SC_ZERO_TIME,retry=sc_core::SC_ZERO_TIME;
    sc_time thoff=sc_core::SC_ZERO_TIME,tloff=sc_core::SC_ZERO_TIME;
    sc_time tend=sc_core::SC_ZERO_TIME,treverse=sc_core::SC_ZERO_TIME;
    sc_time tsc=sc_core::SC_ZERO_TIME;
    bool ov_timer=false,short_timer=false;
};
struct Inputs {
    double vin=5,en=2,iload=0,rload=1e12,temperature=25,pg_pullup=5;
};
struct Outputs {
    double vout=0,sw=0,il=0,ss=0,fb=0,pg=0,ipg=0,iin=0,ien=0;
    double power=0,external_loss=0,energy=0;
    bool high=false,low=false,active=false;
    int state=0;
};
// Shared engine for TDF and numerical tests. sample() is called on the DT grid.
// At t>0 integrates the preceding switch command, then samples the VA controller.
class Engine {
public:
    explicit Engine(const nlohmann::json& config);
    Outputs sample(sc_time now,const Inputs& in);
    const Parameters& parameters() const { return p; }
    const ControlState& control_state() const { return control.state(); }
private:
    Parameters p;
    Controller control;
    double sw=0,il=0,vc=0,ss=0,out=0;
    double gh=0,gl=0,grun=0,gpg=0;
    double target_h=0,target_l=0,target_run=0,target_pg=0;
    sc_time previous=sc_core::SC_ZERO_TIME;
    bool started=false;
    Outputs advance(double h,const Inputs& in);
};

class MP2183 : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"},en{"en"},iload{"iload"};
    sca_tdf::sca_in<double> rload{"rload"},temperature{"temperature"},pg_pullup{"pg_pullup"};
    sca_tdf::sca_out<double> vout{"vout"},sw{"sw"},il{"il"},ss{"ss"},fb{"fb"};
    sca_tdf::sca_out<double> pg{"pg"},ipg{"ipg"},iin{"iin"},ien{"ien"},power{"power"};
    sca_tdf::sca_out<double> external_loss{"external_loss"},energy{"energy"};
    sca_tdf::sca_out<bool> high{"high"},low{"low"},active{"active"};
    sca_tdf::sca_out<int> state{"state"};
    MP2183(sc_core::sc_module_name name,const std::string& config_file);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    std::string get_category() const;
    sc_time get_latency() const; // EN-on delay; total startup also depends on CSS/load
    sc_time get_timestep() const;
    double get_power_dissipation() const { return last_power; }
    const nlohmann::json& get_area() const { return cfg.at("area_mm2").at("value"); }
    const nlohmann::json& get_cost() const { return cfg.at("cost").at("value"); }
    const nlohmann::json& configuration() const { return cfg; }
private:
    nlohmann::json cfg;
    Engine engine;
    double last_power=0;
};
nlohmann::json load_config(const std::string& path);
} // namespace mp2183
