// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <string>
// Sources: TPS63021_switching_v4.va; TI SLVS916I Rev. I (2019-10).
// Application-level model of one fixed-output TPS63021, with internal L/C/PG network.
// Signal ports are not electrical pins. Do not add the same external L/C twice.
namespace tps63021 {
struct Parameters {
    double VOUT_NOM;
    double FSW_NOM_HZ;
    double FSW_MIN_HZ;
    double FSW_MAX_HZ;
    double VIN_OPER_MIN;
    double VIN_OPER_MAX;
    double UVLO_FALL;
    double UVLO_HYST;
    double EN_VIL;
    double EN_VIH;
    double ILIMIT_NOM_A;
    double ILIMIT_START_A;
    double ILIMIT_LOWVIN_A;
    double RSW_ON_OHM;
    double RSW_OFF_OHM;
    double IQ_VINA_A;
    double IQ_VOUT_A;
    double ISHUTDOWN_A;
    double FB_RIN_OHM;
    double OVP_V;
    double TSD_C;
    double TSD_HYST_C;
    double SOFTSTART_S;
    double LOOP_BW_HZ;
    double KP_DUTY_PER_V;
    double KI_DUTY_PER_V_S;
    double INT_DUTY_MAX;
    double INT_RESET_S;
    double DAMP_FILTER_HZ;
    double ACTIVE_DAMP_GAIN;
    double MODE_WINDOW_V;
    double DUTY_MIN;
    double DUTY_MAX;
    double DEADTIME_S;
    double EDGE_S;
    double STEPS_PER_CYCLE;
    double PSM_LOW_RATIO;
    double PSM_HIGH_RATIO;
    double SYNC_TIMEOUT_S;
    double P_SWITCH_PWM_W;
    double P_SWITCH_PSM_W;
    double PG_RLOW_OHM;
    double PG_ROFF_OHM;
    double PG_GOOD_RATIO;
    double PG_BAD_RATIO;
    double BODY_VF_V;
    double BODY_RD_OHM;
    double SMOOTH_V;
    double DT;
    double LOUT;
    double LDCR;
    double COUT;
    double ESR;
    double PG_PULLUP_V;
    double PG_PULLUP_R;
    double INITIAL_VOUT;
    explicit Parameters(const nlohmann::json& config);
};
struct Inputs {
    double vin=3.6,vina=3.6,en=3.6,ps_sync=3.6,temperature=25;
    double fb_override=-1,iload=0,rload=6.6;
};
struct Output {
    double vout=0;
    double l1=0;
    double l2=0;
    double il=0;
    double i_vin=0;
    double i_vina=0;
    double i_en=0;
    double i_ps=0;
    double i_pg=0;
    double power=0;
    double external_loss=0;
    double energy=0;
    double pg=0;
    double ilim=0;
    double duty_buck=0;
    double duty_boost=0;
    double frequency=0;
    double softstart=0;
    double ga_h=0;
    double ga_l=0;
    double gb_h=0;
    double gb_l=0;
    bool active=false;
    bool psm=false;
    bool burst=false;
    bool sync_active=false;
    bool pg_good=false;
    bool thermal_ok=false;
    int mode=0; // 0 off; 1 buck; 2 boost; 3 alternating transition region
};
nlohmann::json load_config(const std::string& path);
class Engine {
public:
    explicit Engine(const nlohmann::json& config);
    Output sample(double time_seconds,const Inputs& in);
    const Parameters& parameters() const {return p;}
private:
    Parameters p;
    double current_limit_cycle=-1, current_limit_phase=1;
    bool initialized=false,en_ok=false,uvlo=false,thermal=true,burst=true,pgood=false;
    double previous=0,last_edge=-1,prev_ps=0,fsync=0;
    double vc=0,vo=0,il=0,a=0,b=0,ss=0,err=0,integ=0,damp=0,pg_g=0;
    double gates[4]={0,0,0,0};
    Output advance(double h,const Inputs& in,const double* g,double limit,bool run,bool power_save);
};
class TPS63021 : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"};
    sca_tdf::sca_in<double> vina{"vina"};
    sca_tdf::sca_in<double> en{"en"};
    sca_tdf::sca_in<double> ps_sync{"ps_sync"};
    sca_tdf::sca_in<double> temperature{"temperature"};
    sca_tdf::sca_in<double> fb_override{"fb_override"};
    sca_tdf::sca_in<double> iload{"iload"};
    sca_tdf::sca_in<double> rload{"rload"};
    sca_tdf::sca_out<double> vout{"vout"};
    sca_tdf::sca_out<double> l1{"l1"};
    sca_tdf::sca_out<double> l2{"l2"};
    sca_tdf::sca_out<double> il{"il"};
    sca_tdf::sca_out<double> i_vin{"i_vin"};
    sca_tdf::sca_out<double> i_vina{"i_vina"};
    sca_tdf::sca_out<double> i_en{"i_en"};
    sca_tdf::sca_out<double> i_ps{"i_ps"};
    sca_tdf::sca_out<double> i_pg{"i_pg"};
    sca_tdf::sca_out<double> power{"power"};
    sca_tdf::sca_out<double> external_loss{"external_loss"};
    sca_tdf::sca_out<double> energy{"energy"};
    sca_tdf::sca_out<double> pg{"pg"};
    sca_tdf::sca_out<double> ilim{"ilim"};
    sca_tdf::sca_out<double> duty_buck{"duty_buck"};
    sca_tdf::sca_out<double> duty_boost{"duty_boost"};
    sca_tdf::sca_out<double> frequency{"frequency"};
    sca_tdf::sca_out<double> softstart{"softstart"};
    sca_tdf::sca_out<double> ga_h{"ga_h"};
    sca_tdf::sca_out<double> ga_l{"ga_l"};
    sca_tdf::sca_out<double> gb_h{"gb_h"};
    sca_tdf::sca_out<double> gb_l{"gb_l"};
    sca_tdf::sca_out<bool> active{"active"};
    sca_tdf::sca_out<bool> psm{"psm"};
    sca_tdf::sca_out<bool> burst{"burst"};
    sca_tdf::sca_out<bool> sync_active{"sync_active"};
    sca_tdf::sca_out<bool> pg_good{"pg_good"};
    sca_tdf::sca_out<bool> thermal_ok{"thermal_ok"};
    sca_tdf::sca_out<int> mode{"mode"};
    TPS63021(sc_core::sc_module_name name,const std::string& config_path);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    std::string get_category() const;
    double get_power_dissipation() const {return last_power;}
    double get_timestep_s() const {return engine.parameters().DT;}
    const nlohmann::json& get_area() const {return config.at("area_mm2").at("value");}
    const nlohmann::json& get_latency() const {return config.at("physical_latency_s").at("value");}
    const nlohmann::json& get_cost() const {return config.at("cost").at("value");}
    const nlohmann::json& configuration() const {return config;}
private:
    nlohmann::json config;
    Engine engine;
    double last_power=0;
};
} // namespace tps63021
