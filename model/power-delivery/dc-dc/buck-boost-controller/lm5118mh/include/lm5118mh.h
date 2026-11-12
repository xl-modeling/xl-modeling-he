// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <string>
// Sources: supplied LM5118MH_rev6.va and TI SNVS566J Rev. J.
// Controller only. External RC/power stage must solve the returned pin currents.
// HO is relative to HS, LO to PGND; outputs are Thevenin targets with R_DRIVER.
// Positive pin current flows INTO the chip. No SYNC/thermal-loop/efficiency model.
namespace lm5118mh {
struct Parameters {
    double VREF;
    double VCC_REG;
    double VCC_ILIM;
    double UVLO_RISE;
    double UVLO_FALL;
    double EN_RISE;
    double EN_FALL;
    double VCC_RISE;
    double VCC_FALL;
    double VCCX_RISE;
    double VCCX_FALL;
    double ISS;
    double IQ_RUN;
    double IQ_OFF;
    double TON_MIN;
    double TOFF_MIN;
    double T_RISE_HO;
    double T_FALL_HO;
    double T_RISE_LO;
    double T_FALL_LO;
    double VCS_BUCK;
    double VCS_BB;
    double GM_ERR;
    double ICOMP_MAX;
    double R_DRIVER;
    double R_VCC;
    double R_SS_DIS;
    double R_UVLO_HIC;
    double TJ_C;
    double FSW_HZ;
    double USE_RT_PIN;
    double COMP_LOW;
    double COMP_HIGH;
    double R_COMP_CLAMP;
    double STEPS_PER_CYCLE;
    double CS_PWM_GAIN;
    double PWM_OFFSET;
    double R_RAMP_RESET;
    double T_RAMP_SWITCH;
    double DT;
    explicit Parameters(const nlohmann::json& config);
};
struct Inputs {
    double vin=24;
    double uvlo=2;
    double rt_current=0;
    double en=5;
    double ramp=0;
    double ss=1.23;
    double fb=1.23;
    double comp=1;
    double vout=12;
    double sync=0;
    double cs=0;
    double csg=0;
    double vcc=7;
    double vccx=0;
    double hb_hs=7;
};
struct Output {
    double ho=0;
    double lo=0;
    double i_vin=0;
    double i_vcc=0;
    double i_uvlo=0;
    double i_ss=0;
    double i_comp=0;
    double i_ramp=0;
    double i_fb=0;
    double i_vout=0;
    double i_cs=0;
    double i_csg=0;
    double i_hb=0;
    double i_en=0;
    double i_sync=0;
    double rt_voltage=0;
    double frequency=0;
    double phase=0;
    double blend=0;
    double cs_threshold=0;
    double valley_cs=0;
    double power_bias=0;
    bool active=false;
    bool limited=false;
    bool hiccup=false;
    bool pwm_ho=false;
    bool pwm_lo=false;
    int consecutive_limit=0;
};
nlohmann::json load_config(const std::string& path);
class Engine {
public:
    explicit Engine(const nlohmann::json& config):p(config){}
    Output sample(double time_seconds,const Inputs& input);
    const Parameters& parameters() const { return p; }
private:
    Parameters p;
    bool initialized=false,en_ok=false,uvlo_ok=false,vcc_ok=false,vccx_ok=false,vin_ok=false;
    bool done=true,limited=false,hiccup=false;
    int count=0;
    double last_time=0,phase=0,valley=0,prev_uvlo=0,prev_compare=0;
    double gh=0,gl=0,rw=0;
};
class LM5118MH : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> vin{"vin"};
    sca_tdf::sca_in<double> uvlo{"uvlo"};
    sca_tdf::sca_in<double> rt_current{"rt_current"};
    sca_tdf::sca_in<double> en{"en"};
    sca_tdf::sca_in<double> ramp{"ramp"};
    sca_tdf::sca_in<double> ss{"ss"};
    sca_tdf::sca_in<double> fb{"fb"};
    sca_tdf::sca_in<double> comp{"comp"};
    sca_tdf::sca_in<double> vout{"vout"};
    sca_tdf::sca_in<double> sync{"sync"};
    sca_tdf::sca_in<double> cs{"cs"};
    sca_tdf::sca_in<double> csg{"csg"};
    sca_tdf::sca_in<double> vcc{"vcc"};
    sca_tdf::sca_in<double> vccx{"vccx"};
    sca_tdf::sca_in<double> hb_hs{"hb_hs"};
    sca_tdf::sca_out<double> ho{"ho"};
    sca_tdf::sca_out<double> lo{"lo"};
    sca_tdf::sca_out<double> i_vin{"i_vin"};
    sca_tdf::sca_out<double> i_vcc{"i_vcc"};
    sca_tdf::sca_out<double> i_uvlo{"i_uvlo"};
    sca_tdf::sca_out<double> i_ss{"i_ss"};
    sca_tdf::sca_out<double> i_comp{"i_comp"};
    sca_tdf::sca_out<double> i_ramp{"i_ramp"};
    sca_tdf::sca_out<double> i_fb{"i_fb"};
    sca_tdf::sca_out<double> i_vout{"i_vout"};
    sca_tdf::sca_out<double> i_cs{"i_cs"};
    sca_tdf::sca_out<double> i_csg{"i_csg"};
    sca_tdf::sca_out<double> i_hb{"i_hb"};
    sca_tdf::sca_out<double> i_en{"i_en"};
    sca_tdf::sca_out<double> i_sync{"i_sync"};
    sca_tdf::sca_out<double> rt_voltage{"rt_voltage"};
    sca_tdf::sca_out<double> frequency{"frequency"};
    sca_tdf::sca_out<double> phase{"phase"};
    sca_tdf::sca_out<double> blend{"blend"};
    sca_tdf::sca_out<double> cs_threshold{"cs_threshold"};
    sca_tdf::sca_out<double> valley_cs{"valley_cs"};
    sca_tdf::sca_out<double> power_bias{"power_bias"};
    sca_tdf::sca_out<bool> active{"active"};
    sca_tdf::sca_out<bool> limited{"limited"};
    sca_tdf::sca_out<bool> hiccup{"hiccup"};
    sca_tdf::sca_out<bool> pwm_ho{"pwm_ho"};
    sca_tdf::sca_out<bool> pwm_lo{"pwm_lo"};
    LM5118MH(sc_core::sc_module_name name,const std::string& json_path);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    std::string get_category() const;
    double get_power_bias() const {return last_power;}
    const nlohmann::json& get_area() const {return config.at("area_mm2").at("value");}
    const nlohmann::json& get_latency() const {return config.at("physical_latency_s").at("value");}
    const nlohmann::json& get_cost() const {return config.at("cost").at("value");}
    const nlohmann::json& configuration() const {return config;}
private:
    nlohmann::json config;
    Engine engine;
    double last_power=0;
};
} // namespace lm5118mh
