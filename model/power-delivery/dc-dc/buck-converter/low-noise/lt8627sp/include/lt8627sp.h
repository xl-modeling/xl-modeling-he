// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <string>

// Application-level model of one ADI LT8627SP. Sources: supplied LT8627SP.va,
// ADI LT8627SP Rev. B, and the supplied switching/noise verification report.
// External L/C and SET/PG application networks are represented internally.
// Not an ADI transistor model, EMI model, PolyPhase or external-SYNC model.
namespace lt8627sp {
struct Parameters {
    double FSW,LOUT,LDCR,RDS_TOP,RDS_BOT,ROFF,RSW_LEAK,CSW;
    double DIODE_VF,DIODE_RD,DIODE_SMOOTH,ISET,IQ_ON,IQ_OFF;
    double ILIM_TOP,ILIM_BOT,IOUT_MAX,VEN_RISE,VEN_FALL,VIN_MIN;
    double VINTVCC,VHEAD,KP,PG_RON,SET_RPD,VOUT_MAX;
    double NOISE_DUTY,VNOISE_WHITE,VNOISE_LF_1HZ,VNOISE_LF_EXP;
    double COUT,ESR,RSET,CSET,PGFB_RATIO,PG_PULLUP_V,PG_PULLUP_R;
    double INITIAL_VOUT;
    double ESR_RESIDUAL_FIT=0, SWITCH_CAP_FIT=0;
    bool ENABLE_CLKOUT,ENABLE_NOISE,NOISE_CHAR_MODE;
    sc_core::sc_time TREF,DEADTIME,TEDGE,DT,period;
    explicit Parameters(const nlohmann::json& config);
};
struct Inputs {
    double pvin=12,svin=12,en_uvlo=3.3,sync_mode=3.3;
    double set_override=-1,sense_offset=0,iload=0,rload=.5;
};
struct Output {
    double vout=0,sw=0,il=0,duty=0,gh=0,gl=0;
    double ipvin=0,isvin=0,power=0,external_loss=0,energy=0;
    double set_voltage=0,vref=0,intvcc=0,pg=0,clkout=0,ilim=0;
    bool active=false,limited=false;
    int state=0;
};
nlohmann::json load_config(const std::string& path);
class Engine {
public:
    explicit Engine(const nlohmann::json& config);
    Output sample(sc_core::sc_time now,const Inputs& inputs);
    const Parameters& parameters() const {return p;}
    // Averaged fixed-duty power-stage noise, not closed-loop PSS/Pnoise.
    // These queries reject NOISE_CHAR_MODE=0 and frequencies >=FSW/10.
    double noise_density(double frequency_hz,double load_ohm) const;
    double noise_rms(double low_hz,double high_hz,double load_ohm) const;
private:
    Parameters p;
    double vc=0,vo=0,sw=0,il=0,setv=0,ref=0,duty=0,gt=0,gb=0;
    bool enabled=false,pulse=false,started=false,high=false,low=false,limited=false;
    sc_core::sc_time previous=sc_core::SC_ZERO_TIME,next=sc_core::SC_ZERO_TIME;
    sc_core::sc_time cycle=sc_core::SC_ZERO_TIME,top_start,top_stop,bot_start;
    void update_enable(const Inputs& in);
    void tick(sc_core::sc_time now,const Inputs& in);
    void commands(sc_core::sc_time now,const Inputs& in);
    sc_core::sc_time next_event(sc_core::sc_time now,sc_core::sc_time end) const;
    Output advance(double h,const Inputs& in);
};
class LT8627SP : public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<double> pvin{"pvin"},svin{"svin"},en_uvlo{"en_uvlo"},sync_mode{"sync_mode"};
    sca_tdf::sca_in<double> set_override{"set_override"},sense_offset{"sense_offset"},iload{"iload"},rload{"rload"};
    sca_tdf::sca_out<double> vout{"vout"},sw{"sw"},il{"il"},duty{"duty"},gh{"gh"},gl{"gl"};
    sca_tdf::sca_out<double> ipvin{"ipvin"},isvin{"isvin"},power{"power"},external_loss{"external_loss"},energy{"energy"};
    sca_tdf::sca_out<double> set_voltage{"set_voltage"},vref{"vref"},intvcc{"intvcc"},pg{"pg"},clkout{"clkout"},ilim{"ilim"};
    sca_tdf::sca_out<bool> active{"active"},limited{"limited"};
    sca_tdf::sca_out<int> state{"state"};
    LT8627SP(sc_core::sc_module_name name,const std::string& json_path);
    void set_attributes() override;
    void processing() override;
    std::string get_module_name() const;
    double get_version() const;
    std::string get_category() const;
    sc_core::sc_time get_timestep() const {return engine.parameters().DT;}
    double get_power_dissipation() const {return last_power;}
    const nlohmann::json& get_area() const {return config.at("area_mm2").at("value");}
    const nlohmann::json& get_latency() const {return config.at("physical_latency_s").at("value");}
    const nlohmann::json& get_cost() const {return config.at("cost").at("value");}
    double get_noise_density(double frequency_hz,double load_ohm) const {return engine.noise_density(frequency_hz,load_ohm);}
    double get_noise_rms(double low_hz,double high_hz,double load_ohm) const {return engine.noise_rms(low_hz,high_hz,load_ohm);}
    const nlohmann::json& configuration() const {return config;}
private:
    nlohmann::json config;
    Engine engine;
    double last_power=0;
};
} // namespace lt8627sp
