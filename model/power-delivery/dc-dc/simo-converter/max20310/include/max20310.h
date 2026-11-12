// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
// MAX20310 power-core SystemC-AMS model v1.0. Sources: supplied MAX20310.va
// (2026-10-18) and ADI Rev 3 (12/21). Not a full PMIC or transistor model.
// GND is battery positive. vbat = V(GND,BATN), BATN = -vbat.
// Internal application L/DCR and four output capacitors; do not duplicate.
#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <array>
#include <string>
#include <cstdint>
namespace max20310 {
using json=nlohmann::json;
struct Parameters {
  double VBB1=1.8;
  double VBB2=1.2;
  double VLDO1=1.5;
  double VLDO2=1;
  double BB1_EN=1;
  double BB2_EN=1;
  double LDO1_EN=1;
  double LDO2_EN=1;
  double LDO1_SWITCH=0;
  double LDO2_SWITCH=0;
  double IPEAK=0.59999999999999998;
  double PIN_LIMIT=0.25;
  double IIN_LIMIT=0.20000000000000001;
  double UV_START=0.80000000000000004;
  double UV_STOP=0.69999999999999996;
  double ILDO_MAX=0.050000000000000003;
  double RDROP=2;
  double RLOADSW=1;
  double DISCHARGE=1;
  double RDIS=10000;
  double RCHG=0.14999999999999999;
  double ROUTSW=0.17999999999999999;
  double ROFF=1000000000000;
  double RDUMP=20;
  double CLX=1.9999999999999999e-11;
  double VDIODE=0.34999999999999998;
  double RDIODE=0.5;
  double HYST=0.0080000000000000002;
  double TCTRL=1e-08;
  double TEDGE=2.0000000000000001e-09;
  double TDEAD=2e-08;
  double TIDLE=4.0000000000000001e-08;
  double TCHG_MAX=1.0000000000000001e-05;
  double TXFER_MAX=3.0000000000000001e-05;
  double IZCD=0.001;
  double TLIMIT=1.0000000000000001e-05;
  double START_DELAY=0;
  double TSOFT=0.00020000000000000001;
  double GM_LDO=10;
  double TAU_LDO=1.9999999999999999e-07;
  double IQ_ON=2.0000000000000002e-05;
  double IQ_OFF=1e-08;
  double QSW=0;
  double DT=2.0000000000000001e-09;
  double LOUT=2.2000000000000001e-06;
  double DCR=0.02;
  double CBB1=1.0000000000000001e-05;
  double CBB2=1.0000000000000001e-05;
  double CLDO1=2.2000000000000001e-06;
  double CLDO2=2.2000000000000001e-06;
  void validate() const;
};
Parameters load_config(const std::string& path, json& config);
struct Inputs {
  double vbat=1.2;
  double en_model=1;
  double temperature=27;
  double rbb1=180;
  double rbb2=120;
  double rldo1=150;
  double rldo2=100;
  double ibb1_load=0;
  double ibb2_load=0;
  double ildo1_load=0;
  double ildo2_load=0;
};
struct Output {
  double bb1=0;
  double bb2=0;
  double ldo1=0;
  double ldo2=0;
  double lx=0;
  double il=0;
  double iin=0;
  double power=0;
  double external_loss=0;
  double energy=0;
  double pin_filtered=0;
  double iin_filtered=0;
  double softstart=0;
  double peak_limit=0;
  double gate_charge=0;
  double gate_bb1=0;
  double gate_bb2=0;
  double gate_dump=0;
  double pass1=0;
  double pass2=0;
  bool running=false;
  bool thermal_trip=false;
  int state=0;
  int selected=0;
};
class Engine {
 public:
  explicit Engine(const Parameters& p);
  Output step(double time_s,const Inputs& in);
  const Parameters& parameters() const { return p_; }
 private:
  Parameters p_; Output o_; std::array<double,5> v_{};
  double il_=0,lc1_=0,lc2_=0,tboot_=0,tstate_=0,tready_=0,tgate_=-1;
  double pmon_=0,imon_=0,ss_=0,ipk_=.08;
  std::array<double,4> gates_{{0,0,0,1}}, start_{{0,0,0,1}}, target_{{0,0,0,1}}, change_{};
  std::uint64_t tick_=0, control_ticks_=1;
  int state_=0,selected_=1; bool running_=false,thermal_=false,en1_=false,en2_=false,el1_=false,el2_=false;
  void control(double t,const Inputs& in);
  void circuit(double t,const Inputs& in);
};
class MAX20310 : public sca_tdf::sca_module {
 public:
  sca_tdf::sca_in<double> vbat{"vbat"};
  sca_tdf::sca_in<double> en_model{"en_model"};
  sca_tdf::sca_in<double> temperature{"temperature"};
  sca_tdf::sca_in<double> rbb1{"rbb1"};
  sca_tdf::sca_in<double> rbb2{"rbb2"};
  sca_tdf::sca_in<double> rldo1{"rldo1"};
  sca_tdf::sca_in<double> rldo2{"rldo2"};
  sca_tdf::sca_in<double> ibb1_load{"ibb1_load"};
  sca_tdf::sca_in<double> ibb2_load{"ibb2_load"};
  sca_tdf::sca_in<double> ildo1_load{"ildo1_load"};
  sca_tdf::sca_in<double> ildo2_load{"ildo2_load"};
  sca_tdf::sca_out<double> bb1{"bb1"};
  sca_tdf::sca_out<double> bb2{"bb2"};
  sca_tdf::sca_out<double> ldo1{"ldo1"};
  sca_tdf::sca_out<double> ldo2{"ldo2"};
  sca_tdf::sca_out<double> lx{"lx"};
  sca_tdf::sca_out<double> il{"il"};
  sca_tdf::sca_out<double> iin{"iin"};
  sca_tdf::sca_out<double> power{"power"};
  sca_tdf::sca_out<double> external_loss{"external_loss"};
  sca_tdf::sca_out<double> energy{"energy"};
  sca_tdf::sca_out<double> pin_filtered{"pin_filtered"};
  sca_tdf::sca_out<double> iin_filtered{"iin_filtered"};
  sca_tdf::sca_out<double> softstart{"softstart"};
  sca_tdf::sca_out<double> peak_limit{"peak_limit"};
  sca_tdf::sca_out<double> gate_charge{"gate_charge"};
  sca_tdf::sca_out<double> gate_bb1{"gate_bb1"};
  sca_tdf::sca_out<double> gate_bb2{"gate_bb2"};
  sca_tdf::sca_out<double> gate_dump{"gate_dump"};
  sca_tdf::sca_out<double> pass1{"pass1"};
  sca_tdf::sca_out<double> pass2{"pass2"};
  sca_tdf::sca_out<bool> running{"running"};
  sca_tdf::sca_out<bool> thermal_trip{"thermal_trip"};
  sca_tdf::sca_out<int> state{"state"};
  sca_tdf::sca_out<int> selected{"selected"};
  MAX20310(sc_core::sc_module_name name,const std::string& config_path);
  std::string get_module_name() const;
  double get_version() const;
  std::string get_category() const;
  json get_area() const; json get_latency() const; json get_cost() const;
  double get_power_dissipation() const {return last_.power;}
  double get_timestep_s() const {return p_.DT;}
  const json& configuration() const {return config_;}
  void set_attributes() override;
  void processing() override;
 private: json config_; Parameters p_; Engine engine_; Output last_;
};
} // namespace max20310
