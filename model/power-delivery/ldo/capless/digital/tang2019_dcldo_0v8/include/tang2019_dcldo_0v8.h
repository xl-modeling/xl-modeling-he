#pragma once
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <string>
#include <experimental/optional>
// Tang et al., IEEE TIE 2019, DOI 10.1109/TIE.2018.2840492.
// Functional system-level abstraction; see matching JSON and model description.
class Tang2019_DCLDO : public sca_tdf::sca_module {
public:
 sca_tdf::sca_in<double> vin{"vin"},iload{"iload"},rload{"rload"},ripple{"ripple"};
 sca_tdf::sca_out<double> vout{"vout"},vcore{"vcore"},iin{"iin"},iq{"iq"},ipass{"ipass"},power{"power"},psrr_voltage{"psrr_voltage"},eta_current{"eta_current"},eta_power{"eta_power"},unmet_load{"unmet_load"};
 sca_tdf::sca_out<int> code{"code"},action{"action"},state{"state"};
 Tang2019_DCLDO(sc_core::sc_module_name,const std::string&);
 void set_attributes() override; void processing() override;
 std::string get_module_name() const; double get_version() const;
 std::string get_category() const; double get_timestep_s() const;
 double get_latency() const; double get_active_area_mm2() const;
 double get_power_dissipation() const{return last_power_;}
 std::experimental::optional<double> get_area() const{return std::experimental::nullopt;}
 double get_power() const{return last_power_;}
 const nlohmann::json& get_parameters() const{return c_;}
private:
 nlohmann::json c_;double dt_,v_=0,lp_=0,xprev_=0,timer_=0,last_power_=0;int k_=0;bool first_=true;
 double n(const std::string&)const;std::string s(const std::string&)const;bool b(const std::string&)const;
 double target(double)const;double capacity(int)const;double pass(double,double,int)const;
 double solve(double,double,double,int,double,double)const;
};
