#pragma once
#include <systemc-ams>
#include "LDO_table.h"

// IBIAS is a physical current in A. Voltage ports are relative to AVSS.
class LDO : public sca_tdf::sca_module {
public:
    struct configs { nlohmann::json document; double timestep_s; };
    static configs get_configs_from_json(const char* path);
    sca_tdf::sca_in<double> IBIAS,AVSS,VREF,VDD,AVDD;
    sca_tdf::sca_in<double> load_current,load_conductance,external_capacitance,temperature;
    sca_tdf::sca_out<double> VOUT,vdd_current,avdd_current,ibias_current,pass_current;
    LDO(sc_core::sc_module_name name,const configs& cfg);
    void set_attributes() override;
    void processing() override;
    void ac_processing() override;
    const char* get_version() const {return "1.1";}
    struct dc_operating_point {
        double vdd=0,avdd=0,vref=0,bias=0,load=0,conductance=0,capacitance=0,temperature=0,voltage=0,current=0,capacitive_current=0;
        bool valid=false;
    };
    const dc_operating_point& get_dc_operating_point() const {return op;}
    const char* get_ac_region() const {return "regulation";}
    bool get_ac_branch_boundary() const {return false;}
private:
    LDO_table model;
    double dt,voltage=0,previous_supply=0,previous_load=0,startup_state=0;
    std::array<double,3> lowpass{};
    std::array<std::array<double,2>,2> load_states{};
    bool initialized=false;
    dc_operating_point op;
};
