#pragma once
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

class LDO_table {
public:
    std::vector<double> supplies, references, currents, temperatures;
    std::vector<double> current_coordinates, ac_supplies, ac_temperatures;
    std::vector<double> ac_poles;
    struct LoadBranch {
        double slow_tau, fast_tau, gain;
        std::vector<double> current_axis, scale;
    };
    std::vector<LoadBranch> load_branches;
    std::vector<std::vector<std::array<double,3>>> ac_coefficients;
    std::vector<double> data;
    double current_offset, internal_capacitance, transition_current;
    double startup_tau, startup_supply, noise_white, noise_flicker, noise_exponent;
    double negative_supply_limit;

    explicit LDO_table(const nlohmann::json& parameters) {
        supplies=parameters.at("supply_grid").get<std::vector<double>>();
        references=parameters.at("reference_grid").get<std::vector<double>>();
        currents=parameters.at("load_grid").get<std::vector<double>>();
        temperatures=parameters.at("temperature_grid").get<std::vector<double>>();
        ac_supplies=parameters.at("ac_supplies").get<std::vector<double>>();
        ac_temperatures=parameters.at("ac_temperatures").get<std::vector<double>>();
        ac_poles=parameters.at("ac_poles_Hz").get<std::vector<double>>();
        ac_coefficients=parameters.at("ac_gains").get<std::vector<std::vector<std::array<double,3>>>>();
        const auto& c=parameters.at("constants");
        current_offset=c.at("load_coordinate_offset_A");
        internal_capacitance=c.at("c_int");
        transition_current=c.at("transition_current_A");
        startup_tau=c.at("startup_tau_s");
        startup_supply=c.at("startup_supply_V");
        negative_supply_limit=c.at("negative_supply_limit_V");
        for(const auto& branch:c.at("load_branches")) {
            load_branches.push_back({branch.at("tau_slow_s"),branch.at("tau_fast_s"),branch.at("gain_ohm"),
                branch.at("current_axis_A").get<std::vector<double>>(),branch.at("current_scale").get<std::vector<double>>()});
        }
        noise_white=c.at("noise_white_V2_Hz");
        noise_flicker=c.at("noise_flicker_power");
        noise_exponent=c.at("noise_flicker_exponent");
        for(const auto* axis:{&supplies,&references,&currents,&temperatures,&ac_supplies,&ac_temperatures}) {
            if(axis->size()<2 || std::adjacent_find(axis->begin(),axis->end(),std::greater_equal<double>())!=axis->end())
                throw std::invalid_argument("LDO grid must be strictly increasing");
        }
        if(current_offset<=0 || internal_capacitance<=0 || transition_current<=0 || startup_tau<=0)
            throw std::invalid_argument("LDO constants must be positive");
        for(double current:currents)current_coordinates.push_back(std::log10(current+current_offset));
        const size_t expected=temperatures.size()*supplies.size()*references.size()*currents.size()*4;
        std::ifstream table(parameters.at("resolved_table_path").get<std::string>());
        if(!table)throw std::invalid_argument("LDO shared data file is missing");
        data.reserve(expected);
        double value;
        while(table>>value) {
            if(!std::isfinite(value))throw std::invalid_argument("LDO table contains a nonfinite value");
            data.push_back(value);
        }
        if(!table.eof() || data.size()!=expected)throw std::invalid_argument("LDO table has an invalid size or number");
        if(ac_poles.size()!=3 || ac_coefficients.size()!=ac_temperatures.size() || load_branches.size()!=2)
            throw std::invalid_argument("LDO dynamic parameter dimensions are invalid");
        for(const auto& row:ac_coefficients)if(row.size()!=ac_supplies.size())throw std::invalid_argument("LDO AC grid dimensions are invalid");
    }

    static size_t segment(const std::vector<double>& grid,double value) {
        auto position=std::upper_bound(grid.begin(),grid.end(),value);
        if(position==grid.begin())return 0;
        return std::min(static_cast<size_t>(position-grid.begin()-1),grid.size()-2);
    }

    double table_value(size_t thermal,size_t supply,size_t reference,size_t current,int field) const {
        size_t index=(((thermal*supplies.size()+supply)*references.size()+reference)*currents.size()+current)*4+field;
        return data[index];
    }

    double supply_node(size_t supply,double reference,double current,double temperature,int field) const {
        size_t ref_index=segment(references,reference),temp_index=segment(temperatures,temperature);
        double coordinate=std::log10(std::max(current,0.0)+current_offset);
        size_t load_index=segment(current_coordinates,coordinate);
        double ref_weight=(reference-references[ref_index])/(references[ref_index+1]-references[ref_index]);
        double load_weight=(coordinate-current_coordinates[load_index])/(current_coordinates[load_index+1]-current_coordinates[load_index]);
        double temp_weight=(temperature-temperatures[temp_index])/(temperatures[temp_index+1]-temperatures[temp_index]);
        double result=0;
        for(size_t thermal=0;thermal<2;++thermal) {
            for(size_t ref=0;ref<2;++ref) {
                double low=table_value(temp_index+thermal,supply,ref_index+ref,load_index,field);
                double high=table_value(temp_index+thermal,supply,ref_index+ref,load_index+1,field);
                double weight=(thermal?temp_weight:1-temp_weight)*(ref?ref_weight:1-ref_weight);
                result+=weight*((1-load_weight)*low+load_weight*high);
            }
        }
        return result;
    }

    double supply_slope(size_t index,double reference,double current,double temperature,int field) const {
        size_t low=index?index-1:index,high=std::min(index+1,supplies.size()-1);
        return (supply_node(high,reference,current,temperature,field)-supply_node(low,reference,current,temperature,field))/(supplies[high]-supplies[low]);
    }

    double surface(double supply,double reference,double current,double temperature,int field) const {
        size_t index=segment(supplies,supply);
        double width=supplies[index+1]-supplies[index],fraction=(supply-supplies[index])/width;
        double square=fraction*fraction,cube=square*fraction;
        return (2*cube-3*square+1)*supply_node(index,reference,current,temperature,field)
            +(cube-2*square+fraction)*width*supply_slope(index,reference,current,temperature,field)
            +(-2*cube+3*square)*supply_node(index+1,reference,current,temperature,field)
            +(cube-square)*width*supply_slope(index+1,reference,current,temperature,field);
    }

    double derivative(double supply,double reference,double current,double temperature,int field,int axis) const {
        if(axis==0) {
            size_t index=segment(supplies,supply);
            double width=supplies[index+1]-supplies[index],fraction=(supply-supplies[index])/width;
            return ((6*fraction*fraction-6*fraction)*supply_node(index,reference,current,temperature,field)
                +(-6*fraction*fraction+6*fraction)*supply_node(index+1,reference,current,temperature,field))/width
                +(3*fraction*fraction-4*fraction+1)*supply_slope(index,reference,current,temperature,field)
                +(3*fraction*fraction-2*fraction)*supply_slope(index+1,reference,current,temperature,field);
        }
        double step=axis==1?1e-7:(axis==2?1e-9:1e-4);
        if((axis==1 && reference>=references.back()) || (axis==2 && current>=currents.back()) || (axis==3 && temperature>=temperatures.back()))step=-step;
        double value=surface(supply,reference,current,temperature,field);
        return (surface(supply,reference+(axis==1?step:0),current+(axis==2?step:0),temperature+(axis==3?step:0),field)-value)/step;
    }

    std::array<double,3> ac_gains(double supply,double temperature) const {
        size_t supply_index=segment(ac_supplies,supply),temp_index=segment(ac_temperatures,temperature);
        double supply_weight=(supply-ac_supplies[supply_index])/(ac_supplies[supply_index+1]-ac_supplies[supply_index]);
        double temp_weight=(temperature-ac_temperatures[temp_index])/(ac_temperatures[temp_index+1]-ac_temperatures[temp_index]);
        std::array<double,3> result{};
        for(size_t thermal=0;thermal<2;++thermal) {
            double weight=thermal?temp_weight:1-temp_weight;
            for(size_t term=0;term<3;++term)result[term]+=weight*((1-supply_weight)*ac_coefficients[temp_index+thermal][supply_index][term]+supply_weight*ac_coefficients[temp_index+thermal][supply_index+1][term]);
        }
        return result;
    }

    double noise_density(double frequency) const {
        return std::sqrt(noise_white+noise_flicker/std::pow(frequency,noise_exponent));
    }

    double branch_gain(size_t branch_index,double current) const {
        const auto& branch=load_branches[branch_index];
        current=std::clamp(current,branch.current_axis.front(),branch.current_axis.back());
        size_t index=segment(branch.current_axis,current);
        double fraction=(current-branch.current_axis[index])/(branch.current_axis[index+1]-branch.current_axis[index]);
        return branch.gain*((1-fraction)*branch.scale[index]+fraction*branch.scale[index+1]);
    }
};
