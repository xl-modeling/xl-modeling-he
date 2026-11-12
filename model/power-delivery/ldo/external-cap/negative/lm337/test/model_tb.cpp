#include "../include/lm337.h"
#include "common.hpp"

int sc_main(int argc,char**argv){
 if(argc!=4)return 2;
 try{
 sc_core::sc_set_time_resolution(1,sc_core::SC_PS);
 json scenario;{std::ifstream s(argv[2]);if(!s)throw std::runtime_error("missing scenario");s>>scenario;}
 double dt=scenario.at("dt_s");std::map<const sc_core::sc_object*,std::string>logical;
 LM337 dut("dut",std::string(argv[1]));logical[&dut.vin]="vin";logical[&dut.iload]="iload";logical[&dut.rload]="rload";logical[&dut.control]="control";logical[&dut.ripple]="ripple";logical[&dut.vout]="vout";logical[&dut.iin]="iin";logical[&dut.ignd]="ignd";logical[&dut.power]="power";
 std::vector<sc_core::sc_object*>objects;descendants(&dut,objects);std::vector<std::string>observed;for(auto*c:objects)if(dynamic_cast<sca_out<double>*>(c))observed.push_back(logical.count(c)?logical[c]:c->basename());

 std::vector<std::string>bn,in,un;for(auto*c:objects){auto name=logical.count(c)?logical[c]:c->basename();if(dynamic_cast<sca_out<bool>*>(c))bn.push_back(name);if(dynamic_cast<sca_out<int>*>(c))in.push_back(name);if(dynamic_cast<sca_out<unsigned>*>(c))un.push_back(name);}
 std::unique_ptr<DiscreteMonitor<bool>>bm;std::unique_ptr<DiscreteMonitor<int>>im;std::unique_ptr<DiscreteMonitor<unsigned>>um;
 if(!bn.empty())bm=std::make_unique<DiscreteMonitor<bool>>("bool_monitor",dt,scenario,std::string(argv[3])+".bool.csv",bn);
 if(!in.empty())im=std::make_unique<DiscreteMonitor<int>>("int_monitor",dt,scenario,std::string(argv[3])+".int.csv",in);
 if(!un.empty())um=std::make_unique<DiscreteMonitor<unsigned>>("uint_monitor",dt,scenario,std::string(argv[3])+".uint.csv",un);
 Monitor monitor("monitor",dt,scenario,argv[3],observed);std::vector<std::unique_ptr<sca_signal<double>>> signals;std::vector<std::unique_ptr<Source>>sources;
 std::vector<std::unique_ptr<sca_signal<bool>>>bools;std::vector<std::unique_ptr<sca_signal<int>>>ints;std::vector<std::unique_ptr<sca_signal<unsigned>>>uints;
 json bound=json::array();
 for(auto*c:objects){std::string name=logical.count(c)?logical[c]:c->basename();
  if(auto*p=dynamic_cast<sca_in<double>*>(c)){
   if(!scenario.at("inputs").contains(name))throw std::runtime_error("Missing explicit input: "+name);
   json spec=scenario["inputs"][name];std::function<double(double)>fn;
   fn=[spec](double t){return stimulus(spec,t);};
   auto sig=std::make_unique<sca_signal<double>>(sc_core::sc_gen_unique_name("input_signal"));(*p)(*sig);
   auto src=std::make_unique<Source>(sc_core::sc_gen_unique_name("source"),dt,fn);src->out(*sig);sources.push_back(std::move(src));signals.push_back(std::move(sig));bound.push_back(name);
  }else if(auto*p=dynamic_cast<sca_out<double>*>(c)){
   auto sig=std::make_unique<sca_signal<double>>(sc_core::sc_gen_unique_name("output_signal"));(*p)(*sig);monitor.add(*sig,name);signals.push_back(std::move(sig));
  }else if(auto*p=dynamic_cast<sca_out<bool>*>(c)){auto s=std::make_unique<sca_signal<bool>>(sc_core::sc_gen_unique_name("bool"));(*p)(*s);bm->add(*s);bools.push_back(std::move(s));
  }else if(auto*p=dynamic_cast<sca_out<int>*>(c)){auto s=std::make_unique<sca_signal<int>>(sc_core::sc_gen_unique_name("int"));(*p)(*s);im->add(*s);ints.push_back(std::move(s));
  }else if(auto*p=dynamic_cast<sca_out<unsigned>*>(c)){auto s=std::make_unique<sca_signal<unsigned>>(sc_core::sc_gen_unique_name("unsigned"));(*p)(*s);um->add(*s);uints.push_back(std::move(s));}
 }
 if(bound.empty()||monitor.names.empty())throw std::runtime_error("No DUT ports bound");
 sc_core::sc_start(scenario.at("stop_s").get<double>(),sc_core::SC_SEC);
 json out=monitor.result();if(bm)out["discrete_bool"]=bm->result();if(im)out["discrete_int"]=im->result();if(um)out["discrete_uint"]=um->result();out["inputs_bound"]=bound;out["evidence_scope"]="TDF model behavior at declared fixture";
 std::cout<<"EXPERT_RESULT "<<out.dump()<<std::endl;return monitor.bad?4:0;
 }catch(const std::exception&e){std::cerr<<"ERROR "<<e.what()<<std::endl;return 3;}
}
