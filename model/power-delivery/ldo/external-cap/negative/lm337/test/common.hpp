
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <vector>
#include <cmath>
#include <functional>
#include <algorithm>
using json=nlohmann::json;using sca_tdf::sca_in;using sca_tdf::sca_out;using sca_tdf::sca_signal;
double stimulus(const json& spec,double t){
 if(spec.is_number())return spec.get<double>();
 if(spec.contains("follow"))throw std::runtime_error("follow must be bound by adapter");
 const auto& p=spec.at("pwl");if(p.empty())throw std::runtime_error("empty PWL");
 if(t<=p[0][0].get<double>())return p[0][1].get<double>();
 for(size_t k=1;k<p.size();++k){double end=p[k][0],start=p[k-1][0];if(t<=end){if(end<=start)throw std::runtime_error("PWL times not strictly increasing");double a=p[k-1][1],b=p[k][1];return a+(b-a)*(t-start)/(end-start);}}
 return p.back()[1].get<double>();
}
struct Source:sca_tdf::sca_module{
 sca_out<double> out;double dt;std::function<double(double)> value;
 Source(sc_core::sc_module_name n,double h,std::function<double(double)> f):sca_tdf::sca_module(n),out("out"),dt(h),value(f){}
 void set_attributes()override{set_timestep(dt,sc_core::SC_SEC);}
 void processing()override{out.write(value(get_time().to_seconds()));}
};
struct Stat{double sum=0,sq=0,lo=1e300,hi=-1e300,last=0;long count=0;void add(double v){sum+=v;sq+=v*v;lo=std::min(lo,v);hi=std::max(hi,v);last=v;++count;}};
struct Monitor:sca_tdf::sca_module{
 double dt,start,stop,capture;std::ofstream csv;std::vector<std::unique_ptr<sca_in<double>>> ports;
 std::vector<std::string> names;std::vector<Stat> stats;long count=0,bad=0;json event_defs,event_results=json::object();std::vector<double> prev;
 size_t bound=0;
 Monitor(sc_core::sc_module_name n,double h,const json&s,const char*path,const std::vector<std::string>&outputs):sca_tdf::sca_module(n),dt(h),start(s.at("window_s")[0]),stop(s.at("window_s")[1]),capture(s.value("capture_from_s",0.)),csv(path),event_defs(s.value("events",json::array())){csv<<std::setprecision(17);for(auto&name:outputs){ports.push_back(std::make_unique<sca_in<double>>(sc_core::sc_gen_unique_name("observe")));names.push_back(name);stats.emplace_back();prev.push_back(0);}}
 void add(sca_signal<double>&sig,const std::string&name){if(bound>=ports.size()||names[bound]!=name)throw std::runtime_error("Monitor order mismatch");(*ports[bound++])(sig);}
 void set_attributes()override{set_timestep(dt,sc_core::SC_SEC);}
 void processing()override{
  double t=get_time().to_seconds();if(count==0){csv<<"time_s,in_window";for(auto&n:names)csv<<","<<n;csv<<"\n";}
  if(t>=capture)csv<<t<<","<<(t>=start&&t<=stop?1:0);
  for(size_t k=0;k<ports.size();++k){double v=ports[k]->read();if(!std::isfinite(v))++bad;if(t>=start&&t<=stop)stats[k].add(v);if(t>=capture)csv<<","<<v;
   for(const auto&e:event_defs){if(e.at("output")==names[k]&&t>=e.value("after_s",0.)){std::string id=e.at("id");double level=e.at("threshold");bool rising=e.value("rising",true);if(!event_results.contains(id)&&count>0&&((rising&&prev[k]<level&&v>=level)||(!rising&&prev[k]>level&&v<=level))){double tc=t-dt+(level-prev[k])/(v-prev[k])*dt;event_results[id]=tc;}}}
   prev[k]=v;
  }if(t>=capture)csv<<"\n";++count;
 }
 json result(){json j={{"samples",count},{"nonfinite_samples",bad},{"statistics",json::object()},{"events",event_results}};for(size_t k=0;k<names.size();++k){auto&s=stats[k];if(!s.count)throw std::runtime_error("empty measurement window");double mean=s.sum/s.count;j["statistics"][names[k]]={{"mean",mean},{"min",s.lo},{"max",s.hi},{"last",s.last},{"pp",s.hi-s.lo},{"rms",std::sqrt(std::max(0.,s.sq/s.count))},{"stddev",std::sqrt(std::max(0.,s.sq/s.count-mean*mean))},{"samples",s.count}};}return j;}
};
void descendants(sc_core::sc_object*o,std::vector<sc_core::sc_object*>&v){for(auto*c:o->get_child_objects()){v.push_back(c);descendants(c,v);}}

template<class T>struct DiscreteMonitor:sca_tdf::sca_module{
 double dt,start,stop;std::ofstream csv;long samples=0;size_t bound=0;
 std::vector<std::unique_ptr<sca_in<T>>> ports;std::vector<std::string>names;std::vector<Stat>stats;
 DiscreteMonitor(sc_core::sc_module_name n,double h,const json&j,const std::string&path,const std::vector<std::string>&ns):sca_tdf::sca_module(n),dt(h),start(j.at("window_s")[0]),stop(j.at("window_s")[1]),csv(path),names(ns){
  csv<<std::setprecision(17)<<"time_s";for(auto&name:names){ports.push_back(std::make_unique<sca_in<T>>(sc_core::sc_gen_unique_name("discrete")));stats.emplace_back();csv<<","<<name;}csv<<"\n";
 }
 void set_attributes()override{set_timestep(dt,sc_core::SC_SEC);}
 void add(sca_signal<T>&sig){(*ports.at(bound++))(sig);}
 void processing()override{double t=get_time().to_seconds();if(t>=start&&t<=stop){csv<<t;for(size_t k=0;k<ports.size();++k){double v=static_cast<double>(ports[k]->read());stats[k].add(v);csv<<","<<v;}csv<<"\n";++samples;}}
 json result(){json r=json::object();for(size_t k=0;k<names.size();++k){auto&s=stats[k];r[names[k]]={{"min",s.lo},{"max",s.hi},{"last",s.last},{"samples",s.count}};}return r;}
};
