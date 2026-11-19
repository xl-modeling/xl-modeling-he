#include "s28.h"
#include "detail/json_value.h"
#include "metrics.h"
#include <filesystem>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>

namespace fs=std::filesystem;
using s28_test::number;
struct Record { double sample_s,output_s,vin; unsigned code,bus; };
struct TestConfig {
    unsigned m=16384,k=147,discard=128,harmonics=10,static_m=16384,static_k=147;
    double fs=100e6,amplitude=.95,bandwidth=50e6,static_amplitude=.95,dc_resolution_lsb=.01;
    unsigned dc_settle=4,dc_read=4;
    static TestConfig load(const fs::path& path) {
        auto j=s28_json::load(path.string()); TestConfig t;
        auto n=[&](const std::string& k){return j.at(k).as_number();};
        auto u=[&](const std::string& k){double v=n(k);if(v<0 || v>100000000 || std::floor(v)!=v)throw std::runtime_error("Invalid integer test option");return static_cast<unsigned>(v);};
        t.m=u("M");t.k=u("k");t.discard=u("startup_discard_samples");t.harmonics=u("harmonic_order");
        t.fs=n("sample_rate_hz");t.amplitude=n("differential_peak_v");t.bandwidth=n("analysis_bandwidth_hz");
        t.static_m=u("static_samples");t.static_k=u("static_k");t.static_amplitude=n("static_differential_peak_v");
        t.dc_resolution_lsb=n("dc_resolution_lsb");t.dc_settle=u("dc_settle_cycles");t.dc_read=u("dc_read_cycles");
        if(j.at("window").as_string()!="rectangular" ||
           j.at("fft_normalization").as_string()!="one-sided power |FFT/M|^2; double interior bins only" ||
           std::abs(n("dynamic_frequency_hz")-t.k*t.fs/t.m)>1e-6)
            throw std::runtime_error("Unsupported window/normalization or inconsistent frequency");
        if(t.m<256 || (t.m&(t.m-1)) || t.k==0 || t.k>=t.m/2 || std::gcd(t.k,t.m)!=1 || t.discard<2 ||
           t.fs<=0 || t.fs>100e6 || t.amplitude<=0 || t.amplitude>=1 || t.harmonics<2 || t.harmonics>100 ||
           t.bandwidth<=0 || t.bandwidth>t.fs/2 || t.static_m<16384 || t.static_k==0 || t.static_k>=t.static_m/2 ||
           std::gcd(t.static_k,t.static_m)!=1 || t.static_amplitude<=0 || t.static_amplitude>1.1 ||
           t.dc_resolution_lsb<=0 || t.dc_resolution_lsb>.01 || t.dc_settle<2 || t.dc_read<1)
            throw std::runtime_error("Invalid or noncoherent test configuration");
        return t;
    }
};
class Stimulus:public sca_tdf::sca_module {
public:
    sca_tdf::sca_out<double> ip{"ip"},in{"in"},cp{"cp"},cn{"cn"},ref{"ref"},ctl{"ctl"},vcmp{"vcmp"};
    sca_tdf::sca_out<double> avdd{"avdd"},dvdd{"dvdd"},vdd{"vdd"},gnd{"gnd"};
    sca_tdf::sca_out<double> io25{"io25"};
    double level=0,amplitude=.95,frequency=0,dc=0;
    bool sine=false;
    s28::Config cfg;
    Stimulus(sc_core::sc_module_name n,const s28::Config& c):sca_tdf::sca_module(n),cfg(c){}
    void set_attributes() override {set_timestep(cfg.timestep_s,sc_core::SC_SEC);}
    void processing() override {
        const double now=sc_core::sc_time_stamp().to_seconds(),period=1/cfg.sample_rate_hz;
        const auto half=static_cast<std::uint64_t>(std::llround(.5*period/cfg.timestep_s));
        const auto tick=static_cast<std::uint64_t>(std::llround(now/cfg.timestep_s));
        bool high=(tick%(2*half))<half;
        // Phase zero at the first falling sample edge; no reference to ADC outputs.
        double v=sine?dc+amplitude*std::sin(2*s28_test::pi*frequency*(now-.5*period)):level;
        ip.write(cfg.common_mode_v+v/2);in.write(cfg.common_mode_v-v/2);
        cp.write(high?cfg.supply_v:0);cn.write(high?0:cfg.supply_v);
        ref.write(cfg.reference_v);ctl.write(cfg.ctl_center_v);vcmp.write(cfg.vcmp_center_v);
        avdd.write(cfg.supply_v);dvdd.write(cfg.supply_v);vdd.write(cfg.supply_v);gnd.write(0);
        io25.write(cfg.pad_ring_v);
    }
};
class Capture:public sca_tdf::sca_module {
public:
    sca_tdf::sca_in<unsigned> code{"code"},seq{"seq"};
    sca_tdf::sca_in<double> d1{"d1"},d2{"d2"},d3{"d3"},d4{"d4"},d5{"d5"},d6{"d6"};
    sca_tdf::sca_in<double> sample_s{"sample_s"},output_s{"output_s"},vin{"vin"},latch{"latch"};
    std::vector<Record> records;
    unsigned last=0,errors=0;
    double supply;
    Capture(sc_core::sc_module_name n,double v):sca_tdf::sca_module(n),supply(v){}
    void processing() override {
        if(seq.read()==last)return;
        last=seq.read();
        unsigned bus=(d1.read()>supply/2?1:0)|(d2.read()>supply/2?2:0)|(d3.read()>supply/2?4:0)|
            (d4.read()>supply/2?8:0)|(d5.read()>supply/2?16:0)|(d6.read()>supply/2?32:0);
        if(bus!=code.read() || bus>63 || latch.read()<supply/2)++errors;
        records.push_back({sample_s.read(),output_s.read(),vin.read(),code.read(),bus});
    }
};
class Bench:public sc_core::sc_module {
public:
    sca_tdf::sca_signal<double> ip,in,cp,cn,ref,cm,ctl,vcmp,avdd,dvdd,vdd,gnd,io25;
    sca_tdf::sca_signal<double> d1,d2,d3,d4,d5,d6,latch,sample_s,output_s,vin;
    sca_tdf::sca_signal<unsigned> code,seq;
    Stimulus source;s28::S28 adc;Capture capture;
    Bench(sc_core::sc_module_name n,const s28::Config& c):sc_core::sc_module(n),source("stimulus",c),adc("s28",c),capture("capture",c.supply_v){
        source.ip(ip);source.in(in);source.cp(cp);source.cn(cn);source.ref(ref);source.ctl(ctl);source.vcmp(vcmp);
        source.avdd(avdd);source.dvdd(dvdd);source.vdd(vdd);source.gnd(gnd);
        source.io25(io25);
        adc.IP(ip);adc.IN(in);adc.CLK_IP(cp);adc.CLK_IN(cn);adc.VRP(ref);adc.VCM(cm);adc.CTL(ctl);adc.VCMP(vcmp);
        adc.AVDD(avdd);adc.DVDD(dvdd);adc.VDD(vdd);adc.AVSS(gnd);
        adc.VSS(gnd);adc.DVSS(gnd);adc.CLKVDD(vdd);adc.CLKVSS(gnd);
        adc.VSS2(gnd);adc.SAVDD(vdd);adc.SAVSS(gnd);adc.VSSD(gnd);adc.VDDD(io25);
        adc.DVDDIO(vdd);adc.DVSSIO(gnd);adc.INVSS(gnd);adc.INVDD(vdd);
        adc.VSS1(gnd);adc.SAVDD1(vdd);adc.SAVSS1(gnd);
        adc.D1OUT(d1);adc.D2OUT(d2);adc.D3OUT(d3);adc.D4OUT(d4);adc.D5OUT(d5);adc.D6OUT(d6);adc.CLKOUT1(latch);
        adc.code_out(code);adc.sequence_out(seq);adc.sample_time_out(sample_s);adc.output_time_out(output_s);adc.sampled_diff_out(vin);
        capture.code(code);capture.seq(seq);capture.d1(d1);capture.d2(d2);capture.d3(d3);capture.d4(d4);capture.d5(d5);capture.d6(d6);
        capture.sample_s(sample_s);capture.output_s(output_s);capture.vin(vin);capture.latch(latch);
    }
    void cycles(unsigned n){sc_core::sc_start(n/adc.config().sample_rate_hz,sc_core::SC_SEC);}
    unsigned dc(double v,unsigned settle,unsigned read,std::ofstream* trace=nullptr){
        source.sine=false;source.level=v;const auto begin=capture.records.size();cycles(settle+read);
        if(capture.records.size()<begin+settle+read-1)throw std::runtime_error("Missing DC output frames");
        std::vector<unsigned> a;
        for(std::size_t i=capture.records.size()-read;i<capture.records.size();++i){
            const auto& r=capture.records[i];a.push_back(r.code);
            if(trace)*trace<<std::setprecision(17)<<r.sample_s<<','<<r.output_s<<','<<v<<','<<r.code<<','<<r.bus<<'\n';
        }
        std::sort(a.begin(),a.end());return a[a.size()/2];
    }
};
static void require(bool b,const std::string& text){if(!b)throw std::runtime_error(text);}
struct TraceGuard {
    sca_util::sca_trace_file* file=nullptr;
    void close(){ if(file){ sca_util::sca_close_vcd_trace_file(file); file=nullptr; } }
    ~TraceGuard(){ close(); }
};
static void create_trace(TraceGuard& guard,const fs::path& out,Bench& b){
    guard.file=sca_util::sca_create_vcd_trace_file((out/"waveforms").string().c_str());
    if(!guard.file) throw std::runtime_error("Could not create VCD trace file");
    sca_util::sca_trace(guard.file,b.ip,"IP"); sca_util::sca_trace(guard.file,b.in,"IN");
    sca_util::sca_trace(guard.file,b.cp,"CLK_IP"); sca_util::sca_trace(guard.file,b.cn,"CLK_IN");
    sca_util::sca_trace(guard.file,b.cm,"VCM"); sca_util::sca_trace(guard.file,b.ref,"VRP");
    sca_util::sca_trace(guard.file,b.code,"code"); sca_util::sca_trace(guard.file,b.d1,"D1OUT");
    sca_util::sca_trace(guard.file,b.d2,"D2OUT"); sca_util::sca_trace(guard.file,b.d3,"D3OUT");
    sca_util::sca_trace(guard.file,b.d4,"D4OUT"); sca_util::sca_trace(guard.file,b.d5,"D5OUT");
    sca_util::sca_trace(guard.file,b.d6,"D6OUT"); sca_util::sca_trace(guard.file,b.latch,"CLKOUT1");
}
static void write_records(const fs::path& p,const std::vector<Record>& a,std::size_t first=0,std::size_t count=0){
    std::ofstream f(p);f<<"sample_time_s,output_time_s,vin_diff_v,code,bus_code,D1OUT,D2OUT,D3OUT,D4OUT,D5OUT,D6OUT\n";
    auto end=count?std::min(a.size(),first+count):a.size();
    for(std::size_t i=first;i<end;++i){const auto&r=a[i];f<<std::setprecision(17)<<r.sample_s<<','<<r.output_s<<','<<r.vin<<','<<r.code<<','<<r.bus;
        for(unsigned b=0;b<6;++b)f<<','<<((r.bus>>b)&1);
        f<<'\n';}
    require(bool(f),"Could not write raw records");
}
static void svg(const fs::path& path,const std::vector<double>& y,const std::string& title,const std::string& ylabel,
                double xstep=1,double xorigin=0,const std::string& xlabel="Sample index"){
    if(y.empty())return;
    std::ofstream f(path);double lo=*std::min_element(y.begin(),y.end()),hi=*std::max_element(y.begin(),y.end());
    if(hi-lo<1e-9){lo-=.5;hi+=.5;}double pad=(hi-lo)*.1;lo-=pad;hi+=pad;
    f<<"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1000\" height=\"450\" viewBox=\"0 0 1000 450\"><rect width=\"1000\" height=\"450\" fill=\"white\"/><g font-family=\"serif\" fill=\"black\"><text x=\"80\" y=\"25\" font-size=\"20\">"<<title<<"</text>";
    f<<"<path d=\"M80 45 V390 H970\" fill=\"none\" stroke=\"black\"/><text x=\"460\" y=\"440\">"<<xlabel<<"</text><text x=\"5\" y=\"55\">"<<ylabel<<"</text>";
    for(int i=0;i<=4;++i){double v=lo+(hi-lo)*i/4;f<<"<text x=\"5\" y=\""<<390-345*i/4<<"\">"<<std::setprecision(4)<<v<<"</text>";}
    for(int i=0;i<=4;++i){double v=xorigin+xstep*(y.size()-1)*i/4;f<<"<text text-anchor=\"middle\" x=\""<<80+890*i/4<<"\" y=\"415\">"<<std::setprecision(5)<<v<<"</text>";}
    f<<"</g><polyline fill=\"none\" stroke=\"#1558a0\" stroke-width=\"1\" points=\"";
    for(std::size_t i=0;i<y.size();++i)f<<80+890.0*i/std::max<std::size_t>(1,y.size()-1)<<','<<390-345*(y[i]-lo)/(hi-lo)<<' ';
    f<<"\"/></svg>";
}
struct Transition {double lo,hi;double mid()const{return (lo+hi)/2;}};
static Transition refine(Bench& b,unsigned k,double lo,double hi,const TestConfig&t,std::ofstream&trace){
    require(b.dc(lo,t.dc_settle,t.dc_read,&trace)<k && b.dc(hi,t.dc_settle,t.dc_read,&trace)>=k,"DC bracket does not contain transition");
    while(hi-lo>t.dc_resolution_lsb*b.adc.config().lsb()){
        double m=(lo+hi)/2;if(b.dc(m,t.dc_settle,t.dc_read,&trace)>=k)hi=m;else lo=m;
    }return {lo,hi};
}
static void resolution_case(Bench& b,const fs::path& out,const TestConfig&t){
    // Scenario-10 resolution: slow differential ramp, then fine scan code-0/code-62 transitions.
    const double vref=b.adc.config().reference_v, differential_lsb=b.adc.config().lsb();
    // Two settle and two read cycles are sufficient for this deterministic DC check.
    TestConfig fast=t; fast.dc_settle=2; fast.dc_read=2;
    const unsigned points=512; const double ramp_lo=-1.10*vref, ramp_hi=1.10*vref;
    std::array<Transition,64> brackets{}; std::array<bool,64> seen{}; std::set<unsigned> codes;
    std::ofstream trace(out/"resolution_ramp.csv");
    trace<<"vin_diff_v,vin_pin_p_v,vin_pin_n_v,code,expected_code\n";
    double oldv=ramp_lo; unsigned oldc=b.dc(oldv,fast.dc_settle,fast.dc_read), mismatches=0;
    codes.insert(oldc);
    for(unsigned i=1;i<=points;++i){
        const double v=ramp_lo+(ramp_hi-ramp_lo)*static_cast<double>(i)/points;
        const unsigned c=b.dc(v,fast.dc_settle,fast.dc_read);
        const unsigned expected=static_cast<unsigned>(std::max(0.,std::min(63.,std::floor((v+vref)/differential_lsb))));
        codes.insert(c); if(c!=expected)++mismatches;
        for(unsigned k=oldc+1;k<=c && k<64;++k){brackets[k]={oldv,v};seen[k]=true;}
        trace<<std::setprecision(17)<<v<<','<<b.adc.config().common_mode_v+v/2<<','<<b.adc.config().common_mode_v-v/2<<','<<c<<','<<expected<<"\n";
        oldv=v; oldc=c;
    }
    require(seen[1]&&seen[63],"Resolution ramp did not bracket code-0/code-1 and code-62/code-63 transitions");
    std::ofstream transition_trace(out/"resolution_transition_trace.csv");
    transition_trace<<"transition,bracket_low_v,bracket_high_v,estimate_v,resolution_lsb\n";
    const auto v1=refine(b,1,brackets[1].lo,brackets[1].hi,fast,transition_trace);
    const auto v63=refine(b,63,brackets[63].lo,brackets[63].hi,fast,transition_trace);
    const double average_lsb=(v63.mid()-v1.mid())/62.0;
    transition_trace<<1<<','<<std::setprecision(17)<<v1.lo<<','<<v1.hi<<','<<v1.mid()<<','<<(v1.hi-v1.lo)/differential_lsb<<"\n";
    transition_trace<<63<<','<<std::setprecision(17)<<v63.lo<<','<<v63.hi<<','<<v63.mid()<<','<<(v63.hi-v63.lo)/differential_lsb<<"\n";
    const bool lsb_pass=average_lsb>=0.028125&&average_lsb<=0.034375;
    std::ofstream f(out/"result.json");
    f<<"{\"case\":1,\"test_id\":\"ADC-001-10-1-1\",\"scenario_id\":\"02-10-1\"," 
     <<"\"excitation\":\"slow_differential_ramp\",\"ramp_start_v\":"<<ramp_lo<<",\"ramp_stop_v\":"<<ramp_hi
     <<",\"ramp_points\":"<<points+1<<",\"resolution_bits\":6,\"nominal_lsb_v\":"<<differential_lsb
     <<",\"transition_code_1_v\":"<<v1.mid()<<",\"transition_code_63_v\":"<<v63.mid()
     <<",\"dc_settle_cycles\":"<<fast.dc_settle<<",\"dc_read_cycles\":"<<fast.dc_read
     <<",\"average_lsb_v\":"<<average_lsb<<",\"acceptance_lsb_v\":[0.028125,0.034375]"
     <<",\"codes_seen\":"<<codes.size()<<",\"code_range_min\":"<<(codes.empty()?0:*codes.begin())
     <<",\"code_range_max\":"<<(codes.empty()?0:*codes.rbegin())<<",\"ramp_code_mismatches\":"<<mismatches
     <<",\"bus_mismatches\":"<<b.capture.errors<<",\"passed\":"<<(lsb_pass&&b.capture.errors==0?"true":"false")<<"}\n";
    write_records(out/"samples.csv",b.capture.records);
    std::vector<double> y; for(auto&r:b.capture.records)y.push_back(r.code);
    svg(out/"resolution_ramp.svg",y,"Resolution differential ramp","code");
    require(lsb_pass&&b.capture.errors==0,"Resolution average LSB is outside the scenario-10 acceptance range");
}
static void range_case(Bench& b,const fs::path& out,const TestConfig&t){
    // Scenario-10 range: the same -1.1 V to +1.1 V ramp, with V1/V63 boundary extraction.
    const double vref=b.adc.config().reference_v, differential_lsb=b.adc.config().lsb(); const unsigned points=2200;
    const double ramp_lo=-1.10*vref, ramp_hi=1.10*vref;
    std::array<Transition,64> brackets{}; std::array<bool,64> seen{}; std::set<unsigned> codes; std::array<std::uint64_t,64> counts{};
    std::ofstream trace(out/"range_scan.csv"); trace<<"vin_diff_v,vin_pin_p_v,vin_pin_n_v,code,expected_code\n";
    double oldv=ramp_lo; unsigned oldc=b.dc(oldv,t.dc_settle,t.dc_read), nonmono=0, skipped=0; bool first=true;
    codes.insert(oldc); ++counts[oldc];
    for(unsigned i=1;i<=points;++i){
        const double v=ramp_lo+(ramp_hi-ramp_lo)*static_cast<double>(i)/points;
        const unsigned c=b.dc(v,t.dc_settle,t.dc_read);
        const unsigned expected=static_cast<unsigned>(std::max(0.,std::min(63.,std::floor((v+vref)/differential_lsb))));
        codes.insert(c); ++counts[c]; if(!first){if(c<oldc)++nonmono;if(c>oldc+1)++skipped;} first=false;
        for(unsigned k=oldc+1;k<=c && k<64;++k){brackets[k]={oldv,v};seen[k]=true;}
        trace<<std::setprecision(17)<<v<<','<<b.adc.config().common_mode_v+v/2<<','<<b.adc.config().common_mode_v-v/2<<','<<c<<','<<expected<<"\n";
        oldv=v; oldc=c;
    }
    require(seen[1]&&seen[63],"Range ramp did not bracket V1 and V63 transitions");
    std::ofstream transition_trace(out/"range_transition_trace.csv"); transition_trace<<"transition,bracket_low_v,bracket_high_v,estimate_v\n";
    const auto v1=refine(b,1,brackets[1].lo,brackets[1].hi,t,transition_trace);
    const auto v63=refine(b,63,brackets[63].lo,brackets[63].hi,t,transition_trace);
    transition_trace<<1<<','<<std::setprecision(17)<<v1.lo<<','<<v1.hi<<','<<v1.mid()<<"\n";
    transition_trace<<63<<','<<std::setprecision(17)<<v63.lo<<','<<v63.hi<<','<<v63.mid()<<"\n";
    const unsigned low=b.dc(-1.10*vref,t.dc_settle,t.dc_read), high=b.dc(1.10*vref,t.dc_settle,t.dc_read);
    const bool all=codes.size()==64&&*codes.begin()==0&&*codes.rbegin()==63;
    const bool range_pass=v1.mid()<=-0.9*vref&&v63.mid()>=0.9*vref;
    const bool saturated=low==0&&high==63;
    std::ofstream histogram(out/"range_histogram.csv"); histogram<<"code,count\n"; for(unsigned code=0;code<64;++code) histogram<<code<<','<<counts[code]<<"\n";
     std::ofstream f(out/"result.json"); f<<"{\"case\":2,\"test_id\":\"ADC-001-10-1-2\",\"scenario_id\":\"02-10-1\",\"excitation\":\"slow_differential_ramp\",\"ramp_start_v\":"<<ramp_lo<<",\"ramp_stop_v\":"<<ramp_hi<<",\"v1_code1_v\":"<<v1.mid()<<",\"v63_code63_v\":"<<v63.mid()<<",\"actual_range_v\":["<<v1.mid()<<','<<v63.mid()<<"],\"acceptance_range_v\":["<<-0.9*vref<<','<<0.9*vref<<"],\"codes_seen\":"<<codes.size()<<",\"all_codes_0_to_63\":"<<(all?"true":"false")<<",\"below_range_code\":"<<low<<",\"above_range_code\":"<<high<<",\"saturation_verified\":"<<(saturated?"true":"false")<<",\"nonmonotonic_steps\":"<<nonmono<<",\"skipped_codes\":"<<skipped<<",\"bus_mismatches\":"<<b.capture.errors<<",\"passed\":"<<(range_pass&&all&&saturated&&nonmono==0&&skipped==0&&b.capture.errors==0?"true":"false")<<"}\n";
    write_records(out/"samples.csv",b.capture.records); std::vector<double> y; for(auto&r:b.capture.records)y.push_back(r.code); svg(out/"range_scan.svg",y,"Input range ramp","code");
    require(range_pass&&all&&saturated&&nonmono==0&&skipped==0&&b.capture.errors==0,"Input range, saturation or monotonicity check failed");
}
static void static_case(Bench& b,const fs::path& out,const TestConfig&t){
    b.source.sine=true;b.source.amplitude=t.static_amplitude;b.source.frequency=t.static_k*t.fs/t.static_m;
    b.cycles(t.discard+t.static_m);const auto&a=b.capture.records;require(a.size()>=t.discard+t.static_m,"Incomplete histogram record");
    const std::size_t start=t.discard;std::array<std::uint64_t,64> counts{};
    double b0=0,ss=0,cc=0;
    for(unsigned i=0;i<t.static_m;++i){const auto&r=a[start+i];++counts[r.code];double phi=2*s28_test::pi*b.source.frequency*(r.sample_s-.5/t.fs);b0+=r.vin;ss+=r.vin*std::sin(phi);cc+=r.vin*std::cos(phi);}
    b0/=t.static_m;double amp=2*std::hypot(ss,cc)/t.static_m;const double lsb=b.adc.config().lsb();
    require(std::isfinite(amp)&&amp>0,"Invalid fitted sine amplitude");
    // Only measured internal transitions define the endpoint line. This is
    // also the convention used by the delivered MATLAB postprocessor.
    std::array<double,63> thresholds{};
    thresholds.fill(std::numeric_limits<double>::quiet_NaN());
    std::uint64_t cum=0;unsigned first=63,last=0;
    for(unsigned i=0;i<63;++i){
        cum+=counts[i];
        if(cum>0&&cum<t.static_m){
            thresholds[i]=b0-amp*std::cos(s28_test::pi*static_cast<double>(cum)/t.static_m);
            first=std::min(first,i);last=i;
        }
    }
    require(first<last,"Insufficient measured histogram transitions");
    const double endpoint_lsb=(thresholds[last]-thresholds[first])/(last-first);
    require(endpoint_lsb>0,"Invalid histogram endpoint line");
    auto cdf=[&](double x){return .5+std::asin(std::max(-1.,std::min(1.,(x-b0)/amp)))/s28_test::pi;};
    std::ofstream f(out/"linearity.csv");f<<"code,count,ideal_count,dnl_lsb,transition_v,inl_endpoint_lsb\n";
    std::vector<double> dnl,inl;double minD=1e9,maxD=-1e9,minI=1e9,maxI=-1e9;unsigned empty=0;
    for(unsigned code=0;code<64;++code){f<<code<<','<<counts[code]<<',';
        const bool valid_code=code>=first+1&&code<last+1;
        if(valid_code){double lower=thresholds[first]+(code-first-1)*endpoint_lsb,upper=lower+endpoint_lsb;
            if(counts[code]==0)++empty;
            double ideal=t.static_m*(cdf(upper)-cdf(lower));double d=counts[code]/ideal-1;dnl.push_back(d);minD=std::min(minD,d);maxD=std::max(maxD,d);number(f,ideal);f<<',';number(f,d);
        }else {f<<",";}
        f<<',';if(code>0&&std::isfinite(thresholds[code-1])){double v=thresholds[code-1],e=(v-(thresholds[first]+(code-1-first)*endpoint_lsb))/lsb;
            number(f,v);f<<',';number(f,e);inl.push_back(e);minI=std::min(minI,e);maxI=std::max(maxI,e);}else f<<',';f<<'\n';}
    const unsigned unexpected_empty=empty;
    std::ofstream r(out/"result.json");r<<"{\"case\":3,\"test_id\":\"ADC-001-10-1-3\",\"scenario_id\":\"02-10-1\",\"excitation\":\"low_frequency_differential_sine\",\"method\":\"sine histogram with sampled-input amplitude and DC correction; endpoint INL\",\"samples\":"<<t.static_m<<",\"k\":"<<t.static_k<<",\"frequency_hz\":";number(r,b.source.frequency);
    r<<",\"fitted_input_peak_v\":";number(r,amp);r<<",\"fitted_input_dc_v\":";number(r,b0);
    r<<",\"dnl_min_lsb\":";number(r,minD);r<<",\"dnl_max_lsb\":";number(r,maxD);r<<",\"inl_min_lsb\":";number(r,minI);r<<",\"inl_max_lsb\":";number(r,maxI);
    unsigned internal_empty=0;for(unsigned code=1;code<63;++code)if(counts[code]==0)++internal_empty;
    r << ",\"internal_zero_count_codes\":" << internal_empty;
    r << ",\"missing_valid_codes\":" << empty;
    r << ",\"valid_code_min\":" << first+1 << ",\"valid_code_max\":" << last;
    r << ",\"first_measured_transition\":" << first+1 << ",\"last_measured_transition\":" << last+1;
    r << ",\"edge_codes_excluded_from_dnl\":[";
    bool comma=false;for(unsigned code=0;code<64;++code)if(code<first+1||code>=last+1){if(comma)r<<',';r<<code;comma=true;}r<<']';
    r << ",\"passed\":"
      << (unexpected_empty==0 && b.capture.errors==0
          && std::max(std::abs(minD),std::abs(maxD))<=.5
          && std::max(std::abs(minI),std::abs(maxI))<=.5 ? "true" : "false")
      << "}\n";
    write_records(out/"samples.csv",a,start,t.static_m);svg(out/"dnl.svg",dnl,"Sine histogram DNL (fully observed codes)","LSB",1,first+1,"Output code");svg(out/"inl.svg",inl,"Endpoint INL (measured transitions)","LSB",1,first+1,"Transition index");
    require(unexpected_empty==0&&b.capture.errors==0,"Histogram coverage or bus consistency failed");
    require(std::max(std::abs(minD),std::abs(maxD))<=.5&&std::max(std::abs(minI),std::abs(maxI))<=.5,"Static design-goal limits exceeded");
}
static std::vector<Record> load_effective_records(const fs::path& path,unsigned& bus_errors){
    std::ifstream f(path);require(bool(f),"Could not open shared sample record: "+path.string());
    std::string line;require(bool(std::getline(f,line)),"Shared sample record is empty: "+path.string());
    std::vector<Record> records;records.reserve(16384);bus_errors=0;
    while(std::getline(f,line)){
        if(line.empty())continue;
        std::stringstream ss(line);std::string field;std::vector<std::string> fields;
        while(std::getline(ss,field,','))fields.push_back(field);
        require(fields.size()>=5,"Malformed shared sample record: "+path.string());
        Record r{};r.sample_s=std::stod(fields[0]);r.output_s=std::stod(fields[1]);r.vin=std::stod(fields[2]);
        r.code=static_cast<unsigned>(std::stoul(fields[3]));r.bus=static_cast<unsigned>(std::stoul(fields[4]));
        if(r.bus!=r.code || r.bus>63) { ++bus_errors; }
        records.push_back(r);
    }
    return records;
}
static void dynamic_analyze(Bench& b,const fs::path& out,const TestConfig&t,const std::vector<Record>& a,unsigned bus_errors){
    require(a.size()>=t.m,"Incomplete dynamic record");
    b.source.amplitude=t.amplitude;b.source.frequency=t.k*t.fs/t.m;
    std::vector<double> y; y.reserve(t.m);for(unsigned i=0;i<t.m;++i)y.push_back(a[i].code*b.adc.config().lsb());
    const auto s=s28_test::analyze(y,t.fs,t.k,t.harmonics,t.bandwidth);
    std::ofstream f(out/"result.json");f<<"{\"case\":4,\"test_id\":\"ADC-001-10-1-4\",\"scenario_id\":\"02-10-1\",\"excitation\":\"coherent_differential_sine\",\"M\":"<<t.m<<",\"k\":"<<t.k<<",\"sample_rate_hz\":"<<t.fs<<",\"frequency_hz\":";number(f,b.source.frequency);
    f<<",\"differential_peak_v\":"<<t.amplitude<<",\"differential_vpp\":"<<2*t.amplitude<<",\"startup_discard_samples\":"<<t.discard<<",\"analysis_bandwidth_hz\":"<<t.bandwidth<<",\"harmonic_order\":"<<t.harmonics;
    f<<",\"window\":\"rectangular\",\"normalization\":\"one-sided |FFT/M|^2; interior bins doubled; DC removed; carrier reference; no ENOB amplitude correction\",\"snr_db\":";number(f,s.snr);f<<",\"thd_db\":";number(f,s.thd);f<<",\"enob_bit\":";number(f,s.enob);
    const double reconstructed=1/(std::pow(10.,-s.snr/10)+std::pow(10.,s.thd/10));
    const bool consistency_pass=bus_errors==0&&std::isfinite(s.sndr)&&s.sndr<=s.snr+1e-9&&std::abs(10*std::log10(reconstructed)-s.sndr)<1e-8;
    const bool range_pass=s.snr>=36.5&&s.snr<=37.6&&s.thd>=-60.0&&s.thd<=-50.0&&s.enob>=5.8&&s.enob<=6.0;
    const bool passed=consistency_pass&&range_pass;
    f<<",\"reference_table\":{\"snr_db\":37.63,\"thd_db\":-53.09,\"enob_bit\":5.93},\"acceptance\":{\"snr_db\":\"36.5..37.6\",\"thd_db\":\"-60..-50\",\"enob_bit\":\"5.8..6.0\"},\"same_frequency_as_datasheet_figure\":"<<(std::abs(b.source.frequency-2697750.)<10?"true":"false")<<",\"passed\":"<<(passed?"true":"false")<<",\"pass_scope\":\"coherence, numeric-pin encoding, metric consistency and acceptance ranges\"}\n";
    std::ofstream h(out/"harmonics.csv");h<<"order,folded_bin,folded_frequency_hz,power_v2,level_dbc,counted_once\n";std::set<unsigned> used;
    for(unsigned i=0;i<s.harmonic_bins.size();++i){unsigned bin=s.harmonic_bins[i];bool counted=bin&&bin!=t.k&&bin*t.fs/t.m<=t.bandwidth&&used.insert(bin).second;
        h<<i+2<<','<<bin<<','<<std::setprecision(17)<<bin*t.fs/t.m<<','<<s.power[bin]<<',';number(h,s28_test::db(s.power[bin],s.signal));h<<','<<(counted?1:0)<<'\n';}
    std::ofstream p(out/"spectrum.csv");p<<"bin,frequency_hz,power_v2,level_dbc\n";std::vector<double> plot;
    for(unsigned i=0;i<s.power.size();++i){p<<i<<','<<std::setprecision(17)<<i*t.fs/t.m<<','<<s.power[i]<<',';double v=s28_test::db(s.power[i],s.signal);number(p,v);p<<'\n';plot.push_back(std::isfinite(v)?std::max(-160.,v):-160.);}
    svg(out/"spectrum.svg",plot,"Output-code coherent FFT","dBc/bin",t.fs/t.m/1e6,0,"Frequency (MHz)");
    write_records(out/"samples.csv",a,0,t.m);
    require(consistency_pass,"Dynamic consistency check failed");
    double expected=1/(std::pow(10.,-s.snr/10)+std::pow(10.,s.thd/10));
    require(std::abs(10*std::log10(expected)-s.sndr)<1e-8,"SNR/THD/SNDR power identity failed");
    require(range_pass,"Dynamic metric is outside the scenario-10 acceptance range");
}
static void dynamic_case(Bench& b,const fs::path& out,const TestConfig&t){
    b.source.sine=true;b.source.amplitude=t.amplitude;b.source.frequency=t.k*t.fs/t.m;
    b.cycles(t.discard+t.m);const auto&all=b.capture.records;require(all.size()>=t.discard+t.m,"Incomplete dynamic record");
    std::vector<Record> a(all.begin()+t.discard,all.begin()+t.discard+t.m);
    dynamic_analyze(b,out,t,a,b.capture.errors);
}
static void dynamic_case_from_shared(Bench& b,const fs::path& out,const TestConfig&t,const fs::path& shared){
    unsigned bus_errors=0;const auto a=load_effective_records(shared,bus_errors);
    dynamic_analyze(b,out,t,a,bus_errors);
}
int sc_main(int argc,char**argv){
    try{
        if(argc!=6&&argc!=7){std::cerr<<"Usage: s28_tb CASE MODEL_JSON TEST_JSON OUTPUT_DIR ROOT_DIR [SHARED_SAMPLES_CSV]\n";return 2;}
        unsigned which=static_cast<unsigned>(std::stoul(argv[1]));require(which>=1&&which<=4,"Case must be 1 to 4");
        fs::path model=argv[2],test=argv[3],out=argv[4];const fs::path shared=argc==7?fs::path(argv[6]):fs::path{};
        require(!fs::exists(out),"Output directory already exists; choose a fresh path");
        auto c=s28::Config::load(model.string());auto t=TestConfig::load(test);require(std::abs(c.sample_rate_hz-t.fs)<1,"Model/test sample-rate mismatch");
        require(std::abs(c.reference_v-1)<1e-12&&std::abs(c.common_mode_v-.5)<1e-12&&std::abs(c.supply_v-1)<1e-12,"These tests require 1 V supply/reference and 0.5 V common mode");
        if(which<=2){c.noise_rms_lsb=0;c.transition_error_lsb.fill(0);c.offset_lsb=0;c.span_error_percent=0;c.quadratic_v_inv=0;c.cubic_v_inv2=0;}
        s28_test::selftest();c.validate();fs::create_directories(out);
        fs::copy_file(model,out/"model_input.json");fs::copy_file(test,out/"test_input.json");
        std::ofstream effective(out/"effective_profile.json");
        effective<<"{\"scenario_id\":\"02-10-1\",\"case\":"<<which<<",\"source\":\"model_input.json\",\"supply_v\":1.0,\"common_mode_v\":0.5,\"cdac_reference_v\":1.0,\"pad_ring_v\":2.5,\"ambient_temperature_c\":2.0,\"cdac_sampling_cap_pf\":4.096,\"noise_rms_lsb\":"<<c.noise_rms_lsb
                 <<",\"overrides\":\""<<(which<=2?"noise, threshold errors, offset, gain and polynomial disabled for independent coding check":"none")<<"\"}\n";
        effective.close();
        sc_core::sc_set_time_resolution(1,sc_core::SC_FS);
        Bench b("platform",c);
        const bool disable_vcd=std::getenv("S28_DISABLE_VCD")!=nullptr;
        TraceGuard tracing; if(!disable_vcd&&(which!=4||shared.empty()))create_trace(tracing,out,b);
        if(which==1)resolution_case(b,out,t);else if(which==2)range_case(b,out,t);else if(which==3)static_case(b,out,t);
        else if(shared.empty())dynamic_case(b,out,t);else dynamic_case_from_shared(b,out,t,shared);
        tracing.close();
        std::ifstream f(out/"result.json");std::cout<<f.rdbuf()<<"RESULT_FOLDER="<<fs::absolute(out)<<'\n';return 0;
    }catch(const std::exception&e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
