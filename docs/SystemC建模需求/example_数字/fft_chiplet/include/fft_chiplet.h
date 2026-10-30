#ifndef FFT_CHIPLET_H
#define FFT_CHIPLET_H

#include <vector>
#include <complex>
#include <string>
#include <fstream>
#include <systemc>
#include <tlm.h>
#include <tlm_utils/simple_target_socket.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace sc_core;
using namespace tlm;
using namespace std;

class FFTChiplet: public sc_module {                              //定义一个名为FFTChiplet的类，继承自sc_module（SystemC模块基类）
public:
    tlm_utils::simple_target_socket<FFTChiplet> target_socket;    //声明一个TLM-2.0简单目标套接字，用于通信
         
    SC_HAS_PROCESS(FFTChiplet);                                   //宏声明，表明该模块包含SystemC进程                        

    FFTChiplet(sc_module_name name, const string& config_file);   //构造函数，接收模块名和配置文件路径
    ~FFTChiplet();                                                //析构函数
    
    void b_transport(tlm_generic_payload& trans, sc_time& delay); //TLM-2.0阻塞传输接口
    void load_config(const string& config_file);                  //加载配置文件的方法
    string get_fft_name(const string& config_file);               //获取FFT名称
    double get_fft_version(const string& config_file);            //获取FFT版本
    double get_fft_latency(const string& config_file);            //获取FFT延迟
    double get_fft_area(const string& config_file);               //获取FFT面积
    double get_fft_power(const string& config_file);              //获取FFT功耗

    enum Registers {
        REG_CONTROL  = 0x00000000,   //控制寄存器地址
        REG_STATUS   = 0x00000004,   //状态寄存器地址
        REG_DATA_IN  = 0x00000010,   //数据输入寄存器起始地址
        REG_DATA_OUT = 0x02000000    //数据输出寄存器起始地址   
    };

    enum ControlBits {
        CONTROL_START = 0x01,     //启动FFT
        CONTROL_RESET = 0x02      //清空输入输出缓冲区
    };

    enum StatusBits {
        STATUS_IDLE  = 0x01,      //FFT处于空闲状态
        STATUS_READY = 0x02,      //数据已写入输入缓冲区
        STATUS_BUSY  = 0x04,      //FFT正在计算
        STATUS_DONE  = 0x08       //计算完成后的数据已写入输出缓冲区
    };
    
private:
    string        name;           //FFT模块名称
    double        version;        //FFT版本号
    bool          ifft_enable;    //是否为IFFT（true表示IFFT，false表示FFT）
    unsigned int  fft_length;     //FFT点数/长度
    double        latency;        //FFT计算延迟
    double        area;           //FFT硬件面积
    double        power;          //FFT功耗

    bool          data_ready;
    bool          comp_busy;
    bool          comp_done;
    sc_event      start_event;
    
    vector<complex<double>> input_data;    //输入数据缓冲区（复数）
    vector<complex<double>> output_data;   //输出数据缓冲区（复数）
    
    //内部函数
    bool read_memory(unsigned int addr, unsigned char* data, unsigned int length);   //读取内存
    bool write_memory(unsigned int addr, unsigned char* data, unsigned int length);  //写入内存
    void computation_thread();                                                       //计算线程（是SC_THREAD）   
    void perform_fft(vector<complex<double>>& data);                                 //执行FFT计算的核心函数
    void perform_ifft(vector<complex<double>>& data);                                //执行IFFT计算的核心函数                                           
    void reset();
};

#endif