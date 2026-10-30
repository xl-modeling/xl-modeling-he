#include "fft_chiplet.h"
#include <systemc>
#include <tlm_utils/simple_initiator_socket.h>
#include <iostream>
#include <iomanip>   //提供输入输出格式化功能
#include <cmath>     //提供数学函数

using namespace sc_core;
using namespace tlm;
using namespace std;

class Test_Initiator: public sc_module {                                   //Test_Initiator类，继承自sc_module
public:
    tlm_utils::simple_initiator_socket<Test_Initiator> initiator_socket;   //TLM发起者套接字，用于向FFT模块发送请求
    
    SC_HAS_PROCESS(Test_Initiator);                                        //SystemC宏，声明该模块包含进程 
    
    Test_Initiator(sc_module_name name): sc_module(name), initiator_socket("initiator_socket") {   //Test_Initiator类构造函数，接收模块名，调用基类sc_module的构造函数，初始化TLM套接字，命名为"initiator_socket"
        SC_THREAD(run_test);                                               //注册SystemC线程，run_test将在仿真开始时自动执行
    }
    
    void run_test() {
        //test_fft();    //执行FFT测试
        test_ifft();   //执行IFFT测试
    }
    
private:
    void test_fft() {
        /*
        const int N = 32;
        vector<complex<double>> input(N);
        for (int i = 0; i < N; i++) {   //生成测试信号：100 * cos(2πi/N)
            input[i] = 100.0 * complex<double>(cos(2 * M_PI * i / N), 0);
        }
        */
        const int N = 8;
        vector<complex<double>> input(N);
        for (int i = 0; i < N; i++) {   //生成测试信号：[1,2,3,4,5,6,7,8]
            input[i] = i + 1;
        }

        //写输入数据
        write_data(FFTChiplet::REG_DATA_IN, reinterpret_cast<unsigned char*>(input.data()), N * sizeof(complex<double>));

        //写寄存器
        write_register(FFTChiplet::REG_CONTROL, FFTChiplet::CONTROL_START);

        //等待完成
        wait_for_completion();
        
        //读取结果
        vector<complex<double>> output(N); 
        read_data(FFTChiplet::REG_DATA_OUT, reinterpret_cast<unsigned char*>(output.data()), N * sizeof(complex<double>));
        
        //打印结果
        cout << "Input:" << endl;
        for (int i = 0; i < N; i++) {
            cout << "  " << input[i] << endl;
        }
        
        cout << "FFT Output:" << endl;
        for (int i = 0; i < N; i++) {
            cout << "  " << output[i] << endl;
        }
    }

    
    void test_ifft() {
        const int N = 8;
        vector<complex<double>> input(N);
        input[0] = {36, 0};
        input[1] = {-4, 9.65685};
        input[2] = {-4, 4};
        input[3] = {-4, 1.65685};
        input[4] = {-4, 0};
        input[5] = {-4, -1.65685};
        input[6] = {-4, -4};
        input[7] = {-4, -9.65685};
        //写输入数据
        write_data(FFTChiplet::REG_DATA_IN, reinterpret_cast<unsigned char*>(input.data()), N * sizeof(complex<double>));

        //写寄存器
        write_register(FFTChiplet::REG_CONTROL, FFTChiplet::CONTROL_START);

        //等待完成
        wait_for_completion();
        
        //读取结果
        vector<complex<double>> output(N); 
        read_data(FFTChiplet::REG_DATA_OUT, reinterpret_cast<unsigned char*>(output.data()), N * sizeof(complex<double>));
        
        //打印结果
        cout << "Input:" << endl;
        for (int i = 0; i < N; i++) {
            cout << "  " << input[i] << endl;
        }
        
        cout << "IFFT Output:" << endl;
        for (int i = 0; i < N; i++) {
            cout << "  " << output[i] << endl;
        }
    }
    
    
    void write_data(uint32_t addr, unsigned char* data, unsigned int length) {
        tlm_generic_payload trans;
        trans.set_command(TLM_WRITE_COMMAND);
        trans.set_address(addr);
        trans.set_data_ptr(data);
        trans.set_data_length(length);
        trans.set_streaming_width(length);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);
        
        sc_time delay = SC_ZERO_TIME;
        initiator_socket->b_transport(trans, delay);
        
        if (trans.get_response_status() != TLM_OK_RESPONSE) {
            cerr << "Error writing data to address 0x" << hex << addr << endl;
        }
    }
    
    
    void write_register(uint32_t addr, uint32_t data) {
        tlm_generic_payload trans;
        trans.set_command(TLM_WRITE_COMMAND);
        trans.set_address(addr);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&data));
        trans.set_data_length(sizeof(data));
        trans.set_streaming_width(sizeof(data));
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);
        
        sc_time delay = SC_ZERO_TIME;
        initiator_socket->b_transport(trans, delay);
        
        if (trans.get_response_status() != TLM_OK_RESPONSE) {
            cerr << "Error writing register to address 0x" << hex << addr << endl;
        }
    }
    
    
    void read_data(uint32_t addr, unsigned char* data, unsigned int length) {
        tlm_generic_payload trans;
        trans.set_command(TLM_READ_COMMAND);
        trans.set_address(addr);
        trans.set_data_ptr(data);
        trans.set_data_length(length);
        trans.set_streaming_width(length);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);
        
        sc_time delay = SC_ZERO_TIME;
        initiator_socket->b_transport(trans, delay);
        
        if (trans.get_response_status() != TLM_OK_RESPONSE) {
            cerr << "Error reading data from address 0x" << hex << addr << endl;
        }
    }
    
    
    void wait_for_completion() {   //读取状态寄存器
        uint32_t status;
        do {
            tlm_generic_payload trans;
            trans.set_command(TLM_READ_COMMAND);
            trans.set_address(FFTChiplet::REG_STATUS);
            trans.set_data_ptr(reinterpret_cast<unsigned char*>(&status));
            trans.set_data_length(sizeof(status));
            trans.set_streaming_width(sizeof(status));
            trans.set_byte_enable_ptr(nullptr);
            trans.set_dmi_allowed(false);
            
            sc_time delay = SC_ZERO_TIME;
            initiator_socket->b_transport(trans, delay);
            
            if (trans.get_response_status() != TLM_OK_RESPONSE) {
                cerr << "Error reading status register from address 0x" << endl;
                return;
            }
            
            wait(10, SC_NS);       //等待10ns再读寄存器
        } 
        while (status != FFTChiplet::STATUS_DONE);
    }
};


int sc_main(int argc, char* argv[]) {
    
    string config_file = (argc > 1) ? argv[1] : "../../json/fft_chiplet.json";
    
    //例化FFTChiplet和Test_Initiator
    FFTChiplet fft_chiplet("fft_chiplet", config_file);
    Test_Initiator test_Initiator("Test_Initiator");
    
    //连接套接字
    test_Initiator.initiator_socket.bind(fft_chiplet.target_socket);

    string name = fft_chiplet.get_fft_name(config_file);
    double version = fft_chiplet.get_fft_version(config_file);
    double fft_latency = fft_chiplet.get_fft_latency(config_file);
    double fft_area = fft_chiplet.get_fft_area(config_file);
    double fft_power = fft_chiplet.get_fft_power(config_file);

    //开始仿真
    cout << "FFTChiplet simulation begins at " << sc_time_stamp()  << endl;
    sc_start();
    
    cout << "FFTChiplet simulation completes at " << sc_time_stamp() << endl;

    return 0;
}