#include "fft_chiplet.h" 
#include <cmath>        //包含数学函数库
#include <iostream>     //包含输入输出流库
#include <cstring>      //包含内存操作函数库
#include <cstdint>


FFTChiplet::FFTChiplet(sc_module_name name, const string& config_file):    //FFTChiplet类构造函数，接收模块名和配置文件路径
                       sc_module(name), target_socket("target_socket") {   //调用基类sc_module的构造函数，初始化TLM套接字，命名为"target_socket"
    
    load_config(config_file);         //从json文件中加载配置信息
    
    input_data.resize(fft_length);    //根据fft_length调整输入数据缓冲区大小
    output_data.resize(fft_length);   //根据fft_length调整输出数据缓冲区大小

    target_socket.register_b_transport(this, &FFTChiplet::b_transport);    //将b_transport方法注册到target_socket，处理传入的TLM事务
    
    SC_THREAD(computation_thread);    //注册SystemC线程，computation_thread将在仿真开始时自动执行
}


FFTChiplet::~FFTChiplet() {           //FFTChiplet类析构函数
    input_data.clear();               //清理输入数据缓冲区资源
    output_data.clear();              //清理输出数据缓冲区资源
}


//加载配置参数
void FFTChiplet::load_config(const string& config_file) {   //加载配置文件
    try {                                                   //异常处理块
        ifstream f(config_file);                            //创建输入文件流对象，打开配置文件
        json config = json::parse(f);                       //解析json文件内容到config对象
     
        ifft_enable = config["ifft_enable"]["value"];       //从json中获取ifft_enable字段
        fft_length = config["fft_length"]["value"];         //从json中获取fft_length字段
        latency = config["latency"]["value"];               //从json中获取latency字段
        
        cout << "Successful loading FFTChiplet configuration" << endl;            //输出成功加载配置的信息    
    } 
    catch (const exception& e) {                            //捕获所有标准异常
        cerr << "Error loading FFTChiplet configuration: " << e.what() << endl;   //输出错误信息，包括异常描述
    }
}


//TLM阻塞传输函数
void FFTChiplet::b_transport(tlm_generic_payload& trans, sc_time& delay) { //TLM-2.0阻塞传输接口，trans：TLM通用载荷对象，包含传输信息；delay：时间延迟引用，用于添加传输延迟
    tlm_command cmd = trans.get_command();           //获取传输命令（读或写）
    uint64_t addr = trans.get_address();             //获取目标地址
    unsigned char* data = trans.get_data_ptr();      //获取数据指针
    unsigned int length = trans.get_data_length();   //获取数据长度
    
    if (cmd == TLM_READ_COMMAND) {                   //如果是读命令
        if (!read_memory(addr, data, length)) {      //尝试读取内存，如果失败则设置地址错误响应
            trans.set_response_status(TLM_ADDRESS_ERROR_RESPONSE);
        } 
        else {                                       //读取成功则设置OK响应                                
            trans.set_response_status(TLM_OK_RESPONSE);
        }
    } 
    else if (cmd == TLM_WRITE_COMMAND) {             //如果是写命令
        if (!write_memory(addr, data, length)) {     //尝试写入内存，如果失败则设置地址错误响应
            trans.set_response_status(TLM_ADDRESS_ERROR_RESPONSE);
        } 
        else {                                       //写入成功则设置OK响应
            trans.set_response_status(TLM_OK_RESPONSE);
        }
    } 
    else {                                           //如果是其他命令（非法命令），设置命令错误响应
        trans.set_response_status(TLM_COMMAND_ERROR_RESPONSE);
    }
}


bool FFTChiplet::write_memory(unsigned int addr, unsigned char* data, unsigned int length) {  
    if (addr == REG_DATA_IN) {                                               
        memcpy(input_data.data(), data, length);  
        data_ready = true;
        return true;     
    }
    /* 错误：该方法的index永远都是0
    if (addr >= REG_DATA_IN && addr < REG_DATA_IN + length) {     //检查地址是否在输入数据寄存器范围内
        unsigned int index = (addr - REG_DATA_IN) / sizeof(complex<double>);           //计算数据在输入缓冲区中的索引
        if (index < input_data.size()) {                                               //检查索引是否有效
            memcpy(&input_data[index], data, sizeof(complex<double>));                 //将数据复制到输入缓冲区
        }
        data_ready = true;
        return true;
    }
    */
    if (addr == REG_CONTROL) {
        uint32_t control;
        memcpy(&control, data, sizeof(uint32_t));
        if (control == CONTROL_START) {
            start_event.notify();        //触发FFT事件       
        }
        if (control == CONTROL_RESET) {
            reset();
        }
        return true;
    }
    return false;
}


bool FFTChiplet::read_memory(unsigned int addr, unsigned char* data, unsigned int length) {  
    if (addr == REG_DATA_OUT) {                                             
        memcpy(data, output_data.data(), length);              
        comp_done = false;
        return true;
    }    
    /*错误：该方法的index永远都是0
    if (addr >= REG_DATA_OUT && addr < REG_DATA_OUT + length) {   //检查地址是否在输出数据寄存器范围内
        unsigned int index = (addr - REG_DATA_OUT) / sizeof(complex<double>);          //计算数据在输出缓冲区中的索引
        if (index < output_data.size()) {                                              //检查索引是否有效
            memcpy(data, &output_data[index], sizeof(complex<double>));                //将输出缓冲区中的数据复制到data
        }
        comp_done = false;
        return true;
    } 
    */
    if (addr == REG_STATUS) {
        uint32_t status = STATUS_IDLE;
        if (data_ready) {
            status = STATUS_READY;
        }
        else if (comp_busy) {
            status = STATUS_BUSY;
        }
        else if (comp_done) {
            status = STATUS_DONE;
        }
        else {
            status = STATUS_IDLE;
        }
        memcpy(data, &status, sizeof(uint32_t));
        return true;
    }    
    return false;
}


//SystemC线程，负责执行FFT计算
void FFTChiplet::computation_thread() {         
    while (true) {
        wait(start_event);               //等待启动信号
        
        if (!data_ready) {
            continue;
        }

        comp_busy = true;
        data_ready = false;

        vector<complex<double>> working_data = input_data;   //将输入缓冲区中的数据复制到工作缓冲区

        if (ifft_enable) {
            perform_ifft(working_data);      //执行IFFT计算
        }
        else {
            perform_fft(working_data);       //执行FFT计算
        }
    
        output_data = working_data;          //将计算结果复制到输出缓冲区

        //wait(sc_time(fft_latency, SC_NS));   //等待指定的延迟时间，模拟计算耗时

        comp_busy = false;
        comp_done = true;
    }
}


//Cooley-Tukey FFT算法实现，data：输入/输出数据向量（原地变换，即结果会覆盖输入数据）
void FFTChiplet::perform_fft(vector<complex<double>>& data) {   

    int n = data.size();                 //获取输入数据的长度（点数）      
    if (n <= 1) return;                  //递归终止条件：如果数据长度小于等于1，直接返回（不需要变换）

    //1. 分治：奇偶分离
    vector<complex<double>> even(n/2);
    vector<complex<double>> odd(n/2);
    for (int i = 0; i < n/2; i++) {
        even[i] = data[2*i];             //偶数索引元素
        odd[i] = data[2*i+1];            //奇数索引元素
    }
    
    //2. 递归求解
    perform_fft(even);                   //对偶数部分递归执行FFT
    perform_fft(odd);                    //对奇数部分递归执行FFT
    
    //3. 合并：蝶形运算
    for (int i = 0; i < n/2; i++) {
        auto t = polar(1.0, -2 * M_PI * i / n) * odd[i];   //计算旋转因子e^(-j*2π*i/n) * odd
        data[i] = even[i] + t;                             //上分支：even + 旋转因子 * odd
        data[i + n/2] = even[i] - t;                       //下分支：even - 旋转因子 * odd
    }
}


//Cooley-Tukey IFFT算法实现，data：输入/输出数据向量（原地变换）
void FFTChiplet::perform_ifft(vector<complex<double>>& data) {   
    int n = data.size();                 //获取输入数据的长度（点数）        
    for (auto& val : data) {           
        val = conj(val);                 //对输入数据取共轭
    }
    perform_fft(data);                   //对共轭后的数据执行正向FFT
    for (auto& val : data) 
        val = conj(val) / double(n);     //再次取共轭并归一化（除以n）
}


void FFTChiplet::reset() {
    data_ready = false;
    comp_busy  = false;
    comp_done  = false;
    fill(input_data.begin(), input_data.end(), complex<double>(0, 0));
    fill(output_data.begin(), output_data.end(), complex<double>(0, 0));
    cout << "FFTChiplet reset at " << sc_time_stamp() << endl;
}

//获取FFT名称信息
string FFTChiplet::get_fft_name(const string& config_file) {       
    try {
        ifstream f(config_file);                           
        json config = json::parse(f);                      
        name = config["name"]["value"];                
        cerr << "Successfull getting FFTChiplet name: " << name << endl;  
        return name;   
    } 
    catch (const exception& e) {                           
        cerr << "Error getting FFTChiplet name: " << e.what() << endl;    
    }
}

//获取FFT版本信息
double FFTChiplet::get_fft_version(const string& config_file) {    
    try {
        ifstream f(config_file);                           
        json config = json::parse(f);                      
        version = config["version"]["value"];            
        cerr << "Successfull getting FFTChiplet version: " << version << endl;   
        return version;   
    } 
    catch (const exception& e) {                           
        cerr << "Error getting FFTChiplet version: " << e.what() << endl;        
    }
}

//获取FFT延迟参数
double FFTChiplet::get_fft_latency(const string& config_file) {   
    try {
        ifstream f(config_file);                           
        json config = json::parse(f);                      
        latency = config["latency"]["value"];  
        cerr << "Successfull getting FFTChiplet latency: " << latency << "ns" << endl;   
        return latency;   
    } 
    catch (const exception& e) {                           
        cerr << "Error getting FFTChiplet latency: " << e.what() << endl;           
    }
}

//获取FFT面积参数
double FFTChiplet::get_fft_area(const string& config_file) {      
    try {
        ifstream f(config_file);                           
        json config = json::parse(f);                      
        area = config["area"]["value"];           
        cerr << "Successfull getting FFTChiplet area: " << area << "mm²" << endl;         
        return area;   
    } 
    catch (const exception& e) {                          
        cerr << "Error getting FFTChiplet area: " << e.what() << endl;              
    }
}

//获取FFT功耗参数
double FFTChiplet::get_fft_power(const string& config_file) {     
    try {
        ifstream f(config_file);                           
        json config = json::parse(f);                      
        power = config["power"]["value"];       
        cerr << "Successfull getting FFTChiplet power: " << power << "mW" << endl;       
        return power;   
    } 
    catch (const exception& e) {                           
        cerr << "Error getting FFTChiplet power: " << e.what() << endl;
    }
}