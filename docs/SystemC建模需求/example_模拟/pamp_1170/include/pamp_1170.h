#ifndef PAMP_1170_H
#define PAMP_1170_H

#include <systemc>
#include <systemc-ams>
#include <cmath>
#include <fstream>
#include <random>
#include "nlohmann/json.hpp"

using namespace sc_core;
using namespace sca_tdf;
using namespace sca_util;
using json = nlohmann::json;

SCA_TDF_MODULE(PowerAmplifier_MAAP011170) {
public:
    // 实信号端口（时域电压）
    sca_tdf::sca_in<double> pa_in;
    sca_tdf::sca_out<double> pa_out;

    SCA_CTOR(PowerAmplifier_MAAP011170);

    void set_attributes() ;
    void processing() ;

    // 辅助接口
    void get_module_name();
    void get_version();
    void get_area();
    void get_power_dissipation();
    void get_latency();

private:
    void load_config(const std::string& config_file);
    double calculate_gain_db(double freq_hz) const;
    double apply_amplitude_compression(double input_amplitude, double linear_gain) const;

    // 配置参数
    std::string module_name;
    double size_x, size_y;
    double version;
    double signal_latency_s;
    double power_diss_w;

    double current_freq_hz;      // 当前工作频率 (Hz)
    double impedance;            // 匹配阻抗 (Ω)
    double p1db_dbm;             // 1dB压缩点输出功率 (dBm)
    double psat_com;             // 饱和输出功率相对于p1db_dbm补偿值 (dBm)         
    double rapp_smoothness;      // Rapp 模型平滑因子

    bool enable_additive_noise;  // 是否使能加性噪声
    double noise_rms_voltage;    // 输出端加性噪声 RMS 电压 (V)

    // 随机数生成器（用于加性噪声）
    std::default_random_engine rng;
    std::normal_distribution<double> noise_dist;
};

#endif