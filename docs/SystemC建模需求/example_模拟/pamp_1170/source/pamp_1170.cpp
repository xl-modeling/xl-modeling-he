#include "pamp_1170.h"
#include <iostream>
#include <fstream>
#include <cmath>

PowerAmplifier_MAAP011170::PowerAmplifier_MAAP011170(sc_module_name name)
    : sca_tdf::sca_module(name),
      rng(std::random_device{}()),
      noise_dist(0.0, 0.0)
{
    load_config("../../json/pamp_1170.json");
    if (enable_additive_noise) {
        noise_dist = std::normal_distribution<double>(0.0, noise_rms_voltage);
    }
}

void PowerAmplifier_MAAP011170::load_config(const std::string& config_file) {
    try {
        std::ifstream f(config_file);
        if (!f.is_open()) {
            std::cerr << "[MAAP-011170] 无法打开配置文件: " << config_file << std::endl;
            return;
        }
        json data = json::parse(f);

        // 模块基本信息
        module_name = data["name"].value("value","MAAP-011170 37-40 GHz GaAs pHEMT Power Amplifier");
        version = data["version"].value("value", 1.0);
        size_x = data["dimensions"].value("x", 7.0);
        size_y = data["dimensions"].value("y", 7.0);
        signal_latency_s = data["latency"].value("value", 0.0);
        power_diss_w = data["power"].value("value", 4.0);

        // 放大器电参数
        current_freq_hz = data["current_freq_hz"].value("value", 38.5e9);
        impedance = data["matched_resistance"].value("value", 50.0);
        p1db_dbm = data["p1db_dbm"].value("value", 27.0);
        psat_com = data["psat_compensate"].value("value", 1.5);   // 注意 JSON 键为 "psat_compensate"
        rapp_smoothness = data["rapp_smooth_factor"].value("value", 2.2);

        enable_additive_noise = data["enable_additive_noise"].value("value", false);
        noise_rms_voltage = data["noise_rms_voltage"].value("value", 0.001);

    } catch (const std::exception& e) {
        std::cerr << "[MAAP-011170] 配置解析错误: " << e.what() << std::endl;
    }
}

void PowerAmplifier_MAAP011170::set_attributes() {
    // 最高工作频率 40 GHz，采样步长应 ≤ 1/(2*40e9) = 12.5 ps
    set_timestep(1, SC_PS);
    pa_in.set_rate(1);
    pa_out.set_rate(1);
}

// 频率响应：37-40 GHz 内增益平坦
double PowerAmplifier_MAAP011170::calculate_gain_db(double freq_hz) const {
    double freq_ghz = freq_hz / 1e9;
    double current_gain = 29;
    if (freq_ghz < 37.0) {
        return 29.0;
    } else if(freq_ghz < 38.0){ 		
        current_gain= -0.800000000000981*std::pow(freq_ghz,2) +
                      57.2000000000742*std::pow(freq_ghz,1) +
                      -992.100000001403;
        return current_gain;
    } else if(freq_ghz < 38.5){
        current_gain= 26.3+ (freq_ghz-38.0)/0.5*0.7;
        return current_gain;
    }else if(freq_ghz < 39){
        current_gain= 27+ (freq_ghz-38.5)/0.5*2.4;
        return current_gain;
    }else if(freq_ghz < 39.5){
        current_gain= 29.4 + (freq_ghz-39.5)/0.5*0.3;
        return current_gain;
    }else if(freq_ghz < 40) {
        current_gain= 29.1+(freq_ghz-40)/0.5*2.3;
        return current_gain;
    }else {
        return 26.8;
    }
}

// Rapp 模型 AM-AM 压缩
double PowerAmplifier_MAAP011170::apply_amplitude_compression(double input_amplitude, double linear_gain) const {
    if (input_amplitude <= 0.0) return 0.0;

    // 饱和电压（对应 Psat）
    double psat_dbm = p1db_dbm + psat_com;
    double psat_watt = std::pow(10.0, (psat_dbm - 30.0) / 10.0);
    double vsat = std::sqrt(2.0 * impedance * psat_watt);  // 峰值电压

    double v_out_lin = input_amplitude * linear_gain;
    double ratio = v_out_lin / vsat;
    double denom = std::pow(1.0 + std::pow(ratio, 2.0 * rapp_smoothness),
                            1.0 / (2.0 * rapp_smoothness));
    double v_out = v_out_lin / denom;
    return v_out;
}

void PowerAmplifier_MAAP011170::processing() {
    double input_voltage = pa_in.read();
    double input_amplitude = std::abs(input_voltage);

    // 当前频率下的线性增益
    double gain_db = calculate_gain_db(current_freq_hz);
    double gain_linear = std::pow(10.0, gain_db / 20.0);

    // 幅度压缩
    double output_amplitude = apply_amplitude_compression(input_amplitude, gain_linear);

    // 保持符号（相位不变，实信号下符号代表极性）
    double output_voltage = (input_voltage >= 0 ? output_amplitude : -output_amplitude);

    // 加性噪声（可选）
    if (enable_additive_noise) {
        output_voltage += noise_dist(rng);
    }

    pa_out.write(output_voltage);
}

// 辅助接口实现
void PowerAmplifier_MAAP011170::get_module_name() {
    std::cout << "模块名称: " << module_name << std::endl;
}
void PowerAmplifier_MAAP011170::get_version() {
    std::cout << "版本号: " << version << std::endl;
}
void PowerAmplifier_MAAP011170::get_area() {
    std::cout << "尺寸: " << size_x << " x " << size_y << " mm" << std::endl;
}
void PowerAmplifier_MAAP011170::get_power_dissipation() {
    std::cout << "静态功耗: " << power_diss_w << " W" << std::endl;
}
void PowerAmplifier_MAAP011170::get_latency() {
    std::cout << "信号延迟: " << signal_latency_s << " s" << std::endl;
}