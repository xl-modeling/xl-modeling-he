// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
#pragma once
// =============================================================================
// LM5116MH —— TI LM5116 宽输入范围同步降压控制器
// 控制方法: 仿真峰值电流模式 (emulated peak current mode)
// 参考: LM5116 datasheet SNVS499I; Verilog-A 模型 veriloga.va
//
// 本文件为 SystemC-AMS TDF 行为级模型（"Verilog-C"）。
// 端口映射与 veriloga.va 完全一致（20 端口）。
//   SW 在 VA 中声明为 output 但内部未驱动、仅作开关节点监视，本模型按实际角色
//   建模为 sca_in（monitor）。
//   AGND/PGND 为参考地，信号流模型统一以 0 为地，故仅声明为输入端口不读取。
// =============================================================================
#include <systemc-ams>
#include <nlohmann/json.hpp>
#include <string>
#include <limits>

class LM5116MH : public sca_tdf::sca_module {
public:
    // ---- 输入端口 (16) ----
    sca_tdf::sca_in<double>  vin{"vin"}, uvlo{"uvlo"}, rt_sync{"rt_sync"}, en{"en"},
                            ramp{"ramp"}, agnd{"agnd"}, ss{"ss"}, fb{"fb"},
                            vout{"vout"}, demb{"demb"}, cs{"cs"}, csg{"csg"},
                            vccx{"vccx"}, hb{"hb"}, sw{"sw"}, pgnd{"pgnd"};
    // ---- 输出端口 (4) ----
    sca_tdf::sca_out<double> comp{"comp"}, ho{"ho"}, lo{"lo"}, vcc{"vcc"};

    LM5116MH(sc_core::sc_module_name name, const std::string& config_file);
    void set_attributes() override;
    void processing() override;

    // ---- 可外部读取的元数据接口（建模需求 2.3.1 / 芯粒文件夹说明）----
    std::string get_module_name() const { return module_name_; }
    double      get_version()      const { return version_; }
    double      get_area()         const { return std::numeric_limits<double>::quiet_NaN(); }   // mm^2
    double      get_latency()      const { return std::numeric_limits<double>::quiet_NaN(); }                // s
    double      get_power_dissipation() const { return current_power_w_; }           // W
    double get_package_area_mm2() const { return size_x_*size_y_; }
    double get_timestep_s() const { return dt_s_; }
    double get_reference_power_w() const { return pow_; }
    double      get_ibias() const { return ibias_a_; }
    double      get_shutdown_current() const { return istby_a_; }
    double      get_ho_drive_current() const { return ho_drive_i_; }
    double      get_lo_drive_current() const { return lo_drive_i_; }
    double      get_ea_aol_db() const { return ea_aol_db_; }
    double      get_ea_gbw_hz() const { return ea_gbw_hz_; }
    double      get_ea_gain_db(double freq_hz) const;
    const nlohmann::json& configuration() const { return cfg_; }

private:
    void load_config(const std::string& config_file);
    nlohmann::json cfg_;
    double p(const std::string& key) const;   // 读取 cfg_[key]["value"]

    // ---- 功能配置参数（来自 json）----
    double FSW_, RT_OHM_, A_CS_, GM_RAMP_, I_RAMP_OS_, CRAMP_EXT_, VCS_OFFSET_, CS_POL_,
           VCS_TH_, VILIM_REF_, DV_RAMP_MIN_;
    double T_OFF_MIN_, T_ON_MIN_, T_DEAD_HO_, T_DEAD_LO_, T_EDGE_;
    double VREF_, SS_FB_OFFS_;
    double VCC_REG_, VCC_UVLO_, VCC_HYST_, VCCX_SW_, UVLO_THR_, UVLO_HYST_,
           EN_ON_, EN_OFF_, VIN_MIN_OP_, VIN_MAX_OP_;
    double HICCUP_CYCLES_, T_HICCUP_OFF_, TSD_THR_, TSD_HYST_;
    double K_P_, K_I_;
    double VDEM_ZC_, VSS_DEM_EN_;
    double IBIAS_TYP_, ISTDBY_MAX_, IOHH_, IOLH_, IOHL_, IOLL_;
    double EA_AOL_DB_, EA_GBW_HZ_, GATE_C_F_;
    double temp_c_, dt_s_;

    // ---- 性能参数（给定值）----
    double lat_, pow_, size_x_, size_y_;
    double current_power_w_=0;
    std::string module_name_;
    double version_;

    // ---- 内部状态 ----
    double tsw_;
    double vsh_, ho_state_, lo_state_, ho_pending_, lo_pending_;
    double t_on_start_, t_off_start_, ho_pend_t_, cycle_start_;
    double cl_last_, cl_count_, hiccup_state_, hiccup_end_;
    double uvlo_ok_prev_, tsd_ok_prev_, supply_ok_prev_, en_ok_prev_;
    double rt_sync_prev_, sync_last_t_;
    double lo_zc_latch_;
    double integ_, comp_disp_, ho_disp_, lo_disp_, vcc_disp_;
    double ibias_a_, istby_a_, ho_drive_i_, lo_drive_i_;
    double ea_aol_db_, ea_gbw_hz_;
    long   cycle_prev_;
    bool   first_;
};
