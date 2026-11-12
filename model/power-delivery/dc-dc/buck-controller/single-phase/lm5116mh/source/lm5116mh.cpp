// Revision 2026-10-30-r1: see JSON revision_validation for source-based fixes,
// numerical regression evidence and uncalibrated physical quantities.
// =============================================================================
// LM5116MH SystemC-AMS TDF 行为级模型
// 控制方法: 仿真峰值电流模式 (emulated peak current mode)
// 移植自 Verilog-A: veriloga.va  (TI LM5116, SNVS499I)
//
// 端口/参数映射见 json/lm5116mh.json 与 include/lm5116mh.h。
//
// 关键实现说明（与 Verilog-A 的对应关系）:
//   @(initial_step)            -> 构造 / 首个周期初始化
//   @(timer(0, tsw))           -> 用 floor(t/tsw) 检测周期起点，做采样保持 + 打嗝计数
//   idt(integ_in, 0)           -> 反步进欧拉积分  integ += integ_in * dt
//   transition(v, 0, T_EDGE)   -> slew-limit（一阶限摆率）输出
//   SW 为只读监视端口（VA 中未驱动）
// =============================================================================
#include "lm5116mh.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace {

// 限幅辅助
inline double clamp_val(double x, double lo, double hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

}  // namespace

LM5116MH::LM5116MH(sc_core::sc_module_name name, const std::string& config_file)
    : sca_tdf::sca_module(name)
    , tsw_(5.0e-6)
    , vsh_(0.5), ho_state_(0.0), lo_state_(0.0), ho_pending_(0.0), lo_pending_(0.0)
    , t_on_start_(0.0), t_off_start_(0.0), ho_pend_t_(0.0), cycle_start_(0.0)
    , cl_last_(0.0), cl_count_(0.0), hiccup_state_(0.0), hiccup_end_(0.0)
    , uvlo_ok_prev_(0.0), tsd_ok_prev_(0.0), supply_ok_prev_(0.0), en_ok_prev_(0.0)
    , rt_sync_prev_(0.0), sync_last_t_(-1.0)
    , lo_zc_latch_(0.0)
    , integ_(0.0), comp_disp_(0.0), ho_disp_(0.0), lo_disp_(0.0), vcc_disp_(0.0)
    , ibias_a_(0.0), istby_a_(0.0), ho_drive_i_(0.0), lo_drive_i_(0.0)
    , ea_aol_db_(0.0), ea_gbw_hz_(0.0)
    , cycle_prev_(-1)
    , first_(true)
{
    load_config(config_file);
    tsw_ = 1.0 / FSW_;
}

double LM5116MH::p(const std::string& key) const {
    auto it = cfg_.find(key);
    if (it == cfg_.end()) throw std::invalid_argument("Missing parameter: "+key);
    auto vit = it->find("value");
    if (vit == it->end()) throw std::invalid_argument("Missing value: "+key);
    return vit->get<double>();
}

double LM5116MH::get_ea_gain_db(double freq_hz) const {
    const double a0 = std::pow(10.0, ea_aol_db_ / 20.0);
    const double fp = (a0 > 0.0) ? (ea_gbw_hz_ / a0) : 0.0;
    if (fp <= 0.0 || freq_hz <= 0.0) return ea_aol_db_;
    return 20.0 * std::log10(a0 / std::sqrt(1.0 + (freq_hz / fp) * (freq_hz / fp)));
}

void LM5116MH::load_config(const std::string& config_file) {
    try {
        // 与 LDO 参考交付同风格：纯 std::ifstream(std::string) 打开配置，
        // 不依赖 std::filesystem，Ubuntu 18.04 / GCC 7.5 下 C++17 可直接编译。
        // （Windows/MinGW 下中文路径由测试台负责转换为 ASCII 相对路径后传入。）
        std::ifstream f(config_file);
        if (!f.is_open()) {
            throw std::runtime_error("Cannot open LM5116MH config: "+config_file);
        }
        f >> cfg_;

        module_name_ = cfg_["name"].value("value", "lm5116mh");
        version_     = cfg_["version"].value("value", 0.1);
        size_x_      = cfg_["dimensions"].value("x", 6.5);
        size_y_      = cfg_["dimensions"].value("y", 4.4);
        lat_         = cfg_["switching_period_reference_s"].at("value").get<double>();
        pow_         = cfg_["power"].value("value", 0.038);
        dt_s_        = cfg_["dt_s"].value("value", 1.0e-8);

        FSW_        = p("fsw");         RT_OHM_     = p("rt_ohm");
        // LM5116 数据手册给出 RT=16kΩ -> 200kHz、RT=5kΩ -> 535kHz。
        // 这两个工作点使用规格典型值，其余 RT 值采用数据手册设计方程近似。
        if (RT_OHM_ > 0.0) {
            if (std::abs(RT_OHM_ - 16000.0) < 1.0)      FSW_ = 200.0e3;
            else if (std::abs(RT_OHM_ - 5000.0) < 1.0)  FSW_ = 535.0e3;
            else                                        FSW_ = 1.0 / (RT_OHM_ * 284.0e-12 + 450.0e-9);
        }
        A_CS_       = p("a_cs");
        GM_RAMP_    = p("gm_ramp");     I_RAMP_OS_  = p("i_ramp_os");
        CRAMP_EXT_  = p("cramp_ext");   VCS_OFFSET_ = p("vcs_offset");
        CS_POL_     = p("cs_pol");      VCS_TH_     = p("vcs_th");
        VILIM_REF_  = p("vilim_ref");   DV_RAMP_MIN_= p("dv_ramp_min");
        T_OFF_MIN_  = p("t_off_min");   T_ON_MIN_   = p("t_on_min");
        T_DEAD_HO_  = p("t_dead_ho");   T_DEAD_LO_  = p("t_dead_lo");
        T_EDGE_     = p("t_edge");      VREF_       = p("vref");
        SS_FB_OFFS_ = p("ss_fb_offs");  VCC_REG_    = p("vcc_reg");
        VCC_UVLO_   = p("vcc_uvlo");    VCC_HYST_   = p("vcc_hyst");
        VCCX_SW_    = p("vccx_sw");     UVLO_THR_   = p("uvlo_thr");
        UVLO_HYST_  = p("uvlo_hyst");   EN_ON_      = p("en_on");
        EN_OFF_     = p("en_off");      VIN_MIN_OP_ = p("vin_min_op");
        VIN_MAX_OP_ = p("vin_max_op");  HICCUP_CYCLES_ = p("hiccup_cycles");
        T_HICCUP_OFF_ = p("t_hiccup_off"); TSD_THR_  = p("tsd_thr");
        TSD_HYST_   = p("tsd_hyst");            K_P_        = p("k_p");
        K_I_        = p("k_i");         VDEM_ZC_    = p("vdem_zc");
        VSS_DEM_EN_ = p("vss_dem_en");
        IBIAS_TYP_  = p("ibias_typ");    ISTDBY_MAX_ = p("istdby_max");
        IOHH_       = p("iohh");         IOLH_       = p("iolh");
        IOHL_       = p("iohl");         IOLL_       = p("ioll");
        EA_AOL_DB_  = p("ea_aol_db");    EA_GBW_HZ_  = p("ea_gbw_hz");
        GATE_C_F_   = p("gate_c_f");
        ea_aol_db_  = EA_AOL_DB_;
        ea_gbw_hz_  = EA_GBW_HZ_;
        temp_c_     = p("temp_c");
        for(auto it=cfg_.begin();it!=cfg_.end();++it){
            if(it->is_object()&&it->contains("value")&&(*it)["value"].is_number()){
                if(!std::isfinite((*it)["value"].get<double>()))throw std::invalid_argument("Nonfinite "+it.key());
            }
        }
        for(double x:{dt_s_,FSW_,CRAMP_EXT_,T_EDGE_,VREF_,K_P_,GATE_C_F_,EA_GBW_HZ_})
            if(!std::isfinite(x)||x<=0)throw std::invalid_argument("Positive finite timing/electrical parameter required");
        if(dt_s_>1e-8||FSW_>1e6||FSW_<1e4||T_ON_MIN_<=0||T_OFF_MIN_<=0||
           T_ON_MIN_+T_OFF_MIN_>=1/FSW_||T_DEAD_HO_<0||T_DEAD_LO_<0||
           EN_ON_<=EN_OFF_||VIN_MAX_OP_<=VIN_MIN_OP_||K_I_<0||GATE_C_F_<0)
            throw std::invalid_argument("Invalid LM5116MH configuration limits");
    } catch (const std::exception& e) {
        throw std::invalid_argument(std::string("LM5116MH configuration error: ")+e.what());
    }
}

void LM5116MH::set_attributes() {
    set_timestep(dt_s_, sc_core::SC_SEC);
    // 反馈环中存在控制器->功率级->控制器的回路，对控制器回读端口声明 1 步延迟，
    // 由 sca_signal 提供，确保 TDF 调度无代数环。
    vout.set_delay(1);
    fb.set_delay(1);
    cs.set_delay(1);
    csg.set_delay(1);
    demb.set_delay(1);
    sw.set_delay(1);
    for (auto* port : {&vin, &uvlo, &rt_sync, &en, &ramp, &agnd, &ss, &demb,
                       &csg, &vccx, &hb, &pgnd}) {
        port->set_rate(1);
    }
    comp.set_rate(1); ho.set_rate(1); lo.set_rate(1); vcc.set_rate(1);
}

void LM5116MH::processing() {
    const double t  = get_time().to_seconds();
    const double dt = get_timestep().to_seconds();

    // ---- 读取输入 ----
    const double vin_v  = vin.read();
    const double vout_v = vout.read();
    const double vfb_v  = fb.read();
    const double vss_v  = ss.read();
    const double cs_v   = cs.read();
    const double csg_v  = csg.read();
    const double uvlo_v = uvlo.read();
    const double en_v   = en.read();
    const double vccx_v = vccx.read();
    const double sw_v   = sw.read();
    const double demb_v = demb.read();
    const double rt_sync_v = rt_sync.read();

    // RT/SYNC 同步脉冲高于约 4V；有效同步范围 50kHz~1MHz。
    // TDF 步长不变，仅更新内部周期边界。
    if (rt_sync_prev_ < 4.0 && rt_sync_v >= 4.0) {
        if (sync_last_t_ >= 0.0) {
            const double sync_period = t - sync_last_t_;
            if (sync_period >= 1.0e-6 && sync_period <= 20.0e-6) {
                tsw_ = sync_period;
                FSW_ = 1.0 / sync_period;
            }
        }
        sync_last_t_ = t;
    }
    rt_sync_prev_ = rt_sync_v;

    // =====================================================================
    // 1. 供电 / 使能 / UVLO / 过温 判定
    // =====================================================================
    // UVLO 比较器（带迟滞）
    double uvlo_ok;
    if (uvlo_v >= UVLO_THR_)            uvlo_ok = 1.0;
    else if (uvlo_v >= UVLO_THR_ - UVLO_HYST_) uvlo_ok = uvlo_ok_prev_;
    else                                 uvlo_ok = 0.0;
    uvlo_ok_prev_ = uvlo_ok;

    // 过温保护（带迟滞）；temp_c 来自 json（模型无温度端口）
    double tsd_ok;
    if (temp_c_ > TSD_THR_)                        tsd_ok = 0.0;
    else if (temp_c_ < TSD_THR_ - TSD_HYST_)       tsd_ok = 1.0;
    else                                           tsd_ok = tsd_ok_prev_;
    tsd_ok_prev_ = tsd_ok;

    // EN 比较器：0.5V 以下关闭，3.3V 以上开启，中间区间保持状态。
    double en_ok;
    if (en_v >= EN_ON_)              en_ok = 1.0;
    else if (en_v <= EN_OFF_)        en_ok = 0.0;
    else                             en_ok = en_ok_prev_;
    en_ok_prev_ = en_ok;

    // VCC 稳压器 / VCCX 切换
    double vcc_v;
    if (vccx_v > VCCX_SW_)              vcc_v = vccx_v;
    else if (vin_v < 10.6)              vcc_v = vin_v;   // 10.6V 以下 LDO 直通
    else                                vcc_v = VCC_REG_;

    // VCC UVLO（带迟滞）
    double supply_ok;
    if (vcc_v >= VCC_UVLO_)             supply_ok = 1.0;
    else if (vcc_v >= VCC_UVLO_ - VCC_HYST_) supply_ok = supply_ok_prev_;
    else                                 supply_ok = 0.0;
    supply_ok_prev_ = supply_ok;

    double run_ok = (supply_ok > 0.5 && en_ok > 0.5 && uvlo_ok > 0.5 && tsd_ok > 0.5)
                    ? 1.0 : 0.0;
    if (!(vin_v >= VIN_MIN_OP_ && vin_v <= VIN_MAX_OP_)) run_ok = 0.0;
    if (hiccup_state_ > 0.5) run_ok = 0.0;

    // 内部电流观测量：这些是数据手册测试条件下的芯片内部量，不新增物理引脚。
    ibias_a_  = (run_ok > 0.5) ? IBIAS_TYP_ : 0.0;
    istby_a_  = (en_ok < 0.5) ? ISTDBY_MAX_ : 0.0;

    // =====================================================================
    // 2. 周期起点 (@timer(0, tsw))：采样保持 + 打嗝计数
    // =====================================================================
    const long cycle_now = static_cast<long>(t / tsw_ + 1e-9);
    if (first_ || cycle_now != cycle_prev_) {
        // CS/CSG 是差分 Kelvin 检测输入；CS_POL 仅修正外部接线极性。
        const double cs_diff = cs_v - csg_v;
        vsh_ = A_CS_ * CS_POL_ * cs_diff + VCS_OFFSET_;

        // 逐周期过流计数 -> 连续 HICCUP_CYCLES 后进入打嗝
        if (cl_last_ > 0.5) cl_count_ += 1.0;
        else                cl_count_ = 0.0;
        if (cl_count_ >= HICCUP_CYCLES_) {
            hiccup_state_ = 1.0;
            hiccup_end_   = t + T_HICCUP_OFF_;
            cl_count_     = 0.0;
            cl_last_      = 0.0;   // 否则陈旧标志会立刻再次触发打嗝
        }
        if (hiccup_state_ > 0.5 && t >= hiccup_end_) {
            hiccup_state_ = 0.0;
        }

        lo_state_    = 0.0;
        lo_pending_  = 0.0;
        lo_zc_latch_ = 0.0;       // 清零本周期二极管仿真锁存

        if (run_ok > 0.5) {
            ho_pending_ = 1.0;
            ho_pend_t_  = t;
            cycle_start_ = t;
        } else {
            ho_pending_ = 0.0;
            ho_state_   = 0.0;
        }
        cycle_prev_ = cycle_now;
        if (first_)     first_ = false;
    }

    // =====================================================================
    // 3. 误差放大器 PI（内部 Type II 补偿，VA 中折叠进 COMP）
    // =====================================================================
    // SS 在 VCC 欠压、UVLO、EN 或过温故障期间被内部放电；解除故障后，
    // 外部 SS 电容重新决定软启动参考。模型把该内部有效电压记为 ss_eff。
    const double ss_eff = (run_ok > 0.5) ? std::max(0.0, vss_v) : 0.0;
    const double vref_target = std::min(VREF_, ss_eff);

    // SS 未达到 1.215V 时限制误差放大器命令；达到基准后释放限制。
    // 手册规定 SS>3V 后进入 SS-to-FB=160mV 的跟踪区，不能用任意增益钳位。
    double ss_ceiling = (ss_eff < VREF_) ? ss_eff : vcc_v;
    if (ss_ceiling > vcc_v) ss_ceiling = vcc_v;

    const double err_raw = vref_target - vfb_v;
    double integ_in = err_raw;
    double vcomp_pi = K_P_ * err_raw + K_I_ * integ_;

    // 抗饱和：当 EA 输出被 VCC 限幅或被 SS 钳位时冻结积分（避免启动期 wind-up 过冲）
    const double limit_hi = (ss_ceiling < vcc_v) ? ss_ceiling : vcc_v;
    if (vcomp_pi > limit_hi) integ_in = (integ_in > 0.0) ? 0.0 : integ_in;
    if (vcomp_pi < 0.0)      integ_in = (integ_in < 0.0) ? 0.0 : integ_in;
    integ_ += integ_in * dt;

    vcomp_pi = K_P_ * err_raw + K_I_ * integ_;
    vcomp_pi = clamp_val(vcomp_pi, 0.0, vcc_v);
    // 软启动期间进一步钳位到 SS 上限，PWM 比较器读取该有效命令
    const double comp_cmd_eff = clamp_val(vcomp_pi, 0.0, ss_ceiling);

    // COMP 输出限摆率（transition(vcomp_pi, 0, T_EDGE)）
    const double slew = 1.0 / T_EDGE_;
    const double maxstep = slew * dt;
    comp_disp_ += clamp_val(comp_cmd_eff - comp_disp_, -maxstep, maxstep);

    // =====================================================================
    // 4. HO 导通（LO->HO 死区后）
    // =====================================================================
    if (ho_pending_ > 0.5 && (t - ho_pend_t_) >= T_DEAD_HO_) {
        ho_state_   = 1.0;
        ho_pending_ = 0.0;
        t_on_start_ = t;
    }

    // =====================================================================
    // 5. 重构电流信号 vs COMP -> PWM 比较器（峰值电流模式核心）
    // =====================================================================
    double dv_ramp = vin_v - vout_v;
    if (dv_ramp < DV_RAMP_MIN_) dv_ramp = DV_RAMP_MIN_;
    double iramp = GM_RAMP_ * dv_ramp + I_RAMP_OS_;
    const double iramp_floor = GM_RAMP_ * DV_RAMP_MIN_ + I_RAMP_OS_;
    if (iramp < iramp_floor) iramp = iramp_floor;

    const double cs_diff = cs_v - csg_v;
    const double vcs_sense = CS_POL_ * cs_diff;  // = IL * RS（允许负电感电流）

    if (ho_state_ > 0.5) {
        const double ton    = t - t_on_start_;
        const double ramp_v = ramp.read();
        const double ramp_component = (ramp_v > 0.0) ? ramp_v : (iramp / CRAMP_EXT_) * ton;
        const double vemul = vsh_ + ramp_component;   // 外部 RAMP 电压或内部行为级斜率

        double comp_cmd = comp_disp_;
        if (comp_cmd < 0.0)     comp_cmd = 0.0;
        if (comp_cmd > vcc_v)   comp_cmd = vcc_v;

        // 结束本导通区间的几种情形
        const bool current_limit = (vemul >= VILIM_REF_) || (vsh_ >= VILIM_REF_);
        if ((vemul >= comp_cmd) || current_limit ||
            (ton >= (tsw_ - T_OFF_MIN_ - T_DEAD_LO_)) || (run_ok < 0.5)) {
            if (ton >= T_ON_MIN_) {            // 仍遵守最小导通时间
                ho_state_   = 0.0;
                t_off_start_ = t;
                lo_pending_ = 1.0;
                cl_last_    = current_limit ? 1.0 : 0.0;
            }
        }
    }

    // =====================================================================
    // 6. LO 导通（HO->LO 死区 + 二极管仿真）
    // =====================================================================
    if (lo_pending_ > 0.5 && ho_state_ < 0.5 &&
        (t - t_off_start_) >= T_DEAD_LO_) {
        if (lo_zc_latch_ < 0.5) lo_state_ = 1.0;
        // 软启动期间由 SW-DEMB 监测低侧源极电压；达到 3V 后解除强制二极管仿真。
        const double sw_demb = sw_v - demb_v;
        // 真实器件以 SW-DEMB 零交越为主；信号流功率级把体二极管钳位到 0V，
        // 因而不会产生实际的负 SW 电压，保留 CS 差分作为该抽象功率级的等效零交越。
        if ((ss_eff < VSS_DEM_EN_) &&
            ((sw_demb < VDEM_ZC_) || (vcs_sense < VDEM_ZC_))) {
            lo_state_    = 0.0;
            lo_zc_latch_ = 1.0;
        }
    }
    if (lo_pending_ < 0.5) lo_state_ = 0.0;

    // =====================================================================
    // 7. 输出驱动（限摆率）
    // =====================================================================
    const double ho_target = (ho_state_ > 0.5) ? vcc_v : 0.0;
    const double lo_target = (lo_state_ > 0.5) ? vcc_v : 0.0;
    const double vcc_target = vcc_v;
    const double gate_c = std::max(GATE_C_F_, 1.0e-15);
    double ho_step = clamp_val(ho_target - ho_disp_, -maxstep, maxstep);
    double lo_step = clamp_val(lo_target - lo_disp_, -maxstep, maxstep);
    ho_drive_i_ = clamp_val(gate_c * ho_step / dt, -IOLH_, IOHH_);
    lo_drive_i_ = clamp_val(gate_c * lo_step / dt, -IOLL_, IOHL_);
    ho_step = ho_drive_i_ * dt / gate_c;
    lo_step = lo_drive_i_ * dt / gate_c;
    ho_disp_ += ho_step;
    lo_disp_ += lo_step;
    vcc_disp_ += clamp_val(vcc_target - vcc_disp_, -maxstep, maxstep);

    comp.write(comp_disp_);
    lo.write(lo_disp_);
    vcc.write(vcc_disp_);
    // HO 是相对 SW 的自举高侧输出：关闭时约等于 SW，打开时约为 SW+VCC。
    ho.write(sw_v + ho_disp_);
    // Bias and idealized gate-driver dissipation. Gate load GATE_C_F is an
    // explicit application assumption, not a silicon capacitance measurement.
    const double gate_loss=.5*std::max(vcc_v,0.)*(std::abs(ho_drive_i_)+std::abs(lo_drive_i_));
    current_power_w_=std::max(vin_v,0.)*std::max(ibias_a_+istby_a_,0.)+gate_loss;
}
