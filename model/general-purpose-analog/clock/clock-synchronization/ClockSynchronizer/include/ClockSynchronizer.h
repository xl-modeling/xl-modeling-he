#ifndef CLOCKSYNCHRONIZER_H
#define CLOCKSYNCHRONIZER_H

#include <systemc-ams>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>

#include "utils/configurable.h"
#include "utils/traceable.h"
#include "utils/randomvalue.h"
#include "utils/modules/cppll.h"

// ============================================================
// 多通道时钟同步器（参考 ADF4030 的核心功能）
//
//   refin ─┬→ RefinReceiver → RDIV → CPPLL(×NDIV) → lock ─┐
//          │                                              ↓
//          └───────────────────────────────────────────→ Core → bsync[channels]
//
//   - 锁相：REFIN 经过零比较整形、RDIV 分频后送入 CPPLL，CPPLL 给出锁定指示
//           关闭 PLL 时（use_pll = false）视为理想锁定，用于需要粗时间步的仿真
//   - 频率生成：锁定后 fBSYNC = fREFIN × NDIV / (RDIV × ODIV)
//               对 REFIN 上升沿持续做最小二乘直线拟合，通道边沿按拟合出的 REFIN 时间网格换算，
//               等效理想 PLL：输出跟随 REFIN 的平均相位与频率，不传递 REFIN 的逐周期抖动
//   - 上电随机相位：各通道初始相位为 0 ~ ODIV-1 个 VCO 周期
//   - 可调延迟：粗延迟（1/fVCO/8 步进，带符号）+ 细延迟（~1.4 ps 步进，0 ~ fine_max）
//               延迟变化在上升沿后生效，每周期最多改变一个粗延迟步进，避免输出毛刺
//   - TDC：测量通道与参考通道的上升沿时差，平均后噪声 σ = tTDC / √AVG，AVG = 64 × 2^AVGEXP；
//          另有每通道固定的系统误差（均匀分布在 ±tdc_accuracy/2，任意两通道之差不超过 ±tdc_accuracy）
//   - 对齐环路：锁定后依次对齐各通道，目标时差为 TDC_OFFSETx（默认 0），每通道迭代 align_cycles 次
//   - 输出：差分电压，边沿带高斯抖动，线性上升/下降沿；锁定前保持低电平
//
// 实现方式：通道边沿时刻 = 基准时刻 + n × T/2 + 抖动，用 double 精确记录，
// Core 只按精确边沿时刻生成斜坡电压，因此延迟、抖动、时差都不受时间步限制。
// CPPLL 只提供锁定行为，不用于生成通道边沿（其 VCO 边沿量化到时间步）。
//
// 注意：sc_time 的分辨率为 1 ps，会截断亚 ps 部分（如 1.4 ps → 1 ps）。
// 本模型的延迟步进、TDC 噪声、对齐精度都在 ps 及以下，因此这类时间量统一用 double（秒）
//
// 对外提供 name / version / latency / power / area 读取接口（数值来自 json）
//
// 数据来源：ADF4030 数据手册 Rev. A（Analog Devices，doc/adf4030.pdf）
//   - 10 个 BSYNC 通道、REFIN 10~250MHz、PFD 10~20MHz、VCO 2375~2625MHz、电荷泵 360~630µA：Table 1
//   - BSYNC 0.65~200MHz、上升/下降时间 80ps（20%~80%）、占空比 50%、差分摆幅 350mV pk（14mA 驱动）、
//     RMS 抖动 4.3ps、最大对齐误差 ±5ps、细延迟步进 1.4ps、中间延迟步进 50ps、TDC 精度 ±3.6ps：Table 1
//   - TDC 单次时间戳不确定度 tTDC ≈ 86ps、AVG = 64 × 2^AVGEXP、σMEAS = tTDC / √AVG：Table 11 与式 (7)(8)
//   - TDC_OFFSETx 步进 1/fVCO/512 = 0.78125ps：Applications Information
//   - power 取 Configuration 1（REFIN = 100MHz，10 路发送 100MHz，14mA 驱动）典型值 2.3W
//   - area 取 48 引脚 LGA 封装 7mm × 7mm = 49mm²
//   - latency 取一次 TDC 测量时间 AVG / fBSYNC（AVGEXP = 12，fBSYNC = 100MHz 时为 2.62144ms）
//   - RDIV/NDIV/ODIV 取 REFIN = 100MHz 时的一组合法分频（PFD 20MHz，VCO 2.5GHz，BSYNC 100MHz）；
//     PLL 环路滤波器、VCO 增益与锁定判据手册未给出，沿用 xl-modeling ADF4030.perf.json
// ============================================================
SC_MODULE(ClockSynchronizer)
    , public Configurable<ClockSynchronizer>
    , public TraceableSCA
{
public:
    // 参考时钟输入：差分电压，以 0 V 为判决阈值
    sca_tdf::sca_in<double> refin;
    // 输出端口 [channels]：差分电压
    sc_core::sc_vector<sca_tdf::sca_out<double>> bsync;

    CFGS_DECLARE(
        std::string module_name;
        double version;
        double latency_ms;          // 一次 TDC 测量时间 (ms)
        double power_mw;            // 功耗 (mW)
        double area_mm2;            // 面积 (mm²)
        int channels;               // 通道数
        int ref_channel;            // 参考通道
        int rdiv;                   // 参考分频
        int ndiv;                   // 反馈分频
        int odiv;                   // 输出分频
        double fine_step_ps;        // 细延迟步进
        int fine_max;               // 细延迟最大码值
        double tdc_tq_ps;           // TDC 单次测量不确定度
        double tdc_accuracy_ps;     // TDC 系统误差上限（任意两通道之间）
        int avgexp;                 // TDC 平均指数
        int align_cycles;           // 每通道对齐迭代次数
        double jitter_ps;           // 输出抖动 rms
        double tr_tf_ps;            // 上升/下降时间（20% ~ 80%）
        double vod_mv;              // 差分输出幅度（峰值）
        int seed;                   // 随机种子
        double cp_icp_ua;           // PLL 电荷泵电流
        double lpf_c1_pf;           // PLL 环路滤波器 C1
        double lpf_r_ohm;           // PLL 环路滤波器 R
        double lpf_c2_pf;           // PLL 环路滤波器 C2
        double vco_kvco_hz;         // PLL VCO 压控增益 (Hz/V)
        double vco_f0_hz;           // PLL VCO 自由振荡频率
        double lock_phase_rad;      // PLL 锁定相位误差阈值
        int lock_cycles;            // PLL 锁定所需连续达标周期数
    );

    // json 为扁平格式（各字段 {type, value, unit, editable}），不含 meta/parameter/configuration，
    // 因此不使用 CFGS_BEGIN/CFGS_END，直接读取顶层字段
    CFGS_FUNC_DECLARE{
        std::ifstream f(json_path, std::ios::in);
        if (!f.is_open())
            throw std::runtime_error(std::string("Cannot open file: ") + json_path);

        nlohmann::json j = nlohmann::json::parse(f);
        configs cfgs;
        cfgs.module_name     = j.at("name").at("value");
        cfgs.version         = j.at("version").at("value");
        cfgs.latency_ms      = j.at("latency").at("value");
        cfgs.power_mw        = j.at("power").at("value");
        cfgs.area_mm2        = j.at("area").at("value");
        cfgs.channels        = j.at("channels").at("value");
        cfgs.ref_channel     = j.at("ref_channel").at("value");
        cfgs.rdiv            = j.at("rdiv").at("value");
        cfgs.ndiv            = j.at("ndiv").at("value");
        cfgs.odiv            = j.at("odiv").at("value");
        cfgs.fine_step_ps    = j.at("fine_step_ps").at("value");
        cfgs.fine_max        = j.at("fine_max").at("value");
        cfgs.tdc_tq_ps       = j.at("tdc_tq_ps").at("value");
        cfgs.tdc_accuracy_ps = j.at("tdc_accuracy_ps").at("value");
        cfgs.avgexp          = j.at("avgexp").at("value");
        cfgs.align_cycles    = j.at("align_cycles").at("value");
        cfgs.jitter_ps       = j.at("jitter_ps").at("value");
        cfgs.tr_tf_ps        = j.at("tr_tf_ps").at("value");
        cfgs.vod_mv          = j.at("vod_mv").at("value");
        cfgs.seed            = j.at("seed").at("value");
        cfgs.cp_icp_ua       = j.at("cp_icp_ua").at("value");
        cfgs.lpf_c1_pf       = j.at("lpf_c1_pf").at("value");
        cfgs.lpf_r_ohm       = j.at("lpf_r_ohm").at("value");
        cfgs.lpf_c2_pf       = j.at("lpf_c2_pf").at("value");
        cfgs.vco_kvco_hz     = j.at("vco_kvco_hz").at("value");
        cfgs.vco_f0_hz       = j.at("vco_f0_hz").at("value");
        cfgs.lock_phase_rad  = j.at("lock_phase_rad").at("value");
        cfgs.lock_cycles     = j.at("lock_cycles").at("value");
        return cfgs;
    }

    const std::string& get_name() const { return m_cfgs.module_name; }
    double get_version() const { return m_cfgs.version; }
    double get_latency() const { return m_cfgs.latency_ms; }   // ms
    double get_power() const { return m_cfgs.power_mw; }       // mW
    double get_area() const { return m_cfgs.area_mm2; }        // mm²

    // 构造函数
    //   timestep：TDF 时间步，例化时指定。同一集群的激励源必须使用相同的时间步
    //   use_pll ：true 时例化 CPPLL 建模锁相过程（VCO ≈ 2.5 GHz，时间步需 ≤ 20 ps）；
    //             false 时视为理想锁定，可用粗时间步（如对齐仿真）
    ClockSynchronizer(sc_core::sc_module_name nm, const configs& cfgs,
                      sc_core::sc_time timestep, bool use_pll = true)
        : refin("refin")
        , bsync("bsync", cfgs.channels)
        , m_cfgs(cfgs)
        , m_lock("lock")
        , m_core("Core", cfgs, timestep, use_pll)
    {
        m_core.refin(refin);
        m_core.lock(m_lock);
        for (int i = 0; i < cfgs.channels; ++i)
            m_core.bsync[i](bsync[i]);

        if (use_pll){
            m_sq.reset(new sca_tdf::sca_signal<double>("refin_sq"));
            m_pfd_in.reset(new sca_tdf::sca_signal<double>("pfd_in"));
            m_pll_out.reset(new sca_tdf::sca_signal<double>("pll_out"));

            m_rx.reset(new RefinReceiver("RefinReceiver"));
            m_rdiv.reset(new CPPLL::DIV("RDIV", 1.0, 0.0, cfgs.rdiv));
            m_pll.reset(new CPPLL("PLL", 0.5, cfgs.cp_icp_ua * 1e-6,
                                  cfgs.lpf_c1_pf * 1e-12, cfgs.lpf_r_ohm, cfgs.lpf_c2_pf * 1e-12,
                                  cfgs.vco_kvco_hz, cfgs.vco_f0_hz, 1.0, 0.0,
                                  cfgs.ndiv, 1, cfgs.lock_phase_rad, cfgs.lock_cycles));
            m_rx->in(refin);
            m_rx->out(*m_sq);
            m_rdiv->vin(*m_sq);
            m_rdiv->vout(*m_pfd_in);
            m_pll->in(*m_pfd_in);
            m_pll->out(*m_pll_out);
            m_pll->lock(m_lock);
        } else {
            m_ideal.reset(new IdealLock("IdealLock"));
            m_ideal->out(m_lock);
        }
    }

    void trace_me(sca_util::sca_trace_file* tf) override{
        sca_util::sca_trace(tf, refin, refin.name());
        sca_util::sca_trace(tf, m_lock, m_lock.name());
        for (const auto& p : bsync)
            sca_util::sca_trace(tf, p, p.name());
    }

    // ---------------- 查询接口（供验证使用） ----------------

    int channels() const { return m_core.channels(); }
    int ref_channel() const { return m_core.ref_channel(); }

    // 锁定状态：locked() 为当前是否已建立输出基准；lock_time() 为最近一次建立基准的时刻
    bool locked() const { return m_core.locked(); }
    sc_core::sc_time lock_time() const { return m_core.lock_time(); }

    // 以下频率、步进只在 locked() 后有效
    double refin_frequency() const { return m_core.refin_frequency(); }
    double pfd_frequency() const { return m_core.refin_frequency() / m_core.rdiv(); }
    double vco_frequency() const { return m_core.vco_frequency(); }
    double bsync_frequency() const { return m_core.bsync_frequency(); }
    double coarse_step() const { return m_core.coarse_step(); }      // s
    double fine_step() const { return m_core.fine_step(); }          // s
    sc_core::sc_time tdc_time() const { return m_core.tdc_time(); }

    // 对齐是否完成，以及开始、完成时刻
    bool aligned() const { return m_core.aligned(); }
    sc_core::sc_time align_start_time() const { return m_core.align_start_time(); }
    sc_core::sc_time align_done_time() const { return m_core.align_done_time(); }

    // 通道 ch 相对参考通道的上升沿时差（s），折算到 [-T/2, T/2)
    //   ideal_skew：不含抖动，由当前生效的延迟决定
    //   edge_skew ：最近一次实际上升沿（含抖动）
    double ideal_skew(int ch) const { return m_core.ideal_skew(ch); }
    double edge_skew(int ch) const { return m_core.edge_skew(ch); }

    // 手动设置延迟码（用于验证延迟步进）。在之后的上升沿逐步生效
    void set_delay(int ch, long coarse, int fine) { m_core.set_delay(ch, coarse, fine); }
    bool settled(int ch) const { return m_core.settled(ch); }

    // 目标时差（TDC_OFFSETx）：带符号 16 位码值，步进 1/fVCO/512（≈ 0.78125 ps）。
    // 需在该通道开始对齐前设置；对齐后该通道相对参考通道的时差趋近此值
    void set_tdc_offset(int ch, int code) { m_core.set_tdc_offset(ch, code); }
    double tdc_offset(int ch) const { return m_core.tdc_offset(ch); }       // s，锁定后有效
    double tdc_offset_step() const { return m_core.tdc_offset_step(); }     // s，锁定后有效

    // 通道 ch 的 TDC 系统误差（s）：TDC 测得的 ch 相对参考通道时差 = 真实时差 + tdc_error(ch) - tdc_error(ref)
    double tdc_error(int ch) const { return m_core.tdc_error(ch); }

private:
    // ============================================================
    // REFIN 接收器：过零比较，输出 0 / 1 方波（CPPLL::DIV 要求输入为两电平方波）
    // ============================================================
    SCA_TDF_MODULE(RefinReceiver)
    {
        sca_tdf::sca_in<double> in;
        sca_tdf::sca_out<double> out;

        RefinReceiver(sc_core::sc_module_name nm) : in("in"), out("out") {}

        void processing() override{
            out.write(in.read() > 0.0 ? 1.0 : 0.0);
        }
    };

    // ============================================================
    // 理想锁定：关闭 PLL 时恒为锁定
    // ============================================================
    SCA_TDF_MODULE(IdealLock)
    {
        sca_tdf::sca_out<bool> out;

        IdealLock(sc_core::sc_module_name nm) : out("out") {}

        void processing() override{
            out.write(true);
        }
    };

    // ============================================================
    // 通道核心：REFIN 边沿测量、通道边沿生成、TDC 与对齐环路
    // ============================================================
    SCA_TDF_MODULE(Core)
    {
        sca_tdf::sca_in<double> refin;
        sca_tdf::sca_in<bool> lock;
        sc_core::sc_vector<sca_tdf::sca_out<double>> bsync;

        Core(sc_core::sc_module_name nm, const configs& cfgs, sc_core::sc_time timestep, bool use_pll)
            : refin("refin")
            , lock("lock")
            , bsync("bsync", cfgs.channels)
            , m_channels(cfgs.channels)
            , m_ref(cfgs.ref_channel)
            , m_rdiv(cfgs.rdiv)
            , m_ndiv(cfgs.ndiv)
            , m_odiv(cfgs.odiv)
            , m_timestep(timestep)
            , m_use_pll(use_pll)
            , m_fine_step(cfgs.fine_step_ps * 1e-12)
            , m_fine_max(cfgs.fine_max)
            , m_tdc_tq(cfgs.tdc_tq_ps * 1e-12)
            , m_avg(64L << cfgs.avgexp)
            , m_align_cycles(cfgs.align_cycles)
            , m_vod(cfgs.vod_mv * 1e-3)
            , m_ramp(cfgs.tr_tf_ps * 1e-12 / 0.6)    // 线性斜坡总时长：20%~80% 段占 60%
            , m_jitter(0.0, cfgs.jitter_ps * 1e-12, cfgs.seed + 1)
            , m_tdc_noise(0.0, 1.0, cfgs.seed + 2)
            , m_ch(cfgs.channels)
        {
            // 上电随机相位
            UniformRandomValue<int> phase(0, cfgs.odiv - 1, cfgs.seed);
            for (Channel& c : m_ch)
                c.phase_vco = phase();

            // TDC 系统误差：每通道固定，均匀分布在 ±tdc_accuracy/2
            const double half = 0.5 * cfgs.tdc_accuracy_ps * 1e-12;
            UniformRandomValue<double> err(-half, half, cfgs.seed + 3);
            m_tdc_err.resize(cfgs.channels);
            for (double& e : m_tdc_err)
                e = err();
            m_offset_code.assign(cfgs.channels, 0);
        }

        void set_attributes() override{
            set_timestep(m_timestep);
        }

        void processing() override{
            const double t = get_time().to_seconds();

            // 1. REFIN 上升沿（过 0 V，线性插值）
            const double v = refin.read();
            if (m_has_prev && m_vprev <= 0.0 && v > 0.0)
                on_refin_edge(m_tprev + (0.0 - m_vprev) / (v - m_vprev) * (t - m_tprev));
            m_tprev = t;
            m_vprev = v;
            m_has_prev = true;

            // 2. 锁定 / 失锁
            const bool lk = lock.read();
            if (!m_locked){
                if (lk && m_nref >= (m_use_pll ? 2 : IDEAL_REF_EDGES))
                    establish(t);
            } else if (!lk){
                lose_lock();
            }

            // 3. 通道边沿与对齐
            if (m_locked){
                for (Channel& c : m_ch){
                    while (c.next <= t){
                        c.last = c.next;
                        c.has_last = true;
                        c.level = (c.n % 2 == 0);       // 偶数号边沿为上升沿
                        if (c.level){
                            c.last_rise = c.last;
                            step_delay(c);
                        }
                        ++c.n;
                        c.next = base_of(c) + c.n * 0.5 * m_period + m_jitter();
                    }
                }
                step_alignment(t);
            }

            // 4. 输出电压
            for (int i = 0; i < m_channels; ++i)
                bsync[i].write(m_locked ? voltage(m_ch[i], t) : -m_vod);
        }

        int channels() const { return m_channels; }
        int ref_channel() const { return m_ref; }
        int rdiv() const { return m_rdiv; }
        bool locked() const { return m_locked; }
        sc_core::sc_time lock_time() const { return m_lock_time; }
        double refin_frequency() const { return 1.0 / m_tref; }
        double vco_frequency() const { return 1.0 / m_tvco; }
        double bsync_frequency() const { return 1.0 / m_period; }
        double coarse_step() const { return m_tvco / 8.0; }
        double fine_step() const { return m_fine_step; }
        sc_core::sc_time tdc_time() const { return sc_core::sc_time(m_avg * m_period, sc_core::SC_SEC); }
        bool aligned() const { return m_done; }
        sc_core::sc_time align_start_time() const { return sc_core::sc_time(m_align_start, sc_core::SC_SEC); }
        sc_core::sc_time align_done_time() const { return m_done_time; }
        double ideal_skew(int ch) const { return wrap(base_of(m_ch[ch]) - base_of(m_ch[m_ref])); }
        double edge_skew(int ch) const { return wrap(m_ch[ch].last_rise - m_ch[m_ref].last_rise); }

        void set_delay(int ch, long coarse, int fine){
            m_ch[ch].coarse = coarse;
            m_ch[ch].fine = std::max(0, std::min(fine, m_fine_max));
        }

        bool settled(int ch) const { return m_ch[ch].applied == delay_of(m_ch[ch]); }

        void set_tdc_offset(int ch, int code){
            m_offset_code[ch] = std::max(-32768, std::min(code, 32767));
        }
        double tdc_offset_step() const { return m_tvco / 512.0; }
        double tdc_offset(int ch) const { return m_offset_code[ch] * tdc_offset_step(); }
        double tdc_error(int ch) const { return m_tdc_err[ch]; }

    private:
        // 理想锁定时，建立基准前测量的 REFIN 边沿数。锁定后仍持续拟合，这里只决定初始估算的精度
        static constexpr long IDEAL_REF_EDGES = 256;

        struct Channel{
            long phase_vco = 0;         // 上电随机相位（VCO 周期数）
            long coarse = 0;            // 粗延迟码（带符号）
            int fine = 0;               // 细延迟码

            double applied = 0.0;       // 已生效的延迟 (s)
            long n = 0;                 // 下一个边沿的序号
            double next = 0.0;          // 下一个边沿的实际时刻（含抖动）
            double last = 0.0;          // 上一个边沿的实际时刻
            bool has_last = false;
            bool level = false;         // 上一个边沿之后的电平
            double last_rise = -1.0;    // 最近一次上升沿的实际时刻
        };

        // 配置参数
        int m_channels;
        int m_ref;
        int m_rdiv;
        int m_ndiv;
        int m_odiv;
        sc_core::sc_time m_timestep;
        bool m_use_pll;
        double m_fine_step;                 // 细延迟步进 (s)
        int m_fine_max;
        double m_tdc_tq;                    // TDC 单次测量不确定度 (s)
        std::vector<double> m_tdc_err;      // 每通道 TDC 系统误差 (s)
        std::vector<int> m_offset_code;     // 每通道目标时差码（TDC_OFFSETx）
        long m_avg;                         // TDC 平均样本数
        int m_align_cycles;
        double m_vod;                       // 差分输出幅度 (V)
        double m_ramp;                      // 线性斜坡总时长 (s)

        // 随机源
        NormalRandomValue<double> m_jitter;
        NormalRandomValue<double> m_tdc_noise;

        // REFIN 测量：第 k 个上升沿时刻 t_k 的最小二乘拟合 t_k ≈ m_fit_a + k · m_tref
        bool m_has_prev = false;
        double m_tprev = 0.0;
        double m_vprev = 0.0;
        long m_nref = 0;                    // 本次测量的 REFIN 上升沿数
        double m_tfirst = 0.0;              // 本次测量的第一个 REFIN 上升沿（拟合以它为原点，减小数值误差）
        double m_sk = 0.0, m_skk = 0.0;     // Σk、Σk²
        double m_st = 0.0, m_skt = 0.0;     // Σ(t_k - m_tfirst)、Σk·(t_k - m_tfirst)
        double m_fit_a = 0.0;               // 拟合截距（绝对时刻）

        // 时间基准 (s)
        bool m_locked = false;
        sc_core::sc_time m_lock_time;
        double m_tref = 0.0;                // REFIN 周期（拟合值）
        double m_tvco = 0.0;                // VCO 周期
        double m_period = 0.0;              // BSYNC 周期
        double m_k0 = 0.0;                  // 通道边沿起始于第 m_k0 个 REFIN 上升沿
        double m_align_start = 0.0;         // 对齐开始时刻

        // 通道状态
        std::vector<Channel> m_ch;

        // 对齐状态
        enum class State { Measure, Settle, Done };
        State m_state = State::Done;
        int m_target = -1;
        int m_iter = 0;
        double m_meas_end = 0.0;
        bool m_done = false;
        sc_core::sc_time m_done_time;

        // 记录 REFIN 上升沿并更新拟合
        void on_refin_edge(double tc){
            if (m_nref == 0){
                m_tfirst = tc;
                m_sk = m_skk = m_st = m_skt = 0.0;
            }
            const double k = static_cast<double>(m_nref);
            const double x = tc - m_tfirst;
            m_sk += k; m_skk += k * k; m_st += x; m_skt += k * x;
            ++m_nref;
            if (m_nref < 2) return;

            const double n = static_cast<double>(m_nref);
            m_tref = (n * m_skt - m_sk * m_st) / (n * m_skk - m_sk * m_sk);
            m_fit_a = m_tfirst + (m_st - m_tref * m_sk) / n;
            m_tvco = m_tref * m_rdiv / m_ndiv;
            m_period = m_tvco * m_odiv;
        }

        // 第 k0 个 REFIN 上升沿在拟合网格上的时刻
        double grid_t0() const { return m_fit_a + m_k0 * m_tref; }

        // 通道边沿基准时刻 = 起始 REFIN 边沿 + 随机相位 + 已生效延迟
        double base_of(const Channel& c) const{
            return grid_t0() + c.phase_vco * m_tvco + c.applied;
        }

        // 锁定后建立时间基准：通道边沿起始于下一个 REFIN 上升沿
        void establish(double t){
            m_k0 = std::floor((t - m_fit_a) / m_tref) + 1.0;
            const double t0 = grid_t0();

            for (Channel& c : m_ch){
                c.applied = delay_of(c);
                c.n = 0;
                c.next = base_of(c) + m_jitter();
                c.has_last = false;
                c.level = false;
                c.last_rise = -1.0;
            }

            m_locked = true;
            m_lock_time = sc_core::sc_time(t, sc_core::SC_SEC);
            m_align_start = t0;
            m_done = false;
            m_iter = 0;
            m_target = next_target(-1);
            if (m_target >= 0) start_measure(t0);
            else finish(t);
        }

        // 失锁：停止输出与对齐，重新测量 REFIN
        void lose_lock(){
            m_locked = false;
            m_state = State::Done;
            m_done = false;
            m_nref = 0;
        }

        double delay_of(const Channel& c) const{
            return c.coarse * coarse_step() + c.fine * m_fine_step;
        }

        double wrap(double d) const{
            return d - m_period * std::floor(d / m_period + 0.5);
        }

        // 上升沿后让延迟向设定值移动，每周期最多一个粗延迟步进
        void step_delay(Channel& c){
            const double target = delay_of(c);
            const double diff = target - c.applied;
            if (diff == 0.0) return;
            const double cs = coarse_step();
            c.applied = (std::fabs(diff) <= cs) ? target : c.applied + (diff > 0 ? cs : -cs);
        }

        // ---------------- 对齐状态机 ----------------
        int next_target(int cur) const{
            for (int i = cur + 1; i < m_channels; ++i)
                if (i != m_ref) return i;
            return -1;
        }

        void start_measure(double t){
            m_meas_end = t + m_avg * m_period;
            m_state = State::Measure;
        }

        void finish(double t){
            m_state = State::Done;
            m_done = true;
            m_done_time = sc_core::sc_time(t, sc_core::SC_SEC);
        }

        void step_alignment(double t){
            switch (m_state){
            case State::Measure:
                if (t >= m_meas_end){
                    adjust(m_target);
                    m_state = State::Settle;
                }
                break;
            case State::Settle:
                if (!settled(m_target)) break;
                if (++m_iter < m_align_cycles){
                    start_measure(t);
                    break;
                }
                m_iter = 0;
                m_target = next_target(m_target);
                if (m_target < 0) finish(t);
                else start_measure(t);
                break;
            case State::Done:
                break;
            }
        }

        // TDC 测量 + 调整目标通道的延迟码
        void adjust(int ch){
            // TDC：测量值 = 真实时差 + 系统误差 + 平均后的随机噪声
            const double sigma = m_tdc_tq / std::sqrt(static_cast<double>(m_avg));
            const double meas = ideal_skew(ch) + (m_tdc_err[ch] - m_tdc_err[m_ref]) + m_tdc_noise() * sigma;

            // 与目标时差的偏差折算到 [-T/2, T/2)，按最短方向调整
            Channel& c = m_ch[ch];
            const double want = delay_of(c) - wrap(meas - tdc_offset(ch));
            const double cs = coarse_step();
            const long coarse = static_cast<long>(std::floor(want / cs));
            const int fine = static_cast<int>(std::lround((want - coarse * cs) / m_fine_step));
            set_delay(ch, coarse, fine);
        }

        // ---------------- 输出波形 ----------------
        // 线性斜坡，边沿时刻为 50% 点（差分 0 V），斜坡总时长 m_ramp
        double voltage(const Channel& c, double t) const{
            const double half = 0.5 * m_ramp;
            const double vn = c.level ? m_vod : -m_vod;
            if (c.has_last && t - c.last < half){
                const double x = std::min(1.0, 0.5 + (t - c.last) / m_ramp);
                return -vn + 2.0 * vn * x;
            }
            if (c.next - t < half){
                const double x = std::max(0.0, 0.5 - (c.next - t) / m_ramp);
                return vn - 2.0 * vn * x;
            }
            return vn;
        }
    };

    configs m_cfgs;
    sca_tdf::sca_signal<bool> m_lock;
    Core m_core;

    // 锁相环路（use_pll = true）
    std::unique_ptr<sca_tdf::sca_signal<double>> m_sq;
    std::unique_ptr<sca_tdf::sca_signal<double>> m_pfd_in;
    std::unique_ptr<sca_tdf::sca_signal<double>> m_pll_out;
    std::unique_ptr<RefinReceiver> m_rx;
    std::unique_ptr<CPPLL::DIV> m_rdiv;
    std::unique_ptr<CPPLL> m_pll;

    // 理想锁定（use_pll = false）
    std::unique_ptr<IdealLock> m_ideal;
};

#endif // CLOCKSYNCHRONIZER_H
