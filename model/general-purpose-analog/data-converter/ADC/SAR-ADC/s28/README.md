# S28 SystemC ADC 模型与场景十自测试

模型名称：S28；类别：通用模拟类 ADC；模型版本：0.1。模型条件为 6 bit、100 MS/s、AVDD/DVDD=1 V、共模=0.5 V、CDAC 参考=1 V，对应场景 02-10-1。

## 测试入口

在 Ubuntu/WSL 中执行：

```bash
python3 test/run_s28.py --check-only
python3 test/run_s28.py --all
```
单独运行某一项时，直接调用与测试大纲一致的编号脚本。例如测试1使用：

```bash
bash test/ADC-001-10-1-1.sh
```


场景十架构功能仿真模型使用四个相互独立的测试实例：

| 实例 | 脚本 | 激励与判据 | 主要输出 |
|---|---|---|---|
| ADC-001-10-1-1 | test/ADC-001-10-1-1.sh | -1.10 V～+1.10 V 差分斜坡，提取码0→1与码62→63跃迁并计算平均 LSB | 平均 LSB、跃迁电压、VCD |
| ADC-001-10-1-2 | test/ADC-001-10-1-2.sh | -1.10 V～+1.10 V 差分斜坡，提取 V1、V63 并检查饱和 | 实际量程、64码覆盖、直方图、VCD |
| ADC-001-10-1-3 | test/ADC-001-10-1-3.sh | 0.95 V、k=147、M=16384 的差分正弦码密度 | DNL、INL 曲线及极值、VCD |
| ADC-001-10-1-4 | test/ADC-001-10-1-4.sh | k=147、M=16384 的相干正弦输入与 FFT | FFT 频谱、SNR、THD、ENOB、VCD |

分辨率测试采用 -1.10 V～+1.10 V 差分慢速斜坡，提取码0→1和码62→63的跃迁电压，平均 LSB 按两点电压差除以62计算，合格范围为 28.125～34.375 mV/LSB。量程测试采用同范围斜坡，提取 V1 和 V63，实际量程需覆盖 -0.9VRP～+0.9VRP，并检查范围外输出饱和。

测试3采用正弦码密度分析，测试4采用相干FFT分析。输入配置为 fs=100 MS/s、M=16384、k=147、差分峰值0.95 V；动态分析采用矩形窗，带宽为0～50 MHz，谐波阶数为2～10阶。四项记录分别保留对应的输入、输出、指标和VCD。

测试平台记录 IP、IN、采样时钟、VCM、数值码、D1OUT～D6OUT 和 CLKOUT1。编号脚本默认开启 VCD。每次运行生成独立结果目录，默认位置为 `/mnt/d/systemc/results/s28/run_年月日_时分秒_微秒/`，终端末行输出 `RESULT_FOLDER`。

## 目录

- doc：数据手册、模型说明、测试说明、交付清单和自测试结果
- include：SystemC-AMS 头文件及 JSON 解析头文件
- json：模型参数和场景十测试配置
- source：SystemC-AMS 模型实现
- test：CMake 测试平台、四个测试入口和指标计算

模型使用 SystemC 2.3.4、SystemC-AMS 2.3.4 和 C++17。

## MATLAB后处理

报告中的四项曲线由 MATLAB R2024b 读取实际采样数据计算并导出。进入工程根目录后执行：

```matlab
addpath('test');
postprocess_s28('doc/matlab/data','doc/matlab/results');
```

原始采样记录位于 `doc/matlab/data`，计算结果及执行记录位于 `doc/matlab/results`，报告图片位于 `doc/figures`，波形文件位于 `doc/vcd`。静态线性度按实际观测到的有效跃迁进行端点校正，具体统计范围见报告及 MATLAB 数据说明。

构建目录按工程绝对路径自动区分；若指定的目录中留有其他工程的CMake缓存，脚本会选用独立目录，保留原缓存。编号入口和Python总入口使用同一规则。

测试3的C++统计与MATLAB报告统一采用观测到的首末有效跃迁作为端点；本次有效码为2～61、跃迁为2～62。该调整仅涉及采样后的指标计算。
