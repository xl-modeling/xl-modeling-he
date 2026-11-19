#!/usr/bin/env bash
# S28 ADC 架构功能仿真模型（场景 02-10-1）测试入口。
#   ./run.sh                    依次运行 ADC-001-10-1-1～4（一次编译，test/run_s28.py --all）
#   ./run.sh ADC-001-10-1-3     单步运行一个测试用例（等同 bash test/ADC-001-10-1-3.sh；也可写 3 或 02-10-1-3）
#   ./run.sh check              只检查文件、配置和 SystemC 环境，不编译、不仿真
#   ./run.sh all --no-vcd       全部运行但不生成 VCD（单步脚本固定生成 VCD）
# 结果：out/run_<时间戳>/（终端末行 RESULT_FOLDER=...）；构建目录：build/
set -euo pipefail
MODEL_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$MODEL_DIR"
step=${1:-all}
[ "$#" -gt 0 ] && shift
case "$step" in
    all)                       exec "${S28_PYTHON:-python3}" test/run_s28.py --all "$@" ;;
    check)                     exec "${S28_PYTHON:-python3}" test/run_s28.py --check-only "$@" ;;
    [1-4]|02-10-1-[1-4])       exec bash "test/ADC-001-10-1-${step: -1}.sh" "$@" ;;
    ADC-001-10-1-[1-4])        exec bash "test/$step.sh" "$@" ;;
    -h|--help)                 sed -n '2,8p' "$0" ;;
    *) echo "未知步骤：$step（可选 all、check、ADC-001-10-1-1～4）" >&2; exit 2 ;;
esac
