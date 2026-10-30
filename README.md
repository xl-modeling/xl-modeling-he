# xl-modeling-he

xl 建模仓库，模型结构与he老师团队对其

## 环境要求

- C++ 版本：**C++17**，不得使用超出C++17范围的标准库功能和关键字
- SystemC 与 SystemC-AMS 库，版本均为 2.3.4，且使用 C++17 编译
- 可选：Boost 库

除以上列出的库外，不得有其他外部依赖

## 目录说明
- `model/` 按照分级分类组织，最底层放模型
- `doc/` 文档说明。例如：he老师团队的建模要求和例子
- `scripts/` 放脚本。例如：`mmkdir.sh` 可以用来创建符合**SystemC建模需求**的目录结构
- `template/` 放模板。例如：`CMakeLists.txt` 通用的 SystemC/SystemC-AMS CMake构建脚本 
