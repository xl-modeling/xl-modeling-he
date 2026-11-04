/**
 * @file configurable.h
 * @brief 定义Configurable类和相关宏，提供统一从 perf.json 中的配置方式
 * 
 * 用法示例：
 * SC_MODULE(Example)
 *      , Configurable<Example> {
 * public:
 *      CFGS_DECLARE(
 *          std::string meta_value;
 *          int parameter_value;
 *          double configuartion_value;
 *      );
 *      CFGS_FUNC_DECLARE{
 *          CFGS_BEGIN(cfgs)
 *              cfgs.meta_value = meta["value"];
 *              cfgs.parameter_value = parameter["value"];
 *              cfgs.configuartion_value = configuration["value"];  
 *          CFGS_END(cfgs) 
 *      }
 *      ... 
 * };
 * 
 * @author jcl
 * @date 2026-09-18
 */

#ifndef CONFIGABLE_H
#define CONFIGABLE_H

#include "nlohmann/json.hpp"

#include <fstream>
#include <type_traits>

/**
 * @brief CFGS_DECLARE宏：声明配置结构体
 * 
 * 在类中声明一个struct configs结构体
 */
#define CFGS_DECLARE(...)   \
    struct configs{         \
        __VA_ARGS__         \
    }           

/**
 * @brief CFGS_FUNC_DECLARE宏：声明静态成员配置函数
 * 
 * 在类中声明一个返回值为configs，静态成员函数get_configs_from_json
 */

#define CFGS_FUNC_DECLARE       \
    static configs get_configs_from_json(const char* json_path) 

/**
 * @brief CFGS_BEGIN 宏：配置函数
 * 
 * 宏简化了读取json文件的过程，并预先提取了 perf.json 中的三个字段：
 *   - meta: 包含元数据的字段
 *   - parameter: 包含参数的字段
 *   - configuration: 包含配置信息的字段
 * 
 * @note 需要在 CFGS_FUC_DECLARE 中使用，与 CFGS_END 配对
 */

#define CFGS_BEGIN(ret)                 \
    do{                                 \
        using json = nlohmann::json;    \
        configs ret;                    \
        std::ifstream f;                \
        json j;                         \
                                        \
        f.open(json_path, std::ios::in);        \
        if (!f.is_open()){                      \
            std::string err = std::string("Cannot open file: ") + json_path;  \
            throw std::runtime_error(err);      \
        }                                       \
                                                \
        j = json::parse(f);                     \
        const json& meta = j.at("meta");                        \
        const json& parameter = j.at("parameter");              \
        const json& configuration = j.at("configuration");      \
        (void) meta;                                            \
        (void) parameter;                                       \
        (void) configuration;                                   \

/**
 * @brief CFGS_END 宏：声明配置结束
 * @note 需要在 CFGS_FUC_DECLARE 中使用，与 CFGS_BEGIN 配对
 */

#define CFGS_END(ret)           \
        return ret;             \
    }while(0);


template <typename ...>
using void_t = void;

template <typename T, typename = void>
struct has_impl : std::false_type {};

template <typename T>
struct has_impl<T, void_t<decltype(T::get_configs_from_json(nullptr))>> : std::true_type {};


/**
 * @class Configurable
 * @brief 统一模型配置接口
 *
 * 继承Configurable后，将强制子类实现静态成员函数 `get_configs_from_json(const char*)`，否则编译不通过
 */

template<typename Derived>
class Configurable {
public:
    Configurable(){ static_assert(has_impl<Derived>::value == true, "Derived class must implement get_configs_from_json"); }
private:
    auto get_configs_impl(const char* s) const {
        return Derived::get_configs_from_json(s);
    }
};

#endif 