#ifndef VERIFICATION_H
#define VERIFICATION_H

#include <systemc>

#define FMT_HEADER_ONLY
#include "fmt/format.h"
#include "fmt/color.h"
#include "fmt/ranges.h"

#include <type_traits>

inline void check(const char* what, double got, double exp, double tol, const char* unit, size_t& fail_counter, size_t width = 40, size_t precision = 4){
    bool ok = std::fabs(got - exp) <= tol;

    fmt::println("{} {}  {:<{}} {:.{}f} {} (expect {:.{}f} ± {:.{}f})",
        fmt::styled(ok ? "[PASS]" : "[FAIL]", fmt::fg(ok ? fmt::color::green : fmt::color::red)), sc_core::sc_time_stamp().to_string(),
        what, width,
        got, precision,
        unit,
        exp, precision,
        tol, precision
    );

    if (!ok) ++fail_counter;
}


template<typename T, typename Tol>
inline std::enable_if_t<std::is_floating_point<T>::value> check_vec(const char* what, const T* got, const T* exp, size_t size, Tol tol, const char* unit, size_t& fail_counter, size_t width = 40, size_t precision = 4){
    bool ok = true;
    for(size_t i = 0; i < size; i++){
        if (std::fabs(got[i] - exp[i]) > tol) {
            ok = false;
            break;
        }
    }

    fmt::println("{} {}  {:<{}} {:.{}f} {} (expect {:.{}f} ± {:.{}f})",
        fmt::styled(ok ? "[PASS]" : "[FAIL]", fmt::fg(ok ? fmt::color::green : fmt::color::red)), sc_core::sc_time_stamp().to_string(),
        what, width,
        fmt::join(got, got + size, ","), precision,
        unit,
        fmt::join(exp, exp + size, ","), precision,
        tol, precision
    );

    if (!ok) ++fail_counter;
}

template<typename T, typename Tol>
inline std::enable_if_t<std::is_integral<T>::value> check_vec(const char* what, const T* got, const T* exp, size_t size, Tol tol, const char* unit, size_t& fail_counter, size_t width = 40){
    bool ok = true;
    for(size_t i = 0; i < size; i++){
        if (std::abs(got[i] - exp[i]) > tol) {
            ok = false;
            break;
        }
    }

    fmt::println("{} {}  {:<{}} [{}] {} (expect [{}] ± {})",
        fmt::styled(ok ? "[PASS]" : "[FAIL]", fmt::fg(ok ? fmt::color::green : fmt::color::red)), sc_core::sc_time_stamp().to_string(),
        what, width,
        fmt::join(got, got + size, ","), unit,
        fmt::join(exp, exp + size, ","), tol
    );

    if (!ok) ++fail_counter;
}

inline void info(const char* what){
    fmt::println("[INFO] {}  {}", sc_core::sc_time_stamp().to_string(), what);
}

#endif // VERIFICATION_H