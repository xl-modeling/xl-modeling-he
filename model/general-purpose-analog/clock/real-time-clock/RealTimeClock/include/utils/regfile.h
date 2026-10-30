#ifndef REGFILE_H
#define REGFILE_H

#include <systemc>

#include <array>
#include <type_traits>

#include "traceable.h"

template<size_t width, size_t num, typename type>
class RegFile_x 
    : public TraceableSC
{
public:
    using reg_type = type;
    using reg_array = std::array<reg_type, num>;

    struct IF : public sc_core::sc_interface {
        virtual const sc_core::sc_event& default_event() const = 0;
        virtual void write(size_t addr, const reg_type& value) = 0;
        virtual reg_type read(size_t addr) const = 0;
    };

    class CH : public sc_core::sc_prim_channel, public IF {
    public:
        CH(const char* name) : sc_core::sc_prim_channel(name) {}
        CH(const char* name, const reg_array& regs) : sc_core::sc_prim_channel(name), m_regs(regs) {}

        const sc_core::sc_event& default_event() const override { return e; }

        void write(size_t addr, const reg_type& value) override {
            assert(addr < num);
            m_regs[addr] = value;
            e.notify();
        }
        reg_type read(size_t addr) const override {
            assert(addr < num);
            return m_regs[addr];
        }

        void trace_me(sc_core::sc_trace_file* tf) {
            for(size_t i = 0; i < m_regs.size(); i++){
                sc_core::sc_trace(tf, m_regs[i], std::string(this->name()) + "_" + std::to_string(i));
            }
        }
    private:
        sc_core::sc_event e;
        reg_array m_regs;
    };

    void trace_me (sc_core::sc_trace_file* tf) override{ ch.trace_me(tf); }

    sc_core::sc_export<IF> inf;
    RegFile_x(const char* name) : ch(name) { inf.bind(ch); }
    RegFile_x(const char* name, const reg_array& regs) : ch(name, regs) { inf.bind(ch); }
    virtual ~RegFile_x() {}
private:
    CH ch;
};

template<size_t width, size_t num>
using RegFile = RegFile_x<width, num, std::conditional_t<(width <= 64), sc_dt::sc_uint<width>, sc_dt::sc_biguint<width>> >;

template<size_t width, size_t num>
using RegFile_s = RegFile_x<width, num, std::conditional_t<(width <= 64), sc_dt::sc_int<width>, sc_dt::sc_bigint<width>> >;

template<size_t width, size_t num>
using RegFile_l = RegFile_x<width, num, sc_dt::sc_lv<width> >;

template <size_t width, size_t num>
using RegFile_b = RegFile_x<width, num, sc_dt::sc_bv<width> >;

#endif