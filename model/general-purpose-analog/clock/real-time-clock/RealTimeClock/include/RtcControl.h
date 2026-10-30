#ifndef RTC_CONTROL_H
#define RTC_CONTROL_H

#include <systemc>

#include <cstdint>
#include <string>

#include "utils/traceable.h"
#include "RtcRegisters.h"

// ============================================================
// RTC 日历控制：
//   - 1Hz 下降沿推进日历（秒/分/时/星期/日期/月/年，闰年 2000~2099）
//
// 数据来源：Maxim Integrated, DS1307 64 x 8, Serial, I2C Real-Time Clock, REV 3/15（doc/DS1307.pdf）
//   - 日历计数、闰年、12/24 小时模式、CH 位、1Hz 下降沿更新：Clock and Calendar、Control Register
// ============================================================

SC_MODULE(RtcControl)
    , public TraceableSC
{
    using reg = RtcRegFile::reg_type;
    using Table = RtcRegFile::Table;
    sc_core::sc_in<bool> clk_1hz;
    sc_core::sc_port<RtcRegFile::IF> inf;

    SC_HAS_PROCESS(RtcControl);
    explicit RtcControl(sc_core::sc_module_name nm) 
    : clk_1hz("clk_1hz"), inf("inf")
    , m_skip_next(false)
    {}

    void end_of_elaboration() override{
        SC_METHOD(negedge_clk_1hz);     // DS1307 要求下降沿更新日历
        sensitive << clk_1hz.negedge_event();
        dont_initialize();
    }

    void trace_me(sc_core::sc_trace_file* tf) override{
        TRACE_SC_SIGNAL(clk_1hz);
    }

    void skip_next() { m_skip_next = true; }
    
    void negedge_clk_1hz(){
        if(m_skip_next){
            m_skip_next = false;
            return;
        }

        reg sec = inf->read(RtcRegFile::Table::REG_SEC);
        reg min = inf->read(RtcRegFile::Table::REG_MIN);
        reg hour = inf->read(RtcRegFile::Table::REG_HOUR);
        reg day = inf->read(RtcRegFile::Table::REG_DAY);
        reg date = inf->read(RtcRegFile::Table::REG_DATE);
        reg mon = inf->read(RtcRegFile::Table::REG_MONTH);
        reg year = inf->read(RtcRegFile::Table::REG_YEAR);

        bool ch = sec[7];  
        if(ch) return;
        bool is_12h = hour[6];
        bool pm_am = hour[5];

        uint8_t sec_d = bcd2bin(sec.range(6, 0));
        uint8_t min_d = bcd2bin(min.range(6,0));
        uint8_t hour_d = bcd2bin(is_12h ? hour.range(4, 0) : hour.range(5, 0));
        uint8_t day_d = bcd2bin(day.range(2, 0));
        uint8_t date_d = bcd2bin(date.range(5, 0));
        uint8_t mon_d = bcd2bin(mon.range(4, 0));
        uint8_t year_d = bcd2bin(year);

        tick(sec_d, min_d, hour_d, day_d, date_d, mon_d, year_d, pm_am, is_12h);

        inf->write(Table::REG_SEC, ch << 7 | bin2bcd(sec_d));
        inf->write(Table::REG_MIN, bin2bcd(min_d));
        inf->write(Table::REG_HOUR, is_12h ? (is_12h << 6 | pm_am << 5 | bin2bcd(hour_d)) : ( is_12h << 6 | bin2bcd(hour_d)));
        inf->write(Table::REG_DAY, bin2bcd(day_d));
        inf->write(Table::REG_DATE, bin2bcd(date_d));
        inf->write(Table::REG_MONTH, bin2bcd(mon_d));
        inf->write(Table::REG_YEAR, bin2bcd(year_d));
    }
private:
    bool m_skip_next;
    uint8_t bcd2bin(uint8_t bcd) { return 10 * ((bcd & 0xf0) >> 4) + (bcd & 0x0f); }
    uint8_t bin2bcd(uint8_t bin) { uint8_t ten = bin / 10; uint8_t num = bin - ten * 10; return (ten << 4 | num); }
    bool is_leap(size_t year) {return !(year & 0x3); }
    unsigned days_in_month(unsigned mm, unsigned yy){
        static const unsigned days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        return (mm == 2 && is_leap(yy)) ? 29 : days[mm - 1];
    }
    void tick(uint8_t& sec, uint8_t& min, uint8_t& hour, uint8_t& day, uint8_t& date, uint8_t& mon, uint8_t& year, bool& pm_am, bool is_12h){
        sec++;
        if(sec > 59)
            sec = 0;
        else
            return;

        min++;
        if(min > 59)
            min = 0;
        else
            return;

        hour++;
        if (is_12h) {
            if(hour > 12) { hour = 1; }
            if(hour > 11) { pm_am = !pm_am; if(pm_am == 1 /* am to pm */) return; } else return;
        } else {
            if (hour > 23) hour = 0; else return;  
        }

        day++;
        if(day > 7){
            day = 1;
        }

        date++;
        if(date > days_in_month(mon, year))
            date = 1;
        else
            return;

        mon++;
        if(mon > 12)
            mon = 1;
        else
            return;

        year++;
        if(year > 99)
            year = 0;
        else
            return;
    }
};

#endif // RTC_CORE_H
