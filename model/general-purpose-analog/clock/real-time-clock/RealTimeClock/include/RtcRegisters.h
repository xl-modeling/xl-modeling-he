#ifndef RTC_REGISTERS_H
#define RTC_REGISTERS_H

#include "utils/regfile.h"

// ========================================================================================
// RTC寄存器文件：8 位地址，8 位数据
//   | ADDRESS | BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0 | FUNCTION | RANGE |
//   |   00h   |  CH  |       10 SECS      |            SECS           |  Seconds | 00-59 |
//   |   01h   |   0  |       10 MINS      |            MINS           |  Minutes | 00-59 |
//   |   02h   |   0  |  12  | 10H  | 10H  |            HOURS          |  Hours   | 00-23 |
//   |         |      |  24  | PM/AM|      |                           |          |       |
//   |   03h   |          0         |  0   |   0  |     Day            |  Day     | 01-07 |
//   |   04h   |      0      |   10 DATE   |            DATE           |  Date    | 01-31 |
//   |   05h   |          0         |10 MON|            MONTH          |  Month   | 01-12 |
//   |   06h   |       10 YEAR             |            YEAR           |  Year    | 00-99 |
//   |   07h   |  OUT |   0  |  0   | SQWE |   0  |   0  | RS1  | RS0  |  Control |  ---  |
//
// 数据来源：Maxim Integrated, DS1307 64 x 8, Serial, I2C Real-Time Clock, REV 3/15（doc/DS1307.pdf）
//   - 寄存器地址与位定义：Table 2. Timekeeper Registers
// ========================================================================================
SC_MODULE(RtcRegFile) 
    , public RegFile<8, 8>
{
    enum Table : uint8_t {
        REG_SEC     = 0x00,     
        REG_MIN     = 0x01,
        REG_HOUR    = 0x02,     
        REG_DAY     = 0x03,     
        REG_DATE    = 0x04,
        REG_MONTH   = 0x05,
        REG_YEAR    = 0x06,
        REG_CONTROL = 0x07
    };
    RtcRegFile(sc_core::sc_module_name name) : RegFile<8,8>(name) {}
    RtcRegFile(sc_core::sc_module_name name, const RegFile<8,8>::reg_array& regs) : RegFile<8,8>(name, regs) {}
};

#endif // RTC_REGISTERS_H
