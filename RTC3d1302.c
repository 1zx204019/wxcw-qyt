#include "D1302.h"
#include "reg51.h"
#include "STC32G.h" 
#include "config.h"
#include <intrins.h>
#include "uart4.h"
#include "lcd.h"
// DS1302寄存器地址
#define DS1302_SEC   0x80
#define DS1302_MIN   0x82
#define DS1302_HOUR  0x84
#define DS1302_DATE  0x86
#define DS1302_MONTH 0x88
#define DS1302_DAY   0x8A
#define DS1302_YEAR  0x8C
#define DS1302_CTRL  0x8E

// 辅助函数：写入一个字节到DS1302
static void ds1302_write_byte(unsigned char cmd, unsigned char dat) {
    unsigned char i;
    
    DS1302_RST = 0;
    DS1302_SCLK = 0;
    _nop_(); _nop_();
    
    DS1302_RST = 1;  // 启动传输
    _nop_(); _nop_();
    
    // 发送命令字节
    for(i = 0; i < 8; i++) {
        DS1302_IO = cmd & 0x01;
        _nop_(); _nop_();
        DS1302_SCLK = 1;
        _nop_(); _nop_();
        DS1302_SCLK = 0;
        cmd >>= 1;
    }
    
    // 发送数据字节
    for(i = 0; i < 8; i++) {
        DS1302_IO = dat & 0x01;
        _nop_(); _nop_();
        DS1302_SCLK = 1;
        _nop_(); _nop_();
        DS1302_SCLK = 0;
        dat >>= 1;
    }
    
    DS1302_RST = 0;  // 结束传输
}

// 辅助函数：读取一个字节从DS1302
static unsigned char ds1302_read_byte(unsigned char cmd) {
    unsigned char i, dat = 0;
    
    DS1302_RST = 0;
    DS1302_SCLK = 0;
    _nop_(); _nop_();
    
    DS1302_RST = 1;  // 启动传输
    _nop_(); _nop_();
    
    cmd |= 0x01; // 读命令
    
    // 发送命令字节
    for(i = 0; i < 8; i++) {
        DS1302_IO = cmd & 0x01;
        _nop_(); _nop_();
        DS1302_SCLK = 1;
        _nop_(); _nop_();
        DS1302_SCLK = 0;
        cmd >>= 1;
    }
    
    // 释放总线，准备读取
    DS1302_IO = 1;
    
    // 读取数据字节
    for(i = 0; i < 8; i++) {
        dat >>= 1;
        if(DS1302_IO) {
            dat |= 0x80;
        }
        DS1302_SCLK = 1;
        _nop_(); _nop_();
        DS1302_SCLK = 0;
        _nop_(); _nop_();
    }
    
    DS1302_RST = 0;  // 结束传输
    
    return dat;
}

void rtc_init(void) {
    // 关闭写保护
    ds1302_write_byte(DS1302_CTRL, 0x00);
    _nop_(); _nop_();
}

// 修改rtc_read函数，添加更多调试信息
void rtc_read(rtc_time_t *t) {
    unsigned char temp;
    
    // 先读取原始寄存器值用于调试
    rtc_debug_raw();
    
    // 读取时间寄存器（原始BCD码）
    temp = ds1302_read_byte(DS1302_SEC);
    t->sec = temp & 0x7F;  // 去掉CH位
    
    temp = ds1302_read_byte(DS1302_MIN);
    t->min = temp & 0x7F;
    
    temp = ds1302_read_byte(DS1302_HOUR);
    t->hour = temp & 0x3F;  // 24小时制
    
    temp = ds1302_read_byte(DS1302_DATE);
    t->day = temp & 0x3F;
    
    temp = ds1302_read_byte(DS1302_MONTH);
    t->mon = temp & 0x1F;
    
    temp = ds1302_read_byte(DS1302_YEAR);
    t->year = temp & 0xFF;
    
    UART4_SendString("Before BCD conversion: ");
    UART4_SendNumber((unsigned long)t->sec, 2);
    UART4_SendString("-");
    UART4_SendNumber((unsigned long)t->min, 2);
    UART4_SendString("-");
    UART4_SendNumber((unsigned long)t->hour, 2);
    UART4_SendString(" ");
    UART4_SendNumber((unsigned long)t->day, 2);
    UART4_SendString("-");
    UART4_SendNumber((unsigned long)t->mon, 2);
    UART4_SendString("-");
    UART4_SendNumber((unsigned long)t->year, 2);
    UART4_SendString("\r\n");
    
    // BCD转十进制
    t->sec = (t->sec >> 4) * 10 + (t->sec & 0x0F);
    t->min = (t->min >> 4) * 10 + (t->min & 0x0F);
    t->hour = (t->hour >> 4) * 10 + (t->hour & 0x0F);
    t->day = (t->day >> 4) * 10 + (t->day & 0x0F);
    t->mon = (t->mon >> 4) * 10 + (t->mon & 0x0F);
    t->year = (t->year >> 4) * 10 + (t->year & 0x0F);
}
void rtc_write(rtc_time_t *t) {
    unsigned char temp;
    
    // 关闭写保护
    ds1302_write_byte(DS1302_CTRL, 0x00);
    _nop_(); _nop_();
    
    // 写入时间寄存器
    temp = ((t->sec / 10) << 4) | (t->sec % 10);
    ds1302_write_byte(DS1302_SEC, temp);
    
    temp = ((t->min / 10) << 4) | (t->min % 10);
    ds1302_write_byte(DS1302_MIN, temp);
    
    temp = ((t->hour / 10) << 4) | (t->hour % 10);
    ds1302_write_byte(DS1302_HOUR, temp);
    
    temp = ((t->day / 10) << 4) | (t->day % 10);
    ds1302_write_byte(DS1302_DATE, temp);
    
    temp = ((t->mon / 10) << 4) | (t->mon % 10);
    ds1302_write_byte(DS1302_MONTH, temp);
    
    temp = ((t->year / 10) << 4) | (t->year % 10);
    ds1302_write_byte(DS1302_YEAR, temp);
    
    // 开启写保护
    ds1302_write_byte(DS1302_CTRL, 0x80);
}


// 添加一个调试函数，输出原始数据
void rtc_debug_raw(void) {
    unsigned char sec, min, hour, day, mon, year;
    
    sec = ds1302_read_byte(DS1302_SEC);
    min = ds1302_read_byte(DS1302_MIN);
    hour = ds1302_read_byte(DS1302_HOUR);
    day = ds1302_read_byte(DS1302_DATE);
    mon = ds1302_read_byte(DS1302_MONTH);
    year = ds1302_read_byte(DS1302_YEAR);
    
    UART4_SendString("DS1302 Raw Registers: ");
    UART4_SendString("SEC=");
    UART4_SendNumber((unsigned long)sec, 2);
    UART4_SendString(" MIN=");
    UART4_SendNumber((unsigned long)min, 2);
    UART4_SendString(" HOUR=");
    UART4_SendNumber((unsigned long)hour, 2);
    UART4_SendString(" DAY=");
    UART4_SendNumber((unsigned long)day, 2);
    UART4_SendString(" MON=");
    UART4_SendNumber((unsigned long)mon, 2);
    UART4_SendString(" YEAR=");
    UART4_SendNumber((unsigned long)year, 2);
    UART4_SendString("\r\n");
    
    // 检查CH位（秒寄存器的第7位）
    if (sec & 0x80) {
        UART4_SendString("WARNING: CH=1 (Oscillator Halted)\r\n");
    }
}
// 添加一个函数检查并初始化RTC时间
void rtc_check_and_init(void) {
    rtc_time_t t;
    rtc_read(&t);
    
    // 如果时间为0，设置默认时间
    if (t.year == 0 && t.mon == 0 && t.day == 0) {
        UART4_SendString("RTC not initialized. Setting default time...\r\n");
        
        t.year = 25;   // 2025
        t.mon = 12;    // 12月
        t.day = 25;    // 24日
        t.hour = 10;   // 10时
        t.min = 30;    // 30分
        t.sec = 0;     // 0秒
        
        rtc_write(&t);
        UART4_SendString("Default time set: 2025-12-24 10:30:00\r\n");
    } else {
        UART4_SendString("RTC already initialized.\r\n");
    }
}
// 修改后的函数：每次都设置时间
//void rtc_check_and_init(void) {
//    rtc_time_t t;
//    
//    // 强制设置时间
//    t.year = 25;   // 2025
//    t.mon = 12;    // 12月
//    t.day = 29;    // 24日
//    t.hour = 10;   // 10时
//    t.min = 29;    // 30分
//    t.sec = 0;     // 0秒
//    
//    rtc_write(&t); // 写入时间
//    UART4_SendString("Time forced to: 2025-12-24 10:30:00\r\n");
//}
