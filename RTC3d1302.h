#ifndef _RTC_H_
#define _RTC_H_
#include "STC32G.H"
// 结构体定义
typedef struct {
    unsigned char year;
    unsigned char mon;
    unsigned char day;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
} rtc_time_t;

// 函数声明
void rtc_init(void);
void rtc_read(rtc_time_t *t);
void rtc_write(rtc_time_t *t);
void rtc_check_and_init(void);
void rtc_debug_raw(void);

#endif
