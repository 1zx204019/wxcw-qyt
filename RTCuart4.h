#ifndef __UART4_H__
#define __UART4_H__

#include "STC32G.H"
#include "config.h"
#include "led.h"
#include "D1302.h"  // 包含RTC头文件

// 核心分区存储宏定义
#define TOTAL_SLAVES      35
#define RECORDS_PER_SLAVE 4
#define RECENT_RECORDS    1
#define TOTAL_RECORDS     (TOTAL_SLAVES * (RECENT_RECORDS + RECORDS_PER_SLAVE))

// 分区索引定义
#define RECENT_DATA_START 0
#define RECENT_DATA_SIZE  TOTAL_SLAVES
#define HISTORY_DATA_START RECENT_DATA_SIZE
#define HISTORY_DATA_SIZE  (TOTAL_SLAVES * RECORDS_PER_SLAVE)

#ifndef HISTORY_SIZE
#define HISTORY_SIZE      176
#define PAGE7_ITEM_COUNT  5
#endif

#define WARNING_RECORD_SIZE  10
#define RECOVERY_RECORD_SIZE 10

// 硬件引脚定义
sbit CE = P5^4;

// 页面枚举
typedef enum {
    PAGE_1 = 0,
    PAGE_2 = 1,
    PAGE_3 = 2,
    PAGE_4 = 3,
    PAGE_5 = 4,
    PAGE_6 = 5,
    PAGE_7 = 6,
    PAGE_8 = 7,
    PAGE_9 = 8,
    PAGE_10 = 9,
    PAGE_11 = 10,
    PAGE_12 = 11,
    PAGE_13 = 12,
    PAGE_14 = 13,
    PAGE_15 = 14,
    PAGE_16 = 15,
    PAGE_17 = 16,
    PAGE_18 = 17,
    PAGE_19 = 18,
    PAGE_20 = 19,
    PAGE_21 = 20,
    PAGE_22 = 21,
    PAGE_23 = 22,
    PAGE_24 = 23,
    PAGE_25 = 24,
    PAGE_26 = 25
} PageType;

// 菜单项枚举
typedef enum {
    MENU_MEASURE_DATA = 0,
    MENU_SENSOR_STATUS = 1,
    MENU_PARAM_QUERY = 2,
    MENU_HISTORY_RECORD = 3,
    MENU_DEVICE_MAINT = 4,
    PAGE2_ITEM_COUNT = 5,

    MENU_SENSOR_EVENT_1 = 0,
    MENU_SENSOR_EVENT_2 = 1,
    MENU_SENSOR_EVENT_3 = 2,
    MENU_HISTORY_DATA_QUERY = 3,
    MENU_MAX_TEMP_QUERY = 4,
    PAGE7_ITEM_COUNT = 5,
    
    MENU_DATE_TIME = 10,
    MENU_PASSWORD = 11,
    MENU_FACTORY_RESET = 12,
    PAGE8_ITEM_COUNT = 3
} MenuItemType;

// 数据记录结构体
typedef struct {
    unsigned char pid;
    unsigned char aid;
    unsigned char temp;
    unsigned char volt1;
    unsigned char volt2;
    unsigned char fosc;
    unsigned long timestamp;
    unsigned char is_valid;
    unsigned char reserved[3];
} DataRecord;

// 兼容旧代码的DataPoint结构体
typedef struct {
    unsigned char pid;
    unsigned char aid;
    unsigned char temp;
    unsigned char volt1;
    unsigned char volt2;
    unsigned char fosc;
    unsigned char voltage;
} DataPoint;

// 预警记录数据结构
typedef struct {
    unsigned char id;
    unsigned char sensor_id;
    unsigned char temperature;
    unsigned char time_hour;
    unsigned char time_minute;
    unsigned char is_valid;
} WarningRecord;

// 恢复记录数据结构
typedef struct {
    unsigned char id;
    unsigned char sensor_id;
    unsigned char time_hour;
    unsigned char time_minute;
    unsigned char status;
    unsigned char is_valid;
} RecoveryRecord;

// 统计信息结构体
typedef struct {
    unsigned char total_slaves;
    unsigned char total_data_count;
    unsigned char overall_avg_temp;
    unsigned char overall_max_temp;
    unsigned char overall_min_temp;
} Statistics;

// 菜单状态结构体
typedef struct {
    PageType current_page;
    PageType prev_page;
    unsigned char page2_selected;
    unsigned char page7_selected;
    unsigned char page8_selected;
    unsigned char page10_selected;
    unsigned char page11_selected;
    unsigned char page12_selected;
    unsigned char page13_selected;
    unsigned char page14_selected;
    unsigned char page15_selected;
    unsigned char page16_selected;
    unsigned char page17_selected;
    unsigned char page18_selected;
    unsigned char page19_selected;
    unsigned char page20_selected;
    unsigned char page21_selected;
    unsigned char page22_selected;
    unsigned char page23_selected;
    unsigned char page24_selected;
    unsigned char page25_selected;
    unsigned char page26_selected;
    unsigned char page_changed;
    unsigned char menu_initialized;
} MenuState;

// 全局变量声明（extern表示在其他文件中定义）
extern unsigned char uart_rx_buff[UART_BUFF_SIZE];
extern unsigned char uart_rx_len;
extern bit uart_rx_complete;
extern unsigned long delay_count;

// 页面/显示相关
extern PageType current_page;
extern bit display_labels_initialized;

// 菜单状态
extern MenuState menu_state;

// 数据存储核心
extern DataRecord data_summary[TOTAL_RECORDS];
extern unsigned char slave_history_index[TOTAL_SLAVES];
extern Statistics current_stats;

// 预警和恢复记录数组
extern WarningRecord warning_records[WARNING_RECORD_SIZE];
extern RecoveryRecord recovery_records[RECOVERY_RECORD_SIZE];

// 历史数据数组（兼容旧代码）
extern DataPoint history[HISTORY_SIZE];

// RTC相关变量
extern rtc_time_t current_rtc_time;
extern unsigned long rtc_refresh_counter;
extern bit need_rtc_refresh;

// 函数声明
// 串口4核心函数
void UART4_Init(unsigned long baudrate);
void UART4_SendByte(unsigned char dat);
void UART4_SendString(unsigned char *str);
void UART4_SendNumber(unsigned long num, unsigned char digits);
void UART4_ReceiveString(void);
void Debug_ShowParsedData(void);

// 定时器/延时函数
void Timer0_Init(void);
void delay_ms(unsigned long ms);

// LCD按键处理
void LCD_HandleKey(unsigned char key);

// 数据存储核心函数
void InitDataStorage(void);
void AddDataToSummary(unsigned char pid, unsigned char aid, unsigned char temp,
                     unsigned char volt1, unsigned char volt2, unsigned char fosc);
DataRecord* GetRecentDataByAID(unsigned char aid);
void CalculateStatistics(void);
void UpdateSummaryDisplay(void);

// 辅助函数
void ShowSlaveCount(void);
unsigned char CountActiveSlaves(void);
void ExportSummaryData(void);

// 原有菜单/兼容函数
void Menu_Init(void);
void InitStatistics(void);
void AddDataToHistory(unsigned char pid, unsigned char aid, unsigned char temp,
                     unsigned char volt1, unsigned char volt2, unsigned char fosc);
unsigned char CountDifferentSlaves(void);
void ExportHistoryData(void);
void RefreshDisplay(void);
void ShowMaxTempQuery(void);

// 新增预警/恢复记录相关函数
void InitWarningRecords(void);
void InitRecoveryRecords(void);
void DeleteWarningRecord(unsigned char index);
void DeleteRecoveryRecord(unsigned char index);

// RTC刷新函数
void UpdateRTCRefresh(void);

#endif // __UART4_H__


