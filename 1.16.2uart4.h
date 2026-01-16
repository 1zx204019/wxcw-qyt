#ifndef __UART4_H__
#define __UART4_H__

#include "STC32G.H"
#include "config.h"
#include "led.h"
#include "D1302.h"  // 包含RTC头文件

// 核心分区存储宏定义
#define TOTAL_SLAVES        35
#define RECORDS_PER_SLAVE   4
#define RECENT_RECORDS      1
#define TOTAL_RECORDS       (TOTAL_SLAVES * (RECENT_RECORDS + RECORDS_PER_SLAVE))

// 分区索引定义
#define RECENT_DATA_START   0
#define RECENT_DATA_SIZE    TOTAL_SLAVES
#define HISTORY_DATA_START  RECENT_DATA_SIZE
#define HISTORY_DATA_SIZE   (TOTAL_SLAVES * RECORDS_PER_SLAVE)

#define WARNING_RECORD_SIZE     10
#define RECOVERY_RECORD_SIZE    10

#define NTC_R_REF       10000UL       // 参考电阻(10KO)
#define NTC_R0          10000UL       // NTC 25度阻值(10KO)
#define NTC_T0          298.15f       // 25度开尔文温度
#define NTC_B_VALUE     3950.0f       // NTC B值(典型值)
#define UART_BUFF_SIZE  6             // 串口接收缓冲区大小（6字节协议）

#define PAGE3_MAX_MODULES    2         // 假设总共可能有 TOTAL_SLAVES 个模块，或者你定义一个具体的数量，比如 2 个
#define PAGE1_DEV_COUNT      sizeof(page1_devices)/sizeof(Page1_DevInfo)  // 设备数量（2个）

// 报警事件记录数组（最多记录3个报警事件）
#define MAX_ALARM_EVENTS     6
#define MAX_RECOVERY_EVENTS  6  // 最多记录3个恢复事件（与报警事件一致）
// uart4.h 宏定义区（添加在 PAGE18 相关宏附近）
#define PAGE10_DISPLAY_COUNT 3  // PAGE10每页显示3条事件（与PAGE18一致）
#define PAGE10_MAX_PAGE      2  // PAGE10总页数（6条事件→2页）
//历史最高温
#define MAX_MAX_TEMP_EVENTS  3  // 新增：最高温事件最大数量

#define PWD_FLASH_DURATION   30  // 新增：单次闪烁时长（25次调用≈250ms，与系统节拍匹配）
#define FLASH_INTERVAL       25  // 与 PWD_FLASH_DURATION 一致，避免冲突

// 新增：PAGE_1总设备数（自动计算，避免硬编码）
#define PAGE1_TOTAL_DEVICES  sizeof(page1_devices)/sizeof(Page1_DevInfo)



#define PAGE19_READ_INTERVAL 600000    


#define PAGE19_TRIGGER_INTERVAL 1000  // 1秒触发一次（避免高频调用）



#define PAGE18_DISPLAY_COUNT 3  // 每页仍显示3个从站（保持原有布局）
#define PAGE18_MAX_PAGE      ((TOTAL_SLAVES + PAGE18_DISPLAY_COUNT - 1) / PAGE18_DISPLAY_COUNT)  // 总页数（10个从站→4页）

// 原定义：#define PAGE18_MAX_PAGE      ((TOTAL_SLAVES + PAGE18_DISPLAY_COUNT - 1) / PAGE18_DISPLAY_COUNT)
#define PAGE18_MAX_PAGE      4  // 固定4页（0-3），对应TX01-03、04-06、07-09、10
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
    
    // 修改为两个时间字段
    unsigned long timestamp;   // 用于系统计时（毫秒）
    rtc_time_t save_time;      // 用于历史记录的RTC时间
    
    unsigned char is_valid;
    unsigned char reserved[3];
} DataRecord;


typedef struct {
    unsigned char PID;          // 从站ID
    unsigned char AID;          // 区域ID
    unsigned int ADC_Value;     // 12位ADC值
    float Bat_Voltage;          // 电池电压
    unsigned char Check_OK;     // FCS校验结果
    short temperature;          // 温度值(放大10倍)
} Protocol_Data;

typedef struct {
    PageType current_page;
    PageType prev_page;
    unsigned char page2_selected;
    unsigned char page3_selected;
    unsigned char page7_selected;
    unsigned char page8_selected;
    unsigned char page9_selected;
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
		
		unsigned char page11_global_event_idx; // 新增：暂存PAGE_10全局事件索引
} MenuState;//用于跟踪和控制整个用户界面（菜单系统）的当前状态。

// PAGE_1设备信息结构体（简化数据管理）
typedef struct {
    unsigned char aid;          // 对应AID
    unsigned char dev_id[7];    // 设备ID字符串（如"01TX01"）
} Page1_DevInfo;

typedef enum {
    RTC_EDIT_IDLE = 0,    // 空闲状态（仅显示）
    RTC_EDIT_SELECT = 1,  // 选择修改位置状态
    RTC_EDIT_CHANGE = 2   // 调整数字状态
} RTC_EditState;

// 报警事件记录结构体
typedef struct {
    unsigned char pid;           // 从站ID
    unsigned char aid;           // 区域ID
    unsigned char temp;          // 温度（整数）
    unsigned int volt_mv;        // 电压（mV）
    rtc_time_t timestamp;        // 发生时间（年、月、日、时、分、秒）
    unsigned char is_valid;      // 记录是否有效
} AlarmRecord;

//恢复事件记录结构体
typedef struct {
    unsigned char pid;           // 从站ID
    unsigned char aid;           // 区域ID
    unsigned char abnormal_temp; // 报警时温度（关联报警事件）
    unsigned char recovery_temp; // 恢复后温度（≤25℃）
    unsigned int recovery_volt_mv; // 恢复时电压（mV）
    rtc_time_t abnormal_timestamp; // 报警时间（关联报警事件）
    rtc_time_t recovery_timestamp; // 恢复时间
    unsigned char is_valid;      // 记录是否有效
} RecoveryRecord;

// 密码编辑状态枚举
typedef enum {
    PWD_EDIT_IDLE = 0,    // 空闲状态（仅显示）
    PWD_EDIT_SELECT = 1,  // 选择密码位状态
    PWD_EDIT_CHANGE = 2   // 编辑数字状态
} Password_EditState;

// 每日最高温存储结构体（存储近3天，0=当天，1=昨天，2=前天）
typedef struct {
    short max_temp;        // 当日最高温（放大10倍）
    rtc_time_t temp_time;  // 最高温发生时间
    unsigned char pid;     // 对应PID
    unsigned char aid;     // 对应AID
    unsigned short volt_mv;// 对应电压（mV）
    unsigned char is_valid;// 该天是否有有效数据
} DailyMaxTemp;

// uart4.h - 在全局变量声明部分添加
typedef struct {
    unsigned char pid;
    unsigned char aid;
    unsigned char temp;
    unsigned short volt_mv;  // 电压(mV)
    rtc_time_t record_time;  // 记录时间
    unsigned char is_valid;           // 记录是否有效
} Page19_AutoRecord;

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

extern rtc_time_t current_rtc_time;
extern unsigned long rtc_refresh_counter;
extern bit need_rtc_refresh;

extern Protocol_Data parsed_data;
extern DataRecord data_summary[TOTAL_RECORDS];
extern unsigned char history_index[TOTAL_SLAVES];
extern unsigned long system_tick;

// uart4.h 末尾新增
extern RTC_EditState rtc_edit_state;
extern unsigned char rtc_edit_pos;
extern rtc_time_t edit_temp_time;

extern RecoveryRecord recovery_events[MAX_RECOVERY_EVENTS];
extern unsigned char recovery_event_count;
extern unsigned char recovery_event_next_index;
// 报警相关全局变量声明
extern AlarmRecord alarm_events[MAX_ALARM_EVENTS];
extern unsigned char alarm_event_count;
extern unsigned char alarm_event_next_index;
extern unsigned char last_abnormal_status[TOTAL_SLAVES];  // 新增：报警状态跟踪数组声明

// 新增：最高温事件相关全局变量声明
extern DailyMaxTemp daily_max_temps[MAX_MAX_TEMP_EVENTS];
extern unsigned char max_temp_event_count;
extern unsigned char max_temp_next_index;
// 在 uart4.h 的全局变量声明部分添加
extern bit disable_today_max_calc;  // 新增：禁止当天最高温计算标志
// uart4.h 全局变量声明部分添加
extern short max_temps[3];
extern rtc_time_t max_temp_times[3];



extern Page19_AutoRecord page19_auto_records[3][4];  // 3个从站，每个从站4条记录
extern unsigned char page19_record_index[3];          // 每个从站当前存储索引
extern unsigned long page19_last_read_time[3];        // 每个从站上次读取时间

// 函数声明
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
void Menu_Init(void);
void RefreshDisplay(void);

// RTC刷新函数
void UpdateRTCRefresh(void);

void InitDataStorage(void);
void AddDataToSummary(unsigned char pid, unsigned char aid, unsigned char temp, 
                     unsigned char volt1, unsigned char volt2, unsigned char fosc);

void DeleteAlarmEvent(unsigned char display_index);
// 在文件末尾的函数声明部分添加

void ClearAllAlarmEvents(void);
void DeleteMaxTempEvent(unsigned char display_index);
DataRecord* GetRecentDataByAID(unsigned char aid);
unsigned long GetSystemTick(void);
void ClearAllMaxTempEvents(void);



#endif 
