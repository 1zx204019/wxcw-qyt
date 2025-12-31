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


#define WARNING_RECORD_SIZE  10
#define RECOVERY_RECORD_SIZE 10


#define NTC_R_REF 10000UL       // 参考电阻(10KO)
#define NTC_R0 10000UL          // NTC 25度阻值(10KO)
#define NTC_T0 298.15f          // 25度开尔文温度
#define NTC_B_VALUE 3950.0f     // NTC B值(典型值)
#define UART_BUFF_SIZE 6        // 串口接收缓冲区大小（6字节协议）


#define PAGE3_MAX_MODULES 2 // 假设总共可能有 TOTAL_SLAVES 个模块，或者你定义一个具体的数量，比如 2 个


#define PAGE1_DEV_COUNT  sizeof(page1_devices)/sizeof(Page1_DevInfo)  // 设备数量（2个）
	

// 报警事件记录数组（最多记录3个报警事件）
#define MAX_ALARM_EVENTS 3
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

//// 数据记录结构体
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

// 报警相关全局变量声明
extern AlarmRecord alarm_events[MAX_ALARM_EVENTS];
extern unsigned char alarm_event_count;
extern unsigned char alarm_event_next_index;
extern unsigned char last_abnormal_status[TOTAL_SLAVES];  // 新增：报警状态跟踪数组声明

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


// 在文件末尾的函数声明部分添加
void DeleteAlarmEvent(unsigned char index);
void ClearAllAlarmEvents(void);

DataRecord* GetRecentDataByAID(unsigned char aid);
unsigned long GetSystemTick(void);

#endif 

