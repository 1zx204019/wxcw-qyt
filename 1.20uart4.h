#ifndef __UART4_H__
#define __UART4_H__

// 头文件包含（按功能分类，便于维护）
#include "STC32G.H"       // STC32G单片机核心头文件
#include "config.h"       // 系统配置头文件
#include "led.h"          // LED驱动头文件
#include "D1302.h"        // DS1302 RTC时钟驱动头文件

// ------------------- 核心存储配置宏定义 -------------------
#define TOTAL_SLAVES        35          // 从站设备总数（支持35个从站）
#define RECORDS_PER_SLAVE   4           // 每个从站的历史记录条数
#define RECENT_RECORDS      1           // 每个从站的实时数据记录条数
#define TOTAL_RECORDS       (TOTAL_SLAVES * (RECENT_RECORDS + RECORDS_PER_SLAVE))  // 总记录数 = 从站数 ×（实时+历史）

// ------------------- 存储分区索引定义 -------------------
#define RECENT_DATA_START   0           // 实时数据存储起始索引
#define RECENT_DATA_SIZE    TOTAL_SLAVES// 实时数据存储大小（每个从站1条）
#define HISTORY_DATA_START  RECENT_DATA_SIZE  // 历史数据存储起始索引
#define HISTORY_DATA_SIZE   (TOTAL_SLAVES * RECORDS_PER_SLAVE)  // 历史数据存储大小

// ------------------- NTC温度传感器参数 -------------------
#define NTC_R_REF       10000UL       // 参考电阻值（10KΩ）
#define NTC_R0          10000UL       // NTC传感器25℃时阻值（10KΩ）
#define NTC_T0          298.15f       // 25℃对应的开尔文温度（298.15K）
#define NTC_B_VALUE     3950.0f       // NTC传感器B值（典型值3950）

// ------------------- UART通信配置 -------------------
#define UART_BUFF_SIZE  6             // 串口接收缓冲区大小（6字节协议帧）

// ------------------- 页面显示配置 -------------------
#define PAGE3_MAX_MODULES    30         // PAGE_3页面支持的最大模块数（30个）
#define PAGE1_DEV_COUNT      sizeof(page1_devices)/sizeof(Page1_DevInfo)  // PAGE_1设备数（自动计算，避免硬编码）
#define PAGE10_DISPLAY_COUNT 3          // PAGE_10每页显示报警事件数（3条）
#define PAGE10_MAX_PAGE      2          // PAGE_10最大页数（支持6条报警事件）
#define PAGE18_DISPLAY_COUNT 3          // PAGE_18每页显示从站数（3个）
#define PAGE18_MAX_PAGE      ((TOTAL_SLAVES + PAGE18_DISPLAY_COUNT - 1) / PAGE18_DISPLAY_COUNT)  // PAGE_18最大页数（自动计算）
#define PAGE1_TOTAL_DEVICES  sizeof(page1_devices)/sizeof(Page1_DevInfo)  // PAGE_1总设备数（自动计算）

// ------------------- 事件记录配置 -------------------
#define MAX_ALARM_EVENTS     6          // 最大报警事件记录数（6条）
#define MAX_RECOVERY_EVENTS  6          // 最大传感器恢复事件记录数（6条）
#define MAX_MAX_TEMP_EVENTS  3          // 最大最高温事件记录数（3条，近3天）

// ------------------- 闪烁效果配置 -------------------
#define PWD_FLASH_DURATION   30        // 密码闪烁单次时长（30个系统节拍≈250ms）
#define FLASH_INTERVAL       25        // 通用闪烁间隔（25个系统节拍，与密码闪烁保持一致）

// ------------------- PAGE_19自动记录配置 -------------------
#define PAGE19_READ_INTERVAL 600000    // PAGE_19自动读取间隔（600000ms=10分钟）
#define PAGE19_TRIGGER_INTERVAL 1000   // PAGE_19触发间隔（1000ms=1秒，避免高频调用）

// ------------------- IO引脚定义 -------------------
sbit CE = P5^4;                       // 扩展引脚定义（CE引脚映射到P5.4）

// ------------------- 页面枚举（所有页面标识） -------------------
typedef enum {
    PAGE_1 = 0,    // 从站设备列表页面
    PAGE_2 = 1,    // 主功能菜单页面
    PAGE_3 = 2,    // 测量数据详情页面
    PAGE_4 = 3,    // 传感器异常状态页面
    PAGE_5 = 4,    // 参数查询页面
    PAGE_6 = 5,    // 预留页面
    PAGE_7 = 6,    // 历史记录查询子菜单页面
    PAGE_8 = 7,    // 设备维护菜单页面
    PAGE_9 = 8,    // 扩展页面
    PAGE_10 = 9,   // 温度报警事件列表页面
    PAGE_11 = 10,  // 报警事件详细数据页面
    PAGE_12 = 11,  // 报警事件删除确认页面
    PAGE_13 = 12,  // 温度预警事件列表页面
    PAGE_14 = 13,  // 传感器恢复事件列表页面
    PAGE_15 = 14,  // 预警/恢复事件详细数据页面
    PAGE_16 = 15,  // 预警事件删除确认页面
    PAGE_17 = 16,  // 恢复事件删除确认页面
    PAGE_18 = 17,  // 历史数据查询页面
    PAGE_19 = 18,  // 清除历史数据确认页面
    PAGE_20 = 19,  // 历史数据删除确认页面
    PAGE_21 = 20,  // 最高温度查询页面
    PAGE_22 = 21,  // 最高温度清除确认页面
    PAGE_23 = 22,  // 最高温度删除确认页面
    PAGE_24 = 23,  // 修改日期时间页面
    PAGE_25 = 24,  // 修改密码页面
    PAGE_26 = 25   // 恢复出厂设置确认页面
} PageType;

// ------------------- 菜单项枚举（各菜单页面选项标识） -------------------
typedef enum {
    // PAGE_2主功能菜单项
    MENU_MEASURE_DATA = 0,    // 测量数据
    MENU_SENSOR_STATUS = 1,   // 传感器状态
    MENU_PARAM_QUERY = 2,     // 参数查询
    MENU_HISTORY_RECORD = 3,  // 历史记录
    MENU_DEVICE_MAINT = 4,    // 设备维护
    PAGE2_ITEM_COUNT = 5,     // PAGE_2菜单项总数（5个）
    
    // PAGE_7历史记录查询子菜单项
    MENU_SENSOR_EVENT_1 = 0,   // 温度报警事件记录
    MENU_SENSOR_EVENT_2 = 1,   // 温度预警事件记录
    MENU_SENSOR_EVENT_3 = 2,   // 传感器恢复事件记录
    MENU_HISTORY_DATA_QUERY = 3,  // 历史数据查询
    MENU_MAX_TEMP_QUERY = 4,  // 最高历史温度查询
    PAGE7_ITEM_COUNT = 5,     // PAGE_7菜单项总数（5个）
    
    // PAGE_8设备维护菜单项
    MENU_DATE_TIME = 10,      // 修改日期时间
    MENU_PASSWORD = 11,       // 修改密码
    MENU_FACTORY_RESET = 12,  // 恢复出厂设置
    PAGE8_ITEM_COUNT = 3      // PAGE_8菜单项总数（3个）
} MenuItemType;

// ------------------- 数据记录结构体（存储传感器实时/历史数据） -------------------
typedef struct {
    unsigned char pid;           // 从站ID
    unsigned char aid;           // 区域ID（从站地址）
    unsigned char temp;          // 温度值（整数，单位℃）
    unsigned char volt1;         // 电压1（单位0.1V）
    unsigned char volt2;         // 电压2（预留）
    unsigned char fosc;          // 振荡频率（预留）
    unsigned long timestamp;     // 系统时间戳（毫秒级，用于计时）
    rtc_time_t save_time;       // RTC时间（用于历史记录的精确时间戳）
    unsigned char is_valid;      // 记录有效性标志（1=有效，0=无效）
    unsigned char reserved[3];   // 预留字段（用于对齐，增强兼容性）
} DataRecord;

// ------------------- UART协议解析结构体（存储解析后的协议数据） -------------------
typedef struct {
    unsigned char PID;          // 从站ID（协议帧解析结果）
    unsigned char AID;          // 区域ID（协议帧解析结果）
    unsigned int ADC_Value;     // 12位ADC原始值（传感器采集数据）
    float Bat_Voltage;          // 电池电压（单位V，保留1位小数）
    unsigned char Check_OK;     // FCS校验结果（1=校验通过，0=校验失败）
    short temperature;          // 转换后的温度值（放大10倍，保留1位小数）
} Protocol_Data;

// ------------------- 菜单状态结构体（跟踪整个菜单系统的当前状态） -------------------
typedef struct {
    PageType current_page;      // 当前显示页面
    PageType prev_page;         // 上一个页面（用于返回操作）
    // 各页面选中项索引
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
    unsigned char page_changed; // 页面变更标志（1=页面已变更，需刷新）
    unsigned char menu_initialized; // 菜单初始化标志（1=已初始化）
    unsigned char page11_global_event_idx; // PAGE_11页面关联的全局报警事件索引
} MenuState;

// ------------------- PAGE_1设备信息结构体（存储PAGE_1显示的设备信息） -------------------
typedef struct {
    unsigned char aid;          // 设备对应的AID（区域ID）
    unsigned char dev_id[7];    // 设备ID字符串（格式如"01TX01"，长度6+1结束符）
} Page1_DevInfo;

// ------------------- RTC编辑状态枚举（控制日期时间修改流程） -------------------
typedef enum {
    RTC_EDIT_IDLE = 0,    // 空闲状态（仅显示，不允许修改）
    RTC_EDIT_SELECT = 1,  // 选择状态（选择要修改的时间位）
    RTC_EDIT_CHANGE = 2   // 调整状态（调整选中位的数字）
} RTC_EditState;

// ------------------- 报警事件结构体（存储温度报警事件详情） -------------------
typedef struct {
    unsigned char pid;           // 触发报警的从站ID
    unsigned char aid;           // 触发报警的区域ID
    unsigned char temp;          // 报警时的温度值（单位℃）
    unsigned int volt_mv;        // 报警时的电压值（单位mV）
    rtc_time_t timestamp;        // 报警发生的RTC时间（年、月、日、时、分、秒）
    unsigned char is_valid;      // 报警记录有效性标志（1=有效，0=无效）
} AlarmRecord;

// ------------------- 恢复事件结构体（存储传感器恢复正常事件详情） -------------------
typedef struct {
    unsigned char pid;           // 恢复正常的从站ID
    unsigned char aid;           // 恢复正常的区域ID
    unsigned char abnormal_temp; // 报警时的温度值（关联报警事件）
    unsigned char recovery_temp; // 恢复后的温度值（≤25℃）
    unsigned int recovery_volt_mv; // 恢复时的电压值（单位mV）
    rtc_time_t abnormal_timestamp; // 报警发生的RTC时间
    rtc_time_t recovery_timestamp; // 恢复正常的RTC时间
    unsigned char is_valid;      // 恢复记录有效性标志（1=有效，0=无效）
} RecoveryRecord;

// ------------------- 密码编辑状态枚举（控制密码修改流程） -------------------
typedef enum {
    PWD_EDIT_IDLE = 0,    // 空闲状态（仅显示，不允许修改）
    PWD_EDIT_SELECT = 1,  // 选择状态（选择要修改的密码位）
    PWD_EDIT_CHANGE = 2   // 调整状态（调整选中位的数字）
} Password_EditState;

// ------------------- 每日最高温结构体（存储近3天的最高温数据） -------------------
typedef struct {
    short max_temp;        // 当日最高温度（放大10倍，保留1位小数）
    rtc_time_t temp_time;  // 最高温发生的RTC时间
    unsigned char pid;     // 对应从站ID
    unsigned char aid;     // 对应区域ID
    unsigned short volt_mv; // 对应电压值（单位mV）
    unsigned char is_valid; // 记录有效性标志（1=有效，0=无效）
} DailyMaxTemp;

// ------------------- PAGE_19自动记录结构体（存储PAGE_19的自动读取记录） -------------------
typedef struct {
    unsigned char pid;           // 从站ID
    unsigned char aid;           // 区域ID
    unsigned char temp;          // 温度值（单位℃）
    unsigned short volt_mv;      // 电压值（单位mV）
    rtc_time_t record_time;     // 记录的RTC时间
    unsigned char is_valid;      // 记录有效性标志（1=有效，0=无效）
} Page19_AutoRecord;

// ------------------- 全局变量声明（extern表示在其他文件中定义） -------------------
// UART通信相关
extern unsigned char uart_rx_buff[UART_BUFF_SIZE];  // 串口接收缓冲区
extern unsigned char uart_rx_len;                 // 串口接收数据长度
extern bit uart_rx_complete;                    // 串口接收完成标志（1=完成）

// 系统计时相关
extern unsigned long delay_count;               // 延时计数器
extern unsigned long system_tick;               // 系统滴答计时器（毫秒级）

// 页面显示相关
extern PageType current_page;                   // 当前显示页面（外部定义）
extern bit display_labels_initialized;          // 固定标签初始化标志（1=已初始化）

// 菜单状态相关
extern MenuState menu_state;                    // 菜单系统状态
extern rtc_time_t current_rtc_time;             // 当前RTC时间
extern unsigned long rtc_refresh_counter;       // RTC刷新计数器
extern bit need_rtc_refresh;                   // RTC刷新标志（1=需要刷新）

// 协议解析相关
extern Protocol_Data parsed_data;               // 解析后的协议数据

// 数据存储相关
extern DataRecord data_summary[TOTAL_RECORDS];  // 数据记录存储数组（实时+历史）
extern unsigned char history_index[TOTAL_SLAVES];// 每个从站的历史记录索引

// RTC编辑相关
extern RTC_EditState rtc_edit_state;            // RTC编辑状态
extern unsigned char rtc_edit_pos;              // RTC编辑选中位置（0-9：年1-分2）
extern rtc_time_t edit_temp_time;               // 临时存储待修改的RTC时间

// 事件记录相关
extern AlarmRecord alarm_events[MAX_ALARM_EVENTS];          // 报警事件存储数组
extern RecoveryRecord recovery_events[MAX_RECOVERY_EVENTS];  // 恢复事件存储数组
extern unsigned char alarm_event_count;                    // 当前报警事件数量
extern unsigned char alarm_event_next_index;               // 下一个报警事件存储索引
extern unsigned char recovery_event_count;                 // 当前恢复事件数量
extern unsigned char recovery_event_next_index;            // 下一个恢复事件存储索引
extern unsigned char last_abnormal_status[TOTAL_SLAVES];   // 各从站上次异常状态（1=异常，0=正常）

// 最高温相关
extern DailyMaxTemp daily_max_temps[MAX_MAX_TEMP_EVENTS];  // 近3天最高温存储数组
extern unsigned char max_temp_event_count;                // 当前最高温事件数量
extern unsigned char max_temp_next_index;                 // 下一个最高温事件存储索引
extern bit disable_today_max_calc;                       // 禁止当天最高温计算标志（1=禁止）
extern bit disable_max_temp_calc;                        // 禁止最高温计算标志（1=禁止）
extern short max_temps[3];                              // 前3高温度缓存（放大10倍）
extern rtc_time_t max_temp_times[3];                     // 前3高温度对应的时间

// PAGE_19相关
extern Page19_AutoRecord page19_auto_records[3][4];       // PAGE_19自动记录数组（3个从站×4条记录）
extern unsigned char page19_record_index[3];             // 每个从站的PAGE_19记录索引
extern unsigned long page19_last_read_time[3];           // 每个从站的PAGE_19上次读取时间

// ------------------- 函数声明（按功能分类） -------------------
// UART通信函数
void UART4_Init(unsigned long baudrate);                  // UART4初始化（指定波特率）
void UART4_SendByte(unsigned char dat);                  // UART4发送1字节数据
void UART4_SendString(unsigned char *str);               // UART4发送字符串
void UART4_SendNumber(unsigned long num, unsigned char digits);  // UART4发送指定位数的数字
void UART4_ReceiveString(void);                         // UART4接收字符串（协议帧）

// 定时器/延时函数
void Timer0_Init(void);                                 // 定时器0初始化（用于延时和系统滴答）
void delay_ms(unsigned long ms);                        // 毫秒级延时函数

// LCD显示函数
void RefreshDisplay(void);                              // 刷新LCD显示
void LCD_DisplayNumber(unsigned char row, unsigned char col, unsigned long num, unsigned char digits);  // LCD显示指定位数数字
void LCD_HandleKey(unsigned char key);                  // LCD按键处理函数

// 菜单初始化函数
void Menu_Init(void);                                   // 菜单系统初始化

// RTC相关函数
void UpdateRTCRefresh(void);                            // 更新RTC刷新状态

// 数据存储函数
void InitDataStorage(void);                             // 数据存储初始化（清空缓存、初始化索引）
void AddDataToSummary(unsigned char pid, unsigned char aid, unsigned char temp, unsigned char volt1, unsigned char volt2, unsigned char fosc);  // 添加数据到摘要存储
DataRecord* GetRecentDataByAID(unsigned char aid);       // 根据AID获取最新数据

// 事件处理函数
void DeleteAlarmEvent(unsigned char display_index);      // 删除指定报警事件
void ClearAllAlarmEvents(void);                         // 清空所有报警事件
void DeleteMaxTempEvent(unsigned char display_index);    // 删除指定最高温事件
void ClearAllMaxTempEvents(void);                       // 清空所有最高温事件

// 系统工具函数
unsigned long GetSystemTick(void);                      // 获取系统滴答计时器值

#endif  // __UART4_H__
