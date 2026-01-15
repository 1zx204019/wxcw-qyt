
#include "uart4.h"
#include <STC32G.H>
#include "lcd.h"
#include "config.h"
#include "relay.h"
#include "led.h"
#include "D1302.h"
#include <string.h>       // 字符串操作库
#include <math.h>         // 数学库（用于NTC温度计算的对数运算）

#ifndef NULL
#define NULL ((void*)0)
#endif

// ------------------- 全局变量定义 -------------------
// UART通信相关
unsigned char uart_rx_buff[UART_BUFF_SIZE] = {0};  // 接收缓冲区
unsigned char uart_rx_len = 0;                     // 接收长度
bit uart_rx_complete = 0;                          // 接收完成标志

// 系统计时相关
unsigned long delay_count = 0;                     // 延时计数器
unsigned long system_tick = 0;                     // 系统滴答计时器

// RTC时间相关
rtc_time_t current_rtc_time;
unsigned long rtc_refresh_counter = 0;
bit need_rtc_refresh = 1;
unsigned char current_system_date[3] = {0}; // 年、月、日（用于判断跨天）

// 数据存储相关
unsigned char slave_id;
DataRecord *recent_data;
Protocol_Data parsed_data = {0, 0, 0, 0.0f, 0, -990};
DataRecord data_summary[TOTAL_RECORDS] = {0};
unsigned char history_index[TOTAL_SLAVES] = {0};
unsigned char page18_hist_index[TOTAL_SLAVES] = {0}; // 每个从站独立跟踪历史索引

// 显示相关
unsigned char display_count = 0;  // 已显示的从站数量（最多3个）
extern PageType current_page;     // 当前页面
bit display_labels_initialized = 0;  // 固定标签是否已显示

// 菜单状态相关
MenuState menu_state;
unsigned char page1_selected = 0;          // PAGE_1选中项标识（0=TX01，1=TX02）
unsigned char page1_scroll_idx = 0;        // PAGE_1滚动控制：当前显示的起始索引（0-8）
static unsigned char last_prev_start_idx = 0;

// RTC编辑相关
RTC_EditState rtc_edit_state = RTC_EDIT_IDLE;  // 当前编辑状态
unsigned char rtc_edit_pos = 0;                // 选中的修改位置（0-9：年1-年2-月1-月2-日1-日2-时1-时2-分1-分2）
rtc_time_t edit_temp_time;                     // 临时存储待修改的时间

// 密码相关
unsigned char default_password[6] = {1, 2, 3, 4, 5, 6};
unsigned char current_password[6] = {0};
Password_EditState pwd_edit_state = PWD_EDIT_IDLE;
unsigned char pwd_edit_pos = 0;
static unsigned int pwd_flash_timer = 0; // 密码闪烁计时器（250ms）
static bit pwd_flash_state = 1;          // 密码闪烁触发标记（1=闪烁中，0=结束）
static unsigned char edit_temp_pwd[6] = {0}; // 临时密码存储

// 通用闪烁相关
static unsigned int flash_timer = 0;
static bit flash_state = 1;

// 报警事件相关
AlarmRecord alarm_events[MAX_ALARM_EVENTS] = {0};  // 报警事件记录数组
unsigned char alarm_event_count = 0;                // 当前报警事件数量
unsigned char alarm_event_next_index = 0;           // 下一个可用索引（循环覆盖）
unsigned char last_abnormal_status[TOTAL_SLAVES] = {0}; // 从站上一次异常状态

// 恢复事件相关
RecoveryRecord recovery_events[MAX_RECOVERY_EVENTS] = {0};
unsigned char recovery_event_count = 0;
unsigned char recovery_event_next_index = 0;

// 最高温相关
unsigned char max_temp_event_count = 0;
unsigned char max_temp_next_index = 0;
// uart4.c - 在全局变量定义部分添加
bit disable_today_max_calc = 0;  // 0=允许计算当天最高温，1=禁止
bit disable_max_temp_calc = 0; // 禁用最高温计算标记
short max_temps[3] = {-990, -990, -990}; // 前3高温度（初始化无效值）
rtc_time_t max_temp_times[3] = {0};      // 对应温度的发生时间
DailyMaxTemp daily_max_temps[3] = {0};   // 近3天最高温存储数组

Page19_AutoRecord page19_auto_records[3][4] = {0};
unsigned char page19_record_index[3] = {0};
unsigned long page19_last_read_time[3] = {0};

// 在 uart4.c 全局变量定义部分添加
static unsigned char page19_selected_record = 0; // 当前选中的记录索引（0-3）
static unsigned char page19_selected_slave = 0;  // 当前选中的从站索引（0-2）

// uart4.c - 在全局变量定义部分添加
static unsigned long last_save_time[TOTAL_SLAVES] = {0};  // 每个从站上次保存的时间

// 新增全局变量（在uart4.c全局变量区添加）
static unsigned long page19_trigger_timer = 0;  // PAGE19独立触发计时器

// 在全局变量区添加
unsigned char page18_scroll_page = 0;  // 当前滚动页（0-3，对应4页）

// ------------------- 静态函数声明 -------------------
// UART相关
static void UART4_ClearBuffer(unsigned char *ptr, unsigned int len);

// 显示相关
static void DisplayFixedLabels(void);
static void DisplayDetailPage(PageType page);
static void RefreshDisplay(void);

// 页面显示相关
static void DisplayPage1(void);    // 从站设备列表页面（PAGE_1）
static void DisplayPage2(void);    // 主功能菜单页面（PAGE_2）
static void DisplayPage3(void);    // 测量数据详情页面（PAGE_3）
static void DisplayPage4(void);    // 传感器异常状态页面（PAGE_4）
static void DisplayPage7(void);    // 历史记录查询子菜单页面（PAGE_7）
static void DisplayPage8(void);    // 设备维护菜单页面（PAGE_8）
static void DisplayPage10(void);   // 温度报警事件列表页面（PAGE_10）
static void DisplayPage11(void);   // 报警事件详细数据页面（PAGE_11）
static void DisplayPage12(void);   // 报警事件删除确认页面（PAGE_12）
static void DisplayPage13(void);   // 温度预警事件列表页面（PAGE_13）
static void DisplayPage14(void);   // 传感器恢复事件列表页面（PAGE_14）
static void DisplayPage15(void);   // 预警/恢复事件详细数据页面（PAGE_15）
static void DisplayPage16(void);   // 预警事件删除确认页面（PAGE_16）
static void DisplayPage17(void);   // 恢复事件删除确认页面（PAGE_17）
static void DisplayPage18(void);   // 历史数据查询页面（PAGE_18）
static void DisplayPage19(void);   // 清除历史数据确认页面（PAGE_19）
static void DisplayPage20(void);   // 历史数据删除确认页面（PAGE_20）
static void DisplayPage21(void);   // 最高温度查询页面（PAGE_21）
static void DisplayPage22(void);   // 最高温度清除确认页面（PAGE_22）
static void DisplayPage23(void);   // 最高温度删除确认页面（PAGE_23）
static void DisplayPage24(void);   // 修改日期时间页面（PAGE_24）
static void DisplayPage25(void);   // 修改密码页面（PAGE_25）
static void DisplayPage26(void);   // 恢复出厂设置确认页面（PAGE_26）

// 页面更新相关
static void UpdateDisplayForPage1(void);
static void UpdateDisplayForPage3(void);
static void DisplayAlarmEventsOnPage13(void);
static void DisplayAlarmEventsOnPage10(void);
static void DisplayRecoveryEventsOnPage14(void);
static void DisplayPassword(void);
static void DisplayRTCOnPage24(void);
static void RefreshPage2Arrow(void);
static void UpdatePage2MenuItems(unsigned char start_idx);
static void RefreshPage7Arrow(void);
static void UpdatePage7MenuItems(unsigned char start_idx);
static void RefreshPage8Arrow(void);
static void UpdatePage8MenuItems(unsigned char start_idx);

// 菜单辅助相关
static unsigned char GetPage2StartIndex(void);
static unsigned char GetPage7StartIndex(void);
static unsigned char GetPage8StartIndex(void);

// 按键处理相关
static void HandlePrevItem(void);
static void HandleNextItem(void);
static void HandleEnterKey(void);
static void HandleReturnKey(void);

// 协议解析相关
static bit Protocol_Check(unsigned char *frame);
static short ADC_To_Temp(unsigned int adc_val);
static void Protocol_Parse(unsigned char *frame);
static unsigned char TempShortToChar(short temp);

// 数据处理相关
static DataRecord* GetFixedSlaveData(unsigned char aid);
static void AddToHistoryData(unsigned char pid, unsigned char aid, unsigned char temp, unsigned int volt_mv);
static void CheckDailyMaxTemp(void);
static void CheckAndRecordAlarm(void);
static void RecordAlarmEvent(unsigned char pid, unsigned char aid, unsigned char temp, unsigned int volt_mv);
static void RecordRecoveryEvent(unsigned char pid, unsigned char aid, unsigned char abnormal_temp, unsigned char recovery_temp, unsigned int recovery_volt_mv, rtc_time_t abnormal_time);
static void DeleteRecoveryEvent(unsigned char display_index);
static void GetMaxTempRecords(short max_temps[], rtc_time_t max_temp_times[]);
static void GetMaxTempRecords_Ext(short max_temps[], rtc_time_t max_temp_times[], unsigned char *pid, unsigned char *aid, unsigned short *volt_mv, DataRecord **target_record);

// RTC相关
static void GetCurrentRTC(void);
static void UpdateRTCRefresh(void);
static void RTC_Edit_Init(void);
static void RTC_Switch_Pos(signed char step);
static void RTC_Adjust_Num(signed char step);
static void RTC_Save_Edit(void);

// 密码相关
static void PWD_Edit_Init(void);
static void PWD_Switch_Pos(signed char step);
static void PWD_Adjust_Num(signed char step);
static void PWD_Save_Edit(void);



// ------------------- 设备列表定义 -------------------
// PAGE_1设备列表（新增设备直接添加此处）
Page1_DevInfo page1_devices[] = {
    {1,  "01TX01"},  // AID=1，设备ID=01TX01
    {2,  "01TX02"},  // AID=2，设备ID=01TX02
    {3,  "01TX03"},  // AID=3，设备ID=01TX03
    {4,  "01TX04"},  // AID=4，设备ID=01TX04
    {5,  "01TX05"},  // AID=5，设备ID=01TX05
    {6,  "01TX06"},  // AID=6，设备ID=01TX06
    {7,  "01TX07"},  // AID=7，设备ID=01TX07
    {8,  "01TX08"},  // AID=8，设备ID=01TX08
    {9,  "01TX09"},  // AID=9，设备ID=01TX09
    {10, "01TX10"}   // AID=10，设备ID=01TX10
};

// ------------------- 菜单初始化 -------------------
void Menu_Init(void) {
    unsigned char i;
    
    // 页面状态初始化
    menu_state.current_page = PAGE_1;
    menu_state.prev_page = PAGE_1;
    menu_state.page_changed = 1;
    menu_state.menu_initialized = 0;
    
    // 选中项初始化
    menu_state.page2_selected = 0;
    menu_state.page3_selected = 0;
    menu_state.page7_selected = 0;
    menu_state.page8_selected = 0;
    menu_state.page10_selected = 0;
    menu_state.page11_selected = 0;
    menu_state.page12_selected = 0;
    menu_state.page13_selected = 0;
    menu_state.page14_selected = 0;
    menu_state.page15_selected = 0;
    menu_state.page16_selected = 0;
    menu_state.page17_selected = 0;
    menu_state.page18_selected = 0;
    menu_state.page19_selected = 0;
    menu_state.page20_selected = 0;
    menu_state.page21_selected = 0;
    menu_state.page22_selected = 0;
    menu_state.page23_selected = 0;
    menu_state.page24_selected = 0;
    menu_state.page25_selected = 0;
    menu_state.page26_selected = 0;
    
    // 报警状态跟踪数组初始化
    for (i = 0; i < TOTAL_SLAVES; i++) {
        last_abnormal_status[i] = 0;
    }
}

// ------------------- UART4初始化 -------------------
void UART4_Init(unsigned long baudrate) {
    unsigned int reload = (unsigned int)(65536UL - (FOSC / (4UL * baudrate)));
    
    // IO口配置：UART4映射到P5.2(RX)/P5.3(TX)
    EAXFR = 1;
    P_SW2 |= 0x80;
    S4_S = 1;
    P_SW2 &= ~0x80;
    
    // IO模式设置：P5.2高阻输入，P5.3推挽输出
    P5M1 |= (1 << 2);
    P5M0 &= ~(1 << 2);
    P5M1 &= ~(1 << 3);
    P5M0 |= (1 << 3);
    
    // 串口配置：8位数据，可变波特率，允许接收
    S4CON = 0x50;
    T4L = reload & 0xFF;
    T4H = reload >> 8;
    T4T3M |= 0x80;  // 启动定时器4
    T4T3M |= 0x20;  // 设置定时器4为UART4波特率发生器
    T4T3M &= ~0x10; // 定时器4设为1T模式
    
    // 中断配置
    IE2 |= 0x10;    // 开启UART4中断
    AUXR |= 0x80;   // STC32G专用：打开总中断开关
    
    // 系统初始化
    Menu_Init();
    rtc_init();
    rtc_check_and_init();  // 检查并设置时间
    InitDataStorage();
}

// ------------------- 发送函数 -------------------
void UART4_SendByte(unsigned char dat) {
    S4BUF = dat;
    while (!(S4CON & 0x02));  // 等待发送完成
    S4CON &= ~0x02;           // 清除发送完成标志
}

void UART4_SendString(unsigned char *str) {
    while (*str != '\0') {
        UART4_SendByte(*str);
        str++;
    }
}

void UART4_SendNumber(unsigned long num, unsigned char digits) {
    unsigned char i;
    unsigned long divisor = 1;
    
    // 计算除数（10^(digits-1)）
    for (i = 1; i < digits; i++) {
        divisor *= 10;
    }
    
    // 逐位发送数字
    for (i = 0; i < digits; i++) {
        UART4_SendByte((unsigned char)((num / divisor) % 10 + '0'));
        divisor /= 10;
    }
}

// ------------------- 定时器0初始化(用于延时) -------------------
void Timer0_Init(void) {
    TMOD &= 0xF0;  // 清除定时器0设置
    TMOD |= 0x01;  // 定时器0模式1（16位定时器）
    TL0 = 0x30;    // 24MHz 1T模式1ms初值
    TH0 = 0xF8;
    TF0 = 0;       // 清除溢出标志
    TR0 = 1;       // 启动定时器0
    ET0 = 1;       // 开启定时器0中断
    EA = 1;        // 开启总中断
}

// 定时器0中断服务函数(用于延时)
void Timer0_ISR(void) interrupt 1 {
    TL0 = 0x30;
    TH0 = 0xF8;
    
    if (delay_count > 0) {
        delay_count--;
    }
    
    // 改为调用函数
    SystemTick_Increment();  // 替换原来的 system_tick++
}

// ------------------- 清空缓冲区 -------------------
static void UART4_ClearBuffer(unsigned char *ptr, unsigned int len) {
    while (len--) {
        *ptr++ = 0;
    }
}

// ------------------- 协议相关函数 -------------------
static bit Protocol_Check(unsigned char *frame) {
    unsigned char fcs = frame[0] + frame[1] + frame[2] + frame[3] + frame[4];
    return (fcs == frame[5]) ? 1 : 0;
}

static short ADC_To_Temp(unsigned int adc_val) {
    double Rntc;
    double temp_k;
    double temp_c;
    
    // 无效ADC值判断
    if (adc_val == 0 || adc_val >= 4096) {
        return -990;
    }
    
    // 计算NTC电阻值
    Rntc = (4096.0 * (double)NTC_R_REF) / (double)adc_val - (double)NTC_R_REF;
    
    // 计算温度（开尔文→摄氏度）
    temp_k = 1.0 / (log(Rntc / (double)NTC_R0) / NTC_B_VALUE + 1.0 / NTC_T0);
    temp_c = temp_k - 273.15;
    
    // 温度范围校验
    if (temp_c < -40.0 || temp_c > 125.0) {
        return -990;
    }
    
    return (short)(temp_c * 10.0 + 0.5); // 保留1位小数，四舍五入
}

static void Protocol_Parse(unsigned char *frame) {
    unsigned int adc_val;
    
    parsed_data.PID = frame[0];
    parsed_data.AID = frame[1];
    adc_val = ((frame[3] & 0x0F) << 8) | frame[2];
    parsed_data.ADC_Value = adc_val;
    parsed_data.Bat_Voltage = (float)frame[4] / 10.0f;
    parsed_data.Check_OK = Protocol_Check(frame) ? 1 : 0;
    parsed_data.temperature = ADC_To_Temp(adc_val);
}

static unsigned char TempShortToChar(short temp) {
    if (temp == -990) {
        return 0;
    }
    return (unsigned char)(abs(temp) / 10);
}

// ------------------- 数据存储相关函数 -------------------
static DataRecord* GetFixedSlaveData(unsigned char aid) {
    unsigned char idx;
    
    if (aid < 1 || aid > TOTAL_SLAVES) {
        return NULL;
    }
    
    idx = aid - 1;
    if (data_summary[idx].is_valid) {
        return &data_summary[idx];
    }
    
    return NULL;
}

void InitDataStorage(void) {
    unsigned char i, j;
    unsigned char aid;
    unsigned short start_pos;
    
    // 初始化数据摘要存储
    for (i = 0; i < TOTAL_RECORDS; i++) {
        data_summary[i].is_valid = 0;
        data_summary[i].timestamp = 0;
        data_summary[i].save_time.year = 0;    // 新增：初始化save_time
        data_summary[i].save_time.mon = 0;
        data_summary[i].save_time.day = 0;
        data_summary[i].save_time.hour = 0;
        data_summary[i].save_time.min = 0;
        data_summary[i].save_time.sec = 0;
        data_summary[i].aid = 0;
        data_summary[i].pid = 0;
        data_summary[i].temp = 0;
        data_summary[i].volt1 = 0;
        data_summary[i].volt2 = 0;
        data_summary[i].fosc = 0;
    }
    
    // 初始化从站AID
    for (i = 0; i < TOTAL_SLAVES; i++) {
        data_summary[i].aid = i + 1;
    }
    
    // 初始化历史数据存储区
    for (i = 0; i < TOTAL_SLAVES; i++) {
        aid = i + 1;
        start_pos = HISTORY_DATA_START + (i * RECORDS_PER_SLAVE);
        
        for (j = 0; j < RECORDS_PER_SLAVE; j++) {
            data_summary[start_pos + j].aid = aid;
        }
    }
    
    // 初始化历史索引
    for (i = 0; i < TOTAL_SLAVES; i++) {
        history_index[i] = 0;
    }
    
    // 初始化报警事件数组
    for (i = 0; i < MAX_ALARM_EVENTS; i++) {
        alarm_events[i].is_valid = 0;
        alarm_events[i].pid = 0;
        alarm_events[i].aid = 0;
        alarm_events[i].temp = 0;
        alarm_events[i].volt_mv = 0;
    }
    alarm_event_count = 0;
    alarm_event_next_index = 0;
    
    // 初始化报警状态跟踪数组
    for (i = 0; i < TOTAL_SLAVES; i++) {
        last_abnormal_status[i] = 0;
    }
    
    // 初始化恢复事件数组
    for (i = 0; i < MAX_RECOVERY_EVENTS; i++) {
        recovery_events[i].is_valid = 0;
        recovery_events[i].pid = 0;
        recovery_events[i].aid = 0;
        recovery_events[i].abnormal_temp = 0;
        recovery_events[i].recovery_temp = 0;
        recovery_events[i].recovery_volt_mv = 0;
    }
    recovery_event_count = 0;
    recovery_event_next_index = 0;
    
		  // 初始化 PAGE19 自动记录数组
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            page19_auto_records[i][j].is_valid = 0;
        }
        page19_record_index[i] = 0;
        page19_last_read_time[i] = 0;
    }
    // === 新增：初始化时间跟踪数组 ===
    for (i = 0; i < TOTAL_SLAVES; i++) {
        last_save_time[i] = 0;  // 初始化2分钟间隔计时
    }
    
    // 清空UART接收缓冲区
    UART4_ClearBuffer(uart_rx_buff, UART_BUFF_SIZE);
    uart_rx_len = 0;
    uart_rx_complete = 0;
}



// uart4.c - 添加函数
// uart4.c - 修改 Page19_AutoReadHistoryData 函数


void AddDataToSummary(unsigned char pid, unsigned char aid, 
                     unsigned char temp, unsigned char volt1,
                     unsigned char volt2, unsigned char fosc) {
    unsigned char slave_idx;
    unsigned long current_time;
    DataRecord *recent_rec;
    unsigned short hist_start;
    unsigned char hist_idx;
    unsigned short hist_pos;
    DataRecord *hist_rec;
    
    // 无效数据过滤（保留原有逻辑）
    if (pid == 0 && aid == 0 && temp == 0 && 
        volt1 == 0 && volt2 == 0 && fosc == 0) {
        return;
    }
    
    // 从站AID校验（保留原有逻辑）
    if (aid < 1 || aid > TOTAL_SLAVES) {
        return;
    }
    
    slave_idx = aid - 1;
    current_time = GetSystemTick();
    recent_rec = &data_summary[slave_idx];
    
    // ========== 核心恢复：添加5分钟间隔判断 ==========
    if (last_save_time[slave_idx] == 0 || 
        (current_time - last_save_time[slave_idx]) >= 600000) { // 300000ms=5分钟
        last_save_time[slave_idx] = current_time;
        
        // 执行历史数据写入（原有逻辑保留）
        if (recent_rec->is_valid) {
            hist_start = HISTORY_DATA_START + (slave_idx * RECORDS_PER_SLAVE);
            hist_idx = history_index[slave_idx];
            hist_pos = hist_start + hist_idx;
            hist_rec = &data_summary[hist_pos];
            
            // 复制当前数据到历史记录
            hist_rec->pid = recent_rec->pid;
            hist_rec->aid = recent_rec->aid;
            hist_rec->temp = recent_rec->temp;
            hist_rec->volt1 = recent_rec->volt1;
            hist_rec->volt2 = recent_rec->volt2;
            hist_rec->fosc = recent_rec->fosc;
            GetCurrentRTC();  // 更新当前RTC时间
            hist_rec->save_time = current_rtc_time;  // 保存RTC时间
            hist_rec->timestamp = current_time;
            hist_rec->is_valid = 1;
            
            // 更新历史索引（循环覆盖，保留原有逻辑）
            history_index[slave_idx] = (hist_idx + 1) % RECORDS_PER_SLAVE;
        }
    }
    // ========== 结束恢复 ==========
    
    // 更新当前实时数据（保留原有逻辑不变）
    recent_rec->pid = parsed_data.PID;
    recent_rec->aid = parsed_data.AID;
    recent_rec->temp = TempShortToChar(parsed_data.temperature);
    recent_rec->volt1 = (unsigned char)(parsed_data.Bat_Voltage * 10);
    recent_rec->volt2 = 0;
    recent_rec->fosc = 0;
    recent_rec->timestamp = current_time;
    recent_rec->is_valid = parsed_data.Check_OK ? 1 : 0;
}







DataRecord* GetRecentDataByAID(unsigned char aid) {
    unsigned char idx;
    
    if (aid < 1 || aid > TOTAL_SLAVES) {
        return NULL;
    }
    
    idx = aid - 1;
    if (data_summary[idx].is_valid) {
        return &data_summary[idx];
    }
    
    return NULL;
}

// ------------------- 显示固定标签 -------------------
static void DisplayFixedLabels(void) {
    if (display_labels_initialized == 0) {
        LCD_Clear();  // 清屏
        
        // 根据当前页面显示对应内容
        switch (menu_state.current_page) {
            case PAGE_1:
                DisplayPage1();
                break;
                
            case PAGE_2:
                DisplayPage2();
                break;
                
            case PAGE_3:
                DisplayPage3();
                break;
                
            case PAGE_4:
                DisplayPage4();
                break;
                
            case PAGE_7:
                DisplayPage7();
                break;
                
            case PAGE_8:
                DisplayPage8();
                break;
                
            case PAGE_10:
                DisplayPage10();
                break;
                
            case PAGE_11:
                DisplayPage11();
                break;
                
            case PAGE_12:
                DisplayPage12();
                break;
                
            case PAGE_13:
                DisplayPage13();
                break;
                
            case PAGE_14:
                DisplayPage14();
                break;
                
            case PAGE_15:
                DisplayPage15();
                break;
                
            case PAGE_16:
                DisplayPage16();
                break;
                
            case PAGE_17:
                DisplayPage17();
                break;
                
            case PAGE_18:
                DisplayPage18();
                break;
                
            case PAGE_19:
                DisplayPage19();
                break;
                
            case PAGE_20:
                DisplayPage20();
                break;
                
            case PAGE_21:
                DisplayPage21();
                break;
                
            case PAGE_22:
                DisplayPage22();
                break;
                
            case PAGE_23:
                DisplayPage23();
                break;
                
            case PAGE_24:
                DisplayPage24();
                break;
                
            case PAGE_25:
                DisplayPage25();
                break;
                
            case PAGE_26:
                DisplayPage26();
                break;
                
            case PAGE_5:
            case PAGE_9:
                DisplayDetailPage(menu_state.current_page);
                break;
                
            default:
                break;
        }
        
        display_labels_initialized = 1;
    }
}

// ------------------- PAGE_1 显示函数 -------------------
static void DisplayPage1(void) {
    unsigned char i;
    DataRecord* dev_data = NULL;
    unsigned char dev_id_buf[8];  // 存储设备ID
    
    // 第0行：标题栏
    LCD_DISPLAYCHAR_NEW(0, 0, 0, 0);   // "从"
    LCD_DISPLAYCHAR_NEW(0, 8, 1, 0);   // "站"
    LCD_DISPLAYCHAR_NEW(0, 48, 0, 1);  // "温"
    LCD_DISPLAYCHAR_NEW(0, 56, 1, 1);  // "度"
    LCD_DISPLAYCHAR_NEW(0, 64, 0, 6);  // "("
    LCD_DISPLAYCHAR_NEW(0, 72, 0, 2);  // "°"
    LCD_DISPLAYCHAR_NEW(0, 80, 0, 7);  // "C"
    LCD_DISPLAYCHAR_NEW(0, 88, 0, 3);  // "电"
    LCD_DISPLAYCHAR_NEW(0, 96, 1, 3);  // "压"
    LCD_DISPLAYCHAR_NEW(0, 104, 2, 3); // 空
    LCD_DISPLAYCHAR_NEW(0, 112, 3, 3); // 空
    LCD_DISPLAYCHAR_NEW(0, 120, 4, 3); // "(mv)"
    
    // 清空数据显示区域（第2行、第4行）
    for (i = 0; i < 2; i++) {
        unsigned char row = (i == 0) ? 2 : 4;  // 第一个从站→2行，第二个→4行
        LCD_DisplayChar(row, 0, ' ');          // 清空箭头位置
        LCD_DisplayString(row, 8, (unsigned char*)"      ");  // 清空设备ID
        LCD_DisplayString(row, 48, (unsigned char*)"     ");  // 清空温度
        LCD_DisplayString(row, 88, (unsigned char*)"    ");   // 清空电压
    }
    
    // 显示当前2个从站数据（根据滚动索引page1_scroll_idx）
    for (i = 0; i < 2; i++) {
        unsigned char dev_idx = page1_scroll_idx + i;  // 从站在列表中的索引
        unsigned char row = (i == 0) ? 2 : 4;          // 固定有效行
        Page1_DevInfo* dev = &page1_devices[dev_idx];
        
        // 生成设备ID（01TX01-01TX10）
        dev_id_buf[0] = '0';
        dev_id_buf[1] = '1';
        dev_id_buf[2] = 'T';
        dev_id_buf[3] = 'X';
        dev_id_buf[4] = '0' + ((dev_idx + 1) / 10);    // 十位
        dev_id_buf[5] = '0' + ((dev_idx + 1) % 10);    // 个位
        dev_id_buf[6] = '\0';
        LCD_DisplayString(row, 8, dev_id_buf);
        
        // 显示温度
        dev_data = GetFixedSlaveData(dev->aid);
        if (dev_data != NULL && dev_data->is_valid) {
            LCD_DisplayNumber(row, 64, (unsigned long)dev_data->temp, 2);
        } else {
            LCD_DisplayString(row, 64, (unsigned char*)"--");
        }
        
        // 显示电压（mV）
        if (dev_data != NULL && dev_data->is_valid) {
            unsigned int volt_mv = dev_data->volt1 * 100;
            LCD_DisplayNumber(row, 88, (unsigned long)volt_mv, 4);
        } else {
            LCD_DisplayString(row, 88, (unsigned char*)"----");
        }
        
        // 显示选中箭头（仅第一个从站行显示）
        if (i == 0) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // 箭头指向当前起始从站
        }
    }
    
    // 第6行：功能提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "上"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
    LCD_DISPLAYCHAR_NEW(6, 56, 3, 4);  // "下"
    LCD_DISPLAYCHAR_NEW(6, 64, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 72, 2, 4);  // "页"
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 5); // "设"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 5); // "置"
    
    display_labels_initialized = 1;
}


// 辅助函数：根据选中项索引，计算当前应显示的菜单项起始索引（确保选中项在显示范围内）
static unsigned char GetPage2StartIndex(void) {
    unsigned char start_idx = 0;
    unsigned char max_start_idx;
    
    // 选中项≥2时，起始索引=选中项-2（保证选中项在第3个位置）
    if (menu_state.page2_selected >= 2) {
        start_idx = menu_state.page2_selected - 2;
    }
    
    // 边界保护：起始索引不能超过最大可用值（PAGE2_ITEM_COUNT=5，最大起始索引=5-3=2）
    max_start_idx = PAGE2_ITEM_COUNT - 3;
    if (start_idx > max_start_idx) {
        start_idx = max_start_idx;
    }
    
    return start_idx;
}

// 确保这个函数存在且正确定义
static void RefreshPage2Arrow(void) {
    unsigned char start_idx = GetPage2StartIndex();
    unsigned char i;
    
    for (i = 0; i < 3; i++) {
        unsigned char item_idx = start_idx + i;  // 当前显示的菜单项实际索引
        unsigned char row = i * 2;
        
        // 箭头严格对应实际选中项（page2_selected）
        if (item_idx == menu_state.page2_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // 选中箭头
        } else {
            LCD_DisplayChar(row, 0, ' ');        // 清除原箭头
        }
    }
}

// ------------------- 显示PAGE_2(功能菜单) -------------------
// 新增：仅首次初始化时调用（绘制菜单项文本和功能提示）
// 局部更新PAGE_2的3个菜单项文本（仅换内容，不清屏）
static void UpdatePage2MenuItems(unsigned char start_idx) {
    unsigned char i;
    
    for (i = 0; i < 3; i++) {
        unsigned char item_idx = start_idx + i;
        unsigned char row = i * 2;
        
        // 关键：清空整行文本区域（从第8列到第48列，覆盖所有菜单项长度）
        LCD_DisplayString(row, 8, (unsigned char*)"                "); // 16个空格，确保旧文本清除
        
        // 按菜单项索引绘制对应文本，避免错位
        switch(item_idx) {
            case MENU_MEASURE_DATA: // 0：测量数据（4个字符）
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 8);   // "测"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 8);  // "量"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 8);  // "数"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 8);  // "据"
                break;
                
            case MENU_SENSOR_STATUS: // 1：传感器状态（5个字符）
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 9);   // "传"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 9);  // "感"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 9);  // "器"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 9);  // "状"
                LCD_DISPLAYCHAR_NEW(row, 40, 4, 9);  // "态"
                break;
                
            case MENU_PARAM_QUERY: // 2：参数查询（4个字符）
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 10);  // "参"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 10); // "数"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 10); // "查"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 10); // "询"
                break;
                
            case MENU_HISTORY_RECORD: // 3：历史记录（4个字符）
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 13);  // "历"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 13); // "史"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 13); // "记"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 13); // "录"
                break;
                
            case MENU_DEVICE_MAINT: // 4：设备维护（4个字符）
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 14);  // "设"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 14); // "备"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 14); // "维"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 14); // "护"
                break;
                
            default:
                // 无效索引时显示空（避免乱码）
                LCD_DisplayString(row, 8, (unsigned char*)"    ");
                break;
        }
    }
}

// 新增：仅刷新箭头位置（核心局部刷新函数）
static void DisplayPage2(void) {
    unsigned char start_idx = GetPage2StartIndex();
    
    if (display_labels_initialized == 0) {
        LCD_Clear();
        // 绘制功能提示（仅首次绘制）
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);    // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);    // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);   // "上"
        LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 80, 0, 12);  // "进"
        LCD_DISPLAYCHAR_NEW(6, 88, 1, 12);  // "入"
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"
        
        // 首次初始化时绘制文本
        UpdatePage2MenuItems(start_idx);
        display_labels_initialized = 1;
    }
    
    // 总是刷新箭头位置，确保实时性
    RefreshPage2Arrow();
}

static void DisplayPage3(void) {
    unsigned char module_index = menu_state.page3_selected;  // 0-9对应TX01-TX10
    unsigned char tx_number = module_index + 1;             // TX编号=索引+1（1-10）
    unsigned char aid = tx_number;                          // AID与TX编号一一对应（1-10）
    DataRecord* dev_data = GetRecentDataByAID(aid);         // 根据AID获取数据
    
    if (display_labels_initialized == 0) {
        LCD_Clear();
        
        // 第0行：固定标签 "PID" 和 "TX"
        LCD_DisplayString(0, 0, (unsigned char*)"PID");
        LCD_DISPLAYCHAR_NEW(0, 24, 0, 16); // 冒号 ":"
        LCD_DisplayString(0, 72, (unsigned char*)"TX");
        LCD_DISPLAYCHAR_NEW(0, 88, 0, 16); // 冒号 ":"
        
        // 第2行：温度标签
        LCD_DISPLAYCHAR_NEW(2, 0, 0, 1);   // "温"
        LCD_DISPLAYCHAR_NEW(2, 8, 1, 1);   // "度"
        LCD_DISPLAYCHAR_NEW(2, 16, 0, 15); // ":"
        
        // 第4行：电压标签
        LCD_DISPLAYCHAR_NEW(4, 0, 0, 3);   // "电"
        LCD_DISPLAYCHAR_NEW(4, 8, 1, 3);   // "压"
        LCD_DISPLAYCHAR_NEW(4, 16, 0, 15); // ":"
        LCD_DisplayString(4, 64, (unsigned char*)"mv");
        
        // 第6行：功能提示
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);    // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);    // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 4, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 60, 3, 4);   // "上"
        LCD_DISPLAYCHAR_NEW(6, 68, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 76, 4, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"
        
        display_labels_initialized = 1;
    }
    
    // 清空数据显示区域
    LCD_DisplayString(0, 32, (unsigned char*)"  ");    // PID（2位）
    LCD_DisplayString(0, 96, (unsigned char*)"  ");    // TX编号（2位）
    LCD_DisplayString(2, 24, (unsigned char*)"--");    // 温度占位符
    LCD_DisplayString(4, 24, (unsigned char*)"----");  // 电压占位符
    
    // 显示TX编号（1-10，格式两位数：01-10）
    LCD_DisplayNumber(0, 96, tx_number, 2);
    
    // 显示PID：未接数据时默认01，有数据时显示真实PID
    if (dev_data != NULL && dev_data->is_valid) {
        LCD_DisplayNumber(0, 32, dev_data->pid, 2);
    } else {
        LCD_DisplayString(0, 32, (unsigned char*)"01"); // 占位PID=01
    }
    
    // 显示温度：有数据时更新，无数据显示占位符
    if (dev_data != NULL && dev_data->is_valid) {
        LCD_DisplayNumber(2, 24, dev_data->temp, 2);
    }
    
    // 显示电压：有数据时更新，无数据显示占位符
    if (dev_data != NULL && dev_data->is_valid) {
        unsigned int volt_mv = dev_data->volt1 * 100;
        LCD_DisplayNumber(4, 24, volt_mv, 4);
    }
}

// ------------------- 显示PAGE_4(传感器状态) -------------------
static void DisplayPage4(void)
{
    DataRecord* dev_data = NULL;
    unsigned char abnormal_count = 0;
    unsigned char i;
    
    // 清屏
    LCD_Clear();
    
    // 固定字符显示（保持原有格式）
    LCD_DisplayString(0, 0, "PID");
    LCD_DISPLAYCHAR_NEW(0, 24, 0, 16);

    LCD_DisplayString(2, 0, "PID");
    LCD_DISPLAYCHAR_NEW(2, 24, 0, 16);

    LCD_DisplayString(4, 0, "PID");
    LCD_DISPLAYCHAR_NEW(4, 24, 0, 16);

    // 功能提示（保持不变）
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);    // "下"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);    // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);   // "页"
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);   // "上"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);   // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);   // "页"
    LCD_DISPLAYCHAR_NEW(6, 80, 0, 26);  // "详"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 26);  // "情"
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"

    // 检查所有模块，只显示温度大于25度的异常模块
    // 假设有2个模块：TX1 (AID=1) 和 TX2 (AID=2)
    for (i = 0; i < TOTAL_SLAVES; i++) {
        // 获取模块数据（AID从1开始）
        dev_data = GetRecentDataByAID(i + 1);

        // 检查数据是否有效且温度大于25度
        if (dev_data != NULL && dev_data->is_valid && dev_data->temp > 25) {
            // 根据异常模块数量确定显示行
            unsigned char row;
            switch (abnormal_count) {
                case 0: row = 0; break;  // 第一个异常显示在第0行
                case 1: row = 2; break;  // 第二个异常显示在第2行
                case 2: row = 4; break;  // 第三个异常显示在第4行
                default: row = 0; break; // 超过3个只显示前3个
            }

            // 显示PID值
            LCD_DisplayNumber(row, 32, dev_data->pid, 2);

            // 显示"异常"字样
            LCD_DISPLAYCHAR_NEW(row, 56, 1, 31);  // 异
            LCD_DISPLAYCHAR_NEW(row, 64, 2, 31);  // 常

            // 显示TX编号
            LCD_DisplayString(row, 88, "TX");
            LCD_DisplayNumber(row, 104, i + 1, 1);  // 显示TX1或TX2

            abnormal_count++;

            // 最多显示3个异常模块
            if (abnormal_count >= 3) {
                break;
            }
        }
    }
}

// 计算 PAGE_7 起始索引
static unsigned char GetPage7StartIndex(void) {
    unsigned char start_idx = 0;
    unsigned char max_start_idx;
    
    // 选中项≥2时，起始索引=选中项-2（保证选中项在第3个位置，与PAGE_2一致）
    if (menu_state.page7_selected >= 2) {
        start_idx = menu_state.page7_selected - 2;
    }
    
    // 边界保护：不超过最大可用起始索引（PAGE7_ITEM_COUNT=5，最大起始索引=5-3=2）
    max_start_idx = PAGE7_ITEM_COUNT - 3;
    if (start_idx > max_start_idx) {
        start_idx = max_start_idx;
    }
    
    return start_idx;
}

// 局部更新 PAGE_7 菜单项文本
static void UpdatePage7MenuItems(unsigned char start_idx) {
    unsigned char i;
    unsigned char item_idx;
    unsigned char row;
    
    for (i = 0; i < 3; i++) {
        item_idx = start_idx + i;
        row = i * 2; // 与PAGE_2一致，行间距为2

        // 清空当前菜单项文本区域（16个空格，确保覆盖旧文本）
        LCD_DisplayString(row, 8, (unsigned char*)"                ");

        if (item_idx < PAGE7_ITEM_COUNT) {
            // 保留原有PAGE_7菜单项文本逻辑
            switch(item_idx) {
                case MENU_SENSOR_EVENT_1://温度报警事件记录
                    LCD_DISPLAYCHAR_NEW(row, 8, 0, 1);   
                    LCD_DISPLAYCHAR_NEW(row, 16, 1, 1);  
                    LCD_DISPLAYCHAR_NEW(row, 24, 0, 19); 
                    LCD_DISPLAYCHAR_NEW(row, 32, 1, 19); 
                    LCD_DISPLAYCHAR_NEW(row, 40, 2, 19); 
                    LCD_DISPLAYCHAR_NEW(row, 48, 3, 19); 
                    LCD_DISPLAYCHAR_NEW(row, 56, 4, 19); 
                    LCD_DISPLAYCHAR_NEW(row, 64, 5, 19); 
                    break;
                    
                case MENU_SENSOR_EVENT_2://温度预警时间记录
                    LCD_DISPLAYCHAR_NEW(row, 8, 0, 1);   
                    LCD_DISPLAYCHAR_NEW(row, 16, 1, 1);  
                    LCD_DISPLAYCHAR_NEW(row, 24, 0, 20); 
                    LCD_DISPLAYCHAR_NEW(row, 32, 1, 20); 
                    LCD_DISPLAYCHAR_NEW(row, 40, 2, 20); 
                    LCD_DISPLAYCHAR_NEW(row, 48, 3, 20); 
                    LCD_DISPLAYCHAR_NEW(row, 56, 4, 20); 
                    LCD_DISPLAYCHAR_NEW(row, 64, 5, 20); 
                    break;
                    
                case MENU_SENSOR_EVENT_3://传感器恢复事件记录
                    LCD_DISPLAYCHAR_NEW(row, 8, 0, 21);
                    LCD_DISPLAYCHAR_NEW(row, 16, 1, 21);
                    LCD_DISPLAYCHAR_NEW(row, 24, 2, 21);
                    LCD_DISPLAYCHAR_NEW(row, 32, 3, 21);
                    LCD_DISPLAYCHAR_NEW(row, 40, 4, 21);
                    LCD_DISPLAYCHAR_NEW(row, 48, 5, 21);
                    LCD_DISPLAYCHAR_NEW(row, 56, 6, 21);
                    LCD_DISPLAYCHAR_NEW(row, 64, 7, 21);
                    LCD_DISPLAYCHAR_NEW(row, 72, 8, 21);
                    break;
                    
                case MENU_HISTORY_DATA_QUERY://历史数据查询
                    LCD_DISPLAYCHAR_NEW(row, 8, 0, 23); 
                    LCD_DISPLAYCHAR_NEW(row, 16, 1, 23);
                    LCD_DISPLAYCHAR_NEW(row, 24, 2, 23);
                    LCD_DISPLAYCHAR_NEW(row, 32, 3, 23);
                    LCD_DISPLAYCHAR_NEW(row, 40, 4, 23);
                    LCD_DISPLAYCHAR_NEW(row, 48, 5, 23);
                    break;
                    
                case MENU_MAX_TEMP_QUERY://最高历史温度查询
                    LCD_DISPLAYCHAR_NEW(row, 8, 0, 24);  
                    LCD_DISPLAYCHAR_NEW(row, 16, 1, 24); 
                    LCD_DISPLAYCHAR_NEW(row, 24, 0, 23); 
                    LCD_DISPLAYCHAR_NEW(row, 32, 1, 23); 
                    LCD_DISPLAYCHAR_NEW(row, 40, 0, 1);  
                    LCD_DISPLAYCHAR_NEW(row, 48, 1, 1);  
                    LCD_DISPLAYCHAR_NEW(row, 56, 4, 23); 
                    LCD_DISPLAYCHAR_NEW(row, 64, 5, 23); 
                    break;
            }
        } else {
            LCD_DisplayString(row, 8, (unsigned char*)"    ");
        }
    }
}

// 局部刷新 PAGE_7 箭头位置
static void RefreshPage7Arrow(void) {
    unsigned char start_idx = GetPage7StartIndex();
    unsigned char i;
    unsigned char item_idx;
    unsigned char row;
    
    for (i = 0; i < 3; i++) {
        item_idx = start_idx + i;
        row = i * 2;
        
        // 箭头严格对应选中项，与PAGE_2逻辑一致
        if (item_idx == menu_state.page7_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // 显示选中箭头
        } else {
            LCD_DisplayChar(row, 0, ' ');        // 清除未选中项箭头
        }
    }
}

// ------------------- 修正DisplayPage7(显示正确的菜单名称) -------------------
static void DisplayPage7(void) {
    static unsigned char last_start_idx = 0xFF; // 记录上一次起始索引
    unsigned char start_idx = GetPage7StartIndex();
    
    // 首次初始化时清屏并绘制固定功能提示
    if (display_labels_initialized == 0) {
        LCD_Clear();
        
        // 绘制底部功能提示（仅首次绘制）
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);    // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);    // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);   // "上"
        LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 80, 0, 22);  // "进"
        LCD_DISPLAYCHAR_NEW(6, 88, 1, 22);  // "入"
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"

        display_labels_initialized = 1;
        last_start_idx = 0xFF; // 强制首次更新文本
    }
    
    // 仅当起始索引变化时，更新菜单文本（避免冗余刷新）
    if (last_start_idx != start_idx) {
        UpdatePage7MenuItems(start_idx);
        last_start_idx = start_idx;
    }
    
    // 每次调用都刷新箭头，确保选中状态实时反馈
    RefreshPage7Arrow();
}

static void DisplayPage8(void) {
    unsigned char start_idx;
    start_idx = GetPage8StartIndex(); // 固定返回0
    
    if (display_labels_initialized == 0) {
        LCD_Clear();          // 仅首次初始化时清屏
        
        // 绘制功能提示（复用原有底部提示，保持不变）
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);    // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);    // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);   // "上"
        LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 80, 0, 22);  // "进"
        LCD_DISPLAYCHAR_NEW(6, 88, 1, 22);  // "入"
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"
        
        // 首次初始化时绘制一次文本（后续不再重复绘制）
        UpdatePage8MenuItems(start_idx);
        display_labels_initialized = 1;
    }
    
    // 仅刷新箭头，不重复绘制文本（核心优化：减少冗余操作）
    RefreshPage8Arrow();
}

// 计算 PAGE_8 起始索引（3个选项无需滚动，固定返回0）
static unsigned char GetPage8StartIndex(void) {
    unsigned char start_idx;
    start_idx = 0;
    
    // 因只有3个选项，无需滚动，起始索引始终为0
    if (start_idx > (3 - 3)) { // 3个选项，显示3个，最大起始索引0
        start_idx = 3 - 3;
    }
    
    return start_idx;
}

// 局部更新 PAGE_8 菜单项文本（复用原有文本逻辑，仅清空文本区域）
static void UpdatePage8MenuItems(unsigned char start_idx) {
    unsigned char i;
    unsigned char item_idx;
    unsigned char row;     
    
    for (i = 0; i < 3; i++) {
        item_idx = start_idx + i; // 当前显示的菜单项索引（0/1/2）
        row = i * 2; // 菜单项显示行（0、2、4行）

        // 清空当前菜单项文本区域（仅清文本，不清箭头）
        LCD_DisplayString(row, 8, (unsigned char*)"                  ");

        // 显示对应的菜单项文本（复用原有 PAGE_8 文本逻辑）
        switch(item_idx) {
            case 0:  // 修改日期时间
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 12);   // "修"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 12);  // "改"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 12);  // "日"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 12);  // "期"
                LCD_DISPLAYCHAR_NEW(row, 40, 2, 20);  // "时"
                LCD_DISPLAYCHAR_NEW(row, 48, 3, 20);  // "间"
                break;

            case 1:  // 修改密码
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 12);   // "修"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 12);  // "改"
                LCD_DISPLAYCHAR_NEW(row, 24, 4, 12);  // "密"
                LCD_DISPLAYCHAR_NEW(row, 32, 5, 12);  // "码"
                break;

            case 2:  // 恢复出厂设置
                LCD_DISPLAYCHAR_NEW(row, 8, 2, 5);   // "恢"
                LCD_DISPLAYCHAR_NEW(row, 16, 3, 5);  // "复"
                LCD_DISPLAYCHAR_NEW(row, 24, 4, 5);  // "出"
                LCD_DISPLAYCHAR_NEW(row, 32, 5, 5);  // "厂"
                LCD_DISPLAYCHAR_NEW(row, 40, 0, 5);  // "设"
                LCD_DISPLAYCHAR_NEW(row, 48, 1, 5);  // "置"
                break;
        }
    }
}

// 局部刷新 PAGE_8 箭头位置（严格对应选中项）
static void RefreshPage8Arrow(void) {
    unsigned char start_idx = GetPage8StartIndex();
    unsigned char i;
    unsigned char item_idx;
    unsigned char row;
    
    for (i = 0; i < 3; i++) {
        item_idx = start_idx + i; // 当前显示的菜单项实际索引（0/1/2）
        row = i * 2;
        
        // 箭头仅在选中项显示，其他位置清空
        if (item_idx == menu_state.page8_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // 选中箭头
        } else {
            LCD_DisplayChar(row, 0, ' ');        // 清除原箭头
        }
    }
}

// ------------------- 显示PAGE_10(温度报警事件列表) -------------------
static void DisplayPage10(void)
{
    unsigned char i;
    unsigned char row;

    // 只在需要初始化时清屏和绘制固定标签
    if (display_labels_initialized == 0) {
        LCD_Clear();

        // 绘制固定标签
        for (i = 0; i < 3; i++) {
            row = i * 2;  

            if (i == menu_state.page10_selected) {
                LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  
            } else {
                LCD_DisplayChar(row, 0, ' ');
            }

            LCD_DISPLAYCHAR_NEW(row, 8, 2, 19);   // "事"
            LCD_DISPLAYCHAR_NEW(row, 16, 3, 19);  // "件"
            LCD_DisplayChar(row, 24, '1' + i);    // 编号

            // 显示"时间："标签（位置可能需要调整）
            LCD_DISPLAYCHAR_NEW(row, 40, 2, 20); // "时"
            LCD_DISPLAYCHAR_NEW(row, 48, 3, 20); // "间"
            LCD_DISPLAYCHAR_NEW(row, 56, 0, 15); // ":"
        }

        // 功能提示
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);    // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);    // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);   // "上"
        LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);   // "页"
        LCD_DISPLAYCHAR_NEW(6, 80, 0, 26);  // "详"
        LCD_DISPLAYCHAR_NEW(6, 88, 1, 26);  // "情"
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"

        display_labels_initialized = 1;
    }

    // 显示报警事件数据（只刷新数据部分，不清屏）
    DisplayAlarmEventsOnPage10();
}
// ------------------- 显示PAGE_11(报警事件详细数据) -------------------
static void DisplayPage11(void) {
    unsigned char display_index;
    AlarmRecord* event = NULL;
    
    // 只在需要初始化时显示固定内容
    if (display_labels_initialized == 0) {
        LCD_Clear();
        
        // 第一行: ID: PID=    AID=
        LCD_DisplayString(0, 0, "ID");
        LCD_DISPLAYCHAR_NEW(0, 16, 0, 15);  
        LCD_DisplayString(0, 24, "PID");
			  LCD_DISPLAYCHAR_NEW(0, 48, 0, 16);  // 
        LCD_DisplayString(0, 80, "TX");
        LCD_DISPLAYCHAR_NEW(0, 96, 0, 16);  // 
        // 第二行: 温度:
        LCD_DISPLAYCHAR_NEW(2, 0, 0, 1);  // "温"
        LCD_DISPLAYCHAR_NEW(2, 8, 1, 1);  // "度"
        LCD_DISPLAYCHAR_NEW(0, 16, 0, 15); 
        
        // 第三行: 电压:
        LCD_DISPLAYCHAR_NEW(4, 0, 0, 33);  // "电"
        LCD_DISPLAYCHAR_NEW(4, 8, 1, 33);  // "压"
        LCD_DISPLAYCHAR_NEW(0, 16, 0, 15); 
        
        // 第四行: 功能提示
        // 按1删除
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 27);
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 27);
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 27);
        LCD_DISPLAYCHAR_NEW(6, 24, 3, 27);
        
        // 按2返回
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
        
        display_labels_initialized = 1;
    }
    
    // 获取选中的报警事件
    if (menu_state.page10_selected < alarm_event_count) {
        display_index = (alarm_event_next_index - menu_state.page10_selected - 1 + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS;
        event = &alarm_events[display_index];
    }
    
    // 清空数据显示区域
    LCD_DisplayString(0, 56, (unsigned char*)"  ");   // PID显示区域
    LCD_DisplayString(0, 104, (unsigned char*)"  ");  // TX显示区域
    LCD_DisplayString(2, 24, (unsigned char*)"  ");   // 温度显示区域
    LCD_DisplayString(4, 24, (unsigned char*)"    "); // 电压显示区域
    
    // 显示详细数据
    if (event != NULL && event->is_valid) {
        // 显示PID
        LCD_DisplayNumber(0, 56, event->pid, 2);
        
        // 显示AID
        LCD_DisplayNumber(0, 104, event->aid, 2);
        
        // 显示温度
        LCD_DisplayNumber(2, 24, event->temp, 2);
        
        // 显示电压
        LCD_DisplayNumber(4, 24, event->volt_mv, 4);
    } else {
        // 显示"--"表示无数据
        LCD_DisplayString(0, 56, (unsigned char*)"--");
        LCD_DisplayString(0, 104, (unsigned char*)"--");
        LCD_DisplayString(2, 24, (unsigned char*)"--");
        LCD_DisplayString(4, 24, (unsigned char*)"----");
    }
}  

// ------------------- 显示PAGE_12(删除确认) -------------------
static void DisplayPage12(void) {
    LCD_Clear();
    
	  LCD_DISPLAYCHAR_NEW(2, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 28);  // "认"
    LCD_DISPLAYCHAR_NEW(2, 16, 0, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 24, 1, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 32, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 40, 3, 27);  // "除"
	  LCD_DISPLAYCHAR_NEW(2, 48, 2, 28);  // "吗"
    
    // 确认提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 28);  // "认"
    
    // 返回提示
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}

// ------------------- 显示PAGE_13(温度预警时间列表) -------------------
static void DisplayPage13(void) {
    unsigned char i;
    unsigned char row;
    
    if (display_labels_initialized == 0) {
        LCD_Clear();
        
        // 绘制固定标签：箭头随 page13_selected 变化
        for (i = 0; i < 3; i++) {
            row = i * 2;  
            // 关键：箭头指向当前 page13_selected 选中项（而非 page10_selected）
            if (i == menu_state.page13_selected) {
                LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // 显示选中箭头
            } else {
                LCD_DisplayChar(row, 0, ' '); // 清除未选中项箭头
            }
            
            LCD_DISPLAYCHAR_NEW(row, 8, 2, 19);   // "事"
            LCD_DISPLAYCHAR_NEW(row, 16, 3, 19);  // "件"
            LCD_DisplayChar(row, 24, '1' + i);    // 事件编号
            
            LCD_DISPLAYCHAR_NEW(row, 40, 2, 20); // "时"
            LCD_DISPLAYCHAR_NEW(row, 48, 3, 20); // "间"
            LCD_DISPLAYCHAR_NEW(row, 56, 0, 15); // ":"
        }
        
        // 功能提示（与 PAGE_10 一致）
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
        LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "上"
        LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
        LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "页"
        LCD_DISPLAYCHAR_NEW(6, 80, 0, 26); // "详"
        LCD_DISPLAYCHAR_NEW(6, 88, 1, 26); // "情"
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"
        
        display_labels_initialized = 1;
    }
    
    // 显示预警事件数据（复用报警的显示函数，或单独实现预警数据显示）
    DisplayAlarmEventsOnPage13(); // 新增：预警事件数据显示函数
}


// 新增：PAGE_13 预警事件数据显示
// 修复后：PAGE_13 复用报警事件数组，无需单独预警变量
static void DisplayAlarmEventsOnPage13(void) {
    unsigned char i;
    unsigned char row;
    unsigned char display_index;
    AlarmRecord* event = NULL;
    
    for (i = 0; i < 3; i++) {
        row = i * 2;
        LCD_DisplayString(row, 64, (unsigned char*)"            "); // 清空数据区
        
        // 复用报警事件的计数和数组（删除所有 warning_xxx 相关）
        if (i < alarm_event_count) {
            // 复用报警事件的索引计算逻辑（MAX_ALARM_EVENTS 已定义）
            display_index = (alarm_event_next_index - 1 - i + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS;
            event = &alarm_events[display_index]; // 复用报警事件数组
            
            if (event != NULL && event->is_valid) {
                // 显示事件时间（与 PAGE_10 格式一致）
                LCD_DisplayNumber(row, 64, event->timestamp.year, 2);
                LCD_DisplayChar(row, 80, '-');
                LCD_DisplayNumber(row, 88, event->timestamp.mon, 2);
                LCD_DisplayChar(row, 104, '-');
                LCD_DisplayNumber(row, 112, event->timestamp.day, 2);
            } else {
                LCD_DisplayString(row, 64, (unsigned char*)"-- -- --");
            }
        } else {
            LCD_DisplayString(row, 64, (unsigned char*)"           ");
        }
    }
}
// ------------------- 显示PAGE_14(传感器恢复事件列表) -------------------
static void DisplayPage14(void) {
    unsigned char i;
    unsigned char row;
    
    // 原有固定标签绘制逻辑（完全保留，不做任何修改）
    if (display_labels_initialized == 0) {
        LCD_Clear();
        
        for (i = 0; i < 3; i++) {
            row = i * 2; 
            
            if (i == menu_state.page14_selected) {
                LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  
            } else {
                LCD_DisplayChar(row, 0, ' ');
            }
            
            LCD_DISPLAYCHAR_NEW(row, 8, 2, 19);   // "事"
            LCD_DISPLAYCHAR_NEW(row, 16, 3, 19);  // "件"
            LCD_DisplayChar(row, 24, '1' + i);    // 编号
        }
        
        // 原有"时间："标签（不修改）
        LCD_DISPLAYCHAR_NEW(0, 40, 2, 20); 
        LCD_DISPLAYCHAR_NEW(0, 48, 3, 20); 
        LCD_DISPLAYCHAR_NEW(0, 56, 0, 15); 
        LCD_DISPLAYCHAR_NEW(2, 40, 2, 20); 
        LCD_DISPLAYCHAR_NEW(2, 48, 3, 20); 
        LCD_DISPLAYCHAR_NEW(2, 56, 0, 15); 
        LCD_DISPLAYCHAR_NEW(4, 40, 2, 20); 
        LCD_DISPLAYCHAR_NEW(4, 48, 3, 20); 
        LCD_DISPLAYCHAR_NEW(4, 56, 0, 15); 
            
        // 原有功能提示（不修改）
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  
    
        LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  
        LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  
        LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  
    
        LCD_DISPLAYCHAR_NEW(6, 80, 0, 26); 
        LCD_DISPLAYCHAR_NEW(6, 88, 1, 26); 
            
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
        
        display_labels_initialized = 1;
    }
    
    // 仅调用数据刷新函数，不影响固定显示
    DisplayRecoveryEventsOnPage14();
}



static void DisplayRecoveryEventsOnPage14(void) {
    unsigned char i;
    unsigned char row;
    unsigned char display_index;
    RecoveryRecord* event = NULL;

    for (i = 0; i < 3; i++) {
        row = i * 2;
        
        // 清空数据区域（与报警列表一致）
        LCD_DisplayString(row, 64, (unsigned char*)"            ");
        
        // 只显示有效事件（复刻报警的判断逻辑）
        if (i < recovery_event_count) {
            display_index = (recovery_event_next_index - 1 - i + MAX_RECOVERY_EVENTS) % MAX_RECOVERY_EVENTS;
            event = &recovery_events[display_index];
            
            if (event != NULL && event->is_valid) {
                // 显示恢复时间（格式与报警一致）
                LCD_DisplayNumber(row, 64, event->recovery_timestamp.year, 2);
                LCD_DisplayChar(row, 80, '-');
                LCD_DisplayNumber(row, 88, event->recovery_timestamp.mon, 2);
                LCD_DisplayChar(row, 104, '-');
                LCD_DisplayNumber(row, 112, event->recovery_timestamp.day, 2);
            } else {
                LCD_DisplayString(row, 64, (unsigned char*)"-- -- --");
            }
        } else {
            LCD_DisplayString(row, 64, (unsigned char*)"           ");
        }
    }
}

// ------------------- 显示PAGE_15(预警详细数据) -------------------
static void DisplayPage15(void) {
    unsigned char display_index;
    RecoveryRecord* event = NULL;
    
    // 只在需要初始化时显示固定内容（不修改原有固定字符）
    if (display_labels_initialized == 0) {
        LCD_Clear();
        
        // 第一行: ID: PID=    AID=（原有固定字符）
        LCD_DisplayString(0, 0, "ID");
        LCD_DISPLAYCHAR_NEW(0, 16, 0, 15);  // "冒号"
        LCD_DisplayString(0, 24, "PID");
        LCD_DISPLAYCHAR_NEW(0, 48, 0, 16);  // "等号"
        LCD_DisplayString(0, 80, "AID");
        LCD_DISPLAYCHAR_NEW(0, 104, 0, 16);  // "等号"
        
        // 第二行: 恢复前温度:（原有固定字符）
        LCD_DISPLAYCHAR_NEW(2, 0, 0, 32); 
        LCD_DISPLAYCHAR_NEW(2, 8, 1, 32); 
        LCD_DISPLAYCHAR_NEW(2, 16, 2, 32);
        LCD_DISPLAYCHAR_NEW(2, 24, 0, 1); 
        LCD_DISPLAYCHAR_NEW(2, 32, 1, 1); 
        LCD_DISPLAYCHAR_NEW(2, 40, 0, 15);
        
        // 恢复后温度：（原有固定字符）
        LCD_DISPLAYCHAR_NEW(4, 0, 0, 32); 
        LCD_DISPLAYCHAR_NEW(4, 8, 1, 32); 
        LCD_DISPLAYCHAR_NEW(4, 16, 3, 32);
        LCD_DISPLAYCHAR_NEW(4, 24, 0, 1); 
        LCD_DISPLAYCHAR_NEW(4, 32, 1, 1); 
        LCD_DISPLAYCHAR_NEW(4, 40, 0, 15);
        
        // 第四行: 功能提示（原有固定字符）
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 27);
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 27);
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 27);
        LCD_DISPLAYCHAR_NEW(6, 24, 3, 27);
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
        
        display_labels_initialized = 1;
    }
    
    // 获取选中的恢复事件
    if (menu_state.page14_selected < recovery_event_count) {
        display_index = (recovery_event_next_index - menu_state.page14_selected - 1 + MAX_RECOVERY_EVENTS) % MAX_RECOVERY_EVENTS;
        event = &recovery_events[display_index];
    }
    
    // 清空数据显示区域（原有逻辑）
    LCD_DisplayString(0, 56, (unsigned char*)"  ");   // PID显示区域
    LCD_DisplayString(0, 112, (unsigned char*)"  ");  // AID显示区域
    LCD_DisplayString(2, 48, (unsigned char*)"  ");   // 恢复前温度显示区域
    LCD_DisplayString(4, 48, (unsigned char*)"  ");   // 恢复后温度显示区域
    
    // 显示恢复事件数据（新增逻辑）
    if (event != NULL && event->is_valid) {
        // 显示PID
        LCD_DisplayNumber(0, 56, event->pid, 2);
        // 显示AID
        LCD_DisplayNumber(0, 112, event->aid, 2);
        // 显示恢复前温度（报警时的温度）
        LCD_DisplayNumber(2, 48, event->abnormal_temp, 2);
        // 显示恢复后温度（≤25℃的正常温度）
        LCD_DisplayNumber(4, 48, event->recovery_temp, 2);
    } else {
        // 无数据时显示"--"
        LCD_DisplayString(0, 56, (unsigned char*)"--");
        LCD_DisplayString(0, 112, (unsigned char*)"--");
        LCD_DisplayString(2, 48, (unsigned char*)"--");
        LCD_DisplayString(4, 48, (unsigned char*)"--");
    }
}


// ------------------- 显示PAGE_16(预警删除确认) -------------------
static void DisplayPage16(void) {
    LCD_Clear();
    
	  LCD_DISPLAYCHAR_NEW(2, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 28);  // "认"
    LCD_DISPLAYCHAR_NEW(2, 16, 0, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 24, 1, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 32, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 40, 3, 27);  // "除"
	  LCD_DISPLAYCHAR_NEW(2, 48, 2, 28);  // "吗"
    
    // 确认提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 28);  // "认"
    
    // 返回提示
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}

// ------------------- 显示PAGE_17(传感器恢复删除确认) -------------------
static void DisplayPage17(void) {
    LCD_Clear();
    
	  LCD_DISPLAYCHAR_NEW(2, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 28);  // "认"
    LCD_DISPLAYCHAR_NEW(2, 16, 0, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 24, 1, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 32, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 40, 3, 27);  // "除"
	  LCD_DISPLAYCHAR_NEW(2, 48, 2, 28);  // "吗"
    
    // 确认提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 28);  // "认"
    
    // 返回提示
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}


// 局部刷新PAGE_18箭头位置（仅控制选中箭头，不影响其他内容）
static void RefreshPage18Arrow(void) {
    unsigned char i;
    unsigned char row;
    
    for (i = 0; i < PAGE18_DISPLAY_COUNT; i++) {
        row = i * 2;
        if (i == menu_state.page18_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25); // 显示选中箭头
        } else {
            LCD_DisplayChar(row, 0, ' ');
        }
    }
}


static void DisplayPage18(void) {
    unsigned char i;
    unsigned char row;
    unsigned char curr_start_aid; // 当前页面起始从站AID
    unsigned char curr_aid;      // 当前遍历的从站AID
    unsigned char dev_tx_num[8];
    DataRecord* dev_data = NULL;
    static unsigned char last_start_idx = 0xFF;
    unsigned char start_idx = 0;
    
    // 第一步：首次初始化/翻页时绘制固定标签（清屏）
    if (display_labels_initialized == 0) {
        LCD_Clear();
        
        // 固定标签：ID、PID、TX（保留原有格式）
        LCD_DisplayString(0, 8, (unsigned char*)"ID");
        LCD_DISPLAYCHAR_NEW(0, 24, 0, 15); // 冒号
        LCD_DisplayString(0, 32, (unsigned char*)"PID");
        LCD_DISPLAYCHAR_NEW(0, 56, 0, 16); // 等号
        LCD_DisplayString(0, 88, (unsigned char*)"TX");
        
        LCD_DisplayString(2, 8, (unsigned char*)"ID");
        LCD_DISPLAYCHAR_NEW(2, 24, 0, 15);
        LCD_DisplayString(2, 32, (unsigned char*)"PID");
        LCD_DISPLAYCHAR_NEW(2, 56, 0, 16);
        LCD_DisplayString(2, 88, (unsigned char*)"TX");
        
        LCD_DisplayString(4, 8, (unsigned char*)"ID");
        LCD_DISPLAYCHAR_NEW(4, 24, 0, 15);
        LCD_DisplayString(4, 32, (unsigned char*)"PID");
        LCD_DISPLAYCHAR_NEW(4, 56, 0, 16);
        LCD_DisplayString(4, 88, (unsigned char*)"TX");
        
        // 功能提示（保留原有）
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "项"
        LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "上"
        LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
        LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "项"
        LCD_DISPLAYCHAR_NEW(6, 80, 0, 26); // "详"
        LCD_DISPLAYCHAR_NEW(6, 88, 1, 26); // "情"
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"
        
        display_labels_initialized = 1;
        last_start_idx = 0xFF;
    }
    
    if (last_start_idx != start_idx) {
        last_start_idx = start_idx;
    }
    
    // 第二步：刷新箭头（核心：箭头指向当前选中项 menu_state.page18_selected）
    for (i = 0; i < PAGE18_DISPLAY_COUNT; i++) {
        row = i * 2; // 0→0行、1→2行、2→4行
        if (i == menu_state.page18_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25); // 选中项显示箭头
        } else {
            LCD_DisplayChar(row, 0, ' '); // 未选中项清空箭头
        }
    }
    
    // 第三步：清空数据区
    for (i = 0; i < 3; i++) {
        row = i * 2;
        LCD_DisplayString(row, 64, (unsigned char*)"  ");   // PID区
        LCD_DisplayString(row, 104, (unsigned char*)"  ");  // TX编号区
    }
    
    // 第四步：计算当前页面起始AID（核心：无循环，仅显示1-10）
    curr_start_aid = page18_scroll_page * PAGE18_DISPLAY_COUNT + 1; 
    // 页索引0→1（TX01）、1→4（TX04）、2→7（TX07）、3→10（TX10）
    
    // 第五步：显示当前页3个从站（核心：不循环，超出10则显示空）
    for (i = 0; i < PAGE18_DISPLAY_COUNT; i++) {
        row = i * 2;
        curr_aid = curr_start_aid + i;
        
        // 仅显示1-10的从站，超出则显示空
        if (curr_aid > 10) {
            curr_aid = 0;
        }
        
        // 生成TX编号（01-10）
        if (curr_aid != 0) {
            dev_tx_num[0] = '0' + (curr_aid / 10);
            dev_tx_num[1] = '0' + (curr_aid % 10);
            dev_tx_num[2] = '\0';
        } else {
            dev_tx_num[0] = ' ';
            dev_tx_num[1] = ' ';
            dev_tx_num[2] = '\0';
        }
        
        // 显示PID
        if (curr_aid != 0) {
            dev_data = GetRecentDataByAID(curr_aid);
            if (dev_data != NULL && dev_data->is_valid) {
                LCD_DisplayNumber(row, 64, (unsigned long)dev_data->pid, 2);
            } else {
                LCD_DisplayString(row, 64, (unsigned char*)"--");
            }
        } else {
            LCD_DisplayString(row, 64, (unsigned char*)"--");
        }
        
        // 显示TX编号
        LCD_DisplayString(row, 104, dev_tx_num);
    }
}



// ------------------- 显示PAGE_19(清除历史数据确认页面) -------------------
static void DisplayPage19(void) {
    unsigned char curr_aid = page19_selected_slave; // 接收真实AID（1-10）
    unsigned char target_idx = page19_selected_record; // 选中的历史记录索引（0-3）
    DataRecord* hist_data = NULL;
    rtc_time_t curr_time;
    unsigned short hist_start;
    unsigned short hist_pos;
    unsigned char slave_idx;
    
    LCD_Clear();
    GetCurrentRTC();
    curr_time = current_rtc_time;
    
    // 核心修复：将真实AID转换为数组索引（1→0，4→3，10→9）
    slave_idx = curr_aid - 1;
    if (slave_idx >= TOTAL_SLAVES) {
        slave_idx = 0; // 边界保护
    }
    
    // 计算该从站的历史数据存储起始位置（基于真实AID的索引）
    hist_start = HISTORY_DATA_START + (slave_idx * RECORDS_PER_SLAVE);
    // 计算目标记录位置（循环覆盖）
    hist_pos = hist_start + target_idx;
    
    // 读取历史数据（判断有效性）
    if (hist_pos < TOTAL_RECORDS && data_summary[hist_pos].is_valid) {
        hist_data = &data_summary[hist_pos];
    }
    
    // 后续温度、电压、日期时间显示逻辑不变（保留原有代码）
    // 1. 第一行：温度 + 电压
    LCD_DISPLAYCHAR_NEW(0, 0, 0, 1);  // "温"
    LCD_DISPLAYCHAR_NEW(0, 8, 1, 1);  // "度"
    LCD_DISPLAYCHAR_NEW(0, 16, 0, 15); // ":"
    if (hist_data != NULL) {
        LCD_DisplayNumber(0, 24, (unsigned long)hist_data->temp, 2);
    } else {
        LCD_DisplayNumber(0, 24, 0UL, 2);
    }
    LCD_DISPLAYCHAR_NEW(0, 40, 0, 2);  // "℃"
    LCD_DISPLAYCHAR_NEW(0, 56, 0, 3);  // "电"
    LCD_DISPLAYCHAR_NEW(0, 64, 1, 3);  // "压"
    LCD_DISPLAYCHAR_NEW(0, 72, 0, 15); // ":"
    if (hist_data != NULL) {
        unsigned int volt_mv = hist_data->volt1 * 100;
        LCD_DisplayNumber(0, 80, (unsigned long)volt_mv, 4);
    } else {
        LCD_DisplayNumber(0, 80, 0UL, 4);
    }
    LCD_DisplayString(0, 112, (unsigned char*)"mV");
    
    // 2. 第二行：日期（读取历史记录的save_time）
    LCD_DISPLAYCHAR_NEW(2, 0, 2, 12);  // "日"
    LCD_DISPLAYCHAR_NEW(2, 8, 3, 12);  // "期"
    LCD_DISPLAYCHAR_NEW(2, 16, 0, 15); // ":"
    if (hist_data != NULL) {
        LCD_DisplayNumber(2, 24, (unsigned long)hist_data->save_time.mon, 2);
        LCD_DISPLAYCHAR_NEW(2, 40, 1, 15); // "."
        LCD_DisplayNumber(2, 48, (unsigned long)hist_data->save_time.day, 2);
    } else {
        LCD_DisplayNumber(2, 24, (unsigned long)curr_time.mon, 2);
        LCD_DISPLAYCHAR_NEW(2, 40, 1, 15);
        LCD_DisplayNumber(2, 48, (unsigned long)curr_time.day, 2);
    }
    
    // 3. 第三行：时间（读取历史记录的save_time）
    LCD_DISPLAYCHAR_NEW(4, 0, 2, 20);  // "时"
    LCD_DISPLAYCHAR_NEW(4, 8, 3, 20);  // "间"
    LCD_DISPLAYCHAR_NEW(4, 16, 0, 15); // ":"
    if (hist_data != NULL) {
        LCD_DisplayNumber(4, 24, (unsigned long)hist_data->save_time.hour, 2);
        LCD_DISPLAYCHAR_NEW(4, 40, 0, 15); // ":"
        LCD_DisplayNumber(4, 48, (unsigned long)hist_data->save_time.min, 2);
    } else {
        LCD_DisplayNumber(4, 24, (unsigned long)curr_time.hour, 2);
        LCD_DISPLAYCHAR_NEW(4, 40, 0, 15);
        LCD_DisplayNumber(4, 48, (unsigned long)curr_time.min, 2);
    }
    
    // 4. 第四行：功能提示（保留原有）
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "上"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "项"
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "下"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "项"
    LCD_DISPLAYCHAR_NEW(6, 80, 0, 29); // "清"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 29); // "除"
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"
    
    display_labels_initialized = 1;
}

// uart4.c 中添加此函数
static void DeletePage19SelectedRecord(void) {
    unsigned char curr_aid = page19_selected_slave; // 真实AID（1-10）
    unsigned char target_idx = page19_selected_record;
    unsigned char slave_idx = curr_aid - 1; // 转换为数组索引
    unsigned char i;
    
    // 边界保护
    if (slave_idx >= 3 || target_idx < 0 || target_idx >= 4) {
        return;
    }
    
    // 标记为无效（基于真实AID对应的索引）
    page19_auto_records[slave_idx][target_idx].is_valid = 0;
    
    // 查找下一条有效记录（逻辑不变）
    for (i = 1; i <= 4; i++) {
        unsigned char next_idx = (target_idx + i) % 4;
        if (page19_auto_records[slave_idx][next_idx].is_valid) {
            page19_selected_record = next_idx;
            break;
        }
    }
    
    UART4_SendString("PAGE19 record ");
    UART4_SendNumber(target_idx, 1);
    UART4_SendString(" deleted.\r\n");
}



// ------------------- 显示PAGE_20(历史数据删除确认页面) -------------------
static void DisplayPage20(void) {
    LCD_Clear();
    
    // 删除确认提示
		LCD_DISPLAYCHAR_NEW(2, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 28);  // "认"
    LCD_DISPLAYCHAR_NEW(2, 16, 0, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 24, 1, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 32, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 40, 3, 27);  // "除"
	  LCD_DISPLAYCHAR_NEW(2, 48, 2, 28);  // "吗"
//
    
    // 确认提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 28);  // "认"
    
    // 返回提示
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}


static void DisplayPage21(void) {
    const unsigned char row_page[3] = {0, 2, 4}; 
    int i;
    
    // 关键：调用函数更新最高温数据
    GetMaxTempRecords(max_temps, max_temp_times);
    
    LCD_Clear();
    
    for (i = 0; i < 3; i++) {
        unsigned char curr_page = row_page[i];
        unsigned char selected_row = menu_state.page21_selected;
        
        // 显示选中箭头
        if (i == selected_row) {
            LCD_DISPLAYCHAR_NEW(curr_page, 0, 0, 25);
        } else {
            LCD_DisplayChar(curr_page, 0, ' ');
        }
        
        // 显示温度
        if (max_temps[i] != -990 && max_temp_times[i].year != 0) {
            // 显示有效数据
            LCD_DisplayNumber(curr_page, 8, (unsigned long)(max_temps[i]/10), 2);
            LCD_DISPLAYCHAR_NEW(curr_page, 24, 0, 2);
            // 日期
            LCD_DisplayNumber(curr_page, 40, (unsigned long)max_temp_times[i].mon, 2);
            LCD_DISPLAYCHAR_NEW(curr_page, 56, 1, 15);
            LCD_DisplayNumber(curr_page, 64, (unsigned long)max_temp_times[i].day, 2);
            // 时间
            LCD_DisplayNumber(curr_page, 88, (unsigned long)max_temp_times[i].hour, 2);
            LCD_DISPLAYCHAR_NEW(curr_page, 104, 0, 15);
            LCD_DisplayNumber(curr_page, 112, (unsigned long)max_temp_times[i].min, 2);
        } else {
            // 显示空值（已删除或无数据）
            LCD_DisplayString(curr_page, 8, (unsigned char*)"--");
            LCD_DISPLAYCHAR_NEW(curr_page, 24, 0, 2);
            LCD_DisplayString(curr_page, 40, (unsigned char*)"--");
            LCD_DISPLAYCHAR_NEW(curr_page, 56, 1, 15);
            LCD_DisplayString(curr_page, 64, (unsigned char*)"--");
            LCD_DisplayString(curr_page, 88, (unsigned char*)"--");
            LCD_DISPLAYCHAR_NEW(curr_page, 104, 0, 15);
            LCD_DisplayString(curr_page, 112, (unsigned char*)"--");
        }
    }
    
    // 功能提示（保持不变）
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);
    LCD_DISPLAYCHAR_NEW(6, 80, 0, 26);
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 26);
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
}
// ------------------- 显示PAGE_22(最高温度清除确认页面) -------------------
// ------------------- 显示PAGE_22(最高温度清除确认页面) -------------------
static void DisplayPage22(void) {
    unsigned char selected_row = menu_state.page21_selected;
    unsigned char pid = 0, aid = 0;
    unsigned short volt_mv = 0;
    unsigned char temp = 0;
    DataRecord *target_record = NULL;
    short local_max_temps[3];
    rtc_time_t local_max_temp_times[3];
    
    // 关键：每次绘制都重新读取最新数据（删除后会读取到无效值）
    GetMaxTempRecords_Ext(local_max_temps, local_max_temp_times, &pid, &aid, &volt_mv, &target_record);
    
    LCD_Clear();
    
    // 第一行: ID: PID=    AID= - 固定字符
    LCD_DisplayString(0, 0, "ID");
    LCD_DISPLAYCHAR_NEW(0, 16, 0, 15); // ":"
    LCD_DisplayString(0, 24, "PID");
    LCD_DISPLAYCHAR_NEW(0, 48, 0, 16); // ":"
    LCD_DisplayString(0, 80, "AID");
    LCD_DISPLAYCHAR_NEW(0, 104, 0, 16); // ":"
    
    // 第二行: 温度: 附带时间戳 - 固定字符
    LCD_DISPLAYCHAR_NEW(2, 0, 0, 1);  // "温"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 1);  // "度"
    LCD_DISPLAYCHAR_NEW(2, 16, 0, 15); // ":"
    LCD_DISPLAYCHAR_NEW(2, 64, 2, 20); // "时"
    LCD_DISPLAYCHAR_NEW(2, 72, 3, 20); // "间"
    LCD_DISPLAYCHAR_NEW(2, 80, 0, 15); // ":"
    
    // 第三行: 电压: - 固定字符
    LCD_DISPLAYCHAR_NEW(4, 0, 0, 3);  // "电"
    LCD_DISPLAYCHAR_NEW(4, 8, 1, 3);  // "压"
    LCD_DISPLAYCHAR_NEW(4, 16, 0, 15); // ":"
    LCD_DisplayString(4, 64, "mV"); // 电压单位
    
    // 第四行（第6行）: 功能提示 - 固定字符
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 27);  // "按"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 27);  // "1"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(6, 24, 3, 27);  // "除"
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
    
    // 4. 数据显示处理：核心修改 - 联动清空（显示占位符）
    // 清空数据区域（原有逻辑保留）
    LCD_DisplayString(0, 56, (unsigned char*)"  ");   // PID显示区
    LCD_DisplayString(0, 112, (unsigned char*)"  ");  // AID显示区
    LCD_DisplayString(2, 24, (unsigned char*)"  ");   // 温度显示区
    LCD_DisplayString(2, 88, (unsigned char*)"        "); // 时间显示区
    LCD_DisplayString(4, 24, (unsigned char*)"    ");  // 电压显示区
    
    // 关键：判断数据是否已清空（local_max_temps=-990 表示清空）
    if (local_max_temps[selected_row] != -990) {
        temp = (unsigned char)(local_max_temps[selected_row] / 10);
        LCD_DisplayNumber(0, 56, pid, 2);
        LCD_DisplayNumber(0, 112, aid, 2);
        LCD_DisplayNumber(2, 24, temp, 2);
        LCD_DISPLAYCHAR_NEW(2, 40, 0, 2);
        LCD_DisplayNumber(2, 88, local_max_temp_times[selected_row].mon, 2);
        LCD_DISPLAYCHAR_NEW(2, 104, 1, 15);
        LCD_DisplayNumber(2, 112, local_max_temp_times[selected_row].day, 2);
        LCD_DisplayNumber(4, 24, volt_mv, 4);
    } else {
        LCD_DisplayString(0, 56, (unsigned char*)"--");
        LCD_DisplayString(0, 112, (unsigned char*)"--");
        LCD_DisplayString(2, 24, (unsigned char*)"--");
        LCD_DISPLAYCHAR_NEW(2, 40, 0, 2);
        LCD_DisplayString(2, 88, (unsigned char*)"-- --");
        LCD_DisplayString(4, 24, (unsigned char*)"----");
    }
}
// ------------------- 显示PAGE_23(最高温度删除确认页面) -------------------
static void DisplayPage23(void) {
           LCD_Clear();
    
    // 删除确认提示
	  LCD_DISPLAYCHAR_NEW(2, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 28);  // "认"
    LCD_DISPLAYCHAR_NEW(2, 16, 0, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 24, 1, 27);  // 
    LCD_DISPLAYCHAR_NEW(2, 32, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 40, 3, 27);  // "除"
	  LCD_DISPLAYCHAR_NEW(2, 48, 2, 28);  // "吗"
    
    // 确认提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 28);  // "认"
    
    // 返回提示
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}


static void GetCurrentRTC(void) {
    rtc_read(&current_rtc_time);
}

// 更新RTC刷新逻辑
static void UpdateRTCRefresh(void) {
    rtc_refresh_counter++;
    
    // 每1秒刷新一次（假设每次调用间隔约5ms，200次=1秒）
    if (rtc_refresh_counter >= 200) {
        rtc_refresh_counter = 0;
        need_rtc_refresh = 1;
    }
    
    // 需要刷新时更新显示
    if (need_rtc_refresh && menu_state.current_page == PAGE_24) {
        GetCurrentRTC();
        DisplayRTCOnPage24();
        need_rtc_refresh = 0;
    }
}




// 初始化RTC编辑（进入选择状态）
static void RTC_Edit_Init(void) {
    // 如果已经在编辑状态，则不需要重新初始化
    if (rtc_edit_state == RTC_EDIT_IDLE) {
        rtc_edit_state = RTC_EDIT_SELECT;
    }
    
    rtc_edit_pos = 0;                  // 默认选中第一个位置（年十位）
    // 核心优化：每次进入编辑都同步最新的当前时间（而非复用旧临时数据）
    GetCurrentRTC();
    edit_temp_time = current_rtc_time; // 复制最新时间到临时变量
    display_labels_initialized = 0;    // 强制刷新显示
}

// 切换编辑位置（上/下一位）
static void RTC_Switch_Pos(signed char step) {
    // 核心修改：按“年1→年2→月1→月2→日1→日2→时1→时2→分1→分2”顺序切换
    rtc_edit_pos += step;
    
    // 边界保护：0-9之间循环（不跳越，仅相邻切换）
    if (rtc_edit_pos < 0) {
        rtc_edit_pos = 9; // 最前位→最后位
    } else if (rtc_edit_pos > 9) {
        rtc_edit_pos = 0; // 最后位→最前位
    }
    
    display_labels_initialized = 0;    // 刷新显示
}

// 调整选中位置的数字（加/减）
static void RTC_Adjust_Num(signed char step) {
    unsigned char ten, unit;  // 十位、个位临时变量
    // 根据选中位置确定修改的数字位和边界
    switch(rtc_edit_pos) {
        // 年（拆分为十位和个位，单独修改）
        case 0: // 年十位（0-9）
            ten = edit_temp_time.year / 10;
            unit = edit_temp_time.year % 10;
            ten += step;
            ten = (ten > 9) ? 9 : (ten < 0) ? 0 : ten;
            edit_temp_time.year = ten * 10 + unit;
            break;
        case 1: // 年个位（0-9）
            ten = edit_temp_time.year / 10;
            unit = edit_temp_time.year % 10;
            unit += step;
            unit = (unit > 9) ? 9 : (unit < 0) ? 0 : unit;
            edit_temp_time.year = ten * 10 + unit;
            break;
        
        // 月（保持原有逻辑，1-12）
        case 2: // 月十位（0-1）
            ten = edit_temp_time.mon / 10;
            unit = edit_temp_time.mon % 10;
            ten += step;
            ten = (ten > 1) ? 1 : (ten < 0) ? 0 : ten;
            edit_temp_time.mon = ten * 10 + unit;
            if (edit_temp_time.mon > 12) edit_temp_time.mon = 12;
            if (edit_temp_time.mon < 1) edit_temp_time.mon = 1;
            break;
        case 3: // 月个位（1-9/0-2）
            ten = edit_temp_time.mon / 10;
            unit = edit_temp_time.mon % 10;
            unit += step;
            if (ten == 1) unit = (unit > 2) ? 2 : (unit < 0) ? 0 : unit;
            else unit = (unit > 9) ? 9 : (unit < 1) ? 1 : unit;
            edit_temp_time.mon = ten * 10 + unit;
            if (edit_temp_time.mon > 12) edit_temp_time.mon = 12;
            if (edit_temp_time.mon < 1) edit_temp_time.mon = 1;
            break;
        
        // 日（保持原有逻辑，1-31）
        case 4: // 日十位（0-3）
            ten = edit_temp_time.day / 10;
            unit = edit_temp_time.day % 10;
            ten += step;
            ten = (ten > 3) ? 3 : (ten < 0) ? 0 : ten;
            edit_temp_time.day = ten * 10 + unit;
            if (edit_temp_time.day > 31) edit_temp_time.day = 31;
            if (edit_temp_time.day < 1) edit_temp_time.day = 1;
            break;
        case 5: // 日个位（1-9/0-1）
            ten = edit_temp_time.day / 10;
            unit = edit_temp_time.day % 10;
            unit += step;
            if (ten == 3) unit = (unit > 1) ? 1 : (unit < 0) ? 0 : unit;
            else unit = (unit > 9) ? 9 : (unit < 1) ? 1 : unit;
            edit_temp_time.day = ten * 10 + unit;
            if (edit_temp_time.day > 31) edit_temp_time.day = 31;
            if (edit_temp_time.day < 1) edit_temp_time.day = 1;
            break;
        
        // 时（保持原有逻辑，0-23）
        case 6: // 时十位（0-2）
            ten = edit_temp_time.hour / 10;
            unit = edit_temp_time.hour % 10;
            ten += step;
            ten = (ten > 2) ? 2 : (ten < 0) ? 0 : ten;
            edit_temp_time.hour = ten * 10 + unit;
            break;
        case 7: // 时个位（0-9/0-3）
            ten = edit_temp_time.hour / 10;
            unit = edit_temp_time.hour % 10;
            unit += step;
            if (ten == 2) unit = (unit > 3) ? 3 : (unit < 0) ? 0 : unit;
            else unit = (unit > 9) ? 9 : (unit < 0) ? 0 : unit;
            edit_temp_time.hour = ten * 10 + unit;
            break;
        
        // 分（保持原有逻辑，0-59）
        case 8: // 分十位（0-5）
            ten = edit_temp_time.min / 10;
            unit = edit_temp_time.min % 10;
            ten += step;
            ten = (ten > 5) ? 5 : (ten < 0) ? 0 : ten;
            edit_temp_time.min = ten * 10 + unit;
            break;
        case 9: // 分个位（0-9）
            ten = edit_temp_time.min / 10;
            unit = edit_temp_time.min % 10;
            unit += step;
            unit = (unit > 9) ? 9 : (unit < 0) ? 0 : unit;
            edit_temp_time.min = ten * 10 + unit;
            break;
        default: return;
    }
    display_labels_initialized = 0;  // 刷新显示，实时预览修改后的值
}


static void RTC_Save_Edit(void) {
    rtc_write(&edit_temp_time);       // 写入DS1302保存
    current_rtc_time = edit_temp_time; // 更新当前时间变量
    
    // 核心修复1：重置临时编辑数据
    edit_temp_time = current_rtc_time;
    
    // 核心修复2：保存后返回选择状态，而不是空闲状态
    rtc_edit_state = RTC_EDIT_SELECT;
    
    // 核心修复3：重置选中位置到第一个（年十位）
//    rtc_edit_pos = 0;
    
    // 核心修复4：标记显示需要刷新
    display_labels_initialized = 0;
    
    UART4_SendString("RTC time saved successfully! Back to select mode.\r\n");
    RefreshDisplay(); // 强制刷新显示，应用新状态
}
static void DisplayPage24(void)
{
    LCD_Clear();
    
    // 第一行: 标题（保持不变）
    LCD_DISPLAYCHAR_NEW(0, 0, 2, 12);   // "日"
    LCD_DISPLAYCHAR_NEW(0, 8, 3, 12);  // "期"
    LCD_DISPLAYCHAR_NEW(2, 0, 2, 20);  // "时"
    LCD_DISPLAYCHAR_NEW(2, 8, 3, 20);  // "间"
    
    // 第四行: 上一项
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);  // "上"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 5, 4); // "位"
    
	  LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "下"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 5, 4); // "位"
		
		LCD_DISPLAYCHAR_NEW(6, 80, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 28);  // "认"
    
    // 返回按钮（始终显示）
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
    
    GetCurrentRTC();
    DisplayRTCOnPage24();
    
    // 编辑状态下显示选中位置的下划线
    if (rtc_edit_state != RTC_EDIT_IDLE) {
        unsigned char row, col;
        // 根据选中位置确定下划线坐标
        switch(rtc_edit_pos) {
            case 0: row=0; col=48; break;  // 年-十位
            case 1: row=0; col=56; break;  // 年-个位
            case 2: row=0; col=72; break;  // 月-十位
            case 3: row=0; col=80; break;  // 月-个位
            case 4: row=0; col=96; break;  // 日-十位
            case 5: row=0; col=104; break; // 日-个位
            case 6: row=2; col=32; break;  // 时-十位
            case 7: row=2; col=40; break;  // 时-个位
            case 8: row=2; col=56; break;  // 分-十位
            case 9: row=2; col=64; break;  // 分-个位
            default: return;
        }
        LCD_DisplayChar(row, col, '_');  // 显示下划线标识选中
    }
}






static void DisplayRTCOnPage24(void) {
    unsigned char temp;
    rtc_time_t *show_time = (rtc_edit_state != RTC_EDIT_IDLE) ? &edit_temp_time : &current_rtc_time;
    
    // 第2行显示日期：YYYY-MM-DD
    LCD_DisplayChar(0, 32, '2');
    LCD_DisplayChar(0, 40, '0');
    temp = (show_time->year / 10);
    LCD_DisplayChar(0, 48, (temp > 9 ? 9 : temp) + '0');
    temp = (show_time->year % 10);
    LCD_DisplayChar(0, 56, (temp > 9 ? 9 : temp) + '0');
    LCD_DisplayChar(0, 64, '-');
    temp = (show_time->mon / 10);
    LCD_DisplayChar(0, 72, (temp > 9 ? 9 : temp) + '0');
    temp = (show_time->mon % 10);
    LCD_DisplayChar(0, 80, (temp > 9 ? 9 : temp) + '0');
    LCD_DisplayChar(0, 88, '-');
    temp = (show_time->day / 10);
    LCD_DisplayChar(0, 96, (temp > 9 ? 9 : temp) + '0');
    temp = (show_time->day % 10);
    LCD_DisplayChar(0, 104, (temp > 9 ? 9 : temp) + '0');
    
    // 第4行显示时间：HH:MM:SS
    temp = (show_time->hour / 10);
    LCD_DisplayChar(2, 32, (temp > 9 ? 9 : temp) + '0');
    temp = (show_time->hour % 10);
    LCD_DisplayChar(2, 40, (temp > 9 ? 9 : temp) + '0');
    LCD_DISPLAYCHAR_NEW(2, 48, 0, 15);
    temp = (show_time->min / 10);
    LCD_DisplayChar(2, 56, (temp > 9 ? 9 : temp) + '0');
    temp = (show_time->min % 10);
    LCD_DisplayChar(2, 64, (temp > 9 ? 9 : temp) + '0');
    LCD_DISPLAYCHAR_NEW(2, 72, 0, 15);
    temp = (show_time->sec / 10);
    LCD_DisplayChar(2, 80, (temp > 9 ? 9 : temp) + '0');
    temp = (show_time->sec % 10);
    LCD_DisplayChar(2, 88, (temp > 9 ? 9 : temp) + '0');
}
// ------------------- 显示PAGE_25(修改密码详情) -------------------
static void DisplayPage25(void) {
    //LCD_Clear(); // 状态切换时全屏清屏（与RTC编辑一致）
    
    // 绘制固定标签（标题“密码”）
    LCD_DISPLAYCHAR_NEW(2, 8, 4, 12);  // "密"
    LCD_DISPLAYCHAR_NEW(2, 16, 5, 12);  // "码"
    
    // 绘制功能提示（固定内容，与RTC功能提示逻辑一致）
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);  // "上"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 5, 4); // "位"
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "下"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 5, 4); // "位"
    LCD_DISPLAYCHAR_NEW(6, 80, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 28);  // "认"
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
    
    // 加载当前密码（首次进入时初始化）
    if (pwd_edit_state == PWD_EDIT_IDLE) {
        memcpy(current_password, default_password, sizeof(current_password));
    }
    
    // 调用闪烁绘制函数（基础界面+闪烁效果）
    DisplayPassword();
}
// ------------------- 显示PAGE_26(恢复出厂设置详情) -------------------
static void DisplayPage26(void)
{
    LCD_Clear();
    LCD_DISPLAYCHAR_NEW(2, 0, 0, 30);  // "确"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 30);  // "认"
    LCD_DISPLAYCHAR_NEW(2, 16, 2, 30);  // "确"
    LCD_DISPLAYCHAR_NEW(2, 24, 3, 30);  // "认"
	  LCD_DISPLAYCHAR_NEW(2, 32, 4, 30);  // "确"
    LCD_DISPLAYCHAR_NEW(2, 40, 5, 30);  // "认"
	  LCD_DISPLAYCHAR_NEW(2, 48, 6, 30);  // "确"
    LCD_DISPLAYCHAR_NEW(2, 56, 7, 30);  // "认"
    // 按键1: 确认

    LCD_DISPLAYCHAR_NEW(6, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 28);  // "认"
    
    // 按键2: 返回
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}



// ------------------- 测量数据显示详情页面 -------------------
static void DisplayDetailPage(PageType page)
{
    LCD_Clear();
    
    switch(page) {
        case PAGE_3:  // 数据汇总页面
            DisplayPage3();
            break;
            
        case PAGE_4:  // 传感器状态
            DisplayPage4();
            break;
            
        case PAGE_5:  // 参数查询
            LCD_DISPLAYCHAR_NEW(0, 0, 0, 17);
            LCD_DISPLAYCHAR_NEW(0, 32, 1, 17);
            LCD_DISPLAYCHAR_NEW(0, 48, 2, 17);
            LCD_DISPLAYCHAR_NEW(0, 72, 3, 17);
            
            LCD_DISPLAYCHAR_NEW(2, 0, 0, 17);
            LCD_DISPLAYCHAR_NEW(2, 32, 1, 17);
            LCD_DISPLAYCHAR_NEW(2, 48, 2, 17);
            LCD_DISPLAYCHAR_NEW(2, 72, 3, 17);
            
            LCD_DISPLAYCHAR_NEW(4, 0, 0, 17);
            LCD_DISPLAYCHAR_NEW(4, 32, 1, 17);
            LCD_DISPLAYCHAR_NEW(4, 48, 2, 17);
            LCD_DISPLAYCHAR_NEW(4, 72, 3, 17);
            
            LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);
            LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);
            LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);
            
            LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);
            LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);
            LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);
            
            LCD_DISPLAYCHAR_NEW(6, 80, 0, 12);
            LCD_DISPLAYCHAR_NEW(6, 88, 1, 12);
            
            LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
            LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
            break;
            
        case PAGE_7:  // 历史记录查询
            DisplayPage7();
            break;
            
        case PAGE_8:  // 设备维护
            DisplayPage8();
            break;
            
        case PAGE_9:  // 扩展页面
            LCD_DISPLAYCHAR_NEW(0, 0, 0, 17);
            LCD_DISPLAYCHAR_NEW(0, 32, 1, 17);
            LCD_DISPLAYCHAR_NEW(2, 48, 2, 17);
            LCD_DISPLAYCHAR_NEW(2, 72, 3, 17);
            
            LCD_DISPLAYCHAR_NEW(6, 0, 0, 18);
            LCD_DISPLAYCHAR_NEW(6, 8, 1, 18);
            LCD_DISPLAYCHAR_NEW(6, 62, 0, 18);
            LCD_DISPLAYCHAR_NEW(6, 70, 2, 18);
            LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
            LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
            break;
        
        case PAGE_10:  // 温度报警事件列表
            DisplayPage10();
            break;
            
        case PAGE_13:  // 温度预警时间列表
            DisplayPage13();
            break;
            
        case PAGE_14:  // 传感器恢复事件列表
            DisplayPage14();
            break;
            
        case PAGE_15:  // 预警/恢复详细数据
            DisplayPage15();
            break;
            
        case PAGE_16:  // 预警删除确认
            DisplayPage16();
            break;
            
        case PAGE_17:  // 新增：恢复删除确认
            DisplayPage17();
            break;
            
        case PAGE_18:  // 历史数据查询页面
            DisplayPage18();
            break;
            
        case PAGE_19:  // 清除历史数据确认页面
            DisplayPage19();
            break;
            
        case PAGE_20:  // 历史数据删除确认页面
            DisplayPage20();
            break;   
            
        case PAGE_21:  // 最高温度查询页面
            DisplayPage21();
            break;
            
        case PAGE_22:  // 最高温度清除确认页面
            DisplayPage22();
            break;
            
        case PAGE_23:  // 最高温度删除确认页面
            DisplayPage23();
            break;   
            
        case PAGE_24:
            DisplayPage24();  // 显示修改日期时间详情
            break;
            
        case PAGE_25:
            DisplayPage25();  // 显示修改密码详情
            break;
            
        case PAGE_26:
            DisplayPage26();  // 显示恢复出厂设置详情
            break;
            
        default:
            break;
    }
}

// ------------------- UART4中断服务函数 -------------------
void UART4_ISR(void) interrupt 18
{
    if (S4CON & 0x01)  // 接收中断
    {
        S4CON &= ~0x01; // 清除接收标志
        
        if (uart_rx_len < 6)  // 接收数据(6字节)
        {
            uart_rx_buff[uart_rx_len++] = S4BUF;
            if (uart_rx_len == 6)  // 收到6字节,设置完成标志
            {
                uart_rx_complete = 1;
            }
        }
        else  // 缓冲区满,清空
        {
            uart_rx_len = 0;
            uart_rx_complete = 0;
        }
    }
}

void UART4_ReceiveString(void) {
    UpdateRTCRefresh(); 
    DisplayFixedLabels();
    
	    // 新增：调用PAGE19自动读取函数
    
    if (uart_rx_complete == 1) {
        Protocol_Parse(uart_rx_buff);
        
        // 调试：显示解析的数据
        UART4_SendString("Parsed Data: PID=");
        UART4_SendNumber(parsed_data.PID, 2);
        UART4_SendString(", AID=");
        UART4_SendNumber(parsed_data.AID, 2);
        UART4_SendString(", Temp=");
        UART4_SendNumber((unsigned long)parsed_data.temperature, 4);
        UART4_SendString("\r\n");
        
        AddDataToSummary(parsed_data.PID, parsed_data.AID,
                        TempShortToChar(parsed_data.temperature),
                        (unsigned char)(parsed_data.Bat_Voltage * 10),
                        0, 0);
        
        // 检查并记录报警事件
        CheckAndRecordAlarm();
        
        // 调试：显示当前报警事件数量
        UART4_SendString("Alarm events count: ");
        UART4_SendNumber(alarm_event_count, 2);
        UART4_SendString("\r\n");
        
        if (menu_state.current_page == PAGE_1) {
            UpdateDisplayForPage1(); 
        } else if (menu_state.current_page == PAGE_3) {
            UpdateDisplayForPage3();
        } else if (menu_state.current_page == PAGE_10) {
            DisplayAlarmEventsOnPage10(); // 更新PAGE_10的报警事件显示
        } // 【新增核心逻辑】：当前页面是PAGE_21时，强制刷新闪烁
        else if (menu_state.current_page == PAGE_14) {
            DisplayRecoveryEventsOnPage14(); // 新增：刷新PAGE_14恢复事件
        } 
        else if (menu_state.current_page == PAGE_25 && pwd_flash_state) {
            DisplayPassword(); // 仅在闪烁中时刷新，提升效率
        }
//          else if (menu_state.current_page == PAGE_21) {
//            display_labels_initialized = 0;  // 强制重新绘制
//            DisplayFixedLabels();
//        }
        UART4_ClearBuffer(uart_rx_buff, 6);
        uart_rx_complete = 0;
        uart_rx_len = 0;
    }
}

// 在 uart4.c 文件中，添加这个函数//page1局部刷新
// (通常放在 UART4_ReceiveString 函数定义之后，或者在文件顶部声明 static void UpdateDisplayForPage1(void);)
static void UpdateDisplayForPage1(void) {
    unsigned char i;
    DataRecord* dev_data = NULL;
    unsigned char dev_id_buf[8];

    // 仅刷新第2行、第4行的有效数据
    for (i = 0; i < 2; i++) {
        unsigned char dev_idx = page1_scroll_idx + i;
        unsigned char row = (i == 0) ? 2 : 4;  // 固定有效行
        Page1_DevInfo* dev = &page1_devices[dev_idx];

        // 生成设备ID
        dev_id_buf[0] = '0';
        dev_id_buf[1] = '1';
        dev_id_buf[2] = 'T';
        dev_id_buf[3] = 'X';
        dev_id_buf[4] = '0' + ((dev_idx + 1) / 10);
        dev_id_buf[5] = '0' + ((dev_idx + 1) % 10);
        dev_id_buf[6] = '\0';
        LCD_DisplayString(row, 8, dev_id_buf);

        // 刷新温度
        dev_data = GetFixedSlaveData(dev->aid);
        if (dev_data != NULL && dev_data->is_valid) {
            LCD_DisplayNumber(row, 64, (unsigned long)dev_data->temp, 2);
        } else {
            LCD_DisplayString(row, 64, (unsigned char*)"--");
        }

        // 刷新电压
        if (dev_data != NULL && dev_data->is_valid) {
            unsigned int volt_mv = dev_data->volt1 * 100;
            LCD_DisplayNumber(row, 88, (unsigned long)volt_mv, 4);
        } else {
            LCD_DisplayString(row, 88, (unsigned char*)"----");
        }

        // 刷新箭头
        if (i == 0) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);
        } else {
            LCD_DisplayChar(row, 0, ' ');
        }
    }
}

// --- 1. (可选) 定义 UpdateDisplayForPage3 ---
static void UpdateDisplayForPage3(void) {
    unsigned char module_index = menu_state.page3_selected;
    unsigned char tx_number = module_index + 1;
    unsigned char aid = tx_number;
    DataRecord* dev_data = GetRecentDataByAID(aid);
    
    // 刷新TX编号（固定）
    LCD_DisplayNumber(0, 96, tx_number, 2);
    
    // 刷新PID
    if (dev_data != NULL && dev_data->is_valid) {
        LCD_DisplayNumber(0, 32, dev_data->pid, 2);
    } else {
        LCD_DisplayString(0, 32, (unsigned char*)"01");
    }
    
    // 刷新温度
    if (dev_data != NULL && dev_data->is_valid) {
        LCD_DisplayNumber(2, 24, dev_data->temp, 2);
    } else {
        LCD_DisplayString(2, 24, (unsigned char*)"--");
    }
    
    // 刷新电压
    if (dev_data != NULL && dev_data->is_valid) {
        unsigned int volt_mv = dev_data->volt1 * 100;
        LCD_DisplayNumber(4, 24, volt_mv, 4);
    } else {
        LCD_DisplayString(4, 24, (unsigned char*)"----");
    }
}

// ------------------- LCD按键处理 -------------------
void LCD_HandleKey(unsigned char key) {
    PageType current_page = menu_state.current_page;

    switch(key) {
        case 1: // 按键1：PAGE_11进入PAGE_12；PAGE_12确认删除；PAGE_15进入PAGE_16；PAGE_17确认删除；其他页上一项
            if (current_page == PAGE_1) {
                if (page1_scroll_idx > 0) {
                    page1_scroll_idx--;  // 索引减1，显示前一组（如TX3-TX4→TX2-TX3）
                } else {
                    page1_scroll_idx = PAGE1_TOTAL_DEVICES - 2;  // 循环到最后一组（TX9-TX10）
                }
                display_labels_initialized = 0;
            } else if (current_page == PAGE_2) {
                HandlePrevItem();//其中已经包含了立即刷新逻辑
            } else if (current_page == PAGE_16) {
                // 关键：使用Page14的选中索引（与报警的page10_selected一致）
                DeleteRecoveryEvent(menu_state.page14_selected);
                
                // 回退到恢复列表页（Page14），与报警回退Page10一致
                menu_state.current_page = PAGE_14;
                menu_state.page_changed = 1;
                display_labels_initialized = 0; // 强制重绘，不显示已删除事件
                RefreshDisplay(); // 主动刷新，确保立即生效
                return;
            } else if (current_page == PAGE_24) {
                if (rtc_edit_state == RTC_EDIT_SELECT) {
                    RTC_Switch_Pos(-1);  // 选择状态：上一位
                } else if (rtc_edit_state == RTC_EDIT_CHANGE) {
                    RTC_Adjust_Num(1);   // 修改状态：数字加
                }
                return;
            } else if (current_page == PAGE_25) {
                if (pwd_edit_state == PWD_EDIT_SELECT) {
                    PWD_Switch_Pos(-1);  // 选择状态：上一个密码位
                } else if (pwd_edit_state == PWD_EDIT_CHANGE) {
                    PWD_Adjust_Num(1);   // 修改状态：密码数字加1
                }
                display_labels_initialized = 0; // 触发全屏刷新
                return;
            } else if (current_page == PAGE_1) {
                // 上一项：从TX02→TX01，边界判断（与PAGE_2一致）
                if (page1_selected > 0) {
                    page1_selected--;
                    display_labels_initialized = 0;  // 刷新显示
                }
            } else if (current_page == PAGE_20) {
     
    // 删除 PAGE19 选中的记录
    DeletePage19SelectedRecord();
    
    // 返回 PAGE19，显示下一条记录
    menu_state.current_page = PAGE_19;
    menu_state.page_changed = 1;
    display_labels_initialized = 0;
    return;

						}   else if (current_page == PAGE_19) {
        // 上一项：减少选中的记录索引（0-3循环）
        if (page19_selected_record > 0) {
            page19_selected_record--;
        } else {
            page19_selected_record = 3; // 循环到最后一条
        }
        display_labels_initialized = 0; // 刷新显示
        return;
    
            } else if (current_page == PAGE_11) {
                // 从报警详细页面进入删除确认
                menu_state.prev_page = PAGE_11;
                menu_state.current_page = PAGE_12;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_12) {
                if (menu_state.page10_selected < alarm_event_count) {
                    DeleteAlarmEvent(menu_state.page10_selected);
                }
                
                // 返回到PAGE_10（报警事件列表）
                menu_state.current_page = PAGE_10;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
    //        
    //        // 发送调试信息
    //        UART4_SendString("Alarm event deleted and returning to list.\r\n");
                return;
            } else if (current_page == PAGE_15) {
                // 从预警详细页面进入删除确认（新增）
                menu_state.prev_page = PAGE_15;
                menu_state.current_page = PAGE_16;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_17) {
                menu_state.current_page = PAGE_15;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_22) {
    //                // 确认删除最高温度数据
    ////                ClearMaxTemperatureData();
                  // 返回到PAGE_21
                  menu_state.current_page = PAGE_23;
                  menu_state.page_changed = 1;
                  display_labels_initialized = 0;
            } else if (current_page == PAGE_23) {
    unsigned char actual_index = menu_state.page21_selected; // 获取PAGE21选中行（0-2）
    
    // 1. 调用删除函数，清空数据源+全局缓存（标记为无效）
    DeleteMaxTempEvent(actual_index);
    
    // 2. 强制PAGE21/PAGE22显示空值（关键：手动置空缓存）
    max_temps[actual_index] = -990; // PAGE21显示空值标记
    memset(&max_temp_times[actual_index], 0, sizeof(rtc_time_t)); // 清空时间缓存
    
    // 3. 切换回PAGE21，强制刷新显示（立即呈现空值）
    menu_state.current_page = PAGE_21;
    display_labels_initialized = 0; // 重置显示标记，强制重绘
    menu_state.page_changed = 1;
    
    // 4. 关键：不要重新调用 CheckDailyMaxTemp()！这会重新计算数据
    
    // 5. 直接刷新显示即可
    DisplayPage21(); // 手动调用绘制函数，立即显示空值
    RefreshDisplay();
    
    // 6. 发送调试信息确认删除
    UART4_SendString("PAGE_23: Max temp ");
    UART4_SendNumber(actual_index, 1);
    UART4_SendString(" deleted.\r\n");
    
    return;

            } else if (current_page == PAGE_26) {
                menu_state.current_page = PAGE_8;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else {
                HandlePrevItem();
            }
            break;

        case 2: // 按键2：PAGE_12返回PAGE_11；PAGE_11返回PAGE_10；PAGE_16返回PAGE_15；PAGE_17返回PAGE_15；PAGE_15返回PAGE_13；其他页下一项
            if (current_page == PAGE_1) {
                if (page1_scroll_idx < PAGE1_TOTAL_DEVICES - 2) {
                    page1_scroll_idx++;  // 索引加1，显示后一组（如TX1-TX2→TX2-TX3）
                } else {
                    page1_scroll_idx = 0;  // 循环到第一组（TX1-TX2）
                }
                display_labels_initialized = 0;
            } else if (current_page == PAGE_2) {
                HandleNextItem();//其中已经包含了立即刷新逻辑
            } else if (current_page == PAGE_24) {
                if (rtc_edit_state == RTC_EDIT_SELECT) {
                    RTC_Switch_Pos(1);   // 选择状态：下一位
                } else if (rtc_edit_state == RTC_EDIT_CHANGE) {
                    RTC_Adjust_Num(-1);  // 修改状态：数字减
                }
                return;
            } else if (current_page == PAGE_1) {
                // 下一项：从TX01→TX02，边界判断（与PAGE_2一致）
                if (page1_selected < (PAGE1_DEV_COUNT - 1)) {
                    page1_selected++;
                    display_labels_initialized = 0;  // 刷新显示
                }
            } else if (current_page == PAGE_25) {
                if (pwd_edit_state == PWD_EDIT_SELECT) {
                    PWD_Switch_Pos(1);   // 选择状态：下一个密码位
                } else if (pwd_edit_state == PWD_EDIT_CHANGE) {
                    PWD_Adjust_Num(-1);  // 修改状态：密码数字减1
                }
                display_labels_initialized = 0;
                return;
							} else if (current_page == PAGE_19) {
        // 下一项：增加选中的记录索引（0-3循环）
        page19_selected_record = (page19_selected_record + 1) % 4;
        display_labels_initialized = 0; // 刷新显示
        return;
            } else if (current_page == PAGE_20) {
                // 从PAGE_20返回PAGE_19
                menu_state.current_page = PAGE_19;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_22) {
                // 最高温度清除确认页面 -> 返回PAGE_21
                menu_state.current_page = PAGE_21;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_23) {
                // 从PAGE_23返回PAGE_22
                menu_state.current_page = PAGE_22;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_12) {
                menu_state.current_page = PAGE_11;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_11) {
                menu_state.current_page = PAGE_10;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_16) {
                // 从预警删除确认返回预警详细页面（新增）
                menu_state.current_page = PAGE_15;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_17) {
                // 从恢复删除确认返回恢复详细页面（新增）
                menu_state.current_page = PAGE_11;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_15) {
                // 从预警详细页面返回预警列表（新增）
                if (menu_state.prev_page == PAGE_13) {
                    menu_state.current_page = PAGE_13;
                } else if (menu_state.prev_page == PAGE_14) {
                    menu_state.current_page = PAGE_14;
                } else {
                    menu_state.current_page = PAGE_7; // 默认返回
                }
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_25) {
                // 按键2：选择状态下一位 / 编辑状态数字减
                if (pwd_edit_state == PWD_EDIT_SELECT) {
                    PWD_Switch_Pos(1); // 下一个密码位
                } else if (pwd_edit_state == PWD_EDIT_CHANGE) {
                    PWD_Adjust_Num(-1); // 数字减1（0-9循环）
                }
                display_labels_initialized = 0;
                return;
            } else if (current_page == PAGE_26) {
                // PAGE_26按键2: 返回
                menu_state.current_page = PAGE_8;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else {
                HandleNextItem();
            }
            break;
            
        case 3: // 按键3：进入/选择
            if (current_page == PAGE_24) {
                if (rtc_edit_state == RTC_EDIT_IDLE) {
                    RTC_Edit_Init();      // 空闲状态→进入选择状态
                } else if (rtc_edit_state == RTC_EDIT_SELECT) {
                    rtc_edit_state = RTC_EDIT_CHANGE; // 选择状态→进入修改状态
                } else if (rtc_edit_state == RTC_EDIT_CHANGE) {
                    RTC_Save_Edit();      // 修改状态→保存并返回空闲
                }
                display_labels_initialized = 0;
                return;
            } else if (current_page == PAGE_25) {
                if (pwd_edit_state == PWD_EDIT_IDLE) {
                    PWD_Edit_Init();      // 空闲→选择状态（开始闪烁）
                } else if (pwd_edit_state == PWD_EDIT_SELECT) {
                    pwd_edit_state = PWD_EDIT_CHANGE; // 选择→修改状态
                } else if (pwd_edit_state == PWD_EDIT_CHANGE) {
                    PWD_Save_Edit();      // 修改→保存并返回选择状态
                }
                display_labels_initialized = 0; // 触发全屏刷新
                return;
            } else if (current_page == PAGE_5) {
                menu_state.current_page = PAGE_9;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_9) {
                menu_state.current_page = PAGE_5;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_21) {
                // 从最高温度查询页面进入清除确认页面
    //                menu_state.current_page = PAGE_22;
    //                menu_state.page22_selected = 0;  // 重置选择
    //                menu_state.page_changed = 1;
    //                display_labels_initialized = 0;
                if (menu_state.page21_selected < 3 && 
                    daily_max_temps[menu_state.page21_selected].is_valid && 
                    daily_max_temps[menu_state.page21_selected].max_temp != -990) {
                    // 有有效数据：进入详情页
                    menu_state.current_page = PAGE_22;
                    menu_state.page22_selected = 0;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                }
            } else if (current_page == PAGE_10) {
                // 从报警列表进入报警详细
                menu_state.prev_page = PAGE_10;
                menu_state.current_page = PAGE_11;
                menu_state.page11_selected = 0;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_13) {
                // 从预警列表进入预警详细（新增）
                menu_state.prev_page = PAGE_13;
                menu_state.current_page = PAGE_11;
                menu_state.page15_selected = 0;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_14) {
                // 从恢复列表进入恢复详细（新增）
                menu_state.prev_page = PAGE_14;
                menu_state.current_page = PAGE_15;
                menu_state.page15_selected = 0;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_15) {
                // 从预警详细页面进入删除确认（新增）
                if (menu_state.prev_page == PAGE_13) {
                    menu_state.current_page = PAGE_16;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                } else if (menu_state.prev_page == PAGE_14) {
                    menu_state.current_page = PAGE_17;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                }
            } else if (current_page == PAGE_18) {
                // 从历史数据查询页面进入清除确认页面
							 unsigned char start_aid;
    unsigned char curr_aid;
    menu_state.current_page = PAGE_19;
    menu_state.page_changed = 1;
    display_labels_initialized = 0;
    page19_selected_record = 0;
    
    // 核心修复：计算当前选中的真实从站AID
    start_aid = page18_scroll_page * PAGE18_DISPLAY_COUNT + 1; // 当前页起始AID（0→1，1→4，2→7，3→10）
    curr_aid = start_aid + menu_state.page18_selected;       // 起始AID + 选中项索引 = 真实AID（如4+0=4→TX04）
    
    // 存储真实AID到page19_selected_slave（用于PAGE_19读取数据）
    page19_selected_slave = curr_aid;  // 直接存储AID（1-10），而非索引
    return;
            } else if (current_page == PAGE_19) {
                // 从清除确认页面进入删除确认页面
                menu_state.current_page = PAGE_20;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_18) {
                // 从历史数据查询页面进入清除确认页面
                menu_state.current_page = PAGE_19;
                menu_state.page19_selected = 0;  // 重置选择
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_25) {
                // 按键3：空闲→选择→编辑→保存
                if (pwd_edit_state == PWD_EDIT_IDLE) {
                    PWD_Edit_Init(); // 进入选择模式，默认选中第一个数字
                } else if (pwd_edit_state == PWD_EDIT_SELECT) {
                    pwd_edit_state = PWD_EDIT_CHANGE; // 进入编辑模式
                } else if (pwd_edit_state == PWD_EDIT_CHANGE) {
                    PWD_Save_Edit(); // 保存修改，返回选择模式（保留当前选中位）
                }
                display_labels_initialized = 0;
                return;
            } else if (current_page == PAGE_19) {
                // 从清除确认页面进入删除确认页面
                menu_state.current_page = PAGE_20;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else {
                HandleEnterKey();
            }
            break;
            
        case 4: // 按键4：返回
            // 检查哪些页面不响应按键4
            if (current_page == PAGE_24) {
                // 根据当前编辑状态决定返回行为
                if (rtc_edit_state == RTC_EDIT_IDLE) {
                    // 浏览模式：返回菜单
                    menu_state.current_page = PAGE_8;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    RefreshDisplay();
                } else {
                    // 选择/编辑模式：只退出编辑状态，停留在当前页面
                    rtc_edit_state = RTC_EDIT_IDLE; // 退出编辑状态
                    // 补充：重置临时数据，避免下次进入编辑复用旧值
                    edit_temp_time = current_rtc_time;
                    display_labels_initialized = 0; // 强制刷新显示
                    RefreshDisplay();
                }
                return; // 终止后续逻辑，避免冲突
            } else if (current_page == PAGE_25) {
                // 修复核心：按按键4直接返回上一级菜单（PAGE_8），同时重置编辑状态
                static unsigned char pwd_back_count = 0;
                
                if (pwd_edit_state == PWD_EDIT_CHANGE) {
                    // 编辑模式：按1次退出编辑→选择模式，按2次返回菜单
                    pwd_back_count++;
                    if (pwd_back_count >= 2) {
                        pwd_edit_state = PWD_EDIT_IDLE; // 重置编辑状态
                        pwd_back_count = 0;
                        // 返回上一级菜单（设备维护 PAGE_8）
                        menu_state.current_page = PAGE_8;
                        menu_state.page_changed = 1;
                    } else {
                        pwd_edit_state = PWD_EDIT_SELECT; // 第一次按：退出编辑→选择模式
                    }
                } else if (pwd_edit_state == PWD_EDIT_SELECT) {
                    // 选择模式：按1次直接返回菜单
                    pwd_edit_state = PWD_EDIT_IDLE;
                    pwd_back_count = 0;
                    menu_state.current_page = PAGE_8;
                    menu_state.page_changed = 1;
                } else if (pwd_edit_state == PWD_EDIT_IDLE) {
                    // 空闲模式（仅显示）：按1次返回菜单
                    menu_state.current_page = PAGE_8;
                    menu_state.page_changed = 1;
                }
                
                display_labels_initialized = 0;
                RefreshDisplay();
                return;
            }
            
            switch(current_page) {
                case PAGE_1:     // 主页面不返回
                case PAGE_11:    // 报警详细页面
                case PAGE_12:    // 删除确认页面
                case PAGE_15:    // 预警/恢复详细页面
                case PAGE_16:    // 预警删除确认
                case PAGE_17:    // 恢复删除确认
                case PAGE_20:    // 历史数据删除确认页面
                case PAGE_23:    // 最高温度删除确认页面
                    // 这些页面不响应按键4
                    break;
                    
                case PAGE_18:    // 历史数据查询页面 -> 返回PAGE_7
                    menu_state.current_page = PAGE_7;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
                    
                case PAGE_19:    // 清除历史数据确认页面 -> 返回PAGE_18
                    menu_state.current_page = PAGE_18;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
                    
                case PAGE_21:    // 最高温度查询页面 -> 返回PAGE_7
                    menu_state.current_page = PAGE_7;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
                    
                default:        // 其他页面调用HandleReturnKey
                    HandleReturnKey();
                    break;
            }
            break;
    }
    
    RefreshDisplay();
}

// ------------------- 处理上一项 -------------------
static void HandlePrevItem(void)
{
    unsigned char start_idx;
    static unsigned char last_prev_start_idx;
    
    switch(menu_state.current_page) {
        case PAGE_2:
            if (menu_state.page2_selected > 0) {
                menu_state.page2_selected--; // 立即更新选中索引
                start_idx = GetPage2StartIndex(); // 实时计算新起始索引
                UpdatePage2MenuItems(start_idx);  // 立即更新文本
                RefreshPage2Arrow();              // 立即刷新箭头
            }
            break;

        case PAGE_3: // 新增：处理 PAGE_3 的上一项
            if (menu_state.page3_selected > 0) {
                menu_state.page3_selected--;
                DisplayPage3();
            }
            break;
            
        case PAGE_7:
            if (menu_state.page7_selected > 0) {
                menu_state.page7_selected--; // 数据层：选中索引更新（3→2）
                start_idx = GetPage7StartIndex(); // 实时计算新起始索引
                UpdatePage7MenuItems(start_idx); // 强制更新文本（显示第3项内容）
                RefreshPage7Arrow(); // 强制刷新箭头（指向第3项）
                menu_state.page_changed = 1;
               // display_labels_initialized = 0; // 额外确保显示标记重置，无残留
            }
            break;
            
        case PAGE_8:
            if (menu_state.page8_selected > 0) {
                menu_state.page8_selected--; // 立即更新选中索引
                RefreshPage8Arrow(); // 按键后立即刷新箭头，无延迟
                menu_state.page_changed = 1;
            }
            break;

        case PAGE_10:
            if (menu_state.page10_selected > 0) {
                menu_state.page10_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_11:
            if (menu_state.page11_selected > 0) {
                menu_state.page11_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_13:
            if (menu_state.page13_selected > 0) {
                menu_state.page13_selected--; // 选中索引上移
                display_labels_initialized = 0; // 刷新显示
                RefreshDisplay();
            }
            break;
            
        case PAGE_14:
            if (menu_state.page14_selected > 0) {
                menu_state.page14_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_15:
            if (menu_state.page15_selected > 0) {
                menu_state.page15_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_18:  // 新增：PAGE_18上一项
            if (menu_state.page18_selected > 0) {
        // 1. 页内上一项：选中项-1（2→1、1→0）
        menu_state.page18_selected--;
    } else {
        // 2. 上翻页：当前页第一项，切换到上一页，选中项设为新页最后一个（索引2）
        if (page18_scroll_page > 0) {
            page18_scroll_page--;
            menu_state.page18_selected = PAGE18_DISPLAY_COUNT - 1;  // 新页选中第三个（索引2）
            display_labels_initialized = 0;  // 强制刷新页面，显示新页数据
        }
    }
    RefreshPage18Arrow();  // 刷新箭头位置
    menu_state.page_changed = 1;
            break;
            
  
            
        case PAGE_21:  // 新增：PAGE_21上一项
            if (menu_state.page21_selected > 0) {
                menu_state.page21_selected--;
                DisplayPage21();
            }
            break;
            
        case PAGE_22:  // 新增：PAGE_22上一项
            if (menu_state.page22_selected > 0) {
                menu_state.page22_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_24:
            if (menu_state.page24_selected > 0) {
                menu_state.page24_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_25:
            if (menu_state.page24_selected > 0) {
                menu_state.page24_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        default:
            break;
    }
}

// ------------------- 处理下一项 -------------------
static void HandleNextItem(void)
{
    unsigned char start_idx;
    static unsigned char last_next_start_idx;
    
    switch(menu_state.current_page) {
        case PAGE_2:
            if (menu_state.page2_selected < (PAGE2_ITEM_COUNT - 1)) {
                menu_state.page2_selected++; // 立即更新选中索引
                start_idx = GetPage2StartIndex(); // 实时计算新起始索引
                UpdatePage2MenuItems(start_idx);  // 立即更新文本
                RefreshPage2Arrow();              // 立即刷新箭头
            }
            break;
            
        case PAGE_3: // 新增：处理 PAGE_3 的下一项
            if (menu_state.page3_selected < (PAGE3_MAX_MODULES - 1)) {
                menu_state.page3_selected++;
                DisplayPage3();
            }
            break;
            
        case PAGE_7:
            if (menu_state.page7_selected < (PAGE7_ITEM_COUNT - 1)) {
                menu_state.page7_selected++; // 数据层：选中索引更新（2→3）
                start_idx = GetPage7StartIndex(); // 实时计算新起始索引
                UpdatePage7MenuItems(start_idx); // 强制更新文本（显示第4项内容）
                RefreshPage7Arrow(); // 强制刷新箭头（指向第4项）
                menu_state.page_changed = 1;
                //display_labels_initialized = 0; // 额外确保显示标记重置，无残留
            }
            break;
            
        case PAGE_8:
            if (menu_state.page8_selected < 2) { // 3个选项，最大索引2
                menu_state.page8_selected++; // 立即更新选中索引
                RefreshPage8Arrow(); // 按键后立即刷新箭头，无延迟
                menu_state.page_changed = 1;
            }
            break;
            
        case PAGE_10:
            if (menu_state.page10_selected < 2) {
                menu_state.page10_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_11:
            if (menu_state.page11_selected < (HISTORY_SIZE - 1)) {
                menu_state.page11_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_13:
            if (menu_state.page13_selected < 2) {
                menu_state.page13_selected++; // 选中索引下移
                display_labels_initialized = 0; // 刷新显示
                RefreshDisplay();
            }
            break;
            
        case PAGE_14:
            if (menu_state.page14_selected < 2) {
                menu_state.page14_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_15:
            if (menu_state.page15_selected < 9) {
                menu_state.page15_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_18:  // 新增：PAGE_18下一项
         if (menu_state.page18_selected < PAGE18_DISPLAY_COUNT - 1) {
        // 1. 页内下一项：选中项+1（0→1、1→2）
        menu_state.page18_selected++;
    } else {
        // 2. 下翻页：当前页最后一项，切换到下一页，选中项设为新页第一个（索引0）
        if (page18_scroll_page < PAGE18_MAX_PAGE - 1) {
            page18_scroll_page++;
            menu_state.page18_selected = 0;  // 新页选中第一个（索引0）
            display_labels_initialized = 0;  // 强制刷新页面
        }
    }
    RefreshPage18Arrow();  // 刷新箭头位置
    menu_state.page_changed = 1;
            break;
            
    
            
        case PAGE_21:  // 新增：PAGE_21下一项
            if (menu_state.page21_selected < 2) {  // 0-2共3项
                menu_state.page21_selected++;
                DisplayPage21();
            }
            break;
            
        case PAGE_22:  // 新增：PAGE_22下一项
            if (menu_state.page22_selected < 1) {  // 0-1共2项
                menu_state.page22_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_24:
            if (menu_state.page24_selected < 1) {  // 0-1共2项
                menu_state.page24_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        case PAGE_25:
            if (menu_state.page24_selected < 1) {  // 0-1共2项
                menu_state.page24_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
            
        default:
            break;
    }
}

// ------------------- 处理进入键 -------------------
static void HandleEnterKey(void)
{
    switch(menu_state.current_page) {
        case PAGE_1:
            menu_state.current_page = PAGE_2;
            menu_state.page2_selected = 0;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            //led_toggle(3);
            break;
            
        case PAGE_2:
            switch(menu_state.page2_selected) {
                case MENU_MEASURE_DATA: // 假设这个选项进入 PAGE_3
                    menu_state.current_page = PAGE_3;
                    menu_state.page3_selected = 0; // 进入时，默认选中第1个模块
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0; // 需要初始化标签
                    break;
                    
                case MENU_SENSOR_STATUS:
                    menu_state.current_page = PAGE_4;
                    break;
                    
                case MENU_PARAM_QUERY:
                    menu_state.current_page = PAGE_5;
                    break;
                    
                case MENU_HISTORY_RECORD:
                    menu_state.current_page = PAGE_7;
                    break;
                    
                case MENU_DEVICE_MAINT:
                    menu_state.current_page = PAGE_8;
                    break;
            }
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_7:
            switch(menu_state.page7_selected) {
                case MENU_SENSOR_EVENT_1:
                    // 温度报警事件记录 -> 进入 PAGE_10
                    menu_state.current_page = PAGE_10;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
                    
                case MENU_SENSOR_EVENT_2:
                    // 温度预警时间记录 -> 进入 PAGE_13
                    menu_state.current_page = PAGE_13;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
                    
                case MENU_SENSOR_EVENT_3:
                    // 传感器恢复事件记录 -> 进入 PAGE_14
                    menu_state.current_page = PAGE_14;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
                    
                case MENU_HISTORY_DATA_QUERY:
                    // 历史数据查询 -> 进入 PAGE_18
                    menu_state.current_page = PAGE_18;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
                    
                case MENU_MAX_TEMP_QUERY:
                    menu_state.current_page = PAGE_21;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
            }
            break;
            
        case PAGE_8:
            switch(menu_state.page8_selected) {
                case 0:  // 修改日期时间
                    menu_state.current_page = PAGE_24;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
                    
                case 1:  // 修改密码
                    menu_state.current_page = PAGE_25;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
                    
                case 2:  // 恢复出厂设置
                    menu_state.current_page = PAGE_26;
                    menu_state.page_changed = 1;
                    display_labels_initialized = 0;
                    break;
            }
            break;
            
        case PAGE_10:
            led_toggle(1);
            break;
            
        case PAGE_13:
            led_toggle(2);
            break;
            
        default:
            break;
    }
}

// ------------------- 处理返回键（按键4） -------------------
static void HandleReturnKey(void)
{
    switch(menu_state.current_page) {
        case PAGE_2:
            menu_state.current_page = PAGE_1;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_3:
        case PAGE_4:
        case PAGE_5:
            menu_state.current_page = PAGE_2;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_7:
            menu_state.current_page = PAGE_2;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_8:
            menu_state.current_page = PAGE_2;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_9:
            menu_state.current_page = PAGE_5;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_10:
            menu_state.current_page = PAGE_7;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_11:
//            menu_state.current_page = PAGE_10;
//            menu_state.page_changed = 1;
//            display_labels_initialized = 0;
            break;
            
        case PAGE_13:
            menu_state.current_page = PAGE_7;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_14:
            menu_state.current_page = PAGE_7;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_15:
            break;
            
        case PAGE_18:  // 新增：从PAGE_18返回PAGE_7
            menu_state.current_page = PAGE_7;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_19:  // 新增：从PAGE_19返回PAGE_18
            menu_state.current_page = PAGE_18;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_20:  // 新增：从PAGE_20返回PAGE_19
            menu_state.current_page = PAGE_19;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_21:  // 新增：从PAGE_21返回PAGE_7
            menu_state.current_page = PAGE_7;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_23:  // 新增：从PAGE_23返回PAGE_22
            menu_state.current_page = PAGE_22;
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_24:
            menu_state.current_page = PAGE_8;  // 返回到设备维护菜单
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_25:
            menu_state.current_page = PAGE_8;  // 返回到设备维护菜单
            menu_state.page_changed = 1;
            display_labels_initialized = 0;
            break;
            
        case PAGE_26:
            break;				
            
        default:
            break;
    }
}

// ------------------- 刷新显示 -------------------
static void RefreshDisplay(void) {
    if (menu_state.page_changed) {
        // 关键：添加 PAGE_18 到白名单，与 PAGE_2/PAGE_7/PAGE_8 一致仅刷新箭头
        if (menu_state.current_page != PAGE_2 && menu_state.current_page != PAGE_7 && 
            menu_state.current_page != PAGE_8 && menu_state.current_page != PAGE_18) {
            display_labels_initialized = 0;
            DisplayFixedLabels();
        } else if (menu_state.current_page == PAGE_2) {
            RefreshPage2Arrow();
        } else if (menu_state.current_page == PAGE_7) {
            RefreshPage7Arrow();
        } else if (menu_state.current_page == PAGE_8) {
            RefreshPage8Arrow();
        } else if (menu_state.current_page == PAGE_18) { // 新增 PAGE_18 分支
            RefreshPage18Arrow(); // 仅刷新箭头，不重置固定标签
        }
        // 保留 PAGE_21/PAGE_22 强制刷新逻辑
        else if (menu_state.current_page == PAGE_21 || menu_state.current_page == PAGE_22) {
            display_labels_initialized = 0;
            DisplayFixedLabels();
        }
        menu_state.page_changed = 0;
    }
}

// 检查并记录报警事件
// 检查并记录报警事件（原函数名不变，补充恢复事件记录）
static void CheckAndRecordAlarm(void) {
    unsigned char i;
    DataRecord* dev_data = NULL;
    unsigned char current_abnormal = 0;
    
    for (i = 0; i < TOTAL_SLAVES; i++) {
        dev_data = GetRecentDataByAID(i + 1);
        
        if (dev_data != NULL && dev_data->is_valid) {
            current_abnormal = (dev_data->temp > 25) ? 1 : 0;
            
            if (current_abnormal == 1) {
                // 温度>25℃：异常状态
                if (last_abnormal_status[i] == 0) {
                    // 首次进入异常：生成新报警事件
                    RecordAlarmEvent(dev_data->pid, i + 1, dev_data->temp, dev_data->volt1 * 100);
                    last_abnormal_status[i] = 1;
                    
                    // 调试信息：确认报警被记录
                    UART4_SendString("Alarm recorded: AID=");
                    UART4_SendNumber(i + 1, 2);
                    UART4_SendString(", Temp=");
                    UART4_SendNumber(dev_data->temp, 2);
                    UART4_SendString("\r\n");
                } else {
                    // 持续异常：更新为当前最高温（核心新增逻辑）
                    unsigned char j, alarm_idx;
                    // 查找该从站（AID=i+1）对应的最新有效报警事件
                    for (j = 0; j < alarm_event_count; j++) {
                        alarm_idx = (alarm_event_next_index - j - 1 + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS;
                        // 匹配AID且事件有效
                        if (alarm_events[alarm_idx].is_valid && alarm_events[alarm_idx].aid == (i + 1)) {
                            // 当前温度更高时更新
                            if (dev_data->temp > alarm_events[alarm_idx].temp) {
                                alarm_events[alarm_idx].temp = dev_data->temp; // 更新最高温
                                alarm_events[alarm_idx].volt_mv = dev_data->volt1 * 100; // 同步更新电压（可选）
                                GetCurrentRTC();
                                alarm_events[alarm_idx].timestamp = current_rtc_time; // 同步更新时间戳（可选）
                                
                                // 调试信息：确认温度更新
                                UART4_SendString("Alarm updated: AID=");
                                UART4_SendNumber(i + 1, 2);
                                UART4_SendString(", New Temp=");
                                UART4_SendNumber(dev_data->temp, 2);
                                UART4_SendString("\r\n");
                            }
                            break; // 找到最新事件后退出，避免重复更新
                        }
                    }
                }
            } else if (current_abnormal == 0 && last_abnormal_status[i] == 1) {
                // 温度≤25℃且之前是异常：记录恢复事件（原有逻辑不变）
                unsigned char j, alarm_idx;
                for (j = 0; j < alarm_event_count; j++) {
                    alarm_idx = (alarm_event_next_index - j - 1 + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS;
                    if (alarm_events[alarm_idx].is_valid && alarm_events[alarm_idx].aid == (i + 1)) {
                        RecordRecoveryEvent(
                            dev_data->pid,
                            i + 1,
                            alarm_events[alarm_idx].temp, // 记录最终最高温作为异常温度
                            dev_data->temp,
                            dev_data->volt1 * 100,
                            alarm_events[alarm_idx].timestamp
                        );
                        UART4_SendString("Recovery recorded: AID=");
                        UART4_SendNumber(i + 1, 2);
                        UART4_SendString(", Recovery Temp=");
                        UART4_SendNumber(dev_data->temp, 2);
                        UART4_SendString("\r\n");
                        break;
                    }
                }
                last_abnormal_status[i] = 0;
            }
        }
    }
}
// 记录报警事件
static void RecordAlarmEvent(unsigned char pid, unsigned char aid, unsigned char temp, unsigned int volt_mv) {
    // 强制读取最新RTC时间（避免时间为空）
    GetCurrentRTC();
    
    alarm_events[alarm_event_next_index].pid = pid;
    alarm_events[alarm_event_next_index].aid = aid;
    alarm_events[alarm_event_next_index].temp = temp;
    alarm_events[alarm_event_next_index].volt_mv = volt_mv;
    alarm_events[alarm_event_next_index].timestamp = current_rtc_time;
    alarm_events[alarm_event_next_index].is_valid = 1;
    
    // 保存到历史数据
    AddToHistoryData(pid, aid, temp, volt_mv);
    
    // 更新索引（循环覆盖）
    alarm_event_next_index = (alarm_event_next_index + 1) % MAX_ALARM_EVENTS;
    
    if (alarm_event_count < MAX_ALARM_EVENTS) {
        alarm_event_count++;
    }
    
    // 调试信息：输出报警事件索引和时间
    UART4_SendString("Alarm index=");
    UART4_SendNumber(alarm_event_next_index, 1);
    UART4_SendString(", Time=");
    UART4_SendNumber(current_rtc_time.year, 2);
    UART4_SendString("-");
    UART4_SendNumber(current_rtc_time.mon, 2);
    UART4_SendString("-");
    UART4_SendNumber(current_rtc_time.day, 2);
    UART4_SendString("\r\n");
}

// 在PAGE_10上显示报警事件
static void DisplayAlarmEventsOnPage10(void) {
    unsigned char i;
    unsigned char row;
    unsigned char display_index;
    AlarmRecord* event = NULL;
    
    // 只在需要初始化时显示固定内容
    if (display_labels_initialized == 0) {
        LCD_Clear();
        
        // 绘制固定标签
        for (i = 0; i < 3; i++) {
            row = i * 2;  
            
            if (i == menu_state.page10_selected) {
                LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  
            } else {
                LCD_DisplayChar(row, 0, ' ');
            }
            
            LCD_DISPLAYCHAR_NEW(row, 8, 2, 19);   // "事"
            LCD_DISPLAYCHAR_NEW(row, 16, 3, 19);  // "件"
            LCD_DisplayChar(row, 24, '1' + i);    // 编号
        }
        
        // 显示"时间："标签
        for (i = 0; i < 3; i++) {
            row = i * 2;
            LCD_DISPLAYCHAR_NEW(row, 40, 2, 20); // "时"
            LCD_DISPLAYCHAR_NEW(row, 48, 3, 20); // "间"
            LCD_DISPLAYCHAR_NEW(row, 56, 0, 15); // ":"
        }
        
        // 功能提示
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
        
        LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "上"
        LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
        LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "页"
        
        LCD_DISPLAYCHAR_NEW(6, 80, 0, 26); // "详"
        LCD_DISPLAYCHAR_NEW(6, 88, 1, 26); // "情"
        
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"
        
        display_labels_initialized = 1;
    }
    
   // 显示报警事件数据
for (i = 0; i < 3; i++) {
    row = i * 2;
    
    // 清空数据区域（只清空时间显示区域）
    // 从第64列开始清空（时间数据显示区域）
    LCD_DisplayString(row, 64, (unsigned char*)"            ");
    
    if (i < alarm_event_count) {
        // 计算显示索引（最新的在前）
        display_index = (alarm_event_next_index - 1 - i + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS;
        event = &alarm_events[display_index];
        
        if (event != NULL && event->is_valid) {
            // 只需要显示时间数据，"事件X"和"时间："已经是固定标签了
            // 显示时间（格式：25-12-31）
            // 在"时间："标签右侧开始显示
            
            // 年份（2位）：比如25
            LCD_DisplayNumber(row, 64, event->timestamp.year, 2);
            // 分隔符"-"
            LCD_DisplayChar(row, 80, '-');
            // 月份（2位）：比如12
            LCD_DisplayNumber(row, 88, event->timestamp.mon, 2);
            // 分隔符"-"
            LCD_DisplayChar(row, 104, '-');
            // 日期（2位）：比如31
            LCD_DisplayNumber(row, 112, event->timestamp.day, 2);
            

        } else {
            // 无效数据，显示"-- -- --"
            LCD_DisplayString(row, 64, (unsigned char*)"-- -- --");
        }
    } else {
        // 没有事件记录，显示空
        LCD_DisplayString(row, 64, (unsigned char*)"           ");
    }
}
}
// 添加数据到历史记录
static void AddToHistoryData(unsigned char pid, unsigned char aid, unsigned char temp, unsigned int volt_mv) {
    unsigned char slave_idx;
    unsigned short hist_start;
    unsigned char hist_idx;
    unsigned short hist_pos;
    
    if (aid < 1 || aid > TOTAL_SLAVES) {
        return;
    }
    
    slave_idx = aid - 1;
    hist_start = HISTORY_DATA_START + (slave_idx * RECORDS_PER_SLAVE);
    hist_idx = history_index[slave_idx];
    hist_pos = hist_start + hist_idx;
    
    if (hist_pos < TOTAL_RECORDS) {
        data_summary[hist_pos].pid = pid;
        data_summary[hist_pos].aid = aid;
        data_summary[hist_pos].temp = temp;
        data_summary[hist_pos].volt1 = (unsigned char)(volt_mv / 100);
        data_summary[hist_pos].timestamp = GetSystemTick();
        data_summary[hist_pos].is_valid = 1;
        
        history_index[slave_idx] = (hist_idx + 1) % RECORDS_PER_SLAVE;
    }
}



// ------------------- 删除指定报警事件 -------------------
void DeleteAlarmEvent(unsigned char display_index) {
    unsigned char actual_index;
    unsigned char i;
	unsigned char idx;
	unsigned char j;
    if (display_index >= alarm_event_count) {
        return; // 无效索引
    }
    
    // 计算实际数组索引（最新的在前面）
    actual_index = (alarm_event_next_index - display_index - 1 + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS;
    
    if (alarm_events[actual_index].is_valid) {
        // 标记为无效
        alarm_events[actual_index].is_valid = 0;
        
        // 如果是循环队列中的最后一个有效事件，减少计数
        if (display_index == 0 && alarm_event_count > 0) {
            // 移动索引，覆盖被删除的事件
            alarm_event_next_index = (alarm_event_next_index - 1 + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS;
            alarm_event_count--;
            
            // 更新其他事件的显示索引
            for (i = 0; i < alarm_event_count; i++) {
                idx = (alarm_event_next_index - i - 1 + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS;
                if (!alarm_events[idx].is_valid) {
                    // 找到下一个有效事件
                    for (j = idx; j != alarm_event_next_index; j = (j - 1 + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS) {
                        if (alarm_events[(j + 1) % MAX_ALARM_EVENTS].is_valid) {
                            // 复制下一个有效事件到当前位置
                            alarm_events[j] = alarm_events[(j + 1) % MAX_ALARM_EVENTS];
                            alarm_events[(j + 1) % MAX_ALARM_EVENTS].is_valid = 0;
                        }
                    }
                }
            }
        }
        
        UART4_SendString("Alarm event deleted.\r\n");
    }
}

// ------------------- 清空所有报警事件 -------------------
void ClearAllAlarmEvents(void) {
    unsigned char i;
    
    for (i = 0; i < MAX_ALARM_EVENTS; i++) {
        alarm_events[i].is_valid = 0;
    }
    
    alarm_event_count = 0;
    alarm_event_next_index = 0;
    
    // 重置报警状态跟踪数组
    for (i = 0; i < TOTAL_SLAVES; i++) {
        last_abnormal_status[i] = 0;
    }
    
    UART4_SendString("All alarm events cleared.\r\n");
}


// 保留原GetMaxTempRecords函数（数据读取逻辑不变）
static void GetMaxTempRecords_Ext(short max_temps[], rtc_time_t max_temp_times[], 
                                unsigned char *pid_ptr, unsigned char *aid_ptr, 
                                unsigned short *volt_mv_ptr, DataRecord **target_record) {
    unsigned char selected_row = menu_state.page21_selected;
    unsigned char i;
    
    CheckDailyMaxTemp();
    *pid_ptr = 0;
    *aid_ptr = 0;
    *volt_mv_ptr = 0;
    *target_record = NULL;
    
    for (i = 0; i < 3; i++) {
        // 强化校验：必须is_valid=1且max_temp!=-990才视为有效
        if (daily_max_temps[i].is_valid && daily_max_temps[i].max_temp != -990 && daily_max_temps[i].temp_time.year != 0) {
            max_temps[i] = daily_max_temps[i].max_temp;
            max_temp_times[i] = daily_max_temps[i].temp_time;
        } else {
            max_temps[i] = -990;
            memset(&max_temp_times[i], 0, sizeof(rtc_time_t));
        }
    }
    
    
    // 选中行数据赋值（有效数据优先，无则取实时数据）
    if (selected_row < 3) {
        if (daily_max_temps[selected_row].is_valid && daily_max_temps[selected_row].max_temp != -990) {
            *pid_ptr = daily_max_temps[selected_row].pid;
            *aid_ptr = daily_max_temps[selected_row].aid;
            *volt_mv_ptr = daily_max_temps[selected_row].volt_mv;
        } else {
            DataRecord *realtime_rec = GetRecentDataByAID(selected_row + 1);
            if (realtime_rec != NULL && realtime_rec->is_valid) {
                *pid_ptr = realtime_rec->pid;
                *aid_ptr = realtime_rec->aid;
                *volt_mv_ptr = realtime_rec->volt1 * 100;
            }
        }
    }
}
// 兼容层函数：保持原函数签名，内部调用扩展函数
// 兼容层函数：保持原函数签名，内部调用扩展函数
static void GetMaxTempRecords(short max_temps[], rtc_time_t max_temp_times[]) {
    unsigned char i;
    
    // 强制更新最高温数据
    CheckDailyMaxTemp();
    
    // 将数据复制到传入的数组中
    for (i = 0; i < 3; i++) {
        if (daily_max_temps[i].is_valid && daily_max_temps[i].max_temp != -990) {
            max_temps[i] = daily_max_temps[i].max_temp;
            max_temp_times[i] = daily_max_temps[i].temp_time;
        } else {
            max_temps[i] = -990;
            memset(&max_temp_times[i], 0, sizeof(rtc_time_t));
        }
    }
}

//新增记录恢复事件函数：

static void RecordRecoveryEvent(unsigned char pid, unsigned char aid, 
                               unsigned char abnormal_temp, unsigned char recovery_temp,
                               unsigned int recovery_volt_mv, rtc_time_t abnormal_time) {
    GetCurrentRTC(); // 获取恢复时的实时时间
    
    recovery_events[recovery_event_next_index].pid = pid;
    recovery_events[recovery_event_next_index].aid = aid;
    recovery_events[recovery_event_next_index].abnormal_temp = abnormal_temp;
    recovery_events[recovery_event_next_index].recovery_temp = recovery_temp;
    recovery_events[recovery_event_next_index].recovery_volt_mv = recovery_volt_mv;
    recovery_events[recovery_event_next_index].abnormal_timestamp = abnormal_time;
    recovery_events[recovery_event_next_index].recovery_timestamp = current_rtc_time;
    recovery_events[recovery_event_next_index].is_valid = 1;
    
    // 循环覆盖逻辑（与报警事件一致）
    recovery_event_next_index = (recovery_event_next_index + 1) % MAX_RECOVERY_EVENTS;
    if (recovery_event_count < MAX_RECOVERY_EVENTS) {
        recovery_event_count++;
    }
}
															 
// 实现恢复事件删除函数
static void DeleteRecoveryEvent(unsigned char display_index) {
    unsigned char actual_index;
    unsigned char i;
    unsigned char idx;
    unsigned char j;

    // 1. 无效索引直接返回（与报警删除一致）
    if (display_index >= recovery_event_count) {
        UART4_SendString("Recovery delete: invalid index\r\n");
        return;
    }

    // 2. 计算实际存储索引（完全复刻报警的索引公式）
    actual_index = (recovery_event_next_index - display_index - 1 + MAX_RECOVERY_EVENTS) % MAX_RECOVERY_EVENTS;

    // 3. 标记事件为无效（核心操作）
    if (alarm_events[actual_index].is_valid) {
        recovery_events[actual_index].is_valid = 0;
        UART4_SendString("Recovery event marked invalid\r\n");
    }

    // 4. 调整事件计数和索引（报警删除的核心逻辑，之前可能漏了这步）
    if (display_index == 0 && recovery_event_count > 0) {
        // 移动索引，覆盖被删除事件
        recovery_event_next_index = (recovery_event_next_index - 1 + MAX_RECOVERY_EVENTS) % MAX_RECOVERY_EVENTS;
        recovery_event_count--;

        // 重新整理有效事件，避免列表显示空条目
        for (i = 0; i < recovery_event_count; i++) {
            idx = (recovery_event_next_index - i - 1 + MAX_RECOVERY_EVENTS) % MAX_RECOVERY_EVENTS;
            if (!recovery_events[idx].is_valid) {
                // 找到下一个有效事件，移动到当前位置
                for (j = idx; j != recovery_event_next_index; j = (j - 1 + MAX_RECOVERY_EVENTS) % MAX_RECOVERY_EVENTS) {
                    if (recovery_events[(j + 1) % MAX_RECOVERY_EVENTS].is_valid) {
                        recovery_events[j] = recovery_events[(j + 1) % MAX_RECOVERY_EVENTS];
                        recovery_events[(j + 1) % MAX_RECOVERY_EVENTS].is_valid = 0;
                    }
                }
            }
        }
    }

    UART4_SendString("Recovery delete success. Count=");
    UART4_SendNumber(recovery_event_count, 1);
    UART4_SendString("\r\n");
}





static void DisplayPassword(void) {
    unsigned char i;
    unsigned char base_page = 4;  // 密码显示行（固定）
    unsigned char col_offset = 8; // 密码显示起始列（固定）
    unsigned char ch;
    unsigned char *pwd_ptr = NULL;

    static unsigned int flash_timer = 0;
    static bit flash_state = 1;

    // 闪烁频率控制（保持与PAGE_24一致）
    flash_timer++;
    if (flash_timer >= FLASH_INTERVAL) {
        flash_timer = 0;
        flash_state = !flash_state;
    }

    // 密码指针赋值（遵循PAGE_24指针模式）
    if (pwd_edit_state != PWD_EDIT_IDLE) {
        pwd_ptr = edit_temp_pwd;
    } else {
        pwd_ptr = current_password;
    }

    // 循环显示6位密码（每个数字间加空格，间隔16列）
    for (i = 0; i < 6; i++) {
        // 列偏移计算：每个数字占8列，空格占8列，总间隔16列
        unsigned char curr_col = col_offset + (i * 16); 
        ch = pwd_ptr[i] + '0';

        // 非选中位：正常显示密码数字
        if (i != pwd_edit_pos) {
            LCD_DisplayChar(base_page, curr_col, ch);
            continue;
        }

        // 选中位：闪烁显示（数字/空白）
        if (pwd_edit_state != PWD_EDIT_IDLE) {
            if (flash_state) {
                LCD_DisplayChar(base_page, curr_col, ch);
            } else {
                LCD_DisplayChar(base_page, curr_col, ' ');
            }
        } else {
            LCD_DisplayChar(base_page, curr_col, ch);
        }
    }

    // 无需额外绘制空格：通过列间隔自动实现数字间空白（8列空格宽度）
}
// 1. 初始化选择状态（按键3：空闲→选择）
// 1. 进入选择状态（按键3：空闲→选择）
static void PWD_Edit_Init(void) {
    if (pwd_edit_state == PWD_EDIT_IDLE) {
        // 进入编辑态：临时密码复制当前密码（对应PAGE_24的 edit_temp_time = current_rtc_time）
        memcpy(edit_temp_pwd, current_password, sizeof(edit_temp_pwd));
        pwd_edit_state = PWD_EDIT_SELECT;
        pwd_edit_pos = 0;
        // 重置闪烁状态
        flash_timer = 0;
        flash_state = 1;
    }
    display_labels_initialized = 0;
}

// 4. 切换密码位函数（对齐PAGE_24的RTC_Switch_Pos）
static void PWD_Switch_Pos(signed char step) {
    pwd_edit_pos += step;
    // 边界循环（0-5，对应6位密码）
    if (pwd_edit_pos < 0) pwd_edit_pos = 5;
    if (pwd_edit_pos > 5) pwd_edit_pos = 0;
    // 切换后重置闪烁状态，确保选中位立即开始闪烁（与PAGE_24一致）
//    static unsigned int flash_timer = 0;
//    static bit flash_state = 1;
    flash_timer = 0;
    flash_state = 1;
    display_labels_initialized = 0;
}
// 3. 调整数字（按键1/2：数字加减）
static void PWD_Adjust_Num(signed char step) {
    // 编辑态：修改临时密码（对应PAGE_24的修改 edit_temp_time）
    if (pwd_edit_state == PWD_EDIT_CHANGE) {
        edit_temp_pwd[pwd_edit_pos] += step;
        // 数字循环（0-9）
        if (edit_temp_pwd[pwd_edit_pos] > 9) edit_temp_pwd[pwd_edit_pos] = 0;
        if (edit_temp_pwd[pwd_edit_pos] < 0) edit_temp_pwd[pwd_edit_pos] = 9;
    }
    display_labels_initialized = 0;
}
// 保存密码修改（之前遗漏的函数实现）
static void PWD_Save_Edit(void) {
    // 保存修改：临时密码覆盖当前密码（对应PAGE_24的 rtc_write(&edit_temp_time)）
    memcpy(current_password, edit_temp_pwd, sizeof(current_password));
    memcpy(default_password, current_password, sizeof(default_password));
    pwd_edit_state = PWD_EDIT_SELECT;
    flash_state = 0; // 闪烁停止
    display_labels_initialized = 0;
    UART4_SendString("Password saved successfully!\r\n");
}



//// 判断两个时间是否为同一天
//static bit IsSameDay(rtc_time_t t1, rtc_time_t t2) {
//    return (t1.year == t2.year) && (t1.mon == t2.mon) && (t1.day == t2.day);
//}

//// 从系统时间戳转换为rtc_time_t（需确保系统tick与RTC同步，若已有该函数可复用）
//static rtc_time_t TickToRTC(unsigned long tick) {
//    rtc_time_t time;
//    // 此处需根据系统tick的计时单位（如1ms/1tick）实现转换
//    // 示例逻辑（需根据实际RTC初始化逻辑调整）：
//    time.year = (current_rtc_time.year - 3) + (tick / (86400000UL * 3)) / 365;
//    time.mon = current_rtc_time.mon;
//    time.day = current_rtc_time.day - (tick / 86400000UL);
//    time.hour = 0;
//    time.min = 0;
//    time.sec = 0;
//    return time;
//}
// 检查是否跨天，更新每日最高温数组
static void CheckDailyMaxTemp(void) {
    rtc_time_t curr_time;
    bit is_new_day = 0;
    unsigned char slave_idx = 0;
    unsigned char hist_idx = 0;
    unsigned short hist_start = 0;
    unsigned short hist_pos = 0;
    DataRecord *hist_rec = NULL;
    DataRecord *realtime_rec = NULL;
    short current_temp = 0;
    
    if (disable_max_temp_calc) {
        return;
    }
    
    GetCurrentRTC();
    curr_time = current_rtc_time;
    
    // 初始化当前日期（首次调用时）
    if (current_system_date[0] == 0 && current_system_date[1] == 0 && current_system_date[2] == 0) {
        current_system_date[0] = curr_time.year;
        current_system_date[1] = curr_time.mon;
        current_system_date[2] = curr_time.day;
    }
    
    // 跨天判断
    is_new_day = (curr_time.year != current_system_date[0]) 
               || (curr_time.mon != current_system_date[1]) 
               || (curr_time.day != current_system_date[2]);
    
    if (is_new_day) {
        // 跨天时移动数据
        daily_max_temps[2] = daily_max_temps[1];
        daily_max_temps[1] = daily_max_temps[0];
        
        // 重置当天数据
        memset(&daily_max_temps[0], 0, sizeof(DailyMaxTemp));
        daily_max_temps[0].is_valid = 0;
        daily_max_temps[0].max_temp = -990;
        
        // 关键修复：跨天时重置删除标志（新的一天允许计算最高温）
        disable_today_max_calc = 0;  // 新增这行
        
        // 更新日期记录
        current_system_date[0] = curr_time.year;
        current_system_date[1] = curr_time.mon;
        current_system_date[2] = curr_time.day;
        
        // 调试信息
        UART4_SendString("New day detected, reset max temp calculation.\r\n");
    }
    
    // 关键修改：如果当天最高温已被删除，则不重新计算
    if (disable_today_max_calc && !is_new_day) {
        return;  // 当天最高温已被删除，不再重新计算
    }
    
    // 只有当允许计算时才进行最高温查找
    if (!disable_today_max_calc) {
        // 读取实时数据
        for (slave_idx = 0; slave_idx < TOTAL_SLAVES; slave_idx++) {
            realtime_rec = &data_summary[slave_idx];
            if (realtime_rec->is_valid && realtime_rec->temp > 0) {
                current_temp = realtime_rec->temp * 10;
                
                if (daily_max_temps[0].is_valid == 0) {
                    // 初始化当天记录
                    daily_max_temps[0].is_valid = 1;
                    daily_max_temps[0].max_temp = current_temp;
                    rtc_read(&daily_max_temps[0].temp_time);
                    daily_max_temps[0].pid = realtime_rec->pid;
                    daily_max_temps[0].aid = realtime_rec->aid;
                    daily_max_temps[0].volt_mv = realtime_rec->volt1 * 100;
                } else if (current_temp > daily_max_temps[0].max_temp) {
                    // 更新更高的温度
                    daily_max_temps[0].max_temp = current_temp;
                    rtc_read(&daily_max_temps[0].temp_time);
                    daily_max_temps[0].pid = realtime_rec->pid;
                    daily_max_temps[0].aid = realtime_rec->aid;
                    daily_max_temps[0].volt_mv = realtime_rec->volt1 * 100;
                }
            }
        }
        
        // 读取历史数据
        for (slave_idx = 0; slave_idx < TOTAL_SLAVES; slave_idx++) {
            hist_start = HISTORY_DATA_START + (slave_idx * RECORDS_PER_SLAVE);
            for (hist_idx = 0; hist_idx < RECORDS_PER_SLAVE; hist_idx++) {
                hist_pos = hist_start + hist_idx;
                hist_rec = &data_summary[hist_pos];
                
                if (hist_rec->is_valid && hist_rec->temp > 0) {
                    current_temp = hist_rec->temp * 10;
                    
                    if (daily_max_temps[0].is_valid == 0) {
                        daily_max_temps[0].is_valid = 1;
                        daily_max_temps[0].max_temp = current_temp;
                        rtc_read(&daily_max_temps[0].temp_time);
                        daily_max_temps[0].pid = hist_rec->pid;
                        daily_max_temps[0].aid = hist_rec->aid;
                        daily_max_temps[0].volt_mv = hist_rec->volt1 * 100;
                    } else if (current_temp > daily_max_temps[0].max_temp) {
                        daily_max_temps[0].max_temp = current_temp;
                        rtc_read(&daily_max_temps[0].temp_time);
                        daily_max_temps[0].pid = hist_rec->pid;
                        daily_max_temps[0].aid = hist_rec->aid;
                        daily_max_temps[0].volt_mv = hist_rec->volt1 * 100;
                    }
                }
            }
        }
    }
}

// ------------------- 删除指定最高温事件 -------------------
void DeleteMaxTempEvent(unsigned char display_index) {
    if (display_index >= 3) {
        return;
    }
    
    // 1. 彻底清空目标索引的最高温数据（标记为无效）
    daily_max_temps[display_index].is_valid = 0;
    daily_max_temps[display_index].max_temp = -990;
    daily_max_temps[display_index].pid = 0;
    daily_max_temps[display_index].aid = 0;
    daily_max_temps[display_index].volt_mv = 0;
    memset(&daily_max_temps[display_index].temp_time, 0, sizeof(rtc_time_t));
    
    // 2. 同步清空全局缓存，确保页面实时显示空值
    max_temps[display_index] = -990;
    memset(&max_temp_times[display_index], 0, sizeof(rtc_time_t));
    
    // 3. 移除禁止重算逻辑，删除当天数据后仍允许重新计算
    if (display_index == 0) {
        UART4_SendString("Today's max temp deleted, allow recalculation.\r\n");
    } else {
        UART4_SendString("History max temp deleted.\r\n");
    }
    
    // ========== 新增核心逻辑 ==========
    CheckDailyMaxTemp();  // 主动重算最高温，更新下一个最高值
    display_labels_initialized = 0;  // 强制页面重新绘制
    RefreshDisplay();     // 触发LCD刷新，显示新数据
    // ==================================
}
// 清空所有最高温事件（用于Page23清除Page21/22数据）
// 修改后的ClearAllMaxTempEvents函数
void ClearAllMaxTempEvents(void) {
    unsigned char i;
    
    // 1. 禁用最高温重新计算（避免删除过程中被覆盖）
    disable_max_temp_calc = 1;
    
    // 2. 彻底重置3个最高温记录
    for (i = 0; i < 3; i++) { 
        daily_max_temps[i].is_valid = 0;
        daily_max_temps[i].max_temp = -990;    
        daily_max_temps[i].pid = 0;            
        daily_max_temps[i].aid = 0;            
        daily_max_temps[i].volt_mv = 0;        
        memset(&daily_max_temps[i].temp_time, 0, sizeof(rtc_time_t));
    }
    
    // 3. 重置计数和索引（关键修复）
    max_temp_event_count = 0;
    max_temp_next_index = 0;
    
    // 4. 释放禁用标记（关键修复：允许后续重新计算最高温）
    disable_max_temp_calc = 0;
    
    UART4_SendString("All max temp events cleared.\r\n");
}
