#include "uart4.h"
#include <STC32G.H>
#include "lcd.h"
#include "config.h"
#include "relay.h"
#include "led.h"
#include "D1302.h"
// 定义NULL（STC32G头文件可能没有定义）
#ifndef NULL
#define NULL ((void*)0)
#endif

// ------------------- 全局变量定义 -------------------
unsigned char uart_rx_buff[UART_BUFF_SIZE] = {0};  // 接收缓冲区
unsigned char uart_rx_len = 0;                     // 接收长度
bit uart_rx_complete = 0;                          // 接收完成标志
unsigned long delay_count = 0;                     // 延时计数器
rtc_time_t current_rtc_time;
unsigned long rtc_refresh_counter = 0;
bit need_rtc_refresh = 1;
// 兼容旧代码的历史数据
DataPoint history[HISTORY_SIZE];

// 核心分区存储系统
DataRecord data_summary[TOTAL_RECORDS];            // 分区存储的总数据数组
unsigned char slave_history_index[TOTAL_SLAVES];   // 从站历史索引

// 统计信息
Statistics current_stats;

// 预警记录数组
WarningRecord warning_records[WARNING_RECORD_SIZE];
// 恢复记录数组
RecoveryRecord recovery_records[RECOVERY_RECORD_SIZE];

// ------------------- 菜单状态 -------------------
// 菜单状态变量
MenuState menu_state;

// 显示相关
extern PageType current_page;  // 当前页面
bit display_labels_initialized = 0;  // 固定标签是否已显示

// ------------------- 静态函数声明 -------------------
static void UART4_ClearBuffer(unsigned char *ptr, unsigned int len);
static void UART4_ParseData(void);
static void DisplayFixedLabels(void);
static void ClearDataArea(void);
static void ClearSummaryArea(void);
static void DisplayPage2(void);
static void DisplayDetailPage(PageType page);
static void HandlePrevItem(void);
static void HandleNextItem(void);
static void HandleEnterKey(void);
static void HandleReturnKey(void);
static void RefreshDisplay(void);
static void DisplayPage7(void);
static void DisplayPage10(void);
static void DisplayPage11(void);
static void DisplayPage12(void);
static void DisplayPage13(void);
static void DisplayPage14(void);
static void DisplayPage15(void);
static void DisplayPage16(void);
static void DisplayPage17(void);
static void DisplayPage18(void);
static void DisplayPage19(void);
static void DisplayPage20(void);
static void DisplayPage21(void);
static void DisplayPage22(void);
static void DisplayPage23(void);
static void DisplayPage8(void);
static void DisplayPage24(void);
static void DisplayPage25(void);
static void DisplayPage26(void);
static void InitSlaveHistoryIndices(void);
static void ClearAllHistoryData(void);
static void ClearMaxTemperatureData(void);  // 新增：清除最高温度数据

static void DisplayRTCOnPage24(void);
// ------------------- 菜单初始化 -------------------
void Menu_Init(void) {
    menu_state.current_page = PAGE_1;
    menu_state.prev_page = PAGE_1; 
    menu_state.page2_selected = 0;
    menu_state.page7_selected = 0;
    menu_state.page10_selected = 0; 
    menu_state.page11_selected = 0; 
    menu_state.page12_selected = 0;
    menu_state.page13_selected = 0; 
    menu_state.page14_selected = 0;
    menu_state.page15_selected = 0; 
    menu_state.page16_selected = 0;
    menu_state.page17_selected = 0;
	    menu_state.page18_selected = 0;  // 新增
    menu_state.page19_selected = 0;  // 新增
    menu_state.page20_selected = 0;  // 新增
	    menu_state.page21_selected = 0;  // 新增
    menu_state.page22_selected = 0;  // 新增
    menu_state.page23_selected = 0;  // 新增
	 menu_state.page8_selected = 0;  // 新增
	    menu_state.page24_selected = 0;  // 新增
    menu_state.page25_selected = 0;  // 新增
    menu_state.page26_selected = 0;  // 新增
    menu_state.page_changed = 1;
    menu_state.menu_initialized = 0;
}
// ------------------- 初始化预警记录 -------------------
void InitWarningRecords(void) {
    unsigned char i;
    for (i = 0; i < WARNING_RECORD_SIZE; i++) {
        warning_records[i].id = 0;
        warning_records[i].sensor_id = 0;
        warning_records[i].temperature = 0;
        warning_records[i].time_hour = 0;
        warning_records[i].time_minute = 0;
        warning_records[i].is_valid = 0;
    }
}

// ------------------- 初始化恢复记录 -------------------
void InitRecoveryRecords(void) {
    unsigned char i;
    for (i = 0; i < RECOVERY_RECORD_SIZE; i++) {
        recovery_records[i].id = 0;
        recovery_records[i].sensor_id = 0;
        recovery_records[i].time_hour = 0;
        recovery_records[i].time_minute = 0;
        recovery_records[i].status = 0;
        recovery_records[i].is_valid = 0;
    }
}

// ------------------- 初始化从站历史索引 -------------------
static void InitSlaveHistoryIndices(void) {
    unsigned char i;
    for (i = 0; i < TOTAL_SLAVES; i++) {
        slave_history_index[i] = 0;
    }
}

// ------------------- 初始化统计信息 -------------------
void InitStatistics(void) {
    unsigned int i;
    
    // 初始化历史数据
    for (i = 0; i < HISTORY_SIZE; i++) {
        history[i].pid = 0;
        history[i].aid = 0;
        history[i].temp = 0;
        history[i].volt1 = 0;
        history[i].volt2 = 0;
        history[i].fosc = 0;
        history[i].voltage = 0;
    }
    
    // 初始化核心分区存储
    for (i = 0; i < TOTAL_RECORDS; i++) {
        data_summary[i].pid = 0;
        data_summary[i].aid = 0;
        data_summary[i].temp = 0;
        data_summary[i].volt1 = 0;
        data_summary[i].volt2 = 0;
        data_summary[i].fosc = 0;
        data_summary[i].timestamp = 0;
        data_summary[i].is_valid = 0;
        data_summary[i].reserved[0] = 0;
        data_summary[i].reserved[1] = 0;
        data_summary[i].reserved[2] = 0;
    }
    
    InitSlaveHistoryIndices();
    
    // 初始化预警和恢复记录
    InitWarningRecords();
    InitRecoveryRecords();
    
    // 初始化统计信息
    current_stats.total_slaves = 0;
    current_stats.total_data_count = 0;
    current_stats.overall_avg_temp = 0;
    current_stats.overall_max_temp = 0;
    current_stats.overall_min_temp = 255;
}

// ------------------- UART4初始化 -------------------
void UART4_Init(unsigned long baudrate)
{
    unsigned int reload = (unsigned int)(65536UL - (FOSC / (4UL * baudrate)));
    
    // IO口配置
    EAXFR = 1;
    P_SW2 |= 0x80;
    S4_S = 1;  // UART4映射到P5.2(RX)/P5.3(TX)
    P_SW2 &= ~0x80;
    
    // 设置IO模式:P5.2(RX)高阻输入,P5.3(TX)推挽输出
    P5M1 |= (1 << 2);
    P5M0 &= ~(1 << 2);
    P5M1 &= ~(1 << 3);
    P5M0 |= (1 << 3);
    
    // 串口配置
    S4CON = 0x50;  // 8位数据,可变波特率,允许接收
    T4L = reload & 0xFF;
    T4H = reload >> 8;
    T4T3M |= 0x80;  // 启动定时器4
    T4T3M |= 0x20;  // 设置定时器4为UART4的波特率发生器
    T4T3M &= ~0x10; // 设置定时器4为1T模式
    
    // 中断配置(注意:需开启全局中断)
    IE2 |= 0x10;    // 开启UART4中断
    AUXR |= 0x80;   // STC32G专用:打开总中断开关
    
    // 初始化数据存储和菜单
    InitStatistics();
    Menu_Init();
    InitDataStorage();
		
		   rtc_init();
			     rtc_check_and_init();  // 检查并设置时间
}

// ------------------- 初始化数据存储 -------------------
void InitDataStorage(void)
{
    unsigned int i;
    
    // 初始化数据记录数组
    for (i = 0; i < TOTAL_RECORDS; i++) {
        data_summary[i].pid = 0;
        data_summary[i].aid = 0;
        data_summary[i].temp = 0;
        data_summary[i].volt1 = 0;
        data_summary[i].volt2 = 0;
        data_summary[i].fosc = 0;
        data_summary[i].timestamp = 0;
        data_summary[i].is_valid = 0;
    }
    
    // 初始化从站索引数组
    for (i = 0; i < TOTAL_SLAVES; i++) {
        slave_history_index[i] = 0;
    }
}

// ------------------- 发送函数 -------------------
void UART4_SendByte(unsigned char dat)
{
    S4BUF = dat;
    while (!(S4CON & 0x02));  // 等待发送完成
    S4CON &= ~0x02;           // 清除发送完成标志
}

void UART4_SendString(unsigned char *str)
{
    while (*str != '\0')
    {
        UART4_SendByte(*str);
        str++;
    }
}

void UART4_SendNumber(unsigned long num, unsigned char digits)
{
    unsigned char i;
    unsigned long divisor = 1;
    
    for (i = 1; i < digits; i++) divisor *= 10;
    for (i = 0; i < digits; i++)
    {
        UART4_SendByte((unsigned char)((num / divisor) % 10 + '0'));
        divisor /= 10;
    }
}

// ------------------- 定时器0初始化(用于延时) -------------------
void Timer0_Init(void)
{
    TMOD &= 0xF0;  // 清除定时器0设置
    TMOD |= 0x01;  // 定时器0模式1(16位定时器)
    TL0 = 0x30;    // 24MHz 1T模式1ms初值
    TH0 = 0xF8;
    TF0 = 0;       // 清除溢出标志
    TR0 = 1;       // 启动定时器0
    ET0 = 1;       // 开启定时器0中断
    EA = 1;        // 开启总中断
}

// 定时器0中断服务函数(用于延时)
void Timer0_ISR(void) interrupt 1
{
    TL0 = 0x30;
    TH0 = 0xF8;
    if (delay_count > 0) delay_count--;
}

// ------------------- 清空缓冲区 -------------------
static void UART4_ClearBuffer(unsigned char *ptr, unsigned int len)
{
    while(len--) *ptr++ = 0;
}

// ------------------- 清空数据区域(根据页面) -------------------
static void ClearDataArea(void)
{
    switch (menu_state.current_page) {
        case PAGE_1:
            break;
        case PAGE_2:
            break;
        case PAGE_3:
            ClearSummaryArea();
            break;
    }
}

// ------------------- 清空统计区域 -------------------
static void ClearSummaryArea(void)
{
    // 清空统计区域
}
static void GetCurrentRTC(void) {
    rtc_read(&current_rtc_time);
}

static void DisplayRTCOnPage1(void) {
    unsigned char temp;
    
    // 第二行（page=2）显示日期：YYYY-MM-DD
    // 显示年份（20XX）
    LCD_DisplayChar(2, 0, '2');
    LCD_DisplayChar(2, 8, '0');
    
    temp = (current_rtc_time.year / 10);
    LCD_DisplayChar(2, 16, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.year % 10);
    LCD_DisplayChar(2, 24, (temp > 9 ? 9 : temp) + '0');
    
    // 显示分隔符
    LCD_DisplayChar(2, 32, '-');
    
    // 显示月份
    temp = (current_rtc_time.mon / 10);
    LCD_DisplayChar(2, 40, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.mon % 10);
    LCD_DisplayChar(2, 48, (temp > 9 ? 9 : temp) + '0');
    
    // 显示分隔符
    LCD_DisplayChar(2, 56, '-');
    
    // 显示日期
    temp = (current_rtc_time.day / 10);
    LCD_DisplayChar(2, 64, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.day % 10);
    LCD_DisplayChar(2, 72, (temp > 9 ? 9 : temp) + '0');
    
    // 第三行（page=4）显示时间：HH:MM:SS
    // 显示小时
    temp = (current_rtc_time.hour / 10);
    LCD_DisplayChar(4, 0, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.hour % 10);
    LCD_DisplayChar(4, 8, (temp > 9 ? 9 : temp) + '0');
    
    // 显示冒号
    LCD_DisplayChar(4, 16, ':');
    
    // 显示分钟
    temp = (current_rtc_time.min / 10);
    LCD_DisplayChar(4, 24, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.min % 10);
    LCD_DisplayChar(4, 32, (temp > 9 ? 9 : temp) + '0');
    
    // 显示冒号
    LCD_DisplayChar(4, 40, ':');
    
    // 显示秒
    temp = (current_rtc_time.sec / 10);
    LCD_DisplayChar(4, 48, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.sec % 10);
    LCD_DisplayChar(4, 56, (temp > 9 ? 9 : temp) + '0');
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
    if (need_rtc_refresh && menu_state.current_page == PAGE_1) {
        GetCurrentRTC();
        DisplayRTCOnPage1();
        need_rtc_refresh = 0;
    }
		else if (need_rtc_refresh && menu_state.current_page == PAGE_24) {
        GetCurrentRTC();
        DisplayRTCOnPage24();
        need_rtc_refresh = 0;
    }
}
// ------------------- 显示固定标签 -------------------
static void DisplayFixedLabels(void)
{
    if (display_labels_initialized == 0)
    {
        LCD_Clear();  // 清屏
        
        switch (menu_state.current_page) {
            case PAGE_1:
                // 页面1: 数据监测(主界面)
                // 第1行(行0):从站
                LCD_DISPLAYCHAR_NEW(0, 0, 0, 0);   // "从"
                LCD_DISPLAYCHAR_NEW(0, 8, 1, 0);   // "站"
                
                // "温度(°C)" - 起始列48
                LCD_DISPLAYCHAR_NEW(0, 48, 0, 1);   // "温"
                LCD_DISPLAYCHAR_NEW(0, 56, 1, 1);   // "度"
                LCD_DISPLAYCHAR_NEW(0, 64, 0, 6);   // "("
                LCD_DISPLAYCHAR_NEW(0, 72, 0, 2);   // "°"
                LCD_DISPLAYCHAR_NEW(0, 80, 0, 7);   // "C"
                
                // "电压(mV)" - 起始列88
                LCD_DISPLAYCHAR_NEW(0, 88, 0, 3);   // "电"
                LCD_DISPLAYCHAR_NEW(0, 96, 1, 3);   // "压"
                LCD_DISPLAYCHAR_NEW(0, 104, 0, 6);  // "("
                LCD_DisplayChar(0, 112, 'V');      // "V"
                LCD_DISPLAYCHAR_NEW(0, 120, 0, 7);  // ")"
                
                // 第7行(行6):功能
                // "下一页" - 列0
                LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
                LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
                LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
                
                // "上一页" - 列48
                LCD_DISPLAYCHAR_NEW(6, 48, 3, 4);  // "上"
                LCD_DISPLAYCHAR_NEW(6, 56, 1, 4);  // "一"
                LCD_DISPLAYCHAR_NEW(6, 64, 2, 4);  // "页"
                
                // "返回" - 列112
                LCD_DISPLAYCHAR_NEW(6, 104, 0, 5);  // "返"
                LCD_DISPLAYCHAR_NEW(6, 112, 1, 5);  // "回"
								
								 LCD_ClearPages(2, 5);
                
                // 初始显示RTC时间
                GetCurrentRTC();
                DisplayRTCOnPage1();
                break;
               
                
            case PAGE_2:
                DisplayPage2();  // 显示PAGE_2内容
                break;
                
            case PAGE_3:
            case PAGE_4:
            case PAGE_5:
            case PAGE_7:

            case PAGE_9:

            case PAGE_10:
                DisplayDetailPage(menu_state.current_page);  // 显示详情页
                break;
						case PAGE_8:
							DisplayPage8(); // 显示page11内容
						break;
            case PAGE_11:
                DisplayPage11(); // 显示page11内容
                break;
            case PAGE_12:
                DisplayPage12();  // 显示PAGE_12内容
                break;
            case PAGE_13:
                DisplayPage13();  // 显示PAGE_13内容
                break;
            case PAGE_14:
                DisplayPage14();  // 显示PAGE_14内容
                break;
            case PAGE_15:
                DisplayPage15();  // 显示PAGE_15内容
                break;
            case PAGE_16:
                DisplayPage16();  // 显示PAGE_16内容
                break;
            case PAGE_17:
                DisplayPage17();  // 新增：显示PAGE_17内容
                break;
						case PAGE_18:  // 新增：历史数据查询页面
                DisplayPage18();
                break;
            case PAGE_19:  // 新增：清除历史数据确认页面
                DisplayPage19();
                break;
            case PAGE_20:  // 新增：历史数据删除确认页面
                DisplayPage20();
                break;
						case PAGE_21:  // 新增：最高温度查询页面
                DisplayPage21();
                break;
            case PAGE_22:  // 新增：最高温度清除确认页面
                DisplayPage22();
                break;
            case PAGE_23:  // 新增：最高温度删除确认页面
                DisplayPage23();
                break;
						case PAGE_24:  // 新增：最高温度查询页面
                DisplayPage24();
						
//							 LCD_ClearPages(2, 5);
                
                // 初始显示RTC时间
                GetCurrentRTC();
                DisplayRTCOnPage24();
               
                break;
            case PAGE_25:  // 新增：最高温度清除确认页面
                DisplayPage25();
                break;
            case PAGE_26:  // 新增：最高温度删除确认页面
                DisplayPage26();
                break;
        }
        display_labels_initialized = 1;
    }
}

// ------------------- 显示PAGE_2(功能菜单) -------------------
static void DisplayPage2(void)
{
    unsigned char start_index = 0;  // 起始显示索引
    unsigned char i;
    
    // 滚动显示逻辑，最多显示3项
    if (menu_state.page2_selected >= 3) {
        start_index = menu_state.page2_selected - 2;
        if (start_index > (PAGE2_ITEM_COUNT - 3)) {
            start_index = PAGE2_ITEM_COUNT - 3;
        }
    }
    
    // 清空显示区域
    for (i = 0; i < 3; i++) {
        unsigned char row = i * 2;
        LCD_DisplayChar(row, 0, ' ');
        LCD_DisplayChar(row, 8, ' ');
        LCD_DisplayChar(row, 16, ' ');
        LCD_DisplayChar(row, 24, ' ');
        LCD_DisplayChar(row, 32, ' ');
        LCD_DisplayChar(row, 40, ' ');
    }
    
    // 显示3个菜单项
    for (i = 0; i < 3 && (start_index + i) < PAGE2_ITEM_COUNT; i++) {
        unsigned char item_index = start_index + i;
        unsigned char row = i * 2;
        
        // 选择箭头
        if (item_index == menu_state.page2_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25); 
        } else {
            LCD_DisplayChar(row, 0, ' ');
        }
        
        // 显示菜单项文本
        switch(item_index) {
            case MENU_MEASURE_DATA:
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 8);   // "测"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 8);  // "量"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 8);  // "数"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 8);  // "据"
                break;
                
            case MENU_SENSOR_STATUS:
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 9);   // "传"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 9);  // "感"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 9);  // "器"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 9);  // "状"
                LCD_DISPLAYCHAR_NEW(row, 40, 4, 9);  // "态"
                break;
                
            case MENU_PARAM_QUERY:
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 10);   // "参"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 10);  // "数"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 10);  // "查"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 10);  // "询"
                break;
                
            case MENU_HISTORY_RECORD:
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 13);   // "历"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 13);  // "史"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 13);  // "记"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 13);  // "录"
                break;
                
            case MENU_DEVICE_MAINT:
                LCD_DISPLAYCHAR_NEW(row, 8, 0, 14);   // "设"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 14);  // "备"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 14);  // "维"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 14);  // "护"
                break;
        }
    }
    
    // 第6行:功能提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "上"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 80, 0, 12);  // "进"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 12);  // "入"
    
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}

// ------------------- 修正DisplayPage7(显示正确的菜单名称) -------------------
static void DisplayPage7(void)
{
    unsigned char start_index = 0;  // ??????
    unsigned char i;
    unsigned char col;  // ?????col??
    
    // ??????3?,??????
    // LCD????3?(?0?2?4?),?????
    if (menu_state.page7_selected >= 3) {
        start_index = menu_state.page7_selected - 2;  // ?????????
        if (start_index > (PAGE7_ITEM_COUNT - 3)) {
            start_index = PAGE7_ITEM_COUNT - 3;
        }
    }
    
  for (i = 0; i < 3; i++) {
        unsigned char row = i * 2;
        // ???0?(????)
        LCD_DisplayChar(row, 0, ' ');
        // ???8-48??????
        for (col = 8; col < 56; col += 8) {
            LCD_DisplayChar(row, col, ' ');
        }
    }
    
    // ??3???
    for (i = 0; i < 3 && (start_index + i) < PAGE7_ITEM_COUNT; i++) {
        unsigned char item_index = start_index + i;
        unsigned char row = i * 2;
        
        // ?????(?????????)
        if (item_index == menu_state.page7_selected) {
           LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // ??????
        } else {
            LCD_DisplayChar(row, 0, ' ');  // ???????
        }
        // ?????????????
        switch(item_index) {
            case MENU_SENSOR_EVENT_1://温度报警事件记录
                // ??"?????1"
						    LCD_DISPLAYCHAR_NEW(row, 8, 0, 1);   // "?"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 1);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 24, 0, 19);   // "?"
                LCD_DISPLAYCHAR_NEW(row, 32, 1, 19);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 40, 2, 19);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 48, 3, 19);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 56, 4, 19);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 64, 5, 19);  // "1"
                break;
                
            case MENU_SENSOR_EVENT_2://温度预警时间记录
                // ??"?????2"
                               LCD_DISPLAYCHAR_NEW(row, 8, 0, 1);   // "?"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 1);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 24, 0, 20);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 32, 1, 20);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 40, 2, 20);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 48, 3, 20);  // "2"
						    LCD_DISPLAYCHAR_NEW(row, 56, 4, 20);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 64, 5, 20);  // "2"
                break;
                
            case MENU_SENSOR_EVENT_3://传感器恢复事件记录
                // ??"?????3"
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
                // ??"??????"
                             LCD_DISPLAYCHAR_NEW(row, 8, 0, 23);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 23);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 24, 2, 23);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 32, 3, 23);  // "1"
						    LCD_DISPLAYCHAR_NEW(row, 40, 4, 23);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 48, 5, 23);  // "1"

                break;
                
            case MENU_MAX_TEMP_QUERY://最高历史温度查询
                // ??"??????"
                   LCD_DISPLAYCHAR_NEW(row, 8, 0, 24);   // "?"
                LCD_DISPLAYCHAR_NEW(row, 16, 1, 24);  // "?"
						    LCD_DISPLAYCHAR_NEW(row, 24, 0, 23);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 32, 1, 23);  // "?"
						    LCD_DISPLAYCHAR_NEW(row, 40, 0, 1);   // "?"
                LCD_DISPLAYCHAR_NEW(row, 48, 1, 1);  // "?"
						    LCD_DISPLAYCHAR_NEW(row, 56, 4, 23);  // "?"
                LCD_DISPLAYCHAR_NEW(row, 64, 5, 23);  // "1"

                break;
        }
    }
    
    // ?6?:??????
    // "??"
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "?"
    
    // ???
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "?"
    
    // ??
    LCD_DISPLAYCHAR_NEW(6, 80, 0, 22);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 22);  // "?"
    
    // ??
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "?"

}

static void DisplayPage8(void)
{
    unsigned char start_index = 0;
    unsigned char i;
    
    // 滚动显示逻辑，最多显示3项
    if (menu_state.page8_selected >= 3) {
        start_index = menu_state.page8_selected - 2;
        if (start_index > 0) {
            start_index = 0;  // PAGE_8只有3项，所以start_index最大为0
        }
    }
    
    // 清空显示区域
    for (i = 0; i < 3; i++) {
        unsigned char row = i * 2;
        LCD_DisplayChar(row, 0, ' ');
        LCD_DisplayChar(row, 8, ' ');
        LCD_DisplayChar(row, 16, ' ');
        LCD_DisplayChar(row, 24, ' ');
        LCD_DisplayChar(row, 32, ' ');
    }
    
    // 显示3个菜单项
    for (i = 0; i < 3; i++) {
        unsigned char item_index = start_index + i;
        unsigned char row = i * 2;
        
        // 选择箭头
        if (item_index == menu_state.page8_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25); 
        } else {
            LCD_DisplayChar(row, 0, ' ');
        }
        
        // 显示菜单项文本
        switch(item_index) {
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
    
    // 第6行:功能提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "上"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 80, 0, 22);  // "进"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 22);  // "入"
    
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}
// ------------------- 显示PAGE_10(温度报警事件列表) -------------------
static void DisplayPage10(void)
{
    unsigned char i;
    unsigned char row;
    
    // ??
    LCD_Clear();
    
    // ??3?????
    for (i = 0; i < 3; i++) {
        row = i * 2;  // ?0?2?4?
        
        // ????(???PAGE_2?????)
        if (i == menu_state.page10_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // ?PAGE_2?????
        } else {
            LCD_DisplayChar(row, 0, ' ');
        }
        
        // ??"??"???
        LCD_DISPLAYCHAR_NEW(row, 8, 2, 19);   // "?"
        LCD_DISPLAYCHAR_NEW(row, 16, 3, 19);  // "?"
        LCD_DisplayChar(row, 24, '1' + i);    // ??
    }
    
    // ?6?:????
        LCD_DISPLAYCHAR_NEW(0, 40, 2, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(0, 48, 3, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(0, 56, 0, 15);  // "?"
        LCD_DISPLAYCHAR_NEW(2, 40, 2, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(2, 48, 3, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(2, 56, 0, 15);  // "?"
        LCD_DISPLAYCHAR_NEW(4, 40, 2, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(4, 48, 3, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(4, 56, 0, 15);  // "?"
        
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 26);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 26);  // "?"
        
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "?"
}

// ------------------- 显示PAGE_11(报警事件详细数据) -------------------
static void DisplayPage11(void) {
    LCD_Clear();
    
    // 第一行: ID: PID=    AID=
    LCD_DisplayString(0, 0, "ID:");
    LCD_DisplayString(0, 24, "PID=");
    LCD_DisplayString(0, 80, "AID=");
    
    // 第二行: 温度:
    LCD_DISPLAYCHAR_NEW(2, 0, 0, 1);  // "温"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 1);  // "度"
    LCD_DisplayChar(2, 16, ':');
    
    // 第三行: 电压:
    LCD_DISPLAYCHAR_NEW(4, 0, 0, 3);  // "电"
    LCD_DISPLAYCHAR_NEW(4, 8, 1, 3);  // "压"
    LCD_DisplayChar(4, 16, ':');
    
    // 第四行: 功能提示
    // 按1删除
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 27);
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 27);
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 27);
    LCD_DISPLAYCHAR_NEW(6, 24, 3, 27);
    
    // 按2返回
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
    
    // 显示当前选中的记录数据
    if (menu_state.page11_selected < HISTORY_SIZE) {
        DataPoint *curr = &history[menu_state.page11_selected];
        
        // 显示PID
        if (curr->pid > 0) {
            LCD_DisplayNumber(0, 48, (unsigned long)curr->pid, 3);
        }
        
        // 显示AID
        if (curr->aid > 0) {
            LCD_DisplayNumber(0, 104, (unsigned long)curr->aid, 3);
        }
        
        // 显示温度
        if (curr->temp > 0) {
            LCD_DisplayNumber(2, 24, (unsigned long)curr->temp, 3);
            LCD_DISPLAYCHAR_NEW(2, 40, 0, 2);  // "°"
            LCD_DisplayChar(2, 48, 'C');
        }
        
        // 显示电压
        if (curr->voltage > 0) {
            LCD_DisplayNumber(4, 24, (unsigned long)curr->voltage, 3);
            LCD_DisplayString(4, 40, "mV");
        } else if (curr->volt1 > 0) {
            LCD_DisplayNumber(4, 24, (unsigned long)curr->volt1, 3);
            LCD_DisplayString(4, 40, "mV");
        }
    }
}

// ------------------- 显示PAGE_12(删除确认) -------------------
static void DisplayPage12(void) {
    LCD_Clear();
    
    // 删除确认提示
    LCD_DISPLAYCHAR_NEW(2, 0, 0, 27);  // "按"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 27);  // "1"
    LCD_DISPLAYCHAR_NEW(2, 16, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 24, 3, 27);  // "除"
    
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
    
    // ??
    LCD_Clear();
    
    // ??3?????
    for (i = 0; i < 3; i++) {
        row = i * 2;  // ?0?2?4?
        
        // ????(???PAGE_2?????)
        if (i == menu_state.page13_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // ?PAGE_2?????
        } else {
            LCD_DisplayChar(row, 0, ' ');
        }
        
        // ??"??"???
        LCD_DISPLAYCHAR_NEW(row, 8, 2, 19);   // "?"
        LCD_DISPLAYCHAR_NEW(row, 16, 3, 19);  // "?"
        LCD_DisplayChar(row, 24, '1' + i);    // ??
    }
    
    // ?6?:????
        LCD_DISPLAYCHAR_NEW(0, 40, 2, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(0, 48, 3, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(0, 56, 0, 15);  // "?"
        LCD_DISPLAYCHAR_NEW(2, 40, 2, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(2, 48, 3, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(2, 56, 0, 15);  // "?"
        LCD_DISPLAYCHAR_NEW(4, 40, 2, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(4, 48, 3, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(4, 56, 0, 15);  // "?"
        
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 26);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 26);  // "?"
        
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "?"
}


// ------------------- 显示PAGE_14(传感器恢复事件列表) -------------------
static void DisplayPage14(void) {
       unsigned char i;
    unsigned char row;
    
    // ??
    LCD_Clear();
    
    // ??3?????
    for (i = 0; i < 3; i++) {
        row = i * 2;  // ?0?2?4?
        
        // ????(???PAGE_2?????)
        if (i == menu_state.page14_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // ?PAGE_2?????
        } else {
            LCD_DisplayChar(row, 0, ' ');
        }
        
        // ??"??"???
        LCD_DISPLAYCHAR_NEW(row, 8, 2, 19);   // "?"
        LCD_DISPLAYCHAR_NEW(row, 16, 3, 19);  // "?"
        LCD_DisplayChar(row, 24, '1' + i);    // ??
    }
    
    // ?6?:????
        LCD_DISPLAYCHAR_NEW(0, 40, 2, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(0, 48, 3, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(0, 56, 0, 15);  // "?"
        LCD_DISPLAYCHAR_NEW(2, 40, 2, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(2, 48, 3, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(2, 56, 0, 15);  // "?"
        LCD_DISPLAYCHAR_NEW(4, 40, 2, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(4, 48, 3, 20);   // "?"
    LCD_DISPLAYCHAR_NEW(4, 56, 0, 15);  // "?"
        
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 26);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 26);  // "?"
        
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "?"
}

// ------------------- 显示PAGE_15(预警详细数据) -------------------
static void DisplayPage15(void) {
    LCD_Clear();
    
    // 第一行: ID: PID=    AID=
    LCD_DisplayString(0, 0, "ID:");
    LCD_DisplayString(0, 24, "PID=");
    LCD_DisplayString(0, 80, "AID=");
    
    // 第二行: 温度:
    LCD_DISPLAYCHAR_NEW(2, 0, 0, 1);  // "温"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 1);  // "度"
    LCD_DisplayChar(2, 16, ':');
    
    // 第三行: 电压:
    LCD_DISPLAYCHAR_NEW(4, 0, 0, 3);  // "电"
    LCD_DISPLAYCHAR_NEW(4, 8, 1, 3);  // "压"
    LCD_DisplayChar(4, 16, ':');
    
    // 第四行: 功能提示
    // 按1删除
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 27);
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 27);
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 27);
    LCD_DISPLAYCHAR_NEW(6, 24, 3, 27);
    
    // 按2返回
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
    
    // 显示当前选中的记录数据
    if (menu_state.page15_selected < HISTORY_SIZE) {
        DataPoint *curr = &history[menu_state.page15_selected];
        
        // 显示PID
        if (curr->pid > 0) {
            LCD_DisplayNumber(0, 48, (unsigned long)curr->pid, 3);
        }
        
        // 显示AID
        if (curr->aid > 0) {
            LCD_DisplayNumber(0, 104, (unsigned long)curr->aid, 3);
        }
        
        // 显示温度
        if (curr->temp > 0) {
            LCD_DisplayNumber(2, 24, (unsigned long)curr->temp, 3);
            LCD_DISPLAYCHAR_NEW(2, 40, 0, 2);  // "°"
            LCD_DisplayChar(2, 48, 'C');
        }
        
        // 显示电压
        if (curr->voltage > 0) {
            LCD_DisplayNumber(4, 24, (unsigned long)curr->voltage, 3);
            LCD_DisplayString(4, 40, "mV");
        } else if (curr->volt1 > 0) {
            LCD_DisplayNumber(4, 24, (unsigned long)curr->volt1, 3);
            LCD_DisplayString(4, 40, "mV");
        }
    }
}

// ------------------- 显示PAGE_16(预警删除确认) -------------------
static void DisplayPage16(void) {
    LCD_Clear();
    
    // 删除确认提示
    LCD_DISPLAYCHAR_NEW(2, 0, 0, 27);  // "按"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 27);  // "1"
    LCD_DISPLAYCHAR_NEW(2, 16, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 24, 3, 27);  // "除"
    
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
    
    // 删除确认提示
    LCD_DISPLAYCHAR_NEW(2, 0, 0, 27);  // "按"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 27);  // "1"
    LCD_DISPLAYCHAR_NEW(2, 16, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 24, 3, 27);  // "除"
    
    // 确认提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 28);  // "认"
    
    // 返回提示
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}
static void DisplayPage18(void) {
    LCD_Clear();
    
    // 第一行：标题
	  LCD_DisplayString(0, 0, "ID:");
    LCD_DisplayString(0, 24, "PID=");
    LCD_DisplayString(0, 80, "TX01");
	
		LCD_DisplayString(2, 0, "ID:");
    LCD_DisplayString(2, 24, "PID=");
    LCD_DisplayString(2, 80, "TX02");
	
		LCD_DisplayString(4, 0, "ID:");
    LCD_DisplayString(4, 24, "PID=");
    LCD_DisplayString(4, 80, "TX03");
	
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 26);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 26);  // "?"
        
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "?"
}

// ------------------- 显示PAGE_19(清除历史数据确认页面) -------------------
static void DisplayPage19(void) {
    LCD_Clear();
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "?"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "?"
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 29);  // 清
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 29);  // 除
        
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);   // "?"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "?"
   
}

// ------------------- 显示PAGE_20(历史数据删除确认页面) -------------------
static void DisplayPage20(void) {
    LCD_Clear();
    
    // 删除确认提示
    LCD_DISPLAYCHAR_NEW(2, 0, 0, 27);  // "按"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 27);  // "1"
    LCD_DISPLAYCHAR_NEW(2, 16, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 24, 3, 27);  // "除"
    
    // 确认提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 28);  // "认"
    
    // 返回提示
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}

// ------------------- 显示PAGE_21(最高温度查询页面) -------------------
static void DisplayPage21(void) {
    LCD_Clear();
    
    // 第一行：最高温度记录1
    if (menu_state.page21_selected == 0) {
        LCD_DISPLAYCHAR_NEW(0, 0, 0, 25);  // 选择箭头
    } else {
        LCD_DisplayChar(0, 0, ' ');
    }
    LCD_DisplayString(0, 8, "MAX1:");
    LCD_DisplayString(0, 48, "TEMP:");
    
    // 第二行：最高温度记录2
    if (menu_state.page21_selected == 1) {
        LCD_DISPLAYCHAR_NEW(2, 0, 0, 25);  // 选择箭头
    } else {
        LCD_DisplayChar(2, 0, ' ');
    }
    LCD_DisplayString(2, 8, "MAX2:");
    LCD_DisplayString(2, 48, "TEMP:");
    
    // 第三行：最高温度记录3
    if (menu_state.page21_selected == 2) {
        LCD_DISPLAYCHAR_NEW(4, 0, 0, 25);  // 选择箭头
    } else {
        LCD_DisplayChar(4, 0, ' ');
    }
    LCD_DisplayString(4, 8, "MAX3:");
    LCD_DisplayString(4, 48, "TEMP:");
    
    
    
    // 第四行：功能提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "上"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 26);  // "进"
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 26);  // "入"
    
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);   // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}

// ------------------- 显示PAGE_22(最高温度清除确认页面) -------------------
static void DisplayPage22(void) {
    LCD_Clear();
    
    // 显示选项
    if (menu_state.page22_selected == 0) {
        LCD_DISPLAYCHAR_NEW(2, 0, 0, 25);  // 选择箭头
    } else {
        LCD_DisplayChar(2, 0, ' ');
    }
    LCD_DISPLAYCHAR_NEW(2, 8, 0, 30);   // "清"
    LCD_DISPLAYCHAR_NEW(2, 16, 1, 30);  // "除"
    LCD_DISPLAYCHAR_NEW(2, 24, 0, 24);  // "最"
    LCD_DISPLAYCHAR_NEW(2, 32, 1, 24);  // "高"
    LCD_DISPLAYCHAR_NEW(2, 40, 0, 1);   // "温"
    LCD_DisplayChar(2, 48, '1');
    
    if (menu_state.page22_selected == 1) {
        LCD_DISPLAYCHAR_NEW(4, 0, 0, 25);  // 选择箭头
    } else {
        LCD_DisplayChar(4, 0, ' ');
    }
    LCD_DISPLAYCHAR_NEW(4, 8, 0, 30);   // "清"
    LCD_DISPLAYCHAR_NEW(4, 16, 1, 30);  // "除"
    LCD_DISPLAYCHAR_NEW(4, 24, 0, 24);  // "最"
    LCD_DISPLAYCHAR_NEW(4, 32, 1, 24);  // "高"
    LCD_DISPLAYCHAR_NEW(4, 40, 0, 1);   // "温"
    LCD_DisplayChar(4, 48, '2');
    
    // 第六行：功能提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "上"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 29);  // 清
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 29);  // 除
    
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);   // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}

// ------------------- 显示PAGE_23(最高温度删除确认页面) -------------------
static void DisplayPage23(void) {
           LCD_Clear();
    
    // 删除确认提示
    LCD_DISPLAYCHAR_NEW(2, 0, 0, 27);  // "按"
    LCD_DISPLAYCHAR_NEW(2, 8, 1, 27);  // "1"
    LCD_DISPLAYCHAR_NEW(2, 16, 2, 27);  // "删"
    LCD_DISPLAYCHAR_NEW(2, 24, 3, 27);  // "除"
    
    // 确认提示
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 28);  // "认"
    
    // 返回提示
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}

static void DisplayPage24(void)
{
    LCD_Clear();
    
    // 第一行: 标题
    LCD_DISPLAYCHAR_NEW(0, 0, 2, 12);   // "修"
    LCD_DISPLAYCHAR_NEW(0, 8, 3, 12);  // "改"

    LCD_DISPLAYCHAR_NEW(2, 0, 2, 20);  // "时"
    LCD_DISPLAYCHAR_NEW(2, 8, 3, 20);  // "间"
    
//    // 第二行: 年-月-日
//    LCD_DisplayString(0, 32, "2024-12-22");
//    
//    // 第三行: 时:分:秒
//    LCD_DisplayString(4, 32, "15:30:45");
    
    // 第四行: 上一项
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);  // "上"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 5, 4); // "位"
    
	  LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "下"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 5, 4); // "位"
		
		LCD_DISPLAYCHAR_NEW(6, 80, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 28);  // "认"
    // 返回
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
}


static void DisplayRTCOnPage24(void) {
    unsigned char temp;
    
    // 第2行显示日期：YYYY-MM-DD（与page1位置一致）
    LCD_DisplayChar(0, 32, '2');
    LCD_DisplayChar(0, 40, '0');
    temp = (current_rtc_time.year / 10);
    LCD_DisplayChar(0, 48, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.year % 10);
    LCD_DisplayChar(0, 56, (temp > 9 ? 9 : temp) + '0');
    LCD_DisplayChar(0, 64, '-');
    temp = (current_rtc_time.mon / 10);
    LCD_DisplayChar(0, 72, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.mon % 10);
    LCD_DisplayChar(0, 80, (temp > 9 ? 9 : temp) + '0');
    LCD_DisplayChar(0, 88, '-');
    temp = (current_rtc_time.day / 10);
    LCD_DisplayChar(0, 96, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.day % 10);
    LCD_DisplayChar(0, 104, (temp > 9 ? 9 : temp) + '0');
    
    // 第4行显示时间：HH:MM:SS
    temp = (current_rtc_time.hour / 10);
    LCD_DisplayChar(2, 32, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.hour % 10);
    LCD_DisplayChar(2, 40, (temp > 9 ? 9 : temp) + '0');
//    LCD_DisplayChar(2, 48, ':');
		LCD_DISPLAYCHAR_NEW(2, 48, 0, 15);
    temp = (current_rtc_time.min / 10);
    LCD_DisplayChar(2, 56, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.min % 10);
    LCD_DisplayChar(2, 64, (temp > 9 ? 9 : temp) + '0');
    LCD_DISPLAYCHAR_NEW(2, 72, 0, 15);
    temp = (current_rtc_time.sec / 10);
    LCD_DisplayChar(2, 80, (temp > 9 ? 9 : temp) + '0');
    temp = (current_rtc_time.sec % 10);
    LCD_DisplayChar(2, 88, (temp > 9 ? 9 : temp) + '0');
}
// ------------------- 显示PAGE_25(修改密码详情) -------------------
static void DisplayPage25(void)
{
    LCD_Clear();
    
    // 第一行: 标题

    LCD_DISPLAYCHAR_NEW(0, 0, 4, 12);  // "密"
    LCD_DISPLAYCHAR_NEW(0, 8, 5, 12);  // "码"
    
    
    
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);  // "上"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 5, 4); // "位"
    
	  LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "下"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 5, 4); // "位"
		
		LCD_DISPLAYCHAR_NEW(6, 80, 0, 28);  // "确"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 28);  // "认"
    // 返回
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
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


// ------------------- 清除最高温度数据 -------------------
static void ClearMaxTemperatureData(void) {
    // 这里需要你根据实际的数据结构来实现
    // 示例：清除最高温度记录
    
    // 可以根据menu_state.page22_selected判断要清除哪个选项
    
    // 发送清除成功的提示
    UART4_SendString("\r\nMax temperature data cleared!\r\n");
}
// ------------------- 显示详情页面 -------------------
static void DisplayDetailPage(PageType page)
{
    LCD_Clear();
    
    switch(page) {
        case PAGE_3:  // 数据汇总页面
            // 第0行:PID=1 TX0 温度:    电压:    mv
            LCD_DisplayString(0, 0, "PID");
            LCD_DISPLAYCHAR_NEW(0, 24, 0, 16); 
            LCD_DisplayString(0, 72, "TX");
            LCD_DISPLAYCHAR_NEW(0, 88, 0, 16);
            
            LCD_DISPLAYCHAR_NEW(2, 0, 0, 1);
            LCD_DISPLAYCHAR_NEW(2, 8, 1, 1);
            LCD_DISPLAYCHAR_NEW(2, 16, 0, 15);
            LCD_DISPLAYCHAR_NEW(2, 72, 0, 3);
            LCD_DISPLAYCHAR_NEW(2, 80, 1, 3);
            LCD_DISPLAYCHAR_NEW(2, 88, 0, 15);
            
            LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);
            LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);
            LCD_DISPLAYCHAR_NEW(6, 16, 4, 4);
            
            LCD_DISPLAYCHAR_NEW(6, 60, 3, 4);
            LCD_DISPLAYCHAR_NEW(6, 68, 1, 4);
            LCD_DISPLAYCHAR_NEW(6, 76, 4, 4);
            
            LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
            LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
            break;
            
        case PAGE_4:  // 传感器状态
            LCD_DisplayString(0, 0, "PID");
            LCD_DISPLAYCHAR_NEW(0, 24, 0, 16);
            
            LCD_DisplayString(2, 0, "PID");
            LCD_DISPLAYCHAR_NEW(2, 24, 0, 16);
            
            LCD_DisplayString(4, 0, "PID");
            LCD_DISPLAYCHAR_NEW(4, 24, 0, 16);
            
            LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);
            LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);
            LCD_DISPLAYCHAR_NEW(6, 16, 4, 4);
            
            LCD_DISPLAYCHAR_NEW(6, 60, 3, 4);
            LCD_DISPLAYCHAR_NEW(6, 68, 1, 4);
            LCD_DISPLAYCHAR_NEW(6, 76, 4, 4);
            
            LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
            LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
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
// ------------------- 清除所有历史数据 -------------------
static void ClearAllHistoryData(void) {
    unsigned char i;
    unsigned int j;
    
    // 1. 清空history数组
    for (i = 0; i < HISTORY_SIZE; i++) {
        history[i].pid = 0;
        history[i].aid = 0;
        history[i].temp = 0;
        history[i].volt1 = 0;
        history[i].volt2 = 0;
        history[i].fosc = 0;
        history[i].voltage = 0;
    }
    
    // 2. 清空data_summary数组
    for (j = 0; j < TOTAL_RECORDS; j++) {
        data_summary[j].pid = 0;
        data_summary[j].aid = 0;
        data_summary[j].temp = 0;
        data_summary[j].volt1 = 0;
        data_summary[j].volt2 = 0;
        data_summary[j].fosc = 0;
        data_summary[j].timestamp = 0;
        data_summary[j].is_valid = 0;
        data_summary[j].reserved[0] = 0;
        data_summary[j].reserved[1] = 0;
        data_summary[j].reserved[2] = 0;
    }
    
    // 3. 重置从站历史索引
    for (i = 0; i < TOTAL_SLAVES; i++) {
        slave_history_index[i] = 0;
    }
    
    // 4. 重置统计信息
    current_stats.total_slaves = 0;
    current_stats.total_data_count = 0;
    current_stats.overall_avg_temp = 0;
    current_stats.overall_max_temp = 0;
    current_stats.overall_min_temp = 255;
    
    // 5. 可选：发送清除成功的提示
    UART4_SendString("\r\nAll history data cleared!\r\n");
}
// ------------------- 解析数据 -------------------
static void UART4_ParseData(void)
{
    if (uart_rx_len >= 6)
    {
        unsigned char pid_val = uart_rx_buff[0];
        unsigned char aid_val = uart_rx_buff[1];
        unsigned char temp_val = uart_rx_buff[2];
        unsigned char volt_val = uart_rx_buff[3];
        unsigned char volt2_val = uart_rx_buff[4];
        unsigned char fosc_val = uart_rx_buff[5];
        
        // 添加到历史数据和分区存储
        AddDataToHistory(pid_val, aid_val, temp_val, volt_val, volt2_val, fosc_val);
        AddDataToSummary(pid_val, aid_val, temp_val, volt_val, volt2_val, fosc_val);
    }
}

// ------------------- 添加数据到历史记录(兼容旧代码) -------------------
void AddDataToHistory(unsigned char pid, unsigned char aid, unsigned char temp, 
                     unsigned char volt1, unsigned char volt2, unsigned char fosc)
{
    static unsigned char history_idx = 0;  // 局部变量避免冲突
    
    // 过滤全0数据(可能是无效数据)
    if (pid == 0 && aid == 0 && temp == 0 && volt1 == 0 && volt2 == 0 && fosc == 0) {
        return;
    }
    
    // 保存数据
    if (history_idx < HISTORY_SIZE) {
        history[history_idx].pid = pid;
        history[history_idx].aid = aid;
        history[history_idx].temp = temp;
        history[history_idx].volt1 = volt1;
        history[history_idx].volt2 = volt2;
        history[history_idx].fosc = fosc;
        history[history_idx].voltage = volt1;  // 使用volt1作为电压值
        
        history_idx++;
        if (history_idx >= HISTORY_SIZE) {
            history_idx = 0;  // 循环覆盖
        }
    }
}

// ------------------- 核心：添加数据到分区存储 -------------------
void AddDataToSummary(unsigned char pid, unsigned char aid, unsigned char temp, 
                     unsigned char volt1, unsigned char volt2, unsigned char fosc)
{
    unsigned char slave_id;
    unsigned int recent_index, history_index_val;
    
    // 验证从站ID有效性
    if (aid == 0 || aid > TOTAL_SLAVES) {
        return;  // 无效的从站ID
    }
    
    slave_id = aid - 1;  // 转换为0-based索引
    
    // 1. 更新最近数据区(RECENT_DATA_START + slave_id)
    recent_index = RECENT_DATA_START + slave_id;
    if (recent_index < TOTAL_RECORDS) {
        data_summary[recent_index].pid = pid;
        data_summary[recent_index].aid = aid;
        data_summary[recent_index].temp = temp;
        data_summary[recent_index].volt1 = volt1;
        data_summary[recent_index].volt2 = volt2;
        data_summary[recent_index].fosc = fosc;
        data_summary[recent_index].timestamp = delay_count;  // 使用延时计数器作为时间戳
        data_summary[recent_index].is_valid = 1;
    }
    
    // 2. 更新历史数据区
    history_index_val = HISTORY_DATA_START + (slave_id * RECORDS_PER_SLAVE) + slave_history_index[slave_id];
    if (history_index_val < TOTAL_RECORDS) {
        data_summary[history_index_val].pid = pid;
        data_summary[history_index_val].aid = aid;
        data_summary[history_index_val].temp = temp;
        data_summary[history_index_val].volt1 = volt1;
        data_summary[history_index_val].volt2 = volt2;
        data_summary[history_index_val].fosc = fosc;
        data_summary[history_index_val].timestamp = delay_count;
        data_summary[history_index_val].is_valid = 1;
        
        // 更新从站历史索引
        slave_history_index[slave_id] = (slave_history_index[slave_id] + 1) % RECORDS_PER_SLAVE;
    }
    
    // 3. 重新计算统计信息
    CalculateStatistics();
}

// ------------------- 获取指定从站的最近数据 -------------------
DataRecord* GetRecentDataByAID(unsigned char aid)
{
    unsigned char slave_id;
    
    if (aid == 0 || aid > TOTAL_SLAVES) {
        return (DataRecord*)0;  // 使用0代替NULL
    }
    
    slave_id = aid - 1;
    return &data_summary[RECENT_DATA_START + slave_id];
}

// ------------------- 计算统计信息 -------------------
void CalculateStatistics(void)
{
    unsigned int i;
    unsigned int sum_temp = 0;
    unsigned int valid_count = 0;
    
    // 重置统计信息
    current_stats.total_slaves = 0;
    current_stats.total_data_count = 0;
    current_stats.overall_avg_temp = 0;
    current_stats.overall_max_temp = 0;
    current_stats.overall_min_temp = 255;
    
    // 统计最近数据区的有效数据
    for (i = RECENT_DATA_START; i < RECENT_DATA_START + RECENT_DATA_SIZE; i++) {
        if (data_summary[i].is_valid && data_summary[i].temp > 0) {
            current_stats.total_slaves++;
            current_stats.total_data_count++;
            
            sum_temp += data_summary[i].temp;
            
            if (data_summary[i].temp > current_stats.overall_max_temp) {
                current_stats.overall_max_temp = data_summary[i].temp;
            }
            if (data_summary[i].temp < current_stats.overall_min_temp) {
                current_stats.overall_min_temp = data_summary[i].temp;
            }
            
            valid_count++;
        }
    }
    
    // 计算平均温度
    if (valid_count > 0) {
        current_stats.overall_avg_temp = (unsigned char)(sum_temp / valid_count);
    } else {
        current_stats.overall_min_temp = 0;
    }
}

// ------------------- 计算不同从站数量(兼容旧代码) -------------------
unsigned char CountDifferentSlaves(void)
{
    unsigned int i;
    unsigned int j;
    unsigned char slave_ids[16];
    unsigned char slave_count;
    unsigned char found;
    unsigned char aid_val;
    unsigned char temp_val;
    unsigned char k;
    
    for (k = 0; k < 16; k++) {
        slave_ids[k] = 0;
    }
    slave_count = 0;
    
    for (i = 0; i < HISTORY_SIZE; i++) {
        temp_val = history[i].temp;
        aid_val = history[i].aid;
        
        if (temp_val > 0 && temp_val < 100 && aid_val > 0) {
            found = 0;
            for (j = 0; j < slave_count; j++) {
                if (slave_ids[j] == aid_val) {
                    found = 1;
                    break;
                }
            }
            
            if (found == 0 && slave_count < 16) {
                slave_ids[slave_count] = aid_val;
                slave_count++;
            }
        }
    }
    
    return slave_count;
}

// ------------------- 显示从站数量 -------------------
void ShowSlaveCount(void)
{
    unsigned char slave_count;
    unsigned char buf[10];
    PageType saved_page;
    
    slave_count = CountDifferentSlaves();
    
    saved_page = menu_state.current_page;
    
    LCD_Clear();
    
    LCD_DisplayString(0, 0, "SLAVE COUNT");
    LCD_DisplayString(2, 0, "DIFF AID:");
    
    LCD_DisplayString(4, 0, "COUNT=");
    
    if (slave_count >= 100) {
        buf[0] = (slave_count / 100) + '0';
        buf[1] = ((slave_count % 100) / 10) + '0';
        buf[2] = (slave_count % 10) + '0';
        buf[3] = 0;
        LCD_DisplayString(4, 48, buf);
    } else if (slave_count >= 10) {
        buf[0] = (slave_count / 10) + '0';
        buf[1] = (slave_count % 10) + '0';
        buf[2] = 0;
        LCD_DisplayString(4, 48, buf);
    } else {
        buf[0] = slave_count + '0';
        buf[1] = 0;
        LCD_DisplayString(4, 48, buf);
    }
    
    LCD_DisplayString(6, 0, "AUTO RETURN...");
    
    delay_ms(3000);
    
    menu_state.current_page = saved_page;
    display_labels_initialized = 0;
    RefreshDisplay();
}

// ------------------- 删除预警记录函数 -------------------
static void DeleteWarningRecord(unsigned char index) {
	unsigned char i;
    if (index < WARNING_RECORD_SIZE) {
        // 将后面的记录前移
        for (i = index; i < WARNING_RECORD_SIZE - 1; i++) {
            warning_records[i] = warning_records[i + 1];
        }
        
        // 清空最后一条记录
        warning_records[WARNING_RECORD_SIZE - 1].id = 0;
        warning_records[WARNING_RECORD_SIZE - 1].sensor_id = 0;
        warning_records[WARNING_RECORD_SIZE - 1].temperature = 0;
        warning_records[WARNING_RECORD_SIZE - 1].time_hour = 0;
        warning_records[WARNING_RECORD_SIZE - 1].time_minute = 0;
        warning_records[WARNING_RECORD_SIZE - 1].is_valid = 0;
        
        // 调整选中索引
        if (menu_state.page15_selected > 0) {
            menu_state.page15_selected--;
        }
    }
}

// ------------------- 删除恢复记录函数 -------------------
static void DeleteRecoveryRecord(unsigned char index) {
	unsigned char i;
    if (index < RECOVERY_RECORD_SIZE) {
        // 将后面的记录前移
        for (i = index; i < RECOVERY_RECORD_SIZE - 1; i++) {
            recovery_records[i] = recovery_records[i + 1];
        }
        
        // 清空最后一条记录
        recovery_records[RECOVERY_RECORD_SIZE - 1].id = 0;
        recovery_records[RECOVERY_RECORD_SIZE - 1].sensor_id = 0;
        recovery_records[RECOVERY_RECORD_SIZE - 1].time_hour = 0;
        recovery_records[RECOVERY_RECORD_SIZE - 1].time_minute = 0;
        recovery_records[RECOVERY_RECORD_SIZE - 1].status = 0;
        recovery_records[RECOVERY_RECORD_SIZE - 1].is_valid = 0;
        
        // 调整选中索引
        if (menu_state.page15_selected > 0) {
            menu_state.page15_selected--;
        }
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

// ------------------- 接收数据处理(主循环调用) -------------------
void UART4_ReceiveString(void)
{
	 UpdateRTCRefresh();  // 添加这行，更新RTC显示
    DisplayFixedLabels();  // 显示固定标签
    
    if (uart_rx_complete == 1)
    {
        UART4_ParseData();  // 解析数据
        ClearDataArea();    // 清空数据区域
        
        // 根据当前页面更新显示
        switch (menu_state.current_page) {
            case PAGE_1:
                // 主页面显示实时数据
                break;
            default:
                break;
        }
        
        // 清空缓冲区
        UART4_ClearBuffer(uart_rx_buff, 6);
        uart_rx_complete = 0;
        uart_rx_len = 0;
    }
}

// ------------------- 导出历史数据 -------------------
void ExportHistoryData(void)
{
    unsigned int i;
    
    UART4_SendString("\r\n=== HISTORY DATA ===\r\n");
    UART4_SendString("IDX PID AID TEM VOL1 VOL2 FOSC\r\n");
    
    for (i = 0; i < HISTORY_SIZE; i++) {
        if (history[i].temp > 0 || history[i].volt1 > 0) {
            UART4_SendNumber(i, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)history[i].pid, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)history[i].aid, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)history[i].temp, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)history[i].volt1, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)history[i].volt2, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)history[i].fosc, 3);
            UART4_SendString("\r\n");
        }
    }
    
    // 导出统计信息
    UART4_SendString("\r\n=== STATISTICS ===\r\n");
    UART4_SendString("COUNT:");
    UART4_SendNumber((unsigned long)current_stats.total_data_count, 3);
    UART4_SendString("\r\n");
    
    UART4_SendString("TEMP: AVG:");
    UART4_SendNumber((unsigned long)current_stats.overall_avg_temp, 3);
    UART4_SendString(" MAX:");
    UART4_SendNumber((unsigned long)current_stats.overall_max_temp, 3);
    UART4_SendString(" MIN:");
    UART4_SendNumber((unsigned long)current_stats.overall_min_temp, 3);
    UART4_SendString("\r\n");
}

// ------------------- LCD按键处理 -------------------
void LCD_HandleKey(unsigned char key) {
    unsigned char i;
    PageType current_page = menu_state.current_page;

    switch(key) {
        case 1: // 按键1：PAGE_11进入PAGE_12；PAGE_12确认删除；PAGE_15进入PAGE_16；PAGE_17确认删除；其他页上一项
			
            if (current_page == PAGE_20) {
                // 确认删除历史数据
                ClearAllHistoryData();
                // 返回到PAGE_18
                menu_state.current_page = PAGE_18;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_23) {
                // 确认删除最高温度数据
                ClearMaxTemperatureData();
                // 返回到PAGE_21
                menu_state.current_page = PAGE_21;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
						}else if (current_page == PAGE_11) {
                // 从报警详细页面进入删除确认
                menu_state.prev_page = PAGE_11;
                menu_state.current_page = PAGE_12;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_12) {
                // ✅ 保留原有的删除逻辑
                // 执行删除逻辑
                if (menu_state.page11_selected < HISTORY_SIZE) {
                    // 找到要删除的记录索引
                    unsigned char delete_idx = menu_state.page11_selected;
                    
                    // 将后面的记录前移
                    for (i = delete_idx; i < HISTORY_SIZE - 1; i++) {
                        history[i] = history[i + 1];
                    }
                    
                    // 清空最后一条记录
                    history[HISTORY_SIZE - 1].pid = 0;
                    history[HISTORY_SIZE - 1].aid = 0;
                    history[HISTORY_SIZE - 1].temp = 0;
                    history[HISTORY_SIZE - 1].volt1 = 0;
                    history[HISTORY_SIZE - 1].volt2 = 0;
                    history[HISTORY_SIZE - 1].fosc = 0;
                    history[HISTORY_SIZE - 1].voltage = 0;
                    
                    // 调整选中索引
                    if (menu_state.page11_selected > 0) {
                        menu_state.page11_selected--;
                    }
                }
                menu_state.current_page = PAGE_11;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_15) {
                // 从预警详细页面进入删除确认（新增）
                menu_state.prev_page = PAGE_15;
                menu_state.current_page = PAGE_16;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_16) {
                // 删除预警记录（新增）
                if (menu_state.prev_page == PAGE_13) {
                    DeleteWarningRecord(menu_state.page15_selected);
                }
                menu_state.current_page = PAGE_15;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_17) {
                // 删除恢复记录（新增）
                if (menu_state.prev_page == PAGE_14) {
                    DeleteRecoveryRecord(menu_state.page15_selected);
                }
                menu_state.current_page = PAGE_15;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_26) {
                // PAGE_26按键1: 确认恢复出厂设置
                // 这里添加恢复出厂设置的逻辑
                ClearAllHistoryData();  // 清空所有历史数据
                // 可以添加其他重置逻辑
                
                // 返回到设备维护菜单
                menu_state.current_page = PAGE_8;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }else {
                HandlePrevItem();
            }
            break;

        case 2: // 按键2：PAGE_12返回PAGE_11；PAGE_11返回PAGE_10；PAGE_16返回PAGE_15；PAGE_17返回PAGE_15；PAGE_15返回PAGE_13；其他页下一项
					if (current_page == PAGE_20) {
                // 从PAGE_20返回PAGE_19
                menu_state.current_page = PAGE_19;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
					}else if (current_page == PAGE_23) {
                // 从PAGE_23返回PAGE_22
                menu_state.current_page = PAGE_22;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            else if (current_page == PAGE_12) {
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
                menu_state.current_page = PAGE_15;
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
            if (current_page == PAGE_5) {
                menu_state.current_page = PAGE_9;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_9) {
                menu_state.current_page = PAGE_5;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }  else if (current_page == PAGE_21) {
                // 从最高温度查询页面进入清除确认页面
                menu_state.current_page = PAGE_22;
                menu_state.page22_selected = 0;  // 重置选择
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_22) {
                // 从最高温度清除确认页面进入删除确认页面
                menu_state.current_page = PAGE_23;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }else if (current_page == PAGE_10) {
                // 从报警列表进入报警详细
                menu_state.prev_page = PAGE_10;
                menu_state.current_page = PAGE_11;
                menu_state.page11_selected = 0;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_13) {
                // 从预警列表进入预警详细（新增）
                menu_state.prev_page = PAGE_13;
                menu_state.current_page = PAGE_15;
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
            }
						else if (current_page == PAGE_18) {
                // 从历史数据查询页面进入清除确认页面
                menu_state.current_page = PAGE_19;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_19) {
                // 从清除确认页面进入删除确认页面
                menu_state.current_page = PAGE_20;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
						else if (current_page == PAGE_18) {
                // 从历史数据查询页面进入清除确认页面
                menu_state.current_page = PAGE_19;
                menu_state.page19_selected = 0;  // 重置选择
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_19) {
                // 从清除确认页面进入删除确认页面
                menu_state.current_page = PAGE_20;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }else {
                HandleEnterKey();
            }
            break;
            
  
            
        case 4: // 按键4：返回
            // 检查哪些页面不响应按键4
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
                    
                case PAGE_22:    // 最高温度清除确认页面 -> 返回PAGE_21
                    menu_state.current_page = PAGE_21;
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
    switch(menu_state.current_page) {
        case PAGE_2:
            if (menu_state.page2_selected > 0) {
                menu_state.page2_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
        case PAGE_7:
            if (menu_state.page7_selected > 0) {
                menu_state.page7_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
				case PAGE_8:
            if (menu_state.page8_selected > 0) {
                menu_state.page8_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
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
                menu_state.page13_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
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
                menu_state.page18_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
        case PAGE_19:  // 新增：PAGE_19上一项
            if (menu_state.page19_selected > 0) {
                menu_state.page19_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
				 case PAGE_21:  // 新增：PAGE_21上一项
            if (menu_state.page21_selected > 0) {
                menu_state.page21_selected--;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
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
    switch(menu_state.current_page) {
        case PAGE_2:
            if (menu_state.page2_selected < (PAGE2_ITEM_COUNT - 1)) {
                menu_state.page2_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
        case PAGE_7:
            if (menu_state.page7_selected < (PAGE7_ITEM_COUNT - 1)) {
                menu_state.page7_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
				case PAGE_8:
				 if (menu_state.page8_selected < 2) {  // 0-2共3项
                menu_state.page8_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
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
                menu_state.page13_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
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
            if (menu_state.page18_selected < 2) {  // 0-2共3项
                menu_state.page18_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
        case PAGE_19:  // 新增：PAGE_19下一项
            if (menu_state.page19_selected < 1) {  // 0-1共2项
                menu_state.page19_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }
            break;
				 case PAGE_21:  // 新增：PAGE_21下一项
            if (menu_state.page21_selected < 2) {  // 0-2共3项
                menu_state.page21_selected++;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
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
            led_toggle(3);
            break;
            
        case PAGE_2:
            switch(menu_state.page2_selected) {
                case MENU_MEASURE_DATA:
                    menu_state.current_page = PAGE_3;
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
//            if (menu_state.prev_page == PAGE_13) {
//                menu_state.current_page = PAGE_13;
//            } else if (menu_state.prev_page == PAGE_14) {
//                menu_state.current_page = PAGE_14;
//            } else {
//                menu_state.current_page = PAGE_7;
//            }
//            menu_state.page_changed = 1;
//            display_labels_initialized = 0;
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
            
        case PAGE_22:  // 新增：从PAGE_22返回PAGE_21
            menu_state.current_page = PAGE_21;
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
//            menu_state.current_page = PAGE_8;  // 返回到设备维护菜单
//            menu_state.page_changed = 1;
//            display_labels_initialized = 0;
            break;				
        default:
            break;
    }
}

// ------------------- 刷新显示 -------------------
static void RefreshDisplay(void)
{
    if (menu_state.page_changed) {
        display_labels_initialized = 0;
        DisplayFixedLabels();
        menu_state.page_changed = 0;
    }
}
// ------------------- 导出分区存储数据 -------------------
void ExportSummaryData(void)
{
    unsigned int i;
    
    UART4_SendString("\r\n=== DATA SUMMARY ===\r\n");
    UART4_SendString("IDX PID AID TEMP VOL1 VOL2 FOSC TS VALID\r\n");
    
    for (i = 0; i < TOTAL_RECORDS; i++) {
        if (data_summary[i].is_valid) {
            UART4_SendNumber(i, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)data_summary[i].pid, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)data_summary[i].aid, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)data_summary[i].temp, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)data_summary[i].volt1, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)data_summary[i].volt2, 3);
            UART4_SendString(" ");
            UART4_SendNumber((unsigned long)data_summary[i].fosc, 3);
            UART4_SendString(" ");
            UART4_SendNumber(data_summary[i].timestamp, 5);
            UART4_SendString(" ");
            UART4_SendNumber(data_summary[i].is_valid, 1);
            UART4_SendString("\r\n");
        }
    }
}

// ------------------- 统计活动从站数 -------------------
unsigned char CountActiveSlaves(void)
{
    unsigned int i;
    unsigned char active_count = 0;
    
    for (i = RECENT_DATA_START; i < RECENT_DATA_START + RECENT_DATA_SIZE; i++) {
        if (data_summary[i].is_valid) {
            active_count++;
        }
    }
    
    return active_count;
}

// ------------------- 更新汇总显示 -------------------
void UpdateSummaryDisplay(void)
{
    // 根据当前页面更新显示内容
    // 可根据需要实现具体显示逻辑
}
