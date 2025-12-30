
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
unsigned char uart_rx_buff[UART_BUFF_SIZE] = {0};  // 接收缓冲区
unsigned char uart_rx_len = 0;                     // 接收长度
bit uart_rx_complete = 0;                          // 接收完成标志
unsigned long delay_count = 0;                     // 延时计数器
rtc_time_t current_rtc_time;
unsigned long rtc_refresh_counter = 0;
bit need_rtc_refresh = 1;

unsigned char slave_id;
DataRecord *recent_data;
unsigned char display_count = 0;  // 已显示的从站数量（最多3个）


// 新增：全局变量初始化
Protocol_Data parsed_data = {0, 0, 0, 0.0f, 0, -990};
DataRecord data_summary[TOTAL_RECORDS] = {0};
unsigned char history_index[TOTAL_SLAVES] = {0};
unsigned long system_tick = 0;
// ------------------- 菜单状态 -------------------
// 菜单状态变量
MenuState menu_state;

// 显示相关
extern PageType current_page;  // 当前页面
bit display_labels_initialized = 0;  // 固定标签是否已显示




// 新增：PAGE_1选中项标识（0=TX01，1=TX02）
unsigned char page1_selected = 0;




RTC_EditState rtc_edit_state = RTC_EDIT_IDLE;  // 当前编辑状态
unsigned char rtc_edit_pos = 0;                // 当前选中的修改位置（0-9对应：年1、年2、月1、月2、日1、日2、时1、时2、分1、分2）
rtc_time_t edit_temp_time;                     // 临时存储待修改的时间（避免直接修改原始数据）



// ------------------- 报警事件记录相关 -------------------
AlarmRecord alarm_events[MAX_ALARM_EVENTS] = {0};  // 报警事件记录数组
unsigned char alarm_event_count = 0;                // 当前报警事件数量
unsigned char alarm_event_next_index = 0;           // 下一个可用的报警事件索引（循环覆盖）

// 记录当前从站的上一次状态（用于判断是否从正常变为异常）
unsigned char last_abnormal_status[TOTAL_SLAVES] = {0};


// ------------------- 静态函数声明 -------------------
static void UART4_ClearBuffer(unsigned char *ptr, unsigned int len);

static void DisplayFixedLabels(void);
//static void ClearDataArea(void);
//static void ClearSummaryArea(void);

static void DisplayDetailPage(PageType page);//测量数据显示详情页面
static void HandlePrevItem(void);
static void HandleNextItem(void);
static void HandleEnterKey(void);
static void HandleReturnKey(void);
static void RefreshDisplay(void);
static void DisplayPage1(void); 
static void DisplayPage2(void);
static void DisplayPage3(void);
static void DisplayPage4(void);
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
static void UpdateDisplayForPage1(void);//动态刷新page1

static void UpdateDisplayForPage3(void);//动态刷新page3




static bit Protocol_Check(unsigned char *frame);
static short ADC_To_Temp(unsigned int adc_val);

static void Protocol_Parse(unsigned char *frame);
static unsigned char TempShortToChar(short temp);
static DataRecord* GetFixedSlaveData(unsigned char aid);
static void UpdateDisplayForPage3(void);



static void GetCurrentRTC(void);
static void UpdateRTCRefresh(void);

static void DisplayRTCOnPage24(void);

static void RTC_Save_Edit(void);
static void RTC_Edit_Init(void);
static void RTC_Switch_Pos(signed char step);
static void RTC_Adjust_Num(signed char step);


static void CheckAndRecordAlarm(void);
static void RecordAlarmEvent(unsigned char pid, unsigned char aid, unsigned char temp, unsigned int volt_mv);
static void DisplayAlarmEventsOnPage10(void);
static void AddToHistoryData(unsigned char pid, unsigned char aid, unsigned char temp, unsigned int volt_mv);



// PAGE_1设备列表（后续新增设备直接添加此处）
Page1_DevInfo page1_devices[] = {
    {1, "01TX01"},  // AID=1，设备ID=01TX01
    {2, "01TX02"}   // AID=2，设备ID=01TX02
};


// ------------------- 菜单初始化 -------------------
void Menu_Init(void) {
	unsigned char i;
    menu_state.current_page = PAGE_1;
    menu_state.prev_page = PAGE_1; 
    menu_state.page2_selected = 0;
	  menu_state.page3_selected = 0;
    menu_state.page7_selected = 0;
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
	  menu_state.page8_selected = 0;  
	  menu_state.page24_selected = 0; 
    menu_state.page25_selected = 0; 
    menu_state.page26_selected = 0; 
    menu_state.page_changed = 1;
    menu_state.menu_initialized = 0;
		// 初始化报警状态跟踪数组
		
    for(i = 0; i < TOTAL_SLAVES; i++) {
        last_abnormal_status[i] = 0;
    }
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
    

    Menu_Init();

		rtc_init();
		rtc_check_and_init();  // 检查并设置时间
		InitDataStorage(); 
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
	    system_tick++; 
}

// ------------------- 清空缓冲区 -------------------
static void UART4_ClearBuffer(unsigned char *ptr, unsigned int len)
{
    while(len--) *ptr++ = 0;
}

// ------------------- 清空数据区域(根据页面) -------------------
//static void ClearDataArea(void)
//{
//    switch (menu_state.current_page) {
//        case PAGE_1:
//            break;
//        case PAGE_2:
//            break;
//        case PAGE_3:
//            ClearSummaryArea();
//            break;
//    }
//}

// ------------------- 清空统计区域 -------------------
//static void ClearSummaryArea(void)
//{
//    // 清空统计区域
//}





//移植代码
static bit Protocol_Check(unsigned char *frame) {
    unsigned char fcs = frame[0] + frame[1] + frame[2] + frame[3] + frame[4];
    return (fcs == frame[5]) ? 1 : 0;
}


static short ADC_To_Temp(unsigned int adc_val) {
    double Rntc;
    double temp_k;
    double temp_c;
    
    if (adc_val == 0 || adc_val >= 4096) return -990;
    
    Rntc = (4096.0 * (double)NTC_R_REF) / (double)adc_val - (double)NTC_R_REF;
    temp_k = 1.0 / (log(Rntc / (double)NTC_R0) / NTC_B_VALUE + 1.0 / NTC_T0);
    temp_c = temp_k - 273.15;
    
    if (temp_c < -40.0 || temp_c > 125.0) return -990;
    return (short)(temp_c * 10.0 + 0.5);
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
    if (temp == -990) return 0;
    return (unsigned char)(abs(temp) / 10);
}




static DataRecord* GetFixedSlaveData(unsigned char aid) {
    unsigned char idx;
    if (aid < 1 || aid > TOTAL_SLAVES) return NULL;
    idx = aid - 1;
    if (data_summary[idx].is_valid) return &data_summary[idx];
    return NULL;
}







void InitDataStorage(void) {
    unsigned char i, j;
    unsigned char aid;
    unsigned short start_pos;

    for (i = 0; i < TOTAL_RECORDS; i++) {
        data_summary[i].is_valid = 0;
        data_summary[i].timestamp = 0;
        data_summary[i].aid = 0;
        data_summary[i].pid = 0;
        data_summary[i].temp = 0;
        data_summary[i].volt1 = 0;
        data_summary[i].volt2 = 0;
        data_summary[i].fosc = 0;
    }

    for (i = 0; i < TOTAL_SLAVES; i++) {
        data_summary[i].aid = i + 1;
    }

    for (i = 0; i < TOTAL_SLAVES; i++) {
        aid = i + 1;
        start_pos = HISTORY_DATA_START + (i * RECORDS_PER_SLAVE);
        for (j = 0; j < RECORDS_PER_SLAVE; j++) {
            data_summary[start_pos + j].aid = aid;
        }
    }

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

    UART4_ClearBuffer(uart_rx_buff, UART_BUFF_SIZE);
    uart_rx_len = 0;
    uart_rx_complete = 0;
}




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

    if (pid == 0 && aid == 0 && temp == 0 && 
        volt1 == 0 && volt2 == 0 && fosc == 0) {
        return;
    }

    if (aid < 1 || aid > TOTAL_SLAVES) {
        return;
    }

    slave_idx = aid - 1;
    current_time = GetSystemTick();
    recent_rec = &data_summary[slave_idx];

    if (recent_rec->is_valid) {
        hist_start = HISTORY_DATA_START + (slave_idx * RECORDS_PER_SLAVE);
        hist_idx = history_index[slave_idx];
        hist_pos = hist_start + hist_idx;
        hist_rec = &data_summary[hist_pos];

        hist_rec->pid = recent_rec->pid;
        hist_rec->aid = recent_rec->aid;
        hist_rec->temp = recent_rec->temp;
        hist_rec->volt1 = recent_rec->volt1;
        hist_rec->volt2 = recent_rec->volt2;
        hist_rec->fosc = recent_rec->fosc;
        hist_rec->timestamp = recent_rec->timestamp;
        hist_rec->is_valid = 1;

        history_index[slave_idx] = (hist_idx + 1) % RECORDS_PER_SLAVE;
    }

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
    if (aid < 1 || aid > TOTAL_SLAVES) return NULL;
    idx = aid - 1;
    if (data_summary[idx].is_valid) return &data_summary[idx];
    return NULL;
}





// ------------------- 显示固定标签 -------------------
static void DisplayFixedLabels(void)
{
    if (display_labels_initialized == 0)
    {
        LCD_Clear();  // 清屏
        
        switch (menu_state.current_page) {
            case PAGE_1:
                DisplayPage1();
                
                break;
                
            case PAGE_2:
                DisplayPage2();  // 显示PAGE_2内容
                break;
                
            case PAGE_3:
							DisplayPage3();
						break;
            case PAGE_4:
							DisplayPage4();
						break;
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



// PAGE_1显示函数（完全参照PAGE_2实现逻辑）
static void DisplayPage1(void) {
    unsigned char i;
    DataRecord* dev_data = NULL;
    // 新增：定义临时数组存储动态生成的设备ID
    unsigned char dev_id_buf[8];  // 足够存储"01TX1"这类字符串

    // 第0行：标题栏（保留原有代码，不动）
    LCD_DISPLAYCHAR_NEW(0, 0, 0, 0);   // "从"
    LCD_DISPLAYCHAR_NEW(0, 8, 1, 0);   // "站"
    LCD_DISPLAYCHAR_NEW(0, 48, 0, 1);  // "温"
    LCD_DISPLAYCHAR_NEW(0, 56, 1, 1);  // "度"
    LCD_DISPLAYCHAR_NEW(0, 64, 0, 6);  // "("
    LCD_DISPLAYCHAR_NEW(0, 72, 0, 2);  // "°"
    LCD_DISPLAYCHAR_NEW(0, 80, 0, 7);  // "C"
    LCD_DISPLAYCHAR_NEW(0, 88, 0, 3);  // "电"
    LCD_DISPLAYCHAR_NEW(0, 96, 1, 3);  // "压"
	  LCD_DISPLAYCHAR_NEW(0, 104, 2, 3);  // 
    LCD_DISPLAYCHAR_NEW(0, 112, 3, 3);  // 
	  LCD_DISPLAYCHAR_NEW(0, 120, 4, 3);  // (mv)
   
     // 清空数据显示区域（保留原有代码）
        for (i = 0; i < PAGE1_DEV_COUNT; i++) {
            unsigned char row = i * 2 + 2;  // 第2行、第4行
            LCD_DisplayChar(row, 0, ' ');
            LCD_DisplayString(row, 8, (unsigned char*)"      ");
            LCD_DisplayString(row, 48, (unsigned char*)"     ");
            LCD_DisplayString(row, 88, (unsigned char*)"    ");
        }

        // 选择箭头：根据当前选择状态绘制
        for (i = 0; i < PAGE1_DEV_COUNT; i++) {
            unsigned char row = i * 2 + 2;  // 设备1→第2行，设备2→第4行
            if (i == page1_selected) {
                LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  // 选中箭头
            } else {
                LCD_DisplayChar(row, 0, ' ');
            }
        }

        // 第6行：功能提示（保留原有代码）
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
        LCD_DISPLAYCHAR_NEW(6, 48, 3, 4);  // "上"
        LCD_DISPLAYCHAR_NEW(6, 56, 1, 4);  // "一"
        LCD_DISPLAYCHAR_NEW(6, 64, 2, 4);  // "页"
        LCD_DISPLAYCHAR_NEW(6, 104, 0, 5); // "返"
        LCD_DISPLAYCHAR_NEW(6, 112, 1, 5); // "回"

        // --- 关键修改：在初始化时也显示当前数据 ---
        // 循环显示2个设备的当前数据
        for (i = 0; i < PAGE1_DEV_COUNT; i++) {
            unsigned char row = i * 2 + 2;  // 设备1→第2行，设备2→第4行
            Page1_DevInfo* dev = &page1_devices[i];
            
            // 显示设备ID（核心修改：动态生成，替换原来的dev->dev_id）
            dev_id_buf[0] = '0';
            dev_id_buf[1] = '1';
            dev_id_buf[2] = 'T';
            dev_id_buf[3] = 'X';
            dev_id_buf[4] = '0' + (i + 1);  // i=0→'1'，i=1→'2'
            dev_id_buf[5] = '\0';           // 字符串结束符
            LCD_DisplayString(row, 8, dev_id_buf);  // 显示动态生成的ID

            // 获取设备实时数据
            dev_data = GetFixedSlaveData(dev->aid);

            // 显示温度（修改：显示整数温度）
            if (dev_data != NULL && dev_data->is_valid) {
                short temp_int = (short)dev_data->temp; // 直接使用原始值
                LCD_DisplayNumber(row, 56, (unsigned long)temp_int, 2); // 显示 2 位数字，例如 "20"
            } else {
                LCD_DisplayString(row, 56, (unsigned char*)"--"); // 显示 2 个破折号
            }

            // 显示电压（核心修改：将存储的值乘以100得到mV）
            if (dev_data != NULL && dev_data->is_valid) {
                unsigned int volt_mv = dev_data->volt1 * 100; // 将 0.01V 单位转换为 mV 单位
                LCD_DisplayNumber(row, 88, (unsigned long)volt_mv, 4); // 显示 4 位数字
            } else {
                LCD_DisplayString(row, 88, (unsigned char*)"----"); // 显示4个破折号
            }
        }
        // --- 修改结束 ---

        display_labels_initialized = 1; // 标记初始化完成
    }
    // else {
    //     // 如果 display_labels_initialized == 1，则不执行任何操作
    //     // 数据的更新由 UART4_ReceiveString 中的 UpdateDisplayForPage1 负责
    // }






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




static void DisplayPage3(void) {
    unsigned char module_index = menu_state.page3_selected; // 当前选中的模块索引
	unsigned char tx_number;
    DataRecord* dev_data = NULL;
    
    if (display_labels_initialized == 0) {
        // 清屏
        LCD_Clear();
        
        // 第0行：固定标签 "PID" 和冒号
        LCD_DisplayString(0, 0, (unsigned char*)"PID");
        LCD_DISPLAYCHAR_NEW(0, 24, 0, 16); // 冒号 ":"
        
        // 固定标签 "TX" 和冒号
        LCD_DisplayString(0, 72, (unsigned char*)"TX");
        LCD_DISPLAYCHAR_NEW(0, 88, 0, 16); // 冒号 ":"

        // 第2行：固定标签 "温度:"
        LCD_DISPLAYCHAR_NEW(2, 0, 0, 1);  // "温"
        LCD_DISPLAYCHAR_NEW(2, 8, 1, 1);  // "度"
        LCD_DISPLAYCHAR_NEW(2, 16, 0, 15); // ":"

        // 第4行：固定标签 "电压:"
        LCD_DISPLAYCHAR_NEW(4, 0, 0, 3);  // "电"
        LCD_DISPLAYCHAR_NEW(4, 8, 1, 3);  // "压"
        LCD_DISPLAYCHAR_NEW(4, 16, 0, 15); // ":"
        LCD_DisplayString(4, 64, (unsigned char*)"mv");

        // 第6行：功能提示
        LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
        LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
        LCD_DISPLAYCHAR_NEW(6, 16, 4, 4);  // "页"
        LCD_DISPLAYCHAR_NEW(6, 60, 3, 4);  // "上"
        LCD_DISPLAYCHAR_NEW(6, 68, 1, 4);  // "一"
        LCD_DISPLAYCHAR_NEW(6, 76, 4, 4);  // "页"
        LCD_DISPLAYCHAR_NEW(6, 112, 0, 11); // "返"
        LCD_DISPLAYCHAR_NEW(6, 120, 1, 11); // "回"

        display_labels_initialized = 1;
    }
    
    // 获取当前选中模块的数据
    dev_data = GetRecentDataByAID(module_index + 1); // AID从1开始
    
    // 清空数据显示区域
    LCD_DisplayString(0, 32, (unsigned char*)"  ");    // PID显示区域（2位）
    LCD_DisplayString(0, 96, (unsigned char*)"  ");    // TX显示区域（2位）
    LCD_DisplayString(2, 24, (unsigned char*)"    ");  // 温度显示区域（4位）
    LCD_DisplayString(4, 24, (unsigned char*)"    ");  // 电压显示区域（4位）
    
    // 显示PID
    if (dev_data != NULL && dev_data->is_valid) {
        // 显示2位PID
        LCD_DisplayNumber(0, 32, dev_data->pid, 2);
    } else {
        LCD_DisplayString(0, 32, (unsigned char*)"--");
    }
    
    // 显示TX（设备号），格式为两位数，如"01", "02"
    // TX号 = 当前模块索引 + 1
    tx_number = module_index + 1;
    LCD_DisplayNumber(0, 96, tx_number, 2);
    
    // 显示温度
    if (dev_data != NULL && dev_data->is_valid) {
        // 显示整数温度，2位数字
        unsigned char temp_display = dev_data->temp;
        if (temp_display == 0) {
            LCD_DisplayString(2, 24, (unsigned char*)"--");
        } else {
            LCD_DisplayNumber(2, 24, temp_display, 2);
        }
    } else {
        LCD_DisplayString(2, 24, (unsigned char*)"--");
    }
    
    // 显示电压（转换为mV显示）
    if (dev_data != NULL && dev_data->is_valid) {
        unsigned int volt_mv = dev_data->volt1 * 100;  // 转换为mV
        // 显示4位电压值
        LCD_DisplayNumber(4, 24, volt_mv, 4);
    } else {
        LCD_DisplayString(4, 24, (unsigned char*)"----");
    }
    
    // 在角落显示当前模块编号/总模块数（可选，便于调试）
    // LCD_DisplayNumber(6, 100, module_index + 1, 2);
    // LCD_DisplayChar(6, 116, '/');
    // LCD_DisplayNumber(6, 124, TOTAL_SLAVES, 2);
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
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   // "下"
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   // "一"
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  // "上"
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  // "一"
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  // "页"
    
    LCD_DISPLAYCHAR_NEW(6, 80, 0, 26); // "详"
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 26); // "情"
    
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  // "返"
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  // "回"
    
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
    
    // 如果没有异常模块，显示"无异常"或其他提示
//    if (abnormal_count == 0) {
//        // 在第0行显示"无异常"提示
//        LCD_DISPLAYCHAR_NEW(0, 48, 0, 31);  // 正
//        LCD_DISPLAYCHAR_NEW(0, 56, 1, 31);  // 异
//        LCD_DISPLAYCHAR_NEW(0, 64, 2, 31);  // 常
//    }
}
     

// ------------------- 修正DisplayPage7(显示正确的菜单名称) -------------------
static void DisplayPage7(void)
{
    unsigned char start_index = 0;  
    unsigned char i;
    unsigned char col;  
    
    if (menu_state.page7_selected >= 3) {
        start_index = menu_state.page7_selected - 2;  
        if (start_index > (PAGE7_ITEM_COUNT - 3)) {
            start_index = PAGE7_ITEM_COUNT - 3;
        }
    }
    
  for (i = 0; i < 3; i++) {
        unsigned char row = i * 2;
        
        LCD_DisplayChar(row, 0, ' ');
        
        for (col = 8; col < 56; col += 8) {
            LCD_DisplayChar(row, col, ' ');
        }
    }
    
   
    for (i = 0; i < 3 && (start_index + i) < PAGE7_ITEM_COUNT; i++) {
        unsigned char item_index = start_index + i;
        unsigned char row = i * 2;
        

        if (item_index == menu_state.page7_selected) {
           LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  
        } else {
            LCD_DisplayChar(row, 0, ' '); 
        }
        
        switch(item_index) {
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
                // ??"?????2"
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
                // ??"??????"
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
    }
    
    // ?6?:??????
    // "??"
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);  
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);  
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4); 
    
    // ???
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4); 
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4); 
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4); 
    
    // ??
    LCD_DISPLAYCHAR_NEW(6, 80, 0, 22);
    LCD_DISPLAYCHAR_NEW(6, 88, 1, 22);
    
    // ??
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  

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
        
        display_labels_initialized = 1;
    }
    
    // 获取选中的报警事件
    if (menu_state.page10_selected < alarm_event_count) {
        display_index = (alarm_event_next_index - menu_state.page10_selected - 1 + MAX_ALARM_EVENTS) % MAX_ALARM_EVENTS;
        event = &alarm_events[display_index];
    }
    
    // 清空数据显示区域
    LCD_DisplayString(0, 56, (unsigned char*)"  ");   // PID显示区域
    LCD_DisplayString(0, 104, (unsigned char*)"  ");  // AID显示区域
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
    

    LCD_Clear();
    

    for (i = 0; i < 3; i++) {
        row = i * 2;  

        if (i == menu_state.page13_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25); 
        } else {
            LCD_DisplayChar(row, 0, ' ');
        }
        
        // ??"??"???
        LCD_DISPLAYCHAR_NEW(row, 8, 2, 19);  
        LCD_DISPLAYCHAR_NEW(row, 16, 3, 19); 
        LCD_DisplayChar(row, 24, '1' + i);   
    }
    
    // ?6?:????
    LCD_DISPLAYCHAR_NEW(0, 40, 2, 20); 
    LCD_DISPLAYCHAR_NEW(0, 48, 3, 20); 
    LCD_DISPLAYCHAR_NEW(0, 56, 0, 15); 
    LCD_DISPLAYCHAR_NEW(2, 40, 2, 20); 
    LCD_DISPLAYCHAR_NEW(2, 48, 3, 20); 
    LCD_DISPLAYCHAR_NEW(2, 56, 0, 15); 
    LCD_DISPLAYCHAR_NEW(4, 40, 2, 20); 
    LCD_DISPLAYCHAR_NEW(4, 48, 3, 20); 
    LCD_DISPLAYCHAR_NEW(4, 56, 0, 15); 
        
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);  
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);  
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4); 
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4); 
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4); 
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4); 
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 26);
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 26);
        
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  
}


// ------------------- 显示PAGE_14(传感器恢复事件列表) -------------------
static void DisplayPage14(void) {
       unsigned char i;
    unsigned char row;
    

    LCD_Clear();
    

    for (i = 0; i < 3; i++) {
        row = i * 2; 
        

        if (i == menu_state.page14_selected) {
            LCD_DISPLAYCHAR_NEW(row, 0, 0, 25);  
        } else {
            LCD_DisplayChar(row, 0, ' ');
        }
        
      
        LCD_DISPLAYCHAR_NEW(row, 8, 2, 19);  
        LCD_DISPLAYCHAR_NEW(row, 16, 3, 19); 
        LCD_DisplayChar(row, 24, '1' + i);   
    }
    
   
    LCD_DISPLAYCHAR_NEW(0, 40, 2, 20); 
    LCD_DISPLAYCHAR_NEW(0, 48, 3, 20); 
    LCD_DISPLAYCHAR_NEW(0, 56, 0, 15); 
    LCD_DISPLAYCHAR_NEW(2, 40, 2, 20); 
    LCD_DISPLAYCHAR_NEW(2, 48, 3, 20); 
    LCD_DISPLAYCHAR_NEW(2, 56, 0, 15); 
    LCD_DISPLAYCHAR_NEW(4, 40, 2, 20); 
    LCD_DISPLAYCHAR_NEW(4, 48, 3, 20); 
    LCD_DISPLAYCHAR_NEW(4, 56, 0, 15); 
        
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);   
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);   
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);  
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);  
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);  
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);  
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 26); 
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 26); 
        
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);
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
	
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4);  
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4);  
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4); 
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4); 
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4); 
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4); 
    
    LCD_DISPLAYCHAR_NEW(6, 72, 0, 26);
    LCD_DISPLAYCHAR_NEW(6, 80, 1, 26);
        
    LCD_DISPLAYCHAR_NEW(6, 112, 0, 11);  
    LCD_DISPLAYCHAR_NEW(6, 120, 1, 11);  
}

// ------------------- 显示PAGE_19(清除历史数据确认页面) -------------------
static void DisplayPage19(void) {
    LCD_Clear();
    LCD_DISPLAYCHAR_NEW(6, 0, 0, 4); 
    LCD_DISPLAYCHAR_NEW(6, 8, 1, 4); 
    LCD_DISPLAYCHAR_NEW(6, 16, 2, 4);
    
    LCD_DISPLAYCHAR_NEW(6, 40, 3, 4);
    LCD_DISPLAYCHAR_NEW(6, 48, 1, 4);
    LCD_DISPLAYCHAR_NEW(6, 56, 2, 4);
    
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
//static void RTC_Edit_Init(void) {
//    rtc_edit_state = RTC_EDIT_SELECT;
//    rtc_edit_pos = 0;                  // 默认选中第一个位置（年十位）
//    // 核心优化：每次进入编辑都同步最新的当前时间（而非复用旧临时数据）
//    GetCurrentRTC();
//    edit_temp_time = current_rtc_time; // 复制最新时间到临时变量
//    display_labels_initialized = 0;    // 强制刷新显示
//}

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
    rtc_edit_pos += step;
    // 边界循环：0-9之间切换
    if (rtc_edit_pos < 0) rtc_edit_pos = 9;
    if (rtc_edit_pos > 9) rtc_edit_pos = 0;
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
    rtc_edit_pos = 0;
    
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


// --- 2. 修改 UART4_ReceiveString ---



//void UART4_ReceiveString(void) {
//    // 注释掉PAGE_1的RTC刷新（已不显示时间，无需刷新）
//    UpdateRTCRefresh(); 
//    DisplayFixedLabels(); // 显示固定标签（标题等）
//    
//    if (uart_rx_complete == 1) {
//        Protocol_Parse(uart_rx_buff); // 解析数据
//        AddDataToSummary(parsed_data.PID, parsed_data.AID,
//                        TempShortToChar(parsed_data.temperature),
//                        (unsigned char)(parsed_data.Bat_Voltage * 10),
//                        0, 0); // 将解析后的最新数据添加到汇总区 (data_summary)
//        

//        if (menu_state.current_page == PAGE_1) {
//            // 调用新的局部刷新函数，只更新中间两行数据
//            UpdateDisplayForPage1(); 
//        } else if (menu_state.current_page == PAGE_3) {
//            UpdateDisplayForPage3(); // 保持原有的PAGE_3刷新逻辑
//        } else if (menu_state.current_page == PAGE_21) { // 假设这是最高温度查询页面
//            DisplayPage21(); // 添加对PAGE_21的刷新调用 (需要确保函数存在)
//        }
//        // 可以为其他需要数据刷新的页面添加类似的 else if 分支
//        
//        UART4_ClearBuffer(uart_rx_buff, 6);
//        uart_rx_complete = 0;
//        uart_rx_len = 0;
//    }
//}
void UART4_ReceiveString(void) {
    UpdateRTCRefresh(); 
    DisplayFixedLabels();
    
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
        }
        
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

    // 循环绘制2个设备（修改核心：动态生成设备ID）
    for (i = 0; i < PAGE1_DEV_COUNT; i++) {
        unsigned char row = i * 2 + 2;  // 设备1→第2行，设备2→第4行
        Page1_DevInfo* dev = &page1_devices[i];
        

        unsigned char dev_id_buf[8]; // 局部数组
        dev_id_buf[0] = '0';
        dev_id_buf[1] = '1';
        dev_id_buf[2] = 'T';
        dev_id_buf[3] = 'X';
        dev_id_buf[4] = '0' + (i + 1);  // i=0→'1'，i=1→'2'
        dev_id_buf[5] = '\0';           // 字符串结束符
        LCD_DisplayString(row, 8, dev_id_buf);  // 显示动态生成的ID

        // 3. 获取设备实时数据（保留原有逻辑）
        dev_data = GetFixedSlaveData(dev->aid);

        // 4. 显示温度（修改：显示整数温度，假设dev_data->temp单位是1度）
        if (dev_data != NULL && dev_data->is_valid) {
            // 假设 dev_data->temp 是 1 度单位 (例如 20 代表 20 度)
            short temp_int = (short)dev_data->temp; // 直接使用原始值
            LCD_DisplayNumber(row, 56, (unsigned long)temp_int, 2); // 显示 2 位数字，例如 "20"
        } else {
            LCD_DisplayString(row, 56, (unsigned char*)"--"); // 显示 2 个破折号
        }

        // 5. 显示电压（核心修改：将存储的值乘以100得到mV）
        if (dev_data != NULL && dev_data->is_valid) {
            // 假设 dev_data->volt1 是 0.01V 为单位 (例如 36 代表 0.36V 或 360mV)
            // 要显示 mV，需要将其乘以 100 (36 * 100 = 3600 mV)
            unsigned int volt_mv = dev_data->volt1 * 100; // 将 0.01V 单位转换为 mV 单位
            LCD_DisplayNumber(row, 88, (unsigned long)volt_mv, 4); // 显示 4 位数字
        } else {
            LCD_DisplayString(row, 88, (unsigned char*)"----"); // 显示4个破折号
        }
    }
}



// --- 1. (可选) 定义 UpdateDisplayForPage3 ---
static void UpdateDisplayForPage3(void) {
unsigned char module_index = menu_state.page3_selected; // 当前选中的模块索引
	unsigned char tx_number;
    DataRecord* dev_data = GetRecentDataByAID(module_index + 1); // AID从1开始
    
    // 只刷新数据部分，不刷新固定标签
    
    // 刷新PID显示
    if (dev_data != NULL && dev_data->is_valid) {
        LCD_DisplayNumber(0, 32, dev_data->pid, 2);
    } else {
        LCD_DisplayString(0, 32, (unsigned char*)"--");
    }
    
    // TX号不需要刷新，因为它是根据当前选中的模块索引计算的
    // 显示当前模块的TX号
    tx_number = module_index + 1;
    LCD_DisplayNumber(0, 96, tx_number, 2);
    
    // 刷新温度显示
    if (dev_data != NULL && dev_data->is_valid) {
        unsigned char temp_display = dev_data->temp;
        if (temp_display == 0) {
            LCD_DisplayString(2, 24, (unsigned char*)"--");
        } else {
            LCD_DisplayNumber(2, 24, temp_display, 2);
        }
    } else {
        LCD_DisplayString(2, 24, (unsigned char*)"--");
    }
    
    // 刷新电压显示
    if (dev_data != NULL && dev_data->is_valid) {
        unsigned int volt_mv = dev_data->volt1 * 100;  // 转换为mV
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
					if (current_page == PAGE_24) {
                if (rtc_edit_state == RTC_EDIT_SELECT) {
                    RTC_Switch_Pos(-1);  // 选择状态：上一位
                } else if (rtc_edit_state == RTC_EDIT_CHANGE) {
                    RTC_Adjust_Num(1);   // 修改状态：数字加
                }
                return;
            }
			else if (current_page == PAGE_1) {
        // 上一项：从TX02→TX01，边界判断（与PAGE_2一致）
        if (page1_selected > 0) {
            page1_selected--;
            display_labels_initialized = 0;  // 刷新显示
        }
    } 
           else if (current_page == PAGE_20) {
                // 确认删除历史数据
//                ClearAllHistoryData();
                // 返回到PAGE_18
                menu_state.current_page = PAGE_18;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_23) {
                // 确认删除最高温度数据
//                ClearMaxTemperatureData();
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
         
                menu_state.current_page = PAGE_15;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_17) {

                menu_state.current_page = PAGE_15;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            } else if (current_page == PAGE_26) {
       
                menu_state.current_page = PAGE_8;
                menu_state.page_changed = 1;
                display_labels_initialized = 0;
            }else {
                HandlePrevItem();
            }
            break;

        case 2: // 按键2：PAGE_12返回PAGE_11；PAGE_11返回PAGE_10；PAGE_16返回PAGE_15；PAGE_17返回PAGE_15；PAGE_15返回PAGE_13；其他页下一项
				if (current_page == PAGE_24) {
                if (rtc_edit_state == RTC_EDIT_SELECT) {
                    RTC_Switch_Pos(1);   // 选择状态：下一位
                } else if (rtc_edit_state == RTC_EDIT_CHANGE) {
                    RTC_Adjust_Num(-1);  // 修改状态：数字减
                }
                return;
            }	  
				else if (current_page == PAGE_1) {
        // 下一项：从TX01→TX02，边界判断（与PAGE_2一致）
        if (page1_selected < (PAGE1_DEV_COUNT - 1)) {
            page1_selected++;
            display_labels_initialized = 0;  // 刷新显示
        }
    }else if (current_page == PAGE_20) {
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
            }
           else if (current_page == PAGE_5) {
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
				 case PAGE_3: // 新增：处理 PAGE_3 的上一项
         if (menu_state.page3_selected > 0) {
             menu_state.page3_selected--;
             // menu_state.page_changed = 1; // 不需要切换页面，只刷新内容
             // display_labels_initialized = 0; // 不需要重绘标签，只刷新数据
             DisplayPage3(); // 直接调用显示函数刷新当前选中项的数据
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
				case PAGE_3: // 新增：处理 PAGE_3 的下一项
         if (menu_state.page3_selected < (PAGE3_MAX_MODULES - 1)) { // 范围是 0 到 (MAX-1)
             menu_state.page3_selected++;
             // menu_state.page_changed = 1; // 不需要切换页面，只刷新内容
             // display_labels_initialized = 0; // 不需要重绘标签，只刷新数据
             DisplayPage3(); // 直接调用显示函数刷新当前选中项的数据
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


// 检查并记录报警事件
static void CheckAndRecordAlarm(void) {
    unsigned char i;
    DataRecord* dev_data = NULL;
    unsigned char current_abnormal = 0;
    
    for (i = 0; i < TOTAL_SLAVES; i++) {
        dev_data = GetRecentDataByAID(i + 1);
        
        if (dev_data != NULL && dev_data->is_valid) {
            // 修改：温度>25℃直接记录（去掉"从正常变异常"的限制，便于测试）
            current_abnormal = (dev_data->temp > 25) ? 1 : 0;
            
            if (current_abnormal == 1) {
                // 新增：避免重复记录（同一异常状态只记录一次）
                if (last_abnormal_status[i] == 0) {
                    RecordAlarmEvent(dev_data->pid, i + 1, dev_data->temp, dev_data->volt1 * 100);
                    last_abnormal_status[i] = 1;
                    
                    // 调试信息：确认报警被记录
                    UART4_SendString("Alarm recorded: AID=");
                    UART4_SendNumber(i + 1, 2);
                    UART4_SendString(", Temp=");
                    UART4_SendNumber(dev_data->temp, 2);
                    UART4_SendString("\r\n");
                }
            } else if (current_abnormal == 0 && last_abnormal_status[i] == 1) {
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
            
            // 可选：如果需要显示TX编号，可以在"事件X"后面显示
            // 比如："事件1 TX01 时间：25-12-31"
            // 在第24-32列显示TX编号
            // LCD_DisplayString(row, 24, "TX");
            // LCD_DisplayNumber(row, 32, event->aid, 2);
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





