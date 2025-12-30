#include <reg51.h>
#include <intrins.h>
#include "STC32G.h"
#include "config.h"
#include "led.h"
#include "lcd.h"
#include "key.h"
#include "relay.h"
#include "uart4.h"

void main() {
    unsigned char key;
    
    // 初始化外设
    GPIO_Init(); 
    Timer0_Init();        // 定时器0初始化
    relay_init();         // 继电器初始化
    led_all_off();        // 关闭所有LED
    UART4_Init(9600);     // 串口4初始化（包含菜单和RTC初始化）
    LCD_Init();           // LCD初始化
    LCD_Delay(100);       // 等待LCD稳定
    
  
    rtc_check_and_init();
    led_on(5);            // 打开LED5（系统就绪指示）
    LCD_Delay(50);        // 短暂延时
    

    
    // 输出RTC状态
    UART4_SendString("RTC initialized. Time will be displayed on LCD.\r\n");
    
    while(1) {
        key_scan();               // 扫描按键
        key = Key_GetValue();     // 获取按键值
        
        // 处理串口数据并刷新显示（RTC刷新在UART4_ReceiveString内部处理）
        UART4_ReceiveString();    // 这个函数会调用DisplayFixedLabels()和UpdateRTCRefresh()
        
        // 所有按键都通过 LCD_HandleKey 函数处理
        if (key >= 1 && key <= 4) {  // 按键1、2、3、4
            LCD_HandleKey(key);
        }
        
        // 短暂延时，降低CPU使用率
        LCD_Delay(5);
    }
}

