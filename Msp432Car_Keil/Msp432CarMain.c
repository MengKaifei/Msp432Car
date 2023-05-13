#include <gpio.h>
#include <wdt_a.h>
#include <key.h>
#include <PID.h>
#include <delay.h>
#include <timA.h>
#include <sysinit.h>
#include <driverlib.h>
#include <stdio.h>

//void track_pinint();    //初始化循迹模块引脚
void motor_pinint();    //初始化电机引脚
void start();           //开车
void stop();            //停车

int fputc(int ch, FILE *f)
{
		UART_transmitData(EUSCI_A1_BASE, ch);
}

//初始化
const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,//时钟源   48 MHZ
    26,//BRDIV = 78
    0,//UCxBRF = 2
    111,//UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,//无校验
    EUSCI_A_UART_LSB_FIRST,//低位先行
    EUSCI_A_UART_ONE_STOP_BIT,//一个停止位
    EUSCI_A_UART_MODE,//UART模式
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

/***********************************<<<   定时器设置   >>>************************************/
#define CLKDIV 48   //时钟源分频
#define CCR0 49999  // 50毫秒更新一次PID

int main() {
    // 关闭看门狗
    WDT_A_holdTimer();

    // 时钟配置
    SysInit();

		// 串口配置
		//GPIO配置
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,GPIO_PIN2|GPIO_PIN3,GPIO_PRIMARY_MODULE_FUNCTION);
    //配置UART
    MAP_UART_initModule(EUSCI_A1_BASE,&uartConfig);
    MAP_UART_enableModule(EUSCI_A1_BASE);
		
		//i2c_init();
	
    // 按键初始化
    key_Init();

    // 电机初始化
    motor_pinint();

    // 循迹模块初始化
    //track_pinint();
		TRACK_PAD_PIN_INIT();

    // LED初始化
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);

    // 延时初始化
    delay_init();

    // 电机PWM输出初始化
    TimA0_4_PWM_Init();

    // 舵机PWM输出初始化
    TimA1_4_PWM_Init();
    TimA1_4_PWM_set_angle(90);
		delay_ms(50);

		// 定时器中断
    TimA2_Int_Init(CCR0, CLKDIV);

    Interrupt_enableMaster();
		
    while (1){
				//UART_transmitData(EUSCI_A0_BASE,'A');
        // 处理按键按下事件
        int keyValue = key_Value();
        if (keyValue == KEY1_PRES){
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            start();
					  //delay_ms(1000);
        }else if (keyValue == KEY2_PRES){
            GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
            stop();
        }
				
    }
}

/* 电机引脚初始化 */
void motor_pinint( )
{
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN4 | GPIO_PIN6);
}

/* 控制车前进 */
void start(){
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN6);
    TimA0_4_PWM_set_duty(50);
}

/* 控制车停止 */
void stop()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN6);
    TimA0_4_PWM_stop();
}


