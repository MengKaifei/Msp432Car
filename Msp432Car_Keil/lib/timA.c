/****************************************************/
// MSP432P401R
// 定时器A
// Bilibili：m-RNA
// E-mail:m-RNA@qq.com
// 创建日期:2021/8/26
/****************************************************/

#include "timA.h"
#include <stdio.h>
#include <stdlib.h>

#define DEFINE_SERVO 90    //初始舵机角度

int sensor[8] = {0, 0, 0, 0, 0,0,0,0};    // 8个传感器数值的数组

/***********************************<<<   PID设置   >>>************************************/
float measureValue = 0.0;
#define PID_KP  10.0f
#define PID_KI  0.0f
#define PID_KD  0.4f

#define PID_TAU 0.02f

#define PID_LIM_MIN -50.0f
#define PID_LIM_MAX  50.0f

#define PID_LIM_MIN_INT 0.0f
#define PID_LIM_MAX_INT 0.0f

#define SAMPLE_TIME_S 0.01f
PID trackPid = {PID_KP, PID_KI, PID_KD,
                PID_TAU,
                PID_LIM_MIN, PID_LIM_MAX,
                PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                SAMPLE_TIME_S};												
								
								
/**************************************         TIMA1_4          *******************************************/
// TODO PWM舵机控制
#define TIMEA1_4_AUTO_PERIOD 19999
#define TIMEA1_4_CLOCK_DIVIDER 48


static  Timer_A_PWMConfig TimA1_4_PWMConfig  = {
			.clockSource = TIMER_A_CLOCKSOURCE_SMCLK,             //时钟源
			.clockSourceDivider = TIMEA1_4_CLOCK_DIVIDER,                            //时钟分频 范围1-64
			.timerPeriod = TIMEA1_4_AUTO_PERIOD,                                  //自动重装载值（ARR）
			.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2, //通道4 （引脚定义）
			.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET,   //输出模式
			.dutyCycle = TIMEA1_4_AUTO_PERIOD,                                    //这里是改变占空比的地方 默认100%
};

void TimA1_4_PWM_Init()
{
	/*初始化引脚*/
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

	MAP_Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_4_PWMConfig); /* 初始化比较寄存器以产生 PWM1 */
}

// 0 - 180角度
void TimA1_4_PWM_set_angle(int angle)
{
	TimA1_4_PWMConfig.dutyCycle = ((angle/18.0+2.5) / 100.0) * TIMEA1_4_AUTO_PERIOD;
	MAP_Timer_A_clearTimer(TIMER_A1_BASE);
	MAP_Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_4_PWMConfig); /* 初始化比较寄存器以产生 PWM0 */
	//MAP_Timer_A_setCompareValue(TIMER_A1_BASE,, ((angle/18.0+2.5) / 100.0) * TIMEA1_4_AUTO_PERIOD);
}

void TimA1_4_PWM_stop()
{
	TimA1_4_PWM_set_angle(0);
}

/*********************************************************************************************************/

/**************************************         TIMA0_4          *******************************************/
// TODO PWM电机控制

#define TIMEA0_4_AUTO_PERIOD 19999
#define TIMEA0_4_CLOCK_DIVIDER 48


static  Timer_A_PWMConfig TimA0_4_PWMConfig  = {
			.clockSource = TIMER_A_CLOCKSOURCE_SMCLK,             //时钟源
			.clockSourceDivider = TIMEA0_4_CLOCK_DIVIDER,                            //时钟分频 范围1-64
			.timerPeriod = TIMEA0_4_AUTO_PERIOD,                                  //自动重装载值（ARR）
			.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4, //通道4 （引脚定义）
			.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET,   //输出模式
			.dutyCycle = TIMEA0_4_AUTO_PERIOD,                                    //这里是改变占空比的地方 默认100%
};

void TimA0_4_PWM_Init()
{
	/*初始化引脚*/
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
}

// 0 - 100占空比
void TimA0_4_PWM_set_duty(int duty)
{
	TimA0_4_PWMConfig.dutyCycle = (duty / 100.0) * TIMEA0_4_AUTO_PERIOD;
	MAP_Timer_A_clearTimer(TIMER_A0_BASE);
	MAP_Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_4_PWMConfig); /* 初始化比较寄存器以产生 PWM0 */
}

void TimA0_4_PWM_stop()
{
	TimA0_4_PWM_set_duty(0);
}
/*********************************************************************************************************/

/**************************************         TIMA2          *******************************************/

void TimA2_Int_Init(uint16_t ccr0, uint16_t psc)
{
	// 1.增计数模式初始化
	Timer_A_UpModeConfig upConfig;
	upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;                                      //时钟源
	upConfig.clockSourceDivider = psc;                                                     //时钟分频 范围1-64
	upConfig.timerPeriod = ccr0;                                                           //自动重装载值（ARR）
	upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;                   //禁用 tim溢出中断
	upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //启用 ccr0更新中断
	upConfig.timerClear = TIMER_A_DO_CLEAR;                                                // Clear value

	// 2.初始化定时器A
	MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);

	// 3.选择模式开始计数
	MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

	// 4.清除比较中断标志位
	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	// 5.开启串口端口中断
	MAP_Interrupt_enableInterrupt(INT_TA2_0);

	// PID初始化
	PID_Init(&trackPid);
}


/* 获取循迹偏移值 */
void read_sensor_values()
{
	sensor[0] = GPIO_getInputPinValue(GPIO_PORT_P10,GPIO_PIN5);
	sensor[1] = GPIO_getInputPinValue(GPIO_PORT_P10,GPIO_PIN3);
	sensor[2] = GPIO_getInputPinValue(GPIO_PORT_P10,GPIO_PIN1);
	sensor[3] = GPIO_getInputPinValue(GPIO_PORT_P7,GPIO_PIN7);
	sensor[4] = GPIO_getInputPinValue(GPIO_PORT_P7,GPIO_PIN5);
	sensor[5] = GPIO_getInputPinValue(GPIO_PORT_P9,GPIO_PIN7);
	sensor[6] = GPIO_getInputPinValue(GPIO_PORT_P9,GPIO_PIN5);
  sensor[7] = GPIO_getInputPinValue(GPIO_PORT_P7,GPIO_PIN0);


	// if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 0)) {
	// 		measureValue = 3;//           0 0 0 0 0 0 0 1
	// }  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 0) && (sensor[7] == 0)) {
	// 		measureValue = 2.5;//           0 0 0 0 0 0 1 1			
	// } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 0) && (sensor[7] == 1)) {
	// 		measureValue = 2;//           0 0 0 0 0 0 1 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 0) && (sensor[6] == 0) && (sensor[7] == 1)) {
	// 		measureValue = 1.5;//           0 0 0 0 0 1 1 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 0) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = 1;//           0 0 0 0 0 1 0 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 0) && (sensor[5] == 0) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = 0.5;//           0 0 0 0 1 1 0 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 0) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = 0.1;//         0 0 0 0 1 0 0 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = 0;//           0 0 0 1 1 0 0 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = -0.1;//        0 0 0 1 0 0 0 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (
	// 				sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = -0.5;//        0 0 1 1 0 0 0 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = -1;//        0 0 1 0 0 0 0 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = -1.5;//        0 1 1 0 0 0 0 0
	// } else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = -2;//        0 1 0 0 0 0 0 0
	// } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = -2.5;//        1 1 0 0 0 0 0 0
	// } else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (
	// 				sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 1) && (sensor[7] == 1)) {
	// 		measureValue = -3;//        1 0 0 0 0 0 0 0
	// }
	uint8_t res = TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_1) << 7 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_2) << 6 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_3) << 5 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_4) << 4
								|TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_5) << 3 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_6) << 2 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_7) << 1 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_8);
	printf("%d\r\n", res);
	switch(res){
		case 254:measureValue = 3;break;	// 11111110
		case 252:measureValue = 2.5;break;	// 11111100
		case 253:measureValue = 2;break;	// 11111101
		case 249:measureValue = 1.5;break;	// 11111001
		case 251:measureValue = 1;break;	// 11111011
		case 243:measureValue = 0.5;break;	// 11110011
		case 247:measureValue = 0.1;break;	// 1110111
		case 227:measureValue = 0.1;break;	// 11100011
		case 231:measureValue = 0;break;	// 11100111
		case 199:measureValue = -0.1;break;	// 11000111
		case 239:measureValue = -0.1;break;	// 11101111
		case 207:measureValue = -0.5;break;	// 11001111
		case 223:measureValue = -1;break;	// 11011111
		case 159:measureValue = -1.5;break;	// 10011111
		case 191:measureValue = -2;break;	// 10111111
		case 63:measureValue = -2.5;break;	// 00111111
		case 127:measureValue = -3;break;	// 01111111
		default:measureValue = measureValue;
	}
}

float pidOut = 0;

// 6.编写TIMA ISR
void TA2_0_IRQHandler(void)
{
	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	/*开始填充用户代码*/
	// MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
	read_sensor_values();
	//printf("PID error value is %f\r\n", measureValue);
  PID_Update(&trackPid, 0, measureValue);
	//printf("PID output value is %d\r\n", (int) trackPid.out);
	TimA1_4_PWM_set_angle(trackPid.out+90);
	/*结束填充用户代码*/
}

/*********************************************************************************************************/

