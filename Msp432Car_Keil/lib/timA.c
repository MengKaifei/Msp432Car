/****************************************************/
// MSP432P401R
// 定时器A
/****************************************************/

#include "timA.h"
#include <stdio.h>
#include <stdlib.h>
#include "delay.h"
#include <string.h>

#define DEFINE_SERVO 90    //初始舵机角度

int sensor[8] = {0, 0, 0, 0, 0,0,0,0};    // 8个传感器数值的数组

/***********************************<<<   PID设置   >>>************************************/
// 小车循迹板循迹转向PID
float measureValue = 0.0;
#define PID_KP  10.0f
#define PID_KI  0.0f
#define PID_KD  0.4f

#define PID_TAU 0.02f

#define PID_LIM_MIN -60.0f
#define PID_LIM_MAX  60.0f

#define PID_LIM_MIN_INT 0.0f
#define PID_LIM_MAX_INT 0.0f

#define SAMPLE_TIME_S 0.01f
PID trackPid = {PID_KP, PID_KI, PID_KD,
                PID_TAU,
                PID_LIM_MIN, PID_LIM_MAX,
                PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                SAMPLE_TIME_S};												
			
// 小车超声波跟随控距PID
static float usValue = 0.0;												
#define DC_PID_KP  5.0f
#define DC_PID_KI  0.0f
#define DC_PID_KD  0.0f

#define DC_PID_TAU 0.02f

#define DC_PID_LIM_MIN -100.0f
#define DC_PID_LIM_MAX 0.0f

#define DC_PID_LIM_MIN_INT 0.0f
#define DC_PID_LIM_MAX_INT 0.0f

#define DC_SAMPLE_TIME_S 0.01f
PID trackPid_DC = {DC_PID_KP, DC_PID_KI, DC_PID_KD,
                DC_PID_TAU,
                DC_PID_LIM_MIN, DC_PID_LIM_MAX,
                DC_PID_LIM_MIN_INT, DC_PID_LIM_MAX_INT,
                DC_SAMPLE_TIME_S};	
					
/***********************************<<<   I2C设置   >>>************************************/
/* Slave Address for I2C Slave */
#define SLAVE_ADDRESS 0x70
#define NUM_OF_REC_BYTES 10
/* Variables */
const uint8_t TXData[] = "hello world!";
static uint8_t RXData[NUM_OF_REC_BYTES];
static volatile uint32_t xferIndex;
static volatile bool stopSent;
/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS ,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};
void i2c_init()
{
		/* Select Port 1 for I2C - Set Pin 6, 7 to input Primary Module Function,
     *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
     */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN6 + GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
		MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN6);
		MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN7);
    stopSent = false;
    memset(RXData, 0x00, NUM_OF_REC_BYTES);

    /* Initializing I2C Master to SMCLK at 100khz with no autostop */
    MAP_I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);

    /* Specify slave address */
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_ADDRESS);

    /* Set Master in transmit mode */
    MAP_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

    /* Enable I2C Module to start operations */
    MAP_I2C_enableModule(EUSCI_B0_BASE);

    /* Enable and clear the interrupt flag */
    MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE,
            EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0);
		
		MAP_I2C_enableInterrupt(EUSCI_B0_BASE,
            EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIB0);
		
		MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, TXData[0]);

//		for (int i =0; i < strlen((char *)TXData); i++)
//		{
//			MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, TXData[i]);
//		}
		
}

static uint8_t TXByteCtr = 1;

void EUSCIB0_IRQHandler(void)
{
    uint_fast16_t status;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B0_BASE);
    MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE, status);

    if (status & EUSCI_B_I2C_NAK_INTERRUPT)
    {
        MAP_I2C_masterSendStart(EUSCI_B0_BASE);
        return;
    }

    /* Check the byte counter */
    if (TXByteCtr)
    {
        /* Send the next data and decrement the byte counter */
        MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, TXData[1]);
			TXByteCtr = 0;
    }
    else
    {
        MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
    }

}

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
int CCR0=0; 

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
	PID_Init(&trackPid_DC);

	// 超声波模块初始化
	HCSR04Init();

	CCR0 = ccr0;
}


void HCSR04Init(void)
{
//	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
//	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
  MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);//tring
	MAP_GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN2); 
	
  MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P3, GPIO_PIN2);
    
	MAP_GPIO_disableInterrupt(GPIO_PORT_P3, GPIO_PIN3);
  MAP_GPIO_disableInterrupt(GPIO_PORT_P3, GPIO_PIN2);

}

// 获取超声波模块距离值
float Distance(void)
{
	int count=0,count2=0;
	float distance=0;

	 	GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3);//上拉
		GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN3);//上拉
		delay_us(10);
		GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3);//上拉
		while(GPIO_getInputPinValue(GPIO_PORT_P3,GPIO_PIN2) == 0){};
		count=MAP_Timer_A_getCounterValue(TIMER_A2_BASE);
		while(GPIO_getInputPinValue(GPIO_PORT_P3,GPIO_PIN2) == 1){};
		count2=MAP_Timer_A_getCounterValue(TIMER_A2_BASE);
		if(count > count2){
			count= CCR0 - count + count2;
		}else{
			count = count2-count;
		}
		//v = 340m/s = 34000cm/s = 34000cm/10^6us = 0.034cm/us
		//s = vt/2 = t*0.034/2 = t*0.017 ?t/58	
		distance=((float)count / 58.0);	
	
		return distance;
}

/* 获取循迹偏移值 */
void read_sensor_values()
{
	uint8_t res = TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_1) << 7 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_2) << 6 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_3) << 5 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_4) << 4
								|TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_5) << 3 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_6) << 2 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_7) << 1 | TRACK_PAD_READ_PIN(TRACK_PAD_CHANNEL_8);
	//printf("%d\r\n", res);
	switch(res){
		case 254:measureValue = 4;TimA0_4_PWM_set_duty(80);break;	// 11111110
		case 252:measureValue = 3.3;TimA0_4_PWM_set_duty(80);break;	// 11111100
		case 253:measureValue = 2;TimA0_4_PWM_set_duty(90);break;	// 11111101
		case 249:measureValue = 1.5;TimA0_4_PWM_set_duty(90);break;	// 11111001
		case 251:measureValue = 1;TimA0_4_PWM_set_duty(90);break;	// 11111011
		case 243:measureValue = 0.5;TimA0_4_PWM_set_duty(100);break;	// 11110011
		case 247:measureValue = 0.1;TimA0_4_PWM_set_duty(100);break;	// 1110111
		case 227:measureValue = 0.1;TimA0_4_PWM_set_duty(100);break;	// 11100011
		case 231:measureValue = 0;TimA0_4_PWM_set_duty(100);break;	// 11100111
		case 199:measureValue = -0.1;TimA0_4_PWM_set_duty(100);break;	// 11000111
		case 239:measureValue = -0.1;TimA0_4_PWM_set_duty(100);break;	// 11101111
		case 207:measureValue = -0.5;TimA0_4_PWM_set_duty(100);break;	// 11001111
		case 223:measureValue = -1;TimA0_4_PWM_set_duty(90);break;	// 11011111
		case 159:measureValue = -1.5;TimA0_4_PWM_set_duty(90);break;	// 10011111
		case 191:measureValue = -2;TimA0_4_PWM_set_duty(90);break;	// 10111111
		case 63:measureValue = -3.3;TimA0_4_PWM_set_duty(80);break;	// 00111111
		case 127:measureValue = -4;TimA0_4_PWM_set_duty(80);break;	// 01111111
		default:measureValue = measureValue;
	}
}

/* 获取超声波距离偏移值 */
void read_us_values()
{
	usValue = Distance();
}

float pidOut = 0;

// 6.编写TIMA ISR
void TA2_0_IRQHandler(void)
{
	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	/*开始填充用户代码*/
	read_sensor_values();	// 获取循迹PID误差值
  PID_Update(&trackPid, 0, measureValue);	// 进行PID矫正
	TimA1_4_PWM_set_angle(trackPid.out+90);	// 执行PID舵机角度转向修正
	
	read_us_values();	// 获取超声波测距PID误差值
	// 判断小车是否为跟随模式，是则超声波PID控制介入，否则电机按照默认值转动
	if (usValue < 50.0){
		PID_Update(&trackPid_DC, 20, usValue);	// 进行PID矫正
		TimA0_4_PWM_set_duty(-trackPid_DC.out);	// 执行PID电机转空比修正
	}
	else{
		TimA0_4_PWM_set_duty(50);
	}
	/*结束填充用户代码*/
}

/*********************************************************************************************************/

