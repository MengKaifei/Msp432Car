/****************************************************/
// MSP432P401R
// 定时器A
/****************************************************/

#ifndef __TIMA_H
#define __TIMA_H
#include <driverlib.h>
#include "PID.h"

extern float pidOut;

// 循迹板引脚定义
#define TRACK_PAD_CHANNEL_1 GPIO_PORT_P10,GPIO_PIN1
#define TRACK_PAD_CHANNEL_2 GPIO_PORT_P7,GPIO_PIN7
#define TRACK_PAD_CHANNEL_3 GPIO_PORT_P7,GPIO_PIN5
#define TRACK_PAD_CHANNEL_4 GPIO_PORT_P9,GPIO_PIN7
#define TRACK_PAD_CHANNEL_5 GPIO_PORT_P9,GPIO_PIN5
#define TRACK_PAD_CHANNEL_6 GPIO_PORT_P7,GPIO_PIN0
#define TRACK_PAD_CHANNEL_7 GPIO_PORT_P7,GPIO_PIN2
#define TRACK_PAD_CHANNEL_8 GPIO_PORT_P6,GPIO_PIN3

// 循迹板引脚初始化
#define TRACK_PAD_PIN_INIT()  \
do { \
		GPIO_setAsInputPin(TRACK_PAD_CHANNEL_1);\
    GPIO_setAsInputPin(TRACK_PAD_CHANNEL_2);\
    GPIO_setAsInputPin(TRACK_PAD_CHANNEL_3);\
    GPIO_setAsInputPin(TRACK_PAD_CHANNEL_4);\
    GPIO_setAsInputPin(TRACK_PAD_CHANNEL_5);\
    GPIO_setAsInputPin(TRACK_PAD_CHANNEL_6);\
    GPIO_setAsInputPin(TRACK_PAD_CHANNEL_7);\
    GPIO_setAsInputPin(TRACK_PAD_CHANNEL_8);\
}while(0)\

// 读取循迹板模块返回值
#define TRACK_PAD_READ_PIN(channel) GPIO_getInputPinValue(channel)

extern uint8_t TIMA2_CAP_STA;
extern uint16_t TIMA2_CAP_VAL;

// 定时器
void TimA2_Int_Init(uint16_t ccr0, uint16_t psc);
void TA2_0_IRQHandler(void);
float return_PIDErr(float outPut);

// 电机PWM
void TimA0_4_PWM_Init();
void TimA0_4_PWM_set_duty(int duty);
void TimA0_4_PWM_stop();

// 舵机PWM
void TimA1_4_PWM_Init();
void TimA1_4_PWM_set_angle(int angle);
void TimA1_4_PWM_stop();

// 超声波模块
void HCSR04Init(void);
float Distance(void);
void read_us_values();

void i2c_init();

#endif
