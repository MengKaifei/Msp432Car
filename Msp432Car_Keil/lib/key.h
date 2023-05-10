//
// Created by 22347 on 2023-04-24.
//

#ifndef __KEY_H
#define __KEY_H

#include "driverlib.h"
#include "delay.h"

#define KEY1 GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN1)
#define KEY2 GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN4)

#define KEY1_PRES 1
#define KEY2_PRES 2

void key_Init(void);
int key_Value(void);


#endif
