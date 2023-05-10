//
// Created by 22347 on 2023-04-24.
//

#include "key.h"

void key_Init(void){
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1 | GPIO_PIN4);
}

int key_Value(void){
    if (KEY1 == 0){
        delay_ms(10);
        return KEY1_PRES;
    }else if (KEY2 == 0){
        delay_ms(10);
        return KEY2_PRES;
    }else
        return 0;
}
