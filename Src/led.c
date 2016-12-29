/*
 * led.c
 *
 *  Created on: 2016/12/29
 *      Author: Yuki
 */

#include "led.h"
#include "gpio.h"

void led_on(Led_TypeDef led){
    if(led & LED0){
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin , GPIO_PIN_SET);
    }
    if(led & LED1){
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin , GPIO_PIN_SET);
    }
}
void led_off(Led_TypeDef led){
    if(led & LED0){
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin , GPIO_PIN_RESET);
    }
    if(led & LED1){
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin , GPIO_PIN_RESET);
    }
}
void led_toggle(Led_TypeDef led){
    if(led & LED0){
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }
    if(led & LED1){
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }

}
