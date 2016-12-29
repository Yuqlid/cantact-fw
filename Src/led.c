/*
 * led.c
 *
 *  Created on: 2016/12/29
 *      Author: Yuki
 */

#include "led.h"
#include "gpio.h"

static uint32_t led_laston = 0;
static uint32_t led_lastoff = 0;
/*
void led_on(Led_TypeDef led){
    if(led & LED0){
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin , GPIO_PIN_SET);
    }
    if(led & LED1){
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin , GPIO_PIN_SET);
    }
}
*/
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

// Attempt to turn on status LED
void led_on(void)
{
    // Make sure the LED has been off for at least LED_DURATION before turning on again
    // This prevents a solid status LED on a busy canbus
    if(led_laston == 0 && HAL_GetTick() - led_lastoff > LED_DURATION)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
        led_laston = HAL_GetTick();
    }
}

// Process time-based LED events
void led_process(void)
{
    // If LED has been on for long enough, turn it off
    if(led_laston > 0 && HAL_GetTick() - led_laston > LED_DURATION)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
        led_laston = 0;
        led_lastoff = HAL_GetTick();
    }
}
