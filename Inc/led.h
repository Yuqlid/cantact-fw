/*
 * led.h
 *
 *  Created on: 2016/12/29
 *      Author: Yuki
 */

#ifndef LED_H_
#define LED_H_
#ifdef __cplusplus
 extern "C" {
#endif


typedef enum
{
  LED0 = 1 << 0,
  LED1 = 1 << 1,
  LED_MAX
} Led_TypeDef;

void led_on(Led_TypeDef led);
void led_off(Led_TypeDef led);
void led_toggle(Led_TypeDef led);

#ifdef __cplusplus
}
#endif
#endif /* LED_H_ */
