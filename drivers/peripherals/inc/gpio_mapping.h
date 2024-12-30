/*
 * gpio_mapping.h
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#ifndef __GPIO_MAPPING_H__
#define __GPIO_MAPPING_H__

#include "adc.h"
#include "gpio.h"
#include "lpuart.h"
#include "usart.h"

/*** GPIO MAPPING global variables ***/

// Analog inputs.
#ifdef HW1_1
extern const GPIO_pin_t GPIO_MNTR_EN;
#endif
extern const ADC_gpio_t GPIO_ADC;
// VRS power control.
#ifdef HW1_0
extern const GPIO_pin_t GPIO_RS_POWER_ENABLE;
#endif
// RS485 interface.
extern const LPUART_gpio_t GPIO_RS485_LPUART;
// AT interface.
extern const USART_gpio_t GPIO_AT_USART;
// Board mode.
extern const GPIO_pin_t GPIO_MODE0;
extern const GPIO_pin_t GPIO_MODE1;

#endif /* __GPIO_MAPPING_H__ */
