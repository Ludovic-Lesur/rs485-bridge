/*
 * mapping.h
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#ifndef __MAPPING_H__
#define __MAPPING_H__

#include "gpio.h"

/*** MAPPING global variables ***/

#ifdef HW1_0
// VRS control.
extern const GPIO_pin_t GPIO_RS_POWER_ENABLE;
#endif
#ifdef HW1_1
// Monitoring control.
extern const GPIO_pin_t GPIO_MNTR_EN;
#endif
// LPUART1 (RS485 side).
extern const GPIO_pin_t GPIO_LPUART1_TX;
extern const GPIO_pin_t GPIO_LPUART1_RX;
extern const GPIO_pin_t GPIO_LPUART1_DE;
extern const GPIO_pin_t GPIO_LPUART1_NRE;
// ADC inputs.
extern const GPIO_pin_t GPIO_ADC1_IN4;
extern const GPIO_pin_t GPIO_ADC1_IN5;
// Interface mode.
extern const GPIO_pin_t GPIO_MODE0;
extern const GPIO_pin_t GPIO_MODE1;
// USART (USB side).
extern const GPIO_pin_t GPIO_USART2_TX;
extern const GPIO_pin_t GPIO_USART2_RX;
// Programming.
extern const GPIO_pin_t GPIO_SWDIO;
extern const GPIO_pin_t GPIO_SWCLK;

#endif /* __MAPPING_H__ */
