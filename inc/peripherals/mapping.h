/*
 * mapping.h
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#ifndef __MAPPING_H__
#define __MAPPING_H__

#include "gpio.h"
#include "gpio_reg.h"

// VRS control.
static const GPIO_pin_t GPIO_RS_POWER_ENABLE =	(GPIO_pin_t) {GPIOA, 0, 0, 0};
// LPUART1 (RS485 side).
static const GPIO_pin_t GPIO_LPUART1_TX =		(GPIO_pin_t) {GPIOA, 0, 2, 6}; // AF6 = LPUART1_TX.
static const GPIO_pin_t GPIO_LPUART1_RX =		(GPIO_pin_t) {GPIOA, 0, 3, 6}; // AF6 = LPUART1_RX.
static const GPIO_pin_t GPIO_LPUART1_DE =		(GPIO_pin_t) {GPIOB, 1, 1, 4}; // AF4 = LPUART1_DE.
static const GPIO_pin_t GPIO_LPUART1_NRE =		(GPIO_pin_t) {GPIOA, 0, 1, 0};
// ADC inputs.
static const GPIO_pin_t GPIO_ADC1_IN4 =			(GPIO_pin_t) {GPIOA, 0, 4, 0};
static const GPIO_pin_t GPIO_ADC1_IN5 =			(GPIO_pin_t) {GPIOA, 0, 5, 0};
// Interface mode.
static const GPIO_pin_t GPIO_MODE0 =			(GPIO_pin_t) {GPIOA, 0, 6, 0};
static const GPIO_pin_t GPIO_MODE1 =			(GPIO_pin_t) {GPIOA, 0, 7, 0};
// USART (USB side).
static const GPIO_pin_t GPIO_USART2_TX =		(GPIO_pin_t) {GPIOA, 0, 9, 4}; // AF4 = USART2_TX.
static const GPIO_pin_t GPIO_USART2_RX =		(GPIO_pin_t) {GPIOA, 0, 10, 4}; // AF4 = USART2_RX.
// Programming.
static const GPIO_pin_t GPIO_SWDIO =			(GPIO_pin_t) {GPIOA, 0, 13, 0};
static const GPIO_pin_t GPIO_SWCLK =			(GPIO_pin_t) {GPIOA, 0, 14, 0};

#endif /* __MAPPING_H__ */
