/*
 * mapping.c
 *
 *  Created on: 22 apr. 2024
 *      Author: Ludo
 */

#include "mapping.h"

#include "gpio.h"
#include "gpio_reg.h"

#ifdef HW1_0
// VRS control.
const GPIO_pin_t GPIO_RS_POWER_ENABLE =	(GPIO_pin_t) {GPIOA, 0, 0, 0};
#endif
#ifdef HW1_1
// Monitoring control.
const GPIO_pin_t GPIO_MNTR_EN =			(GPIO_pin_t) {GPIOA, 0, 0, 0};
#endif
// LPUART1 (RS485 side).
const GPIO_pin_t GPIO_LPUART1_TX =		(GPIO_pin_t) {GPIOA, 0, 2, 6}; // AF6 = LPUART1_TX.
const GPIO_pin_t GPIO_LPUART1_RX =		(GPIO_pin_t) {GPIOA, 0, 3, 6}; // AF6 = LPUART1_RX.
const GPIO_pin_t GPIO_LPUART1_DE =		(GPIO_pin_t) {GPIOB, 1, 1, 4}; // AF4 = LPUART1_DE.
const GPIO_pin_t GPIO_LPUART1_NRE =		(GPIO_pin_t) {GPIOA, 0, 1, 0};
// ADC inputs.
const GPIO_pin_t GPIO_ADC1_IN4 =		(GPIO_pin_t) {GPIOA, 0, 4, 0};
const GPIO_pin_t GPIO_ADC1_IN5 =		(GPIO_pin_t) {GPIOA, 0, 5, 0};
// Interface mode.
const GPIO_pin_t GPIO_MODE0 =			(GPIO_pin_t) {GPIOA, 0, 6, 0};
const GPIO_pin_t GPIO_MODE1 =			(GPIO_pin_t) {GPIOA, 0, 7, 0};
// USART (USB side).
const GPIO_pin_t GPIO_USART2_TX =		(GPIO_pin_t) {GPIOA, 0, 9, 4}; // AF4 = USART2_TX.
const GPIO_pin_t GPIO_USART2_RX =		(GPIO_pin_t) {GPIOA, 0, 10, 4}; // AF4 = USART2_RX.
// Programming.
const GPIO_pin_t GPIO_SWDIO =			(GPIO_pin_t) {GPIOA, 0, 13, 0};
const GPIO_pin_t GPIO_SWCLK =			(GPIO_pin_t) {GPIOA, 0, 14, 0};
