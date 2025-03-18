/*
 * mcu_mapping.c
 *
 *  Created on: 22 apr. 2024
 *      Author: Ludo
 */

#include "mcu_mapping.h"

#include "adc.h"
#include "gpio.h"
#include "lpuart.h"
#include "usart.h"

/*** MCU MAPPING local global variables ***/

// Analog inputs.
static const GPIO_pin_t GPIO_ADC_VRS_MEASURE = { GPIOA, 0, 4, 0 };
static const GPIO_pin_t GPIO_ADC_VUSB_MEASURE = { GPIOA, 0, 5, 0 };
// Analog inputs list.
static const GPIO_pin_t* const GPIO_ADC_PINS_LIST[ADC_CHANNEL_INDEX_LAST] = { &GPIO_ADC_VRS_MEASURE, &GPIO_ADC_VUSB_MEASURE };
// LPUART1.
static const GPIO_pin_t GPIO_LPUART1_TX = { GPIOA, 0, 2, 6 };
static const GPIO_pin_t GPIO_LPUART1_RX = { GPIOA, 0, 3, 6 };
static const GPIO_pin_t GPIO_LPUART1_DE = { GPIOB, 1, 1, 4 };
static const GPIO_pin_t GPIO_LPUART1_NRE = { GPIOA, 0, 1, 0 };
// USART2.
static const GPIO_pin_t GPIO_USART2_TX = { GPIOA, 0, 9, 4 };
static const GPIO_pin_t GPIO_USART2_RX = { GPIOA, 0, 10, 4 };

/*** MCU MAPPING global variables ***/

// Analog inputs.
#ifdef HW1_1
const GPIO_pin_t GPIO_MNTR_EN = { GPIOA, 0, 0, 0 };
#endif
const ADC_gpio_t ADC_GPIO = { (const GPIO_pin_t**) &GPIO_ADC_PINS_LIST, ADC_CHANNEL_INDEX_LAST };
// VRS power control.
#ifdef HW1_0
const GPIO_pin_t GPIO_RS_POWER_ENABLE = { GPIOA, 0, 0, 0 };
#endif
// RS485 interface.
const LPUART_gpio_t LPUART_GPIO_RS485 = { &GPIO_LPUART1_TX, &GPIO_LPUART1_RX, &GPIO_LPUART1_DE, &GPIO_LPUART1_NRE };
// AT interface.
const USART_gpio_t USART_GPIO_AT = { &GPIO_USART2_TX, &GPIO_USART2_RX };
// Interface mode.
const GPIO_pin_t GPIO_MODE0 = { GPIOA, 0, 6, 0 };
const GPIO_pin_t GPIO_MODE1 = { GPIOA, 0, 7, 0 };
