/*
 * gpio_mapping.c
 *
 *  Created on: 22 apr. 2024
 *      Author: Ludo
 */

#include "gpio_mapping.h"

#include "adc.h"
#include "gpio.h"
#include "lpuart.h"
#include "usart.h"

/*** GPIO MAPPING local structures ***/

/*!******************************************************************
 * \enum GPIO_adc_channel_t
 * \brief GPIO ADC channels list.
 *******************************************************************/
typedef enum {
    GPIO_ADC_CHANNEL_INDEX_VRS = 0,
    GPIO_ADC_CHANNEL_INDEX_VUSB,
    GPIO_ADC_CHANNEL_INDEX_LAST
} GPIO_adc_channel_index_tt;

/*** GPIO MAPPING local global variables ***/

// Analog inputs.
static const GPIO_pin_t GPIO_ADC_VRS_MEASURE = (GPIO_pin_t) { GPIOA, 0, 4, 0 };
static const GPIO_pin_t GPIO_ADC_VUSB_MEASURE = (GPIO_pin_t) { GPIOA, 0, 5, 0 };
// Analog inputs list.
static const GPIO_pin_t* GPIO_ADC_PINS_LIST[GPIO_ADC_CHANNEL_INDEX_LAST] = { &GPIO_ADC_VRS_MEASURE, &GPIO_ADC_VUSB_MEASURE };
// LPUART1.
static const GPIO_pin_t GPIO_LPUART1_TX = (GPIO_pin_t) { GPIOA, 0, 2, 6 };
static const GPIO_pin_t GPIO_LPUART1_RX = (GPIO_pin_t) { GPIOA, 0, 3, 6 };
static const GPIO_pin_t GPIO_LPUART1_DE = (GPIO_pin_t) { GPIOB, 1, 1, 4 };
static const GPIO_pin_t GPIO_LPUART1_NRE = (GPIO_pin_t) { GPIOA, 0, 1, 0 };
// USART2.
static const GPIO_pin_t GPIO_USART2_TX = (GPIO_pin_t) { GPIOA, 0, 9, 4 };
static const GPIO_pin_t GPIO_USART2_RX = (GPIO_pin_t) { GPIOA, 0, 10, 4 };

/*** GPIO MAPPING global variables ***/

// Analog inputs.
#ifdef HW1_1
const GPIO_pin_t GPIO_MNTR_EN = (GPIO_pin_t) {GPIOA, 0, 0, 0};
#endif
const ADC_gpio_t GPIO_ADC = { (const GPIO_pin_t**) &GPIO_ADC_PINS_LIST, GPIO_ADC_CHANNEL_INDEX_LAST };
// VRS power control.
#ifdef HW1_0
const GPIO_pin_t GPIO_RS_POWER_ENABLE = (GPIO_pin_t) { GPIOA, 0, 0, 0 };
#endif
// RS485 interface.
const LPUART_gpio_t GPIO_RS485_LPUART = { &GPIO_LPUART1_TX, &GPIO_LPUART1_RX, &GPIO_LPUART1_DE, &GPIO_LPUART1_NRE };
// AT interface.
const USART_gpio_t GPIO_AT_USART = { &GPIO_USART2_TX, &GPIO_USART2_RX };
// Interface mode.
const GPIO_pin_t GPIO_MODE0 = (GPIO_pin_t) { GPIOA, 0, 6, 0 };
const GPIO_pin_t GPIO_MODE1 = (GPIO_pin_t) { GPIOA, 0, 7, 0 };
