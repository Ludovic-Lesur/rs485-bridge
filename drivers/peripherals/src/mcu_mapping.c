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
#include "tim.h"
#include "usart.h"

/*** MCU MAPPING local global variables ***/

// Analog inputs.
#ifdef DIM
static const GPIO_pin_t GPIO_ADC_VRS_MEASURE = { GPIOA, 0, 4, 0 };
static const GPIO_pin_t GPIO_ADC_VUSB_MEASURE = { GPIOA, 0, 5, 0 };
#endif
#ifdef RS485_BRIDGE
static const GPIO_pin_t GPIO_ADC_VRS_MEASURE = { GPIOA, 0, 6, 0 };
static const GPIO_pin_t GPIO_ADC_VUSB_MEASURE = { GPIOA, 0, 7, 0 };
#endif
// Analog inputs list.
static const GPIO_pin_t* const GPIO_ADC_PINS_LIST[ADC_CHANNEL_INDEX_LAST] = { &GPIO_ADC_VRS_MEASURE, &GPIO_ADC_VUSB_MEASURE };
#ifdef DIM
// LPUART1.
static const GPIO_pin_t GPIO_LPUART1_TX = { GPIOA, 0, 2, 6 };
static const GPIO_pin_t GPIO_LPUART1_RX = { GPIOA, 0, 3, 6 };
static const GPIO_pin_t GPIO_LPUART1_DE = { GPIOB, 1, 1, 4 };
static const GPIO_pin_t GPIO_LPUART1_NRE = { GPIOA, 0, 1, 0 };
#endif
#ifdef RS485_BRIDGE
// USART2.
static const GPIO_pin_t GPIO_USART2_TX = { GPIOA, 0, 2, 7 };
static const GPIO_pin_t GPIO_USART2_RX = { GPIOA, 0, 3, 7 };
static const GPIO_pin_t GPIO_USART2_DE = { GPIOA, 0, 1, 7 };
static const GPIO_pin_t GPIO_USART2_NRE = { GPIOA, 0, 0, 0 };
#endif
#ifdef DIM
// USART2.
static const GPIO_pin_t GPIO_USART2_TX = { GPIOA, 0, 9, 4 };
static const GPIO_pin_t GPIO_USART2_RX = { GPIOA, 0, 10, 4 };
#endif
#ifdef RS485_BRIDGE
// USART1.
static const GPIO_pin_t GPIO_USART1_TX = { GPIOA, 0, 9, 7 };
static const GPIO_pin_t GPIO_USART1_RX = { GPIOA, 0, 10, 7 };
#endif
#ifdef RS485_BRIDGE
// RS485 bridge
static const GPIO_pin_t GPIO_LED_RED =   { GPIOB, 1, 4, 2 };
static const GPIO_pin_t GPIO_LED_GREEN = { GPIOB, 1, 5, 2 };
static const GPIO_pin_t GPIO_LED_BLUE =  { GPIOB, 1, 7, 10 };
// Timer channels.
static const TIM_channel_gpio_t TIM_CHANNEL_GPIO_LED_RED = { TIM_CHANNEL_LED_RED, &GPIO_LED_RED, TIM_POLARITY_ACTIVE_LOW };
static const TIM_channel_gpio_t TIM_CHANNEL_GPIO_LED_GREEN = { TIM_CHANNEL_LED_GREEN, &GPIO_LED_GREEN, TIM_POLARITY_ACTIVE_LOW };
static const TIM_channel_gpio_t TIM_CHANNEL_GPIO_LED_BLUE = { TIM_CHANNEL_LED_BLUE, &GPIO_LED_BLUE, TIM_POLARITY_ACTIVE_LOW };
// Timer pins list.
static const TIM_channel_gpio_t* const TIM_CHANNEL_GPIO_LIST_LED[TIM_CHANNEL_INDEX_LED_LAST] = { &TIM_CHANNEL_GPIO_LED_RED, &TIM_CHANNEL_GPIO_LED_GREEN, &TIM_CHANNEL_GPIO_LED_BLUE };
#endif

/*** MCU MAPPING global variables ***/

#ifdef RS485_BRIDGE
// TCXO power control
const GPIO_pin_t GPIO_TCXO_POWER_ENABLE = { GPIOA, 0, 15, 0 };
#endif
// Analog inputs.
#if ((defined DIM) && (defined HW1_1))
const GPIO_pin_t GPIO_MNTR_EN = { GPIOA, 0, 0, 0 };
#endif
#ifdef RS485_BRIDGE
const GPIO_pin_t GPIO_MNTR_EN = { GPIOB, 1, 0, 0 };
#endif
const ADC_gpio_t ADC_GPIO = { (const GPIO_pin_t**) &GPIO_ADC_PINS_LIST, ADC_CHANNEL_INDEX_LAST };
// VRS power control.
#if ((defined DIM) && (defined HW1_0))
const GPIO_pin_t GPIO_RS_POWER_ENABLE = { GPIOA, 0, 0, 0 };
#endif
#ifdef RS485_BRIDGE
const GPIO_pin_t GPIO_RS_POWER_ENABLE = { GPIOA, 0, 8, 0 };
#endif
// RS485 interface.
#ifdef RS485_BRIDGE
const GPIO_pin_t GPIO_TRX_POWER_ENABLE = { GPIOA, 0, 5, 0 };
const GPIO_pin_t GPIO_BUS_ENABLE = { GPIOA, 0, 4, 0 };
#endif
#ifdef DIM
const LPUART_gpio_t LPUART_GPIO_RS485 = { &GPIO_LPUART1_TX, &GPIO_LPUART1_RX, &GPIO_LPUART1_DE, &GPIO_LPUART1_NRE };
#endif
#ifdef RS485_BRIDGE
const USART_gpio_t USART_GPIO_RS485 = { &GPIO_USART2_TX, &GPIO_USART2_RX, &GPIO_USART2_DE, &GPIO_USART2_NRE };
#endif
// AT interface.
#ifdef DIM
const USART_gpio_t USART_GPIO_AT = { &GPIO_USART2_TX, &GPIO_USART2_RX };
#endif
#ifdef RS485_BRIDGE
const USART_gpio_t USART_GPIO_AT = { &GPIO_USART1_TX, &GPIO_USART1_RX, NULL, NULL };
#endif
// Interface mode.
#ifdef DIM
const GPIO_pin_t GPIO_MODE0 = { GPIOA, 0, 6, 0 };
const GPIO_pin_t GPIO_MODE1 = { GPIOA, 0, 7, 0 };
#endif
#ifdef RS485_BRIDGE
const GPIO_pin_t GPIO_MODE0 = { GPIOA, 0, 11, 0 };
const GPIO_pin_t GPIO_MODE1 = { GPIOA, 0, 12, 0 };
#endif
#ifdef RS485_BRIDGE
const TIM_gpio_t TIM_GPIO_LED = { (const TIM_channel_gpio_t**) &TIM_CHANNEL_GPIO_LIST_LED, TIM_CHANNEL_INDEX_LED_LAST };
#endif
