/*
 * mcu_mapping.h
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#ifndef __MCU_MAPPING_H__
#define __MCU_MAPPING_H__

#include "adc.h"
#include "gpio.h"
#include "lpuart.h"
#include "tim.h"
#include "usart.h"

/*** MCU MAPPING macros ***/

#ifdef DIM
#define ADC_CHANNEL_VRS         ADC_CHANNEL_IN4
#define ADC_CHANNEL_VUSB        ADC_CHANNEL_IN5
#endif
#ifdef RS485_BRIDGE
#define ADC_INSTANCE_VMCU_TMCU  ADC_INSTANCE_ADC1
#define ADC_INSTANCE_GPIO       ADC_INSTANCE_ADC2
#define ADC_CHANNEL_VRS         ADC_CHANNEL_IN3
#define ADC_CHANNEL_VUSB        ADC_CHANNEL_IN4
#endif

#ifdef DIM
#define USART_INSTANCE_AT       USART_INSTANCE_USART2
#endif
#ifdef RS485_BRIDGE
#define USART_INSTANCE_AT       USART_INSTANCE_USART1
#endif

#ifdef RS485_BRIDGE
#define USART_INSTANCE_RS485    USART_INSTANCE_USART2
#endif

#ifdef RS485_BRIDGE
#define TIM_INSTANCE_LED        TIM_INSTANCE_TIM3
#define TIM_CHANNEL_LED_RED     TIM_CHANNEL_1
#define TIM_CHANNEL_LED_GREEN   TIM_CHANNEL_2
#define TIM_CHANNEL_LED_BLUE    TIM_CHANNEL_4
#endif

/*** MCU MAPPING structures ***/

/*!******************************************************************
 * \enum ADC_channel_index_t
 * \brief ADC channels index.
 *******************************************************************/
typedef enum {
    ADC_CHANNEL_INDEX_VRS = 0,
    ADC_CHANNEL_INDEX_VUSB,
    ADC_CHANNEL_INDEX_LAST
} ADC_channel_index_t;

#ifdef RS485_BRIDGE
/*!******************************************************************
 * \enum TIM_channel_channel_led_t
 * \brief TIM LED channels index.
 *******************************************************************/
typedef enum {
    TIM_CHANNEL_INDEX_LED_RED = 0,
    TIM_CHANNEL_INDEX_LED_GREEN,
    TIM_CHANNEL_INDEX_LED_BLUE,
    TIM_CHANNEL_INDEX_LED_LAST
} TIM_channel_index_led_t;
#endif

/*** MCU MAPPING global variables ***/

#ifdef RS485_BRIDGE
// TCXO power control
extern const GPIO_pin_t GPIO_TCXO_POWER_ENABLE;
#endif
// Analog inputs.
#if (((defined DIM) && (defined HW1_1)) || (defined RS485_BRIDGE))
extern const GPIO_pin_t GPIO_MNTR_EN;
#endif
extern const ADC_gpio_t ADC_GPIO;
// VRS power control.
#if (((defined DIM) && (defined HW1_0)) || (defined RS485_BRIDGE))
extern const GPIO_pin_t GPIO_RS_POWER_ENABLE;
#endif
// RS485 interface.
#ifdef RS485_BRIDGE
extern const GPIO_pin_t GPIO_TRX_POWER_ENABLE;
extern const GPIO_pin_t GPIO_BUS_ENABLE;
#endif
#ifdef DIM
extern const LPUART_gpio_t LPUART_GPIO_RS485;
#endif
#ifdef RS485_BRIDGE
extern const USART_gpio_t USART_GPIO_RS485;
#endif
// AT interface.
extern const USART_gpio_t USART_GPIO_AT;
// Board mode.
extern const GPIO_pin_t GPIO_MODE0;
extern const GPIO_pin_t GPIO_MODE1;
#ifdef RS485_BRIDGE
// RGB LED.
extern const TIM_gpio_t TIM_GPIO_LED;
#endif

#endif /* __MCU_MAPPING_H__ */
