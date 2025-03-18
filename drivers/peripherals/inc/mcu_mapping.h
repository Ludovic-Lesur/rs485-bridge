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
#include "usart.h"

/*** MCU MAPPING macros ***/

#define ADC_CHANNEL_VRS     ADC_CHANNEL_IN4
#define ADC_CHANNEL_VUSB    ADC_CHANNEL_IN5

#define USART_INSTANCE_AT   USART_INSTANCE_USART2

/*** MCU MAPPING structures ***/

/*!******************************************************************
 * \enum ADC_channel_index_t
 * \brief ADC channels index.
 *******************************************************************/
typedef enum {
    ADC_CHANNEL_INDEX_VRS = 0,
    ADC_CHANNEL_INDEX_VUSB,
    ADC_CHANNEL_INDEX_LAST
} ADC_channel_index_tt;

/*** MCU MAPPING global variables ***/

// Analog inputs.
#ifdef HW1_1
extern const GPIO_pin_t GPIO_MNTR_EN;
#endif
extern const ADC_gpio_t ADC_GPIO;
// VRS power control.
#ifdef HW1_0
extern const GPIO_pin_t GPIO_RS_POWER_ENABLE;
#endif
// RS485 interface.
extern const LPUART_gpio_t LPUART_GPIO_RS485;
// AT interface.
extern const USART_gpio_t USART_GPIO_AT;
// Board mode.
extern const GPIO_pin_t GPIO_MODE0;
extern const GPIO_pin_t GPIO_MODE1;

#endif /* __MCU_MAPPING_H__ */
