/*
 * led.h
 *
 *  Created on: 22 aug 2020
 *      Author: Ludo
 */

#ifndef __LED_H__
#define __LED_H__

#include "error.h"
#include "tim.h"
#include "types.h"

/*** LED structures ***/

/*!******************************************************************
 * \enum LED_status_t
 * \brief LED driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    LED_SUCCESS,
    LED_ERROR_NULL_DURATION,
    LED_ERROR_COLOR,
    // Low level drivers errors.
    LED_ERROR_BASE_TIM_OPM = ERROR_BASE_STEP,
    // Last base value.
    LED_ERROR_BASE_LAST = (LED_ERROR_BASE_TIM_OPM + TIM_ERROR_BASE_LAST)
} LED_status_t;

#ifdef RS485_BRIDGE

/*!******************************************************************
 * \enum LED_color_t
 * \brief LED colors list.
 *******************************************************************/
typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_YELLOW,
    LED_COLOR_BLUE,
    LED_COLOR_MAGENTA,
    LED_COLOR_CYAN,
    LED_COLOR_WHITE,
    LED_COLOR_LAST
} LED_color_t;

/*!******************************************************************
 * \enum LED_state_t
 * \brief LED states list.
 *******************************************************************/
typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_ACTIVE,
    LED_STATE_LAST
} LED_state_t;

/*** LED functions ***/

/*!******************************************************************
 * \fn LED_status_t LED_init(void)
 * \brief Init LED driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_init(void);

/*!******************************************************************
 * \fn LED_status_t LED_de_init(void)
 * \brief Release LED driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_de_init(void);

/*!******************************************************************
 * \fn LED_status_t LED_single_pulse(uint32_t pulse_duration_ms, LED_color_t color)
 * \brief Start single pulse.
 * \param[in]   pulse_duration_ms: Pulse duration in ms.
 * \param[in]   color: LED color.
 * \param[in]   pulse_completion_event: Enable internal interrupt at the end of the pulse.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_single_pulse(uint32_t pulse_duration_ms, LED_color_t color, uint8_t pulse_completion_event);

/*!******************************************************************
 * \fn LED_state_t LED_get_state(void)
 * \brief Read LED driver state.
 * \param[in]   none
 * \param[out]  none
 * \retval      LED driver state.
 *******************************************************************/
LED_state_t LED_get_state(void);

/*******************************************************************/
#define LED_exit_error(base) { ERROR_check_exit(led_status, LED_SUCCESS, base) }

/*******************************************************************/
#define LED_stack_error(base) { ERROR_check_stack(led_status, LED_SUCCESS, base) }

/*******************************************************************/
#define LED_stack_exit_error(base, code) { ERROR_check_stack_exit(led_status, LED_SUCCESS, base, code) }

#endif /* RS485_BRIDGE */

#endif /* __LED_H__ */
