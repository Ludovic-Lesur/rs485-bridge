/*
 * led.c
 *
 *  Created on: 22 aug. 2020
 *      Author: Ludo
 */

#include "led.h"

#include "error.h"
#include "error_base.h"
#include "maths.h"
#include "mcu_mapping.h"
#include "tim.h"
#include "types.h"

#ifdef RS485_BRIDGE

/*** LED functions ***/

/*******************************************************************/
LED_status_t LED_init(void) {
    // Local variables.
    LED_status_t status = LED_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Init timer.
    tim_status = TIM_OPM_init(TIM_INSTANCE_LED, (TIM_gpio_t*) &TIM_GPIO_LED);
    TIM_exit_error(LED_ERROR_BASE_TIM_OPM);
errors:
    return status;
}

/*******************************************************************/
LED_status_t LED_de_init(void) {
    // Local variables.
    LED_status_t status = LED_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    // Release timer.
    tim_status = TIM_OPM_de_init(TIM_INSTANCE_LED, (TIM_gpio_t*) &TIM_GPIO_LED);
    TIM_stack_error(ERROR_BASE_LED + LED_ERROR_BASE_TIM_OPM);
    return status;
}

/*******************************************************************/
LED_status_t LED_single_pulse(uint32_t pulse_duration_us, LED_color_t color, uint8_t pulse_completion_event) {
    // Local variables.
    LED_status_t status = LED_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    uint8_t channels_mask = 0;
    uint8_t idx = 0;
    // Check parameters.
    if (pulse_duration_us == 0) {
        status = LED_ERROR_NULL_DURATION;
        goto errors;
    }
    if (color >= LED_COLOR_LAST) {
        status = LED_ERROR_COLOR;
        goto errors;
    }
    // Make pulse on required channels.
    for (idx = 0; idx < TIM_CHANNEL_INDEX_LED_LAST; idx++) {
        // Apply color mask.
        channels_mask |= (((color >> idx) & 0x01) << ((TIM_GPIO_LED.list[idx])->channel));
    }
    // Make pulse on channel.
    tim_status = TIM_OPM_make_pulse(TIM_INSTANCE_LED, channels_mask, 0, pulse_duration_us, pulse_completion_event);
    TIM_exit_error(LED_ERROR_BASE_TIM_OPM);
errors:
    return status;
}

/*******************************************************************/
LED_state_t LED_get_state(void) {
    // Local variables.
    LED_state_t state = LED_STATE_OFF;
    TIM_status_t tim_status = TIM_SUCCESS;
    uint8_t pulse_is_done = 0;
    // Get status.
    tim_status = TIM_OPM_get_pulse_status(TIM_INSTANCE_LED, &pulse_is_done);
    TIM_stack_error(ERROR_BASE_LED + LED_ERROR_BASE_TIM_OPM);
    // Update state.
    state = (pulse_is_done == 0) ? LED_STATE_ACTIVE : LED_STATE_OFF;
    return state;
}

#endif /* RS485_BRIDGE */
