/*
 * power.c
 *
 *  Created on: 22 jul. 2023
 *      Author: Ludo
 */

#include "power.h"

#include "analog.h"
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "types.h"

/*** POWER local global variables ***/

static uint32_t power_domain_state[POWER_DOMAIN_LAST] = { [0 ... (POWER_DOMAIN_LAST - 1)] = 0 };

/*** POWER local functions ***/

/*******************************************************************/
#define _POWER_stack_driver_error(driver_status, driver_success, driver_error_base, power_status) { \
    if (driver_status != driver_success) { \
        ERROR_stack_add(driver_error_base + driver_status); \
        ERROR_stack_add(ERROR_BASE_POWER + power_status); \
    } \
}

/*** POWER functions ***/

/*******************************************************************/
void POWER_init(void) {
    // Local variables.
    uint8_t idx = 0;
    // Init context.
    for (idx = 0; idx < POWER_DOMAIN_LAST; idx++) {
        power_domain_state[idx] = 0;
    }
    // Init power control pins.
#if (((defined DIM) && (defined HW1_1)) || (defined RS485_BRIDGE))
    GPIO_configure(&GPIO_MNTR_EN, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
#ifdef RS485_BRIDGE
    GPIO_configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_TRX_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
#if (((defined DIM) && (defined HW1_0)) || (defined RS485_BRIDGE))
    GPIO_configure(&GPIO_RS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}

/*******************************************************************/
void POWER_enable(POWER_requester_id_t requester_id, POWER_domain_t domain, LPTIM_delay_mode_t delay_mode) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    uint32_t delay_ms = 0;
    uint8_t action_required = 0;
    // Check parameters.
    if (requester_id >= POWER_REQUESTER_ID_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_REQUESTER_ID);
        goto errors;
    }
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    action_required = ((power_domain_state[domain] == 0) ? 1 : 0);
    // Update state.
    power_domain_state[domain] |= (0b1 << requester_id);
    // Directly exit if this is not the first request.
    if (action_required == 0) goto errors;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
        // Enable voltage dividers and init ADC.
#if (((defined DIM) && (defined HW1_1)) || (defined RS485_BRIDGE))
        GPIO_write(&GPIO_MNTR_EN, 1);
        delay_ms = POWER_ON_DELAY_MS_ANALOG;
#endif
        // Init attached drivers.
        analog_status = ANALOG_init();
        _POWER_stack_driver_error(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG, POWER_ERROR_DRIVER_ANALOG);
        break;
#ifdef RS485_BRIDGE
    case POWER_DOMAIN_TCXO:
        GPIO_write(&GPIO_TCXO_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_TCXO;
        break;
    case POWER_DOMAIN_TRX:
        GPIO_write(&GPIO_TRX_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_TRX;
        break;
#endif
#if (((defined DIM) && (defined HW1_0)) || (defined RS485_BRIDGE))
    case POWER_DOMAIN_RS485:
        // Turn RS power supply on.
        GPIO_write(&GPIO_RS_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_RS485;
        break;
#endif
    default:
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    // Power on delay.
    if (delay_ms != 0) {
        lptim_status = LPTIM_delay_milliseconds(delay_ms, delay_mode);
        _POWER_stack_driver_error(lptim_status, LPTIM_SUCCESS, ERROR_BASE_LPTIM, POWER_ERROR_DRIVER_LPTIM);
    }
errors:
    return;
}

/*******************************************************************/
void POWER_disable(POWER_requester_id_t requester_id, POWER_domain_t domain) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    // Check parameters.
    if (requester_id >= POWER_REQUESTER_ID_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_REQUESTER_ID);
        goto errors;
    }
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    if (power_domain_state[domain] == 0) goto errors;
    // Update state.
    power_domain_state[domain] &= ~(0b1 << requester_id);
    // Directly exit if this is not the last request.
    if (power_domain_state[domain] != 0) goto errors;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
        // Release attached drivers.
        analog_status = ANALOG_de_init();
        _POWER_stack_driver_error(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG, POWER_ERROR_DRIVER_ANALOG);
        // Turn analog front-end off.
#if (((defined DIM) && (defined HW1_1)) || (defined RS485_BRIDGE))
        GPIO_write(&GPIO_MNTR_EN, 0);
#endif
        break;
#ifdef RS485_BRIDGE
    case POWER_DOMAIN_TCXO:
        GPIO_write(&GPIO_TCXO_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_TRX:
        GPIO_write(&GPIO_TRX_POWER_ENABLE, 0);
        break;
#endif
#if (((defined DIM) && (defined HW1_0)) || (defined RS485_BRIDGE))
    case POWER_DOMAIN_RS485:
        // Turn RS power supply off.
        GPIO_write(&GPIO_RS_POWER_ENABLE, 0);
        break;
#endif
    default:
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
errors:
    return;
}

/*******************************************************************/
uint8_t POWER_get_state(POWER_domain_t domain) {
    // Local variables.
    uint8_t state = 0;
    // Check parameters.
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    state = (power_domain_state[domain] == 0) ? 0 : 1;
errors:
    return state;
}
