/*
 * power.c
 *
 *  Created on: 22 jul. 2023
 *      Author: Ludo
 */

#include "power.h"

#include "analog.h"
#include "error.h"
#include "gpio.h"
#include "gpio_mapping.h"
#include "lptim.h"
#include "types.h"

/*** POWER local global variables ***/

static uint8_t power_domain_state[POWER_DOMAIN_LAST];

/*** POWER functions ***/

/*******************************************************************/
void POWER_init(void) {
	// Local variables.
	POWER_domain_t domain = 0;
	// Init power control pins.
#ifdef HW1_0
	GPIO_configure(&GPIO_RS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
#ifdef HW1_1
	GPIO_configure(&GPIO_MNTR_EN, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Disable all domains by default.
	for (domain=0 ; domain<POWER_DOMAIN_LAST ; domain++) {
		POWER_disable(domain);
	}
}

/*******************************************************************/
POWER_status_t POWER_enable(POWER_domain_t domain, LPTIM_delay_mode_t delay_mode) {
	// Local variables.
	POWER_status_t status = POWER_SUCCESS;
	ANALOG_status_t analog_status = ANALOG_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
	uint32_t delay_ms = 0;
	// Check domain.
	switch (domain) {
	case POWER_DOMAIN_ANALOG:
		// Enable voltage dividers and init ADC.
#ifdef HW1_1
		GPIO_write(&GPIO_MNTR_EN, 1);
#endif
		analog_status = ANALOG_init();
		ANALOG_exit_error(POWER_ERROR_BASE_ANALOG);
		delay_ms = POWER_ON_DELAY_MS_ANALOG;
		break;
#ifdef HW1_0
	case POWER_DOMAIN_RS485:
		// Turn RS power supply on.
		GPIO_write(&GPIO_RS_POWER_ENABLE, 1);
		delay_ms = POWER_ON_DELAY_MS_RS485;
		break;
#endif
	default:
		status = POWER_ERROR_DOMAIN;
		goto errors;
	}
	// Update state.
	power_domain_state[domain] = 1;
	// Power on delay.
	if (delay_ms != 0) {
		lptim_status = LPTIM_delay_milliseconds(delay_ms, delay_mode);
		LPTIM_exit_error(POWER_ERROR_BASE_LPTIM);
	}
errors:
	return status;
}

/*******************************************************************/
POWER_status_t POWER_disable(POWER_domain_t domain) {
	// Local variables.
	POWER_status_t status = POWER_SUCCESS;
	ANALOG_status_t analog_status = ANALOG_SUCCESS;
	// Check domain.
	switch (domain) {
	case POWER_DOMAIN_ANALOG:
		// Disable voltage dividers and release ADC.
	    analog_status = ANALOG_de_init();
#ifdef HW1_1
		GPIO_write(&GPIO_MNTR_EN, 0);
#endif
		ANALOG_exit_error(POWER_ERROR_BASE_ANALOG);
		break;
#ifdef HW1_0
	case POWER_DOMAIN_RS485:
		// Turn RS power supply off.
		GPIO_write(&GPIO_RS_POWER_ENABLE, 0);
		break;
#endif
	default:
		status = POWER_ERROR_DOMAIN;
		goto errors;
	}
	// Update state.
	power_domain_state[domain] = 0;
errors:
	return status;
}

/*******************************************************************/
POWER_status_t POWER_get_state(POWER_domain_t domain, uint8_t* state) {
	// Local variables.
	POWER_status_t status = POWER_SUCCESS;
	// Check parameters.
	if (domain >= POWER_DOMAIN_LAST) {
		status = POWER_ERROR_DOMAIN;
		goto errors;
	}
	if (state == NULL) {
		status = POWER_ERROR_NULL_PARAMETER;
		goto errors;
	}
	(*state) = power_domain_state[domain];
errors:
	return status;
}
