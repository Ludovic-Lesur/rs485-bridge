/*
 * dip_switch.c
 *
 *  Created on: 30 oct. 2022
 *      Author: Ludo
 */

#include "dip_switch.h"

#include "gpio.h"
#include "gpio_mapping.h"
#include "lptim.h"

/*** DIP SWITCH local macros ***/

#define DIP_SWITCH_GPIO_TX_MODE     GPIO_MODE0

/*** DIP SWITCH functions ***/

/*******************************************************************/
DIP_SWITCH_tx_mode_t DIP_SWITCH_get_tx_mode(void) {
	// Local variables.
    DIP_SWITCH_tx_mode_t tx_mode = DIP_SWITCH_TX_MODE_DISABLED;
	// Activate pull up.
	GPIO_configure(&DIP_SWITCH_GPIO_TX_MODE, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_UP);
	LPTIM_delay_milliseconds(100, 0);
	// Read GPIO.
	if (GPIO_read(&DIP_SWITCH_GPIO_TX_MODE) == 0) {
		tx_mode = DIP_SWITCH_TX_MODE_ENABLED;
	}
	// Disable pull-up.
	GPIO_configure(&DIP_SWITCH_GPIO_TX_MODE, GPIO_MODE_ANALOG, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Return mode.
	return tx_mode;
}
