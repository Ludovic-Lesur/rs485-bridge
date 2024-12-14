/*
 * main.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

// Peripherals.
#include "exti.h"
#include "gpio.h"
#include "gpio_mapping.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvic_priority.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
#include "types.h"
// Utils.
#include "error.h"
// Middleware.
#include "cli.h"
#include "power.h"
// Applicative.
#include "error_base.h"

/*** MAIN local functions ***/

/*******************************************************************/
static void DIM_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	CLI_status_t cli_status = CLI_SUCCESS;
#ifndef DIM_MODE_DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	// Init power module and clock tree.
	PWR_init();
	RCC_init(NVIC_PRIORITY_CLOCK);
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
#ifndef DIM_MODE_DEBUG
	// Start independent watchdog.
	iwdg_status = IWDG_init();
	IWDG_stack_error(ERROR_BASE_IWDG);
	IWDG_reload();
#endif
	// High speed oscillator.
	rcc_status = RCC_switch_to_hsi();
	RCC_stack_error(ERROR_BASE_RCC);
	// Calibrate clocks.
	rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
	RCC_stack_error(ERROR_BASE_RCC);
	// Init RTC.
	rtc_status = RTC_init(NULL, NVIC_PRIORITY_RTC);
	RTC_stack_error(ERROR_BASE_RTC);
	// Init delay timer.
	LPTIM_init(NVIC_PRIORITY_DELAY);
	// Init components.
	POWER_init();
	// Init AT interface.
	cli_status = CLI_init();
	CLI_stack_error(ERROR_BASE_CLI);
}

/*** MAIN function ***/

/*******************************************************************/
int main(void) {
    // Local variables.
    CLI_status_t cli_status = CLI_SUCCESS;
	// Init board.
	DIM_init_hw();
	// Main loop.
	while (1) {
		// Enter sleep mode.
		IWDG_reload();
		PWR_enter_sleep_mode();
		IWDG_reload();
		// Wake-up.
		cli_status = CLI_process();
		CLI_stack_error(ERROR_BASE_CLI);
	}
}
