/*
 * main.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

// Peripherals.
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "mapping.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Utils.
#include "types.h"
// Components.
#include "power.h"
// Applicative.
#include "at_usb.h"
#include "error.h"

/*** MAIN local functions ***/

/*******************************************************************/
static void DIM_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
#ifndef DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	// Init power module and clock tree.
	PWR_init();
	RCC_init();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
#ifndef DEBUG
	// Start independent watchdog.
	iwdg_status = IWDG_init();
	IWDG_stack_error();
	IWDG_reload();
#endif
	// High speed oscillator.
	rcc_status = RCC_switch_to_hsi();
	RCC_stack_error();
	// Calibrate clocks.
	rcc_status = RCC_calibrate();
	RCC_stack_error();
	// Init RTC.
	rtc_status = RTC_init();
	RTC_stack_error();
	// Init delay timer.
	LPTIM1_init();
	// Init components.
	POWER_init();
	// Init AT interface.
	AT_USB_init();
}

/*** MAIN function ***/

/*******************************************************************/
int main(void) {
	// Init board.
	DIM_init_hw();
	// Main loop.
	while (1) {
		// Enter sleep mode.
		IWDG_reload();
		PWR_enter_sleep_mode();
		IWDG_reload();
		// Wake-up.
		AT_USB_task();
	}
}
