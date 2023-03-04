/*
 * main.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

// Peripherals.
#include "adc.h"
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "mapping.h"
#include "node.h"
#include "nvic.h"
#include "nvm.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Applicative.
#include "at_usb.h"
#include "error.h"

/*** MAIN local functions ***/

/* COMMON INIT FUNCTION FOR PERIPHERALS AND COMPONENTS.
 * @param:	None.
 * @return:	None.
 */
static void DIM_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
#ifndef DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	NVM_init();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
	// Init clock and power modules.
	RCC_init();
	PWR_init();
	// Reset RTC.
	RTC_reset();
	// Start oscillators.
	RCC_enable_lsi();
	// Start independent watchdog.
#ifndef DEBUG
	iwdg_status = IWDG_init();
	IWDG_error_check();
#endif
	// High speed oscillator.
	IWDG_reload();
	rcc_status = RCC_switch_to_hsi();
	RCC_error_check();
	// RTC.
	RTC_reset();
	RCC_enable_lse();
	rtc_status = RTC_init();
	RTC_error_check();
	// Init peripherals.
	LPTIM1_init();
	adc1_status = ADC1_init();
	ADC1_error_check();
	LPUART1_init();
	USART2_init();
	// Init node interface.
	NODE_init();
	// Init AT interface.
	AT_USB_init();
}

/*** MAIN function ***/

/* MAIN FUNCTION.
 * @param:	None.
 * @return:	None.
 */
int main(void) {
	// Init board.
	DIM_init_hw();
	// Start periodic wakeup timer.
	RTC_start_wakeup_timer(RTC_WAKEUP_PERIOD_SECONDS);
	// Main loop.
	while (1) {
		// Enter sleep mode.
		PWR_enter_sleep_mode();
		// Wake-up.
		AT_USB_task();
		IWDG_reload();
	}
}
