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
#include "nvic.h"
#include "nvm.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Applicative.
#include "at.h"
#include "error.h"

/*** MAIN local structures ***/

typedef union {
	struct {
		unsigned lse_status : 1;
		unsigned lsi_status : 1;
		unsigned interface_mode : 2;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
	uint8_t all;
} DIM_status_t;

// Device context.
typedef struct {
	// Global.
	DIM_status_t status;
	// Clocks.
	uint32_t lsi_frequency_hz;
	uint8_t lse_running;
} DIM_context_t;

/*** MAIN local global variables ***/

static DIM_context_t dim_ctx;

/*** MAIN local functions ***/

/* COMMON INIT FUNCTION FOR MAIN CONTEXT.
 * @param:	None.
 * @return:	None.
 */
static void DIM_init_context(void) {
	// Init context.
	dim_ctx.lsi_frequency_hz = 0;
	dim_ctx.lse_running = 0;
	dim_ctx.status.all = 0;
}

/* COMMON INIT FUNCTION FOR PERIPHERALS AND COMPONENTS.
 * @param:	None.
 * @return:	None.
 */
static void DIM_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	RS485_address_t node_address;
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
	rcc_status = RCC_enable_lsi();
	RCC_error_check();
	dim_ctx.status.lsi_status = (rcc_status == RCC_SUCCESS) ? 1 : 0;
	rcc_status = RCC_enable_lse();
	RCC_error_check();
	dim_ctx.status.lse_status = (rcc_status == RCC_SUCCESS) ? 1 : 0;
	// Start independent watchdog.
#ifndef DEBUG
	iwdg_status = IWDG_init();
	IWDG_error_check();
#endif
	// High speed oscillator.
	IWDG_reload();
	rcc_status = RCC_switch_to_hsi();
	RCC_error_check();
	// Get LSI effective frequency (must be called after HSI initialization and before RTC inititialization).
	rcc_status = RCC_get_lsi_frequency(&dim_ctx.lsi_frequency_hz);
	RCC_error_check();
	if (rcc_status != RCC_SUCCESS) dim_ctx.lsi_frequency_hz = RCC_LSI_FREQUENCY_HZ;
	IWDG_reload();
	// RTC.
	dim_ctx.lse_running = dim_ctx.status.lse_status;
	rtc_status = RTC_init(&dim_ctx.lse_running, dim_ctx.lsi_frequency_hz);
	RTC_error_check();
	// Update LSE status if RTC failed to start on it.
	if (dim_ctx.lse_running == 0) {
		dim_ctx.status.lse_status = 0;
	}
	IWDG_reload();
	// Read RS485 address in NVM.
	nvm_status = NVM_read_byte(NVM_ADDRESS_RS485_ADDRESS, &node_address);
	NVM_error_check();
	// Init peripherals.
	LPTIM1_init(dim_ctx.lsi_frequency_hz);
	adc1_status = ADC1_init();
	ADC1_error_check();
	lpuart1_status = LPUART1_init(node_address);
	LPUART1_error_check();
	USART2_init();
	// Init AT interface.
	AT_init();
}

/*** MAIN function ***/

/* MAIN FUNCTION.
 * @param:	None.
 * @return:	None.
 */
int main(void) {
	// Init board.
	DIM_init_context();
	DIM_init_hw();
	// Start periodic wakeup timer.
	RTC_start_wakeup_timer(RTC_WAKEUP_PERIOD_SECONDS);
	// Main loop.
	while (1) {
		// Enter sleep mode.
		PWR_enter_sleep_mode();
		// Wake-up.
		AT_task();
		IWDG_reload();
	}
}
