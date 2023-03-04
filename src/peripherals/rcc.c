/*
 * rcc.c
 *
 *  Created on: 25 oct. 2020
 *      Author: Ludo
 */

#include "rcc.h"

#include "flash.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "types.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT				1000000
#define RCC_MSI_RESET_FREQUENCY_KHZ		2100

/*** RCC local global variables ***/

static uint32_t rcc_sysclk_khz;

/*** RCC local functions ***/

/* RCC INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void RCC_IRQHandler(void) {
	// Clear all flags.
	RCC -> CICR |= (0b11 << 0);
}

/*** RCC functions ***/

/* INIT RCC MODULE.
 * @param:	None.
 * @return:	None.
 */
void RCC_init(void) {
	// Default prescalers (HCLK, PCLK1 and PCLK2 must not exceed 32MHz).
	// HCLK = SYSCLK = 16MHz (HPRE='0000').
	// PCLK1 = HCLK = 16MHz (PPRE1='000').
	// PCLK2 = HCLK = 16MHz (PPRE2='000').
	// All peripherals clocked via the corresponding APBx line.
	// Reset clock is MSI 2.1MHz.
	rcc_sysclk_khz = RCC_MSI_RESET_FREQUENCY_KHZ;
	// Enable LSI and LSE ready interrupts.
	RCC -> CIER |= (0b11 << 0);
}

/* CONFIGURE AND USE HSI AS SYSTEM CLOCK (16MHz INTERNAL RC).
 * @param:			None.
 * @return status:	Function execution status.
 */
RCC_status_t RCC_switch_to_hsi(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
	uint32_t loop_count = 0;
	// Set flash latency.
	flash_status = FLASH_set_latency(1);
	FLASH_status_check(RCC_ERROR_BASE_FLASH);
	// Init HSI.
	RCC -> CR |= (0b1 << 0); // Enable HSI (HSI16ON='1').
	// Wait for HSI to be stable.
	while (((RCC -> CR) & (0b1 << 2)) == 0) {
		// Wait for HSIRDYF='1' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_READY;
			goto errors;
		}
	}
	// Switch SYSCLK.
	RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
	RCC -> CFGR |= (0b01 << 0); // Use HSI as system clock (SW='01').
	// Wait for clock switch.
	loop_count = 0;
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)) {
		// Wait for SWS='01' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_SWITCH;
			goto errors;
		}
	}
	// Disable MSI.
	RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
	// Update flag and frequency.
	rcc_sysclk_khz = RCC_HSI_FREQUENCY_KHZ;
errors:
	return status;
}

/* RETURN THE CURRENT SYSTEM CLOCK FREQUENCY.
 * @param:					None.
 * @return rcc_sysclk_khz:	Current system clock frequency in kHz.
 */
uint32_t RCC_get_sysclk_khz(void) {
	return rcc_sysclk_khz;
}

/* ENABLE INTERNAL LOW SPEED OSCILLATOR (38kHz INTERNAL RC).
 * @param:	None.
 * @return:	None.
 */
void RCC_enable_lsi(void) {
	// Enable LSI.
	RCC -> CSR |= (0b1 << 0); // LSION='1'.
	// Enable interrupt.
	NVIC_enable_interrupt(NVIC_INTERRUPT_RCC_CRS);
	// Wait for LSI to be stable.
	while (((RCC -> CSR) & (0b1 << 1)) == 0) {
		PWR_enter_sleep_mode();
	}
	NVIC_disable_interrupt(NVIC_INTERRUPT_RCC_CRS);
}

/* ENABLE EXTERNAL LOW SPEED OSCILLATOR (32.768kHz QUARTZ).
 * @param:	None.
 * @return:	None.
 */
void RCC_enable_lse(void) {
	// Enable LSE (32.768kHz crystal).
	RCC -> CSR |= (0b1 << 8); // LSEON='1'.
	// Enable interrupt.
	NVIC_enable_interrupt(NVIC_INTERRUPT_RCC_CRS);
	// Wait for LSE to be stable.
	while (((RCC -> CSR) & (0b1 << 9)) == 0) {
		PWR_enter_sleep_mode();
	}
	NVIC_disable_interrupt(NVIC_INTERRUPT_RCC_CRS);
}
