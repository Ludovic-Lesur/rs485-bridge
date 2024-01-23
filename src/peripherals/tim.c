/*
 * tim.c
 *
 *  Created on: 03 jul. 2021
 *      Author: Ludo
 */

#include "tim.h"

#include "iwdg.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "rtc.h"
#include "tim_reg.h"
#include "types.h"

/*** TIM local macros ***/

#define TIM_TIMEOUT_COUNT				10000000

#define TIM21_INPUT_CAPTURE_PRESCALER	8

/*** TIM local structures ***/

/*******************************************************************/
typedef struct {
	volatile uint16_t ccr1_start;
	volatile uint16_t ccr1_end;
	volatile uint16_t capture_count;
	volatile uint8_t capture_done;
} TIM21_context_t;

/*** TIM local global variables ***/

static TIM21_context_t tim21_ctx;

/*** TIM local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM21_IRQHandler(void) {
	// TI1 interrupt.
	if (((TIM21 -> SR) & (0b1 << 1)) != 0) {
		// Update flags.
		if (((TIM21 -> DIER) & (0b1 << 1)) != 0) {
			// Check count.
			if (tim21_ctx.capture_count == 0) {
				// Store start value.
				tim21_ctx.ccr1_start = (TIM21 -> CCR1);
				tim21_ctx.capture_count++;
			}
			else {
				// Check rollover.
				if ((TIM21 -> CCR1) > tim21_ctx.ccr1_end) {
					// Store new value.
					tim21_ctx.ccr1_end = (TIM21 -> CCR1);
					tim21_ctx.capture_count++;
				}
				else {
					// Capture complete.
					tim21_ctx.capture_done = 1;
				}
			}
		}
		TIM21 -> SR &= ~(0b1 << 1);
	}
}

/*******************************************************************/
void TIM21_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.
	// Configure timer.
	// Channel input on TI1.
	// Capture done every 8 edges.
	// CH1 mapped on MCO.
	TIM21 -> CCMR1 |= (0b01 << 0) | (0b11 << 2);
	TIM21 -> OR |= (0b111 << 2);
	// Enable interrupt.
	TIM21 -> DIER |= (0b1 << 1); // CC1IE='1'.
	// Generate event to update registers.
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.
}

/*******************************************************************/
void TIM21_de_init(void) {
	// Disable timer.
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 2); // TIM21EN='0'.
}

/*******************************************************************/
TIM_status_t TIM21_mco_capture(uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameters.
	if ((ref_clock_pulse_count == NULL) || (mco_pulse_count == NULL)) {
		status = TIM_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Reset timer context.
	tim21_ctx.ccr1_start = 0;
	tim21_ctx.ccr1_end = 0;
	tim21_ctx.capture_count = 0;
	tim21_ctx.capture_done = 0;
	// Reset counter.
	TIM21 -> CNT = 0;
	TIM21 -> CCR1 = 0;
	// Enable interrupt.
	TIM21 -> SR &= 0xFFFFF9B8; // Clear all flags.
	NVIC_enable_interrupt(NVIC_INTERRUPT_TIM21, NVIC_PRIORITY_TIM21);
	// Enable TIM21 peripheral.
	TIM21 -> CR1 |= (0b1 << 0); // CEN='1'.
	TIM21 -> CCER |= (0b1 << 0); // CC1E='1'.
	// Wait for capture to complete.
	while (tim21_ctx.capture_done == 0) {
		// Manage timeout.
		loop_count++;
		if (loop_count > TIM_TIMEOUT_COUNT) {
			status = TIM_ERROR_CAPTURE_TIMEOUT;
			goto errors;
		}
	}
	// Update results.
	(*ref_clock_pulse_count) = (tim21_ctx.ccr1_end - tim21_ctx.ccr1_start);
	(*mco_pulse_count) = (TIM21_INPUT_CAPTURE_PRESCALER * (tim21_ctx.capture_count - 1));
errors:
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_TIM21);
	// Stop counter.
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM21 -> CCER &= ~(0b1 << 0); // CC1E='0'.
	return status;
}
