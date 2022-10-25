/*
 * lpuart.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#include "lpuart.h"

#include "at.h"
#include "exti.h"
#include "gpio.h"
#include "lpuart_reg.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** LPUART local macros ***/

#define LPUART_BAUD_RATE 			9600
#define LPUART_TIMEOUT_COUNT		100000
#ifdef RSM
#define LPUART_ADDR_LENGTH_BYTES	1
#define LPUART_ADDR_NODE			0x31
#define LPUART_ADDR_MASTER			0x65
#endif

/*** LPUART local global variables ***/

#ifdef RSM
static volatile uint32_t lpuart_irq_count = 0;
#endif

/*** LPUART local functions ***/

void LPUART1_IRQHandler(void) {
	// RXNE interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 5)) != 0) {
#ifdef RSM
		// Increment IRQ count.
		lpuart_irq_count++;
		// Do not transmit address bytes to applicative layer.
		if (lpuart_irq_count > LPUART_ADDR_LENGTH_BYTES) {
			// Fill AT RX buffer with incoming byte.
			AT_fill_rx_buffer(LPUART1 -> RDR);
		}
#else
		AT_fill_rx_buffer(LPUART1 -> RDR);
#endif
		// Clear RXNE flag.
		LPUART1 -> RQR |= (0b1 << 3);

	}
	// Overrun error interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		LPUART1 -> ICR |= (0b1 << 3);
	}
}

/* FILL LPUART1 TX BUFFER WITH A NEW BYTE.
 * @param tx_byte:	Byte to append.
 * @return:			None.
 */
static void _LPUART1_fill_tx_buffer(uint8_t tx_byte) {
	// Fill transmit register.
	LPUART1 -> TDR = tx_byte;
	// Wait for transmission to complete.
	uint32_t loop_count = 0;
	while (((LPUART1 -> ISR) & (0b1 << 7)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > LPUART_TIMEOUT_COUNT) break;
	}
}

/*** LPUART functions ***/

/* CONFIGURE LPUART1.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_init(void) {
	// Select LSE as clock source.
	RCC -> CCIPR |= (0b11 << 10); // LPUART1SEL='11'.
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 18); // LPUARTEN='1'.
	// Configure TX and RX GPIOs.
	GPIO_configure(&GPIO_LPUART1_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LPUART1_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LPUART1_DE, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE); // External pull-down resistor present.
	LPUART1_disable_rx();
	// Configure peripheral.
#ifdef RSM
	LPUART1 -> CR1 |= 0x03FF2822;
	LPUART1 -> CR2 |= ((LPUART_ADDR_NODE & 0x7F) << 24) | (0b1 << 4);
	LPUART1 -> CR3 |= 0x00805000;
#else
	LPUART1 -> CR1 |= 0x03FF0022;
	LPUART1 -> CR3 |= 0x00B05000;
#endif
	// Baud rate.
	uint32_t brr = (RCC_LSE_FREQUENCY_HZ * 256);
	brr /= LPUART_BAUD_RATE;
	LPUART1 -> BRR = (brr & 0x000FFFFF); // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.
	// Configure interrupt.
	NVIC_set_priority(NVIC_INTERRUPT_LPUART1, 0);
	EXTI_configure_line(EXTI_LINE_LPUART1, EXTI_TRIGGER_RISING_EDGE);
	// Enable transmitter.
	LPUART1 -> CR1 |= (0b1 << 3); // TE='1'.
	// Enable peripheral.
	LPUART1 -> CR1 |= (0b1 << 0); // UE='1'.
}

/* EANABLE LPUART RX OPERATION.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_enable_rx(void) {
#ifdef RSM
	// Mute mode request.
	LPUART1 -> RQR |= (0b1 << 2); // MMRQ='1'.
#endif
	// Clear flag and enable interrupt.
	LPUART1 -> RQR |= (0b1 << 3);
	NVIC_enable_interrupt(NVIC_INTERRUPT_LPUART1);
	// Enable receiver.
	LPUART1 -> CR1 |= (0b1 << 2); // RE='1'.
	// Enable RS485 receiver.
	GPIO_configure(&GPIO_LPUART1_NRE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE); // External pull-down resistor present.
}

/* DISABLE LPUART RX OPERATION.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_disable_rx(void) {
#ifdef RSM
	// Reset IRQ count for next command reception.
	lpuart_irq_count = 0;
#endif
	// Disable RS485 receiver.
	GPIO_configure(&GPIO_LPUART1_NRE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_LPUART1_NRE, 1);
	// Disable receiver.
	LPUART1 -> CR1 &= ~(0b1 << 2); // RE='0'.
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_LPUART1);
}

/* SEND A BYTE ARRAY THROUGH LPUART1.
 * @param tx_string:	Byte array to send.
 * @return:				None.
 */
void LPUART1_send_string(char* tx_string) {
#ifdef RSM
	// Send master address.
	_LPUART1_fill_tx_buffer(LPUART_ADDR_MASTER | 0x80);
#endif
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		_LPUART1_fill_tx_buffer((uint8_t) *(tx_string++));
	}
	// Wait for TC flag.
	while (((LPUART1 -> ISR) & (0b1 << 6)) == 0);
}
