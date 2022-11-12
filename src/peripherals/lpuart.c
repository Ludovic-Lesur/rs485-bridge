/*
 * lpuart.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#include "lpuart.h"

#include "exti.h"
#include "gpio.h"
#include "lpuart_reg.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "rs485.h"

/*** LPUART local macros ***/

#define LPUART_BAUD_RATE 			9600
#define LPUART_STRING_LENGTH_MAX	1000
#define LPUART_TIMEOUT_COUNT		100000

#define LPUART_NODE_ADDRESS			0x00

/*** LPUART local structures ***/

typedef enum {
	LPUART_FIELD_INDEX_DESTINATION_ADDRESS = 0,
	LPUART_FIELD_INDEX_SOURCE_ADDRESS,
	LPUART_FIELD_INDEX_COMMAND,
} LPUART_field_index_t;

typedef struct {
	LPUART_mode_t mode;
	volatile uint32_t rx_byte_count;
} LPUART_context_t;

/*** LPUART local global variables ***/

static LPUART_context_t lpuart_ctx;

/*** LPUART local functions ***/

/* LPUART1 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_IRQHandler(void) {
	// Local variables.
	uint8_t rx_byte = 0;
	// RXNE interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 5)) != 0) {
		// Read incoming byte.
		rx_byte = (LPUART1 -> RDR);
		// Check mode.
		if (lpuart_ctx.mode == LPUART_MODE_NODE) {
			// Check field index.
			if (lpuart_ctx.rx_byte_count >= LPUART_FIELD_INDEX_SOURCE_ADDRESS) {
				// Transmit command to applicative layer.
				RS485_fill_rx_buffer(rx_byte);
			}
			// Increment byte count.
			lpuart_ctx.rx_byte_count++;
		}
		else {
			RS485_fill_rx_buffer(rx_byte);
		}
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
 * @return status:	Function execution status.
 */
static LPUART_status_t _LPUART1_fill_tx_buffer(uint8_t tx_byte) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	uint32_t loop_count = 0;
	// Fill transmit register.
	LPUART1 -> TDR = tx_byte;
	// Wait for transmission to complete.
	while (((LPUART1 -> ISR) & (0b1 << 7)) == 0) {
		// Wait for TXE='1' or timeout.
		loop_count++;
		if (loop_count > LPUART_TIMEOUT_COUNT) {
			status = LPUART_ERROR_TX_TIMEOUT;
			goto errors;
		}
	}
errors:
	return status;
}

/*** LPUART functions ***/

/* CONFIGURE LPUART1.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_init(void) {
	// Local variables.
	uint32_t brr = 0;
	// Init context.
	lpuart_ctx.mode = LPUART_MODE_DIRECT;
	lpuart_ctx.rx_byte_count = 0;
	// Select LSE as clock source.
	RCC -> CCIPR |= (0b11 << 10); // LPUART1SEL='11'.
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 18); // LPUARTEN='1'.
	// Configure TX and RX GPIOs.
	GPIO_configure(&GPIO_LPUART1_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LPUART1_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_LPUART1_DE, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE); // External pull-down resistor present.
	LPUART1_disable_rx();
	// Configure peripheral in direct mode by default.
	LPUART1 -> CR1 |= 0x03FF0022;
	LPUART1 -> CR2 |= ((LPUART_NODE_ADDRESS & 0x7F) << 24) | (0b1 << 4);
	LPUART1 -> CR3 |= 0x00B05000;
	// Baud rate.
	brr = (RCC_LSE_FREQUENCY_HZ * 256);
	brr /= LPUART_BAUD_RATE;
	LPUART1 -> BRR = (brr & 0x000FFFFF); // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.
	// Configure interrupt.
	NVIC_set_priority(NVIC_INTERRUPT_LPUART1, 0);
	EXTI_configure_line(EXTI_LINE_LPUART1, EXTI_TRIGGER_RISING_EDGE);
	// Enable transmitter.
	LPUART1 -> CR1 |= (0b1 << 3) | (0b1 << 0); // TE='1' and UE='1'.
}

/* CONFIGURE LPUART MODE.
 * @param mode:		Operation mode.
 * @return status:	Function execution status.
 */
LPUART_status_t LPUART1_set_mode(LPUART_mode_t mode) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	// Disable peripheral.
	LPUART1 -> CR1 &= ~(0b1 << 0);
	// Configure peripheral.
	switch (mode) {
	case LPUART_MODE_DIRECT:
		// Disable mute mode, address detection and wake-up on RXNE.
		LPUART1 -> CR1 &= 0xFFFFD7FF; // MME='0' and WAKE='0'.
		LPUART1 -> CR3 |= 0x00030000; // WUS='11'.
		break;
	case LPUART_MODE_NODE:
		// Enable mute mode, address detection and wake up on address match.
		LPUART1 -> CR1 |= 0x00002800; // MME='1' and WAKE='1'.
		LPUART1 -> CR3 &= 0xFFCFFFFF; // WUS='00'.
		break;
	default:
		status = LPUART_ERROR_MODE;
		goto errors;
	}
	// Update mode.
	lpuart_ctx.mode = mode;
	// Enable peripheral.
	LPUART1 -> CR1 |= (0b1 << 0);
errors:
	return status;
}

/* EANABLE LPUART RX OPERATION.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_enable_rx(void) {
	// Request mute mode if needed.
	if (lpuart_ctx.mode == LPUART_MODE_NODE) {
		LPUART1 -> RQR |= (0b1 << 2); // MMRQ='1'.
	}
	// Reset IRQ count.
	lpuart_ctx.rx_byte_count = 0;
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
	// Disable RS485 receiver.
	GPIO_configure(&GPIO_LPUART1_NRE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_LPUART1_NRE, 1);
	// Disable receiver.
	LPUART1 -> CR1 &= ~(0b1 << 2); // RE='0'.
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_LPUART1);
}

/* SEND A COMMAND TO AN RS485 NODE.
 * @param node_address_enable:	Enable or disable destination address header.
 * @param node_address:			RS485 address of the destination board.
 * @param command:				Command to send.
 * @return:						None.
 */
LPUART_status_t LPUART1_send_command(uint8_t node_address, char_t* command) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameters.
	if (command == NULL) {
		status = LPUART_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (node_address > RS485_ADDRESS_LAST) {
		status = LPUART_ERROR_NODE_ADDRESS;
		goto errors;
	}
	// Send header if required.
	if (lpuart_ctx.mode == LPUART_MODE_NODE) {
		// Send destination and source addresses.
		_LPUART1_fill_tx_buffer(node_address | 0x80);
		_LPUART1_fill_tx_buffer(LPUART_NODE_ADDRESS);
	}
	// Fill TX buffer with new bytes.
	while (*command) {
		status = _LPUART1_fill_tx_buffer((uint8_t) *(command++));
		if (status != LPUART_SUCCESS) goto errors;
		// Check character count.
		loop_count++;
		if (loop_count > LPUART_STRING_LENGTH_MAX) {
			status = LPUART_ERROR_STRING_LENGTH;
			goto errors;
		}
	}
	// Wait for TC flag.
	loop_count = 0;
	while (((LPUART1 -> ISR) & (0b1 << 6)) == 0) {
		// Wait for TC='1' or timeout.
		loop_count++;
		if (loop_count > LPUART_TIMEOUT_COUNT) {
			status = LPUART_ERROR_TC_TIMEOUT;
			break;
		}
	}
errors:
	return status;
}
