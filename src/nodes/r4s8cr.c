/*
 * r4s8cr.c
 *
 *  Created on: Feb 5, 2023
 *      Author: ludo
 */

#include "r4s8cr.h"

#include "dinfox.h"
#include "lpuart.h"
#include "node.h"
#include "node_common.h"
#include "r4s8cr_reg.h"

/*** R4S8CR local macros ***/

#define R4S8CR_NUMBER_OF_RELAYS					8
#define R4S8CR_NUMBER_OF_IDS					15

#define R4S8CR_NODE_ADDRESS						0xFF

#define R4S8CR_BAUD_RATE						9600

#define R4S8CR_BUFFER_SIZE_BYTES				64
#define R4S8CR_REPLY_BUFFER_DEPTH				16

#define R4S8CR_ADDRESS_SIZE_BYTES				1
#define R4S8CR_RELAY_ADDRESS_SIZE_BYTES			1
#define R4S8CR_COMMAND_SIZE_BYTES				1

#define R4S8CR_COMMAND_READ						0xA0
#define R4S8CR_COMMAND_OFF						0x00
#define R4S8CR_COMMAND_ON						0x01

#define R4S8CR_REPLY_PARSING_DELAY_MS			10

#define R4S8CR_REPLY_HEADER_SIZE				(R4S8CR_ADDRESS_SIZE_BYTES + R4S8CR_RELAY_ADDRESS_SIZE_BYTES)
#define R4S8CR_REPLY_SIZE_BYTES					(R4S8CR_REPLY_HEADER_SIZE + R4S8CR_NUMBER_OF_RELAYS)

#define R4S8CR_READ_TIMEOUT_MS					200
#define R4S8CR_WRITE_TIMEOUT_MS					1000

/*** R4S8CR local structures ***/

/*******************************************************************/
typedef struct {
	uint8_t command[R4S8CR_BUFFER_SIZE_BYTES];
	uint8_t command_size;
	volatile uint8_t reply[R4S8CR_BUFFER_SIZE_BYTES];
	volatile uint8_t reply_size;
	DINFOX_bit_representation_t rxst[R4S8CR_NUMBER_OF_RELAYS];
	NODE_print_frame_cb_t print_callback;
} R4S8CR_context_t;

/*** R4S8CR local global variables ***/

static uint32_t R4S8CR_REGISTERS[R4S8CR_REG_ADDR_LAST];

static const uint32_t R4S8CR_REG_ERROR_VALUE[R4S8CR_REG_ADDR_LAST] = {
	0x00000000,
	((DINFOX_BIT_ERROR << 14) | (DINFOX_BIT_ERROR << 12) | (DINFOX_BIT_ERROR << 10) | (DINFOX_BIT_ERROR << 8) |
	 (DINFOX_BIT_ERROR << 6)  | (DINFOX_BIT_ERROR << 4)  | (DINFOX_BIT_ERROR << 2)  | (DINFOX_BIT_ERROR << 0)),
};

static R4S8CR_context_t r4s8cr_ctx;

/*** R4S8CR local functions ***/

/*******************************************************************/
static void _R4S8CR_fill_rx_buffer(uint8_t rx_byte) {
	// Store incoming byte.
	r4s8cr_ctx.reply[r4s8cr_ctx.reply_size] = rx_byte;
	// Manage index.
	r4s8cr_ctx.reply_size = (r4s8cr_ctx.reply_size + 1) % R4S8CR_BUFFER_SIZE_BYTES;
}

/*******************************************************************/
static void _R4S8CR_flush_buffers(void) {
	// Local variables.
	uint8_t idx = 0;
	// Flush buffers.
	for (idx=0 ; idx<R4S8CR_BUFFER_SIZE_BYTES ; idx++) {
		r4s8cr_ctx.command[idx] = 0x00;
		r4s8cr_ctx.reply[idx] = 0x00;
	}
	r4s8cr_ctx.command_size = 0;
	r4s8cr_ctx.reply_size = 0;
}

/*******************************************************************/
static NODE_status_t _R4S8CR_read_relays_state(uint8_t relay_box_id, uint32_t timeout_ms, NODE_access_status_t* read_status) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint32_t reply_time_ms = 0;
	uint8_t idx = 0;
	// Reset relays status.
	for (idx=0 ; idx<R4S8CR_NUMBER_OF_RELAYS ; idx++) {
		r4s8cr_ctx.rxst[idx] = DINFOX_BIT_ERROR;
	}
	// Flush buffers and status.
	_R4S8CR_flush_buffers();
	(read_status -> all) = 0;
	// Build command.
	r4s8cr_ctx.command[0] = R4S8CR_NODE_ADDRESS;
	r4s8cr_ctx.command[1] = (R4S8CR_COMMAND_READ | relay_box_id);
	r4s8cr_ctx.command[2] = 0x00;
	r4s8cr_ctx.command_size = (R4S8CR_ADDRESS_SIZE_BYTES + R4S8CR_RELAY_ADDRESS_SIZE_BYTES + R4S8CR_COMMAND_SIZE_BYTES);
	// Configure physical interface.
	status = R4S8CR_configure_phy();
	if (status != NODE_SUCCESS) goto errors;
	LPUART1_disable_rx();
	// Send command.
	lpuart1_status = LPUART1_write(r4s8cr_ctx.command, r4s8cr_ctx.command_size);
	LPUART1_exit_error(NODE_ERROR_BASE_LPUART);
	// Enable reception.
	LPUART1_enable_rx();
	// Wait reply.
	while (1) {
		// Delay.
		lptim1_status = LPTIM1_delay_milliseconds(R4S8CR_REPLY_PARSING_DELAY_MS, LPTIM_DELAY_MODE_STOP);
		LPTIM1_exit_error(NODE_ERROR_BASE_LPTIM);
		reply_time_ms += R4S8CR_REPLY_PARSING_DELAY_MS;
		// Check number of received bytes.
		if (r4s8cr_ctx.reply_size >= R4S8CR_REPLY_SIZE_BYTES) {
			// Relays loop.
			for (idx=0 ; idx<R4S8CR_NUMBER_OF_RELAYS ; idx++) {
				// Get relay state.
				r4s8cr_ctx.rxst[idx] = (r4s8cr_ctx.reply[R4S8CR_REPLY_HEADER_SIZE + idx] == 0) ? DINFOX_BIT_0 : DINFOX_BIT_1;
			}
			break;
		}
		// Exit if timeout.
		if (reply_time_ms > timeout_ms) {
			// Set status to timeout.
			(read_status -> reply_timeout) = 1;
			break;
		}
	}
errors:
	LPUART1_enable_rx();
	return status;
}

/*******************************************************************/
static NODE_status_t _R4S8CR_read_register(NODE_access_parameters_t* read_params, uint32_t* reg_value, NODE_access_status_t* read_status) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	uint8_t relay_box_id = 0;
	uint32_t unused_mask = 0;
	uint8_t idx = 0;
	// Check parameters.
	if ((read_params == NULL) || (reg_value == NULL) || (read_status == NULL)) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if ((read_params -> reg_addr) >= R4S8CR_REG_ADDR_LAST) {
		status = NODE_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	if (((read_params -> node_addr) < DINFOX_NODE_ADDRESS_R4S8CR_START) || ((read_params -> node_addr) >= (DINFOX_NODE_ADDRESS_R4S8CR_START + DINFOX_NODE_ADDRESS_RANGE_R4S8CR))) {
		status = NODE_ERROR_R4S8CR_ADDRESS;
		goto errors;
	}
	// Reset value.
	(*reg_value) = R4S8CR_REG_ERROR_VALUE[(read_params -> reg_addr)];
	// Update required registers.
	switch (read_params -> reg_addr) {
	case R4S8CR_REG_ADDR_STATUS:
		// Convert node address to ID.
		relay_box_id = ((read_params -> node_addr) - DINFOX_NODE_ADDRESS_R4S8CR_START + 1) & 0x0F;
		// Read relays state.
		status = _R4S8CR_read_relays_state(relay_box_id, ((read_params -> reply_params).timeout_ms), read_status);
		if ((status != NODE_SUCCESS) || ((read_status -> all) != 0)) goto errors;
		// Write register.
		for (idx=0 ; idx<R4S8CR_NUMBER_OF_RELAYS ; idx++) {
			DINFOX_write_field(&(R4S8CR_REGISTERS[R4S8CR_REG_ADDR_STATUS]), &unused_mask, ((uint32_t) r4s8cr_ctx.rxst[idx]), (0b11 << (idx << 1)));
		}
		break;
	default:
		// Nothing to do on other registers.
		break;
	}
	// Read register.
	(*reg_value) = R4S8CR_REGISTERS[(read_params -> reg_addr)];
errors:
	return status;
}

/*** R4S8CR functions ***/

/*******************************************************************/
void R4S8CR_init(NODE_print_frame_cb_t print_callback) {
	// Init context.
	_R4S8CR_flush_buffers();
	// Register callback.
	r4s8cr_ctx.print_callback = print_callback;
}

/*******************************************************************/
NODE_status_t R4S8CR_configure_phy(void) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	LPUART_configuration_t lpuart_config;
	// Configure physical interface.
	lpuart_config.baud_rate = R4S8CR_BAUD_RATE;
	lpuart_config.rx_callback = &_R4S8CR_fill_rx_buffer;
	lpuart1_status = LPUART1_configure(&lpuart_config);
	LPUART1_exit_error(NODE_ERROR_BASE_LPUART);
errors:
	return status;
}

/*******************************************************************/
NODE_status_t R4S8CR_send_command(NODE_command_parameters_t* command_params) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	// Convert ASCII to raw bytes.
	string_status = STRING_hexadecimal_string_to_byte_array((command_params -> command), STRING_CHAR_NULL, r4s8cr_ctx.command, &r4s8cr_ctx.command_size);
	STRING_exit_error(NODE_ERROR_BASE_STRING);
	// Configure physical interface.
	status = R4S8CR_configure_phy();
	if (status != NODE_SUCCESS) goto errors;
	LPUART1_disable_rx();
	// Send command.
	lpuart1_status = LPUART1_write(r4s8cr_ctx.command, r4s8cr_ctx.command_size);
	LPUART1_exit_error(NODE_ERROR_BASE_LPUART);
errors:
	// Enable reception.
	LPUART1_enable_rx();
	return status;
}

/*******************************************************************/
NODE_status_t R4S8CR_scan(NODE_t* nodes_list, uint8_t nodes_list_size, uint8_t* nodes_count) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	NODE_access_parameters_t read_params;
	NODE_access_status_t read_status;
	NODE_address_t node_addr = 0;
	uint8_t node_list_idx = 0;
	uint32_t reg_value = 0;
	// Check parameters.
	if ((nodes_list == NULL) || (nodes_count == NULL)) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Reset count.
	(*nodes_count) = 0;
	// Build read input common parameters.
	read_params.reg_addr = R4S8CR_REG_ADDR_STATUS;
	read_params.reply_params.type = NODE_REPLY_TYPE_VALUE;
	read_params.reply_params.timeout_ms = R4S8CR_READ_TIMEOUT_MS;
	// Loop on all addresses.
	for (node_addr=DINFOX_NODE_ADDRESS_R4S8CR_START ; node_addr<(DINFOX_NODE_ADDRESS_R4S8CR_START + DINFOX_NODE_ADDRESS_RANGE_R4S8CR) ; node_addr++) {
		// Update read parameters.
		read_params.node_addr = node_addr;
		// Ping address.
		status = _R4S8CR_read_register(&read_params, &reg_value, &read_status);
		if (status != NODE_SUCCESS) goto errors;
		// Check reply status.
		if (read_status.all == 0) {
			// Node found.
			(*nodes_count)++;
			// Store address and reset board ID.
			nodes_list[node_list_idx].address = node_addr;
			nodes_list[node_list_idx].board_id = DINFOX_BOARD_ID_R4S8CR;
		}
	}
errors:
	return status;
}

/*******************************************************************/
NODE_status_t R4S8CR_task(void) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	char_t r4s8cr_frame[R4S8CR_BUFFER_SIZE_BYTES] = {STRING_CHAR_NULL};
	// Check number of received bytes.
	if (r4s8cr_ctx.reply_size >= R4S8CR_REPLY_SIZE_BYTES) {
		// Convert to ASCII.
		string_status = STRING_byte_array_to_hexadecimal_string((uint8_t*) r4s8cr_ctx.reply, (uint8_t) r4s8cr_ctx.reply_size, 0, r4s8cr_frame);
		STRING_exit_error(NODE_ERROR_BASE_STRING);
		// Print buffer.
		if (r4s8cr_ctx.print_callback != NULL) {
			r4s8cr_ctx.print_callback(r4s8cr_frame);
		}
		// Flush buffers.
		_R4S8CR_flush_buffers();
	}
errors:
	return status;
}
