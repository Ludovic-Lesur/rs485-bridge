/*
 * r4s8cr.c
 *
 *  Created on: Feb 5, 2023
 *      Author: ludo
 */

#include "r4s8cr.h"

#include "at_usb.h"
#include "dinfox.h"
#include "lpuart.h"
#include "mode.h"
#include "node.h"

/*** R4S8CR local macros ***/

#define R4S8CR_BAUD_RATE					9600

#define R4S8CR_BUFFER_SIZE_BYTES			128
#define R4S8CR_REPLY_BUFFER_DEPTH			16

#define R4S8CR_ADDRESS_SIZE_BYTES			1
#define R4S8CR_RELAY_ADDRESS_SIZE_BYTES		1
#define R4S8CR_COMMAND_SIZE_BYTES			1

#define R4S8CR_COMMAND_READ					0xA0
#define R4S8CR_COMMAND_OFF					0x00
#define R4S8CR_COMMAND_ON					0x01

#define R4S8CR_REPLY_PARSING_DELAY_MS		10

#define R4S8CR_REPLY_SIZE_BYTES				(R4S8CR_ADDRESS_SIZE_BYTES + R4S8CR_RELAY_ADDRESS_SIZE_BYTES + R4S8CR_REGISTER_LAST)

/*** R4S8CR local structures ***/

typedef struct {
	uint8_t command[R4S8CR_BUFFER_SIZE_BYTES];
	uint8_t command_size;
	volatile uint8_t reply[R4S8CR_BUFFER_SIZE_BYTES];
	volatile uint8_t reply_size;
} R4S8CR_context_t;

/*** R4S8CR local global variables ***/

static R4S8CR_context_t r4s8cr_ctx;

/*** LBUS local functions ***/

/* FLUSH COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
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

/* READ R4S8CR NODE REGISTER.
 * @param read_params:	Pointer to the read operation parameters.
 * @param read_data:	Pointer to the read result.
 * @param read_status:	Pointer to the read operation status.
 * @return status:		Function execution status.
 */
NODE_status_t _R4S8CR_read_register(NODE_read_parameters_t* read_params, NODE_read_data_t* read_data, NODE_access_status_t* read_status) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint32_t reply_time_ms = 0;
	uint8_t relay_box_id = 0;
	// Check parameters.
	if ((read_params == NULL) || (read_data == NULL) || (read_status == NULL)) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if ((read_params -> format) != STRING_FORMAT_BOOLEAN) {
		status = NODE_ERROR_REGISTER_FORMAT;
		goto errors;
	}
	if ((read_params -> type) != NODE_REPLY_TYPE_VALUE) {
		status = NODE_ERROR_READ_TYPE;
		goto errors;
	}
	if ((read_params -> register_address) >= R4S8CR_REGISTER_LAST) {
		status = NODE_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	if (((read_params -> node_address) < DINFOX_NODE_ADDRESS_R4S8CR_START) || ((read_params -> node_address) >= (DINFOX_NODE_ADDRESS_R4S8CR_START + DINFOX_NODE_ADDRESS_RANGE_R4S8CR))) {
		status = NODE_ERROR_NODE_ADDRESS;
		goto errors;
	}
	// Convert node address to ID.
	relay_box_id = ((read_params -> node_address) - DINFOX_NODE_ADDRESS_R4S8CR_START + 1) & 0x0F;
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
	lpuart1_status = LPUART1_send(r4s8cr_ctx.command, r4s8cr_ctx.command_size);
	LPUART1_status_check(NODE_ERROR_BASE_LPUART);
	// Enable reception.
	LPUART1_enable_rx();
	// Wait reply.
	while (1) {
		// Delay.
		lptim1_status = LPTIM1_delay_milliseconds(R4S8CR_REPLY_PARSING_DELAY_MS, LPTIM_DELAY_MODE_STOP);
		LPTIM1_status_check(NODE_ERROR_BASE_LPTIM);
		reply_time_ms += R4S8CR_REPLY_PARSING_DELAY_MS;
		// Check number of received bytes.
		if (r4s8cr_ctx.reply_size >= R4S8CR_REPLY_SIZE_BYTES) {
			// Update value.
			(read_data -> value) = r4s8cr_ctx.reply[(read_params -> register_address) + R4S8CR_REPLY_SIZE_BYTES - R4S8CR_REGISTER_LAST];
			break;
		}
		// Exit if timeout.
		if (reply_time_ms > (read_params -> timeout_ms)) {
			// Set status to timeout.
			(read_status -> reply_timeout) = 1;
			break;
		}
	}
errors:
	return status;
}

/*** R4S8CR functions ***/

/* CONFIGURE PHYSICAL INTERFACE FOR R4S8CR.
 * @param:	None.
 * @return:	None.
 */
NODE_status_t R4S8CR_configure_phy(void) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	LPUART_config_t lpuart_config;
	// Configure physical interface.
	lpuart_config.baud_rate = R4S8CR_BAUD_RATE;
	lpuart_config.rx_callback = &R4S8CR_fill_rx_buffer;
	lpuart1_status = LPUART1_configure(&lpuart_config);
	LPUART1_status_check(NODE_ERROR_BASE_LPUART);
errors:
	return status;
}

NODE_status_t R4S8CR_send_command(NODE_command_parameters_t* command_params) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	// Convert ASCII to raw bytes.
	string_status = STRING_hexadecimal_string_to_byte_array((command_params -> command), STRING_CHAR_NULL, r4s8cr_ctx.command, &r4s8cr_ctx.command_size);
	STRING_status_check(NODE_ERROR_BASE_STRING);
	// Configure physical interface.
	status = R4S8CR_configure_phy();
	if (status != NODE_SUCCESS) goto errors;
	LPUART1_disable_rx();
	// Send command.
	lpuart1_status = LPUART1_send(r4s8cr_ctx.command, r4s8cr_ctx.command_size);
	LPUART1_status_check(NODE_ERROR_BASE_LPUART);
	// Enable reception.
	LPUART1_enable_rx();
errors:
	return status;
}

/* SCAN R4S8CR NODES ON BUS.
 * @param nodes_list:		Node list to fill.
 * @param nodes_list_size:	Maximum size of the list.
 * @param nodes_count:		Pointer to byte that will contain the number of LBUS nodes detected.
 * @return status:			Function execution status.
 */
NODE_status_t R4S8CR_scan(NODE_t* nodes_list, uint8_t nodes_list_size, uint8_t* nodes_count) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	NODE_read_parameters_t read_params;
	NODE_read_data_t read_data;
	NODE_access_status_t read_status;
	NODE_address_t node_address = 0;
	uint8_t node_list_idx = 0;
	// Check parameters.
	if ((nodes_list == NULL) || (nodes_count == NULL)) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Reset count.
	(*nodes_count) = 0;
	// Build read input common parameters.
	read_params.format = STRING_FORMAT_BOOLEAN;
	read_params.timeout_ms = R4S8CR_TIMEOUT_MS;
	read_params.register_address = R4S8CR_REGISTER_RELAY_1;
	read_params.type = NODE_REPLY_TYPE_VALUE;
	// Configure read data.
	read_data.raw = NULL;
	read_data.value = 0;
	read_data.byte_array = NULL;
	read_data.extracted_length = 0;
	// Loop on all addresses.
	for (node_address=DINFOX_NODE_ADDRESS_R4S8CR_START ; node_address<(DINFOX_NODE_ADDRESS_R4S8CR_START + DINFOX_NODE_ADDRESS_RANGE_R4S8CR) ; node_address++) {
		// Update read parameters.
		read_params.node_address = node_address;
		// Ping address.
		status = _R4S8CR_read_register(&read_params, &read_data, &read_status);
		if (status != NODE_SUCCESS) goto errors;
		// Check reply status.
		if (read_status.all == 0) {
			// Node found.
			(*nodes_count)++;
			// Store address and reset board ID.
			nodes_list[node_list_idx].address = node_address;
			nodes_list[node_list_idx].board_id = DINFOX_BOARD_ID_R4S8CR;
		}
	}
errors:
	return status;
}

/* MAIN TASK OF R4S8CR INTERFACE.
 * @param:			None.
 * @return status:	Function execution status.
 */
NODE_status_t R4S8CR_task(void) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	char_t r4s8cr_frame[R4S8CR_BUFFER_SIZE_BYTES] = {STRING_CHAR_NULL};
	// Check number of received bytes.
	if (r4s8cr_ctx.reply_size >= R4S8CR_REPLY_SIZE_BYTES) {
		// Convert to ASCII.
		string_status = STRING_byte_array_to_hexadecimal_string((uint8_t*) r4s8cr_ctx.reply, (uint8_t) r4s8cr_ctx.reply_size, 0, r4s8cr_frame);
		STRING_status_check(NODE_ERROR_BASE_STRING);
		// Print buffer.
		AT_USB_print(r4s8cr_frame);
		// Flush buffers.
		_R4S8CR_flush_buffers();
	}
errors:
	return status;
}

/* FILL R4S8CR BUFFER WITH A NEW BYTE (CALLED BY LPUART INTERRUPT).
 * @param rx_byte:	Incoming byte.
 * @return:			None.
 */
void R4S8CR_fill_rx_buffer(uint8_t rx_byte) {
	// Store incoming byte.
	r4s8cr_ctx.reply[r4s8cr_ctx.reply_size] = rx_byte;
	// Manage index.
	r4s8cr_ctx.reply_size = (r4s8cr_ctx.reply_size + 1) % R4S8CR_BUFFER_SIZE_BYTES;
}
