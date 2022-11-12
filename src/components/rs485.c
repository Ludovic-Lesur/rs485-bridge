/*
 * rs485.c
 *
 *  Created on: 28 oct 2022
 *      Author: Ludo
 */

#include "rs485.h"

#include "dinfox.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "string.h"

/*** RS485 local macros ***/

#define RS485_RESPONSE_PARSING_DELAY_MS		10
#define RS485_RESPONSE_PARSING_TIMEOUT_MS	100

#define RS485_RESPONSE_BUFFER_LENGTH_BYTES	128

/*** RS485 local structures ***/

typedef struct {
	// Response buffer.
	volatile char_t response_buf[RS485_RESPONSE_BUFFER_LENGTH_BYTES];
	volatile uint8_t response_buf_idx;
	PARSER_context_t parser;
	volatile uint8_t line_end_flag;
} RS485_context_t;

/*** RS485 local global variables ***/

static RS485_context_t rs485_ctx;

/*** RS485 local functions ***/

/* RESET RS485 PARSER.
 * @param:	None.
 * @return:	None.
 */
static void _RS485_reset_parser(void) {
	// Reset parsing variables.
	rs485_ctx.response_buf_idx = 0;
	rs485_ctx.line_end_flag = 0;
	rs485_ctx.parser.rx_buf = (char_t*) rs485_ctx.response_buf;
	rs485_ctx.parser.rx_buf_length = 0;
	rs485_ctx.parser.separator_idx = 0;
	rs485_ctx.parser.start_idx = 0;
}

/* WAIT A RESPONSE ON RS485 BUS.
 * @param timeout_ms:	Timeout in ms.
 * @return:				Function execution status.
 */
static RS485_status_t _RS485_wait_for_response(uint32_t timeout_ms) {
	// Local variables.
	RS485_status_t status = RS485_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint32_t parsing_time = 0;
	// Wait for respone.
	while (rs485_ctx.line_end_flag == 0) {
		// Delay.
		lptim1_status = LPTIM1_delay_milliseconds(RS485_RESPONSE_PARSING_DELAY_MS, 0);
		LPTIM1_status_check(RS485_ERROR_BASE_LPTIM);
		parsing_time += RS485_RESPONSE_PARSING_DELAY_MS;
		// Exit if timeout.
		if (parsing_time > timeout_ms) {
			status = RS485_ERROR_RESPONSE_TIMEOUT;
			goto errors;
		}
	}
errors:
	return status;
}

/*** RS485 functions ***/

/* SCAN ALL NODES ON RS485 BUS.
 * @param nodes_list:				Node list that will be filled.
 * @param node_list_size:			Size of the list (maximum number of nodes which can be recorded).
 * @param number_of_nodes_found:	Pointer that will contain the effective number of nodes found.
 * @return status:					Function execution status.
 */
RS485_status_t RS485_scan_nodes(RS485_node_t* nodes_list, uint8_t node_list_size, uint8_t* number_of_nodes_found) {
	// Local variables.
	RS485_status_t status = RS485_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	PARSER_status_t parser_status = PARSER_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	uint8_t node_address = 0;
	uint8_t node_list_idx = 0;
	int32_t node_board_id = 0;
	// Reset result.
	(*number_of_nodes_found) = 0;
	// Set mode.
	lpuart1_status = LPUART1_set_mode(LPUART_MODE_NODE);
	LPUART1_status_check(RS485_ERROR_BASE_LPUART);
	// Loop on all addresses.
	for (node_address=0 ; node_address<=RS485_ADDRESS_LAST ; node_address++) {
		// Reset parser.
		_RS485_reset_parser();
		// Send ping command.
		lpuart1_status = LPUART1_send_command(node_address, "RS\r");
		LPUART1_status_check(RS485_ERROR_BASE_LPUART);
		// Wait response.
		LPUART1_enable_rx();
		status = _RS485_wait_for_response(100);
		LPUART1_disable_rx();
		if (status == RS485_SUCCESS) {
			// Parse response.
			rs485_ctx.parser.rx_buf_length = rs485_ctx.response_buf_idx;
			parser_status = PARSER_compare(&rs485_ctx.parser, PARSER_MODE_COMMAND, "OK");
			// Check status.
			if (parser_status == PARSER_SUCCESS) {
				// Node found.
				(*number_of_nodes_found)++;
				// Check size.
				if (node_list_idx < node_list_size) {
					nodes_list[node_list_idx].address = node_address;
					nodes_list[node_list_idx].board_id = DINFOX_BOARD_ID_LAST;
				}
				// Reset parser.
				_RS485_reset_parser();
				// Get board ID.
				lpuart1_status = LPUART1_send_command(node_address, "RS$R=01\r");
				LPUART1_status_check(RS485_ERROR_BASE_LPUART);
				// Wait response.
				LPUART1_enable_rx();
				status = _RS485_wait_for_response(100);
				LPUART1_disable_rx();
				if (status == RS485_SUCCESS) {
					// Parse response.
					string_status = STRING_string_to_value((char_t*) rs485_ctx.response_buf, STRING_FORMAT_HEXADECIMAL, 2, &node_board_id);
					if (string_status == STRING_SUCCESS) {
						nodes_list[node_list_idx].board_id = (uint8_t) node_board_id;
					}
				}
			}
		}
		IWDG_reload();
	}
	return RS485_SUCCESS;
errors:
	// Disable receiver.
	LPUART1_disable_rx();
	return status;
}

/* SEND A COMMAND ON RS485 BUS.
 * @param lpuart_mode:	Transmission mode (direct or addressed).
 * @param node_address:	Slave address.
 * @param command:		Command to send.
 * @param response_ptr:	Pointer that will point to the slave response.
 * @return status:		Function execution status.
 */
RS485_status_t RS485_send_command(LPUART_mode_t lpuart_mode, uint8_t node_address, char_t* command, char_t* response, uint8_t response_size_byte) {
	// Local variables.
	RS485_status_t status = RS485_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	uint8_t idx = 0;
	// Reset parser.
	_RS485_reset_parser();
	// Set mode.
	lpuart1_status = LPUART1_set_mode(lpuart_mode);
	LPUART1_status_check(RS485_ERROR_BASE_LPUART);
	// Send command.
	lpuart1_status = LPUART1_send_command(node_address, command);
	LPUART1_status_check(RS485_ERROR_BASE_LPUART);
	// Wait response.
	LPUART1_enable_rx();
	status = _RS485_wait_for_response(1000);
	LPUART1_disable_rx();
	if (status != RS485_SUCCESS) goto errors;
	// Check if received source address equals the node address.
	if (rs485_ctx.response_buf[0] != node_address) {
		status = RS485_ERROR_SOURCE_ADDRESS_MISMATCH;
		goto errors;
	}
	// Fill response.
	for (idx=0 ; idx<rs485_ctx.response_buf_idx; idx++) {
		// Break if the maximum size is reached.
		if (idx > response_size_byte) break;
		// Fill byte and skip source address.
		response[idx] = rs485_ctx.response_buf[idx + 1];
	}
errors:
	// Disable receiver.
	LPUART1_disable_rx();
	return status;
}

/* FILL RS485 BUFFER WITH A NEW BYTE (CALLED BY LPUART INTERRUPT).
 * @param rx_byte:	Incoming byte.
 * @return:			None.
 */
void RS485_fill_rx_buffer(uint8_t rx_byte) {
	// Append byte if line end flag is not allready set.
	if (rs485_ctx.line_end_flag == 0) {
		// Check ending characters.
		if ((rx_byte == STRING_CHAR_CR) || (rx_byte == STRING_CHAR_LF)) {
			rs485_ctx.response_buf[rs485_ctx.response_buf_idx] = STRING_CHAR_NULL;
			rs485_ctx.line_end_flag = 1;
		}
		else {
			// Store new byte.
			rs485_ctx.response_buf[rs485_ctx.response_buf_idx] = rx_byte;
			// Manage index.
			rs485_ctx.response_buf_idx++;
			if (rs485_ctx.response_buf_idx >= RS485_RESPONSE_BUFFER_LENGTH_BYTES) {
				rs485_ctx.response_buf_idx = 0;
			}
		}
	}
}
