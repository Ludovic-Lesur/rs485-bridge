/*
 * rs485.c
 *
 *  Created on: 28 oct 2022
 *      Author: Ludo
 */

#include "rs485.h"

#include "at.h"
#include "dinfox.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "rs485_common.h"
#include "string.h"

/*** RS485 local macros ***/

#define RS485_BUFFER_SIZE_BYTES			128
#define RS485_REPLY_BUFFER_DEPTH		32

#define RS485_REPLY_PARSING_DELAY_MS	10
#define RS485_REPLY_TIMEOUT_MS			100
#define RS485_SEQUENCE_TIMEOUT_MS		1000

#define RS485_REPLY_OK					"OK"
#define RS485_REPLY_ERROR				"ERROR"

/*** RS485 local structures ***/

typedef enum {
	RS485_REPLY_TYPE_RAW = 0,
	RS485_REPLY_TYPE_OK,
	RS485_REPLY_TYPE_VALUE,
	RS485_REPLY_TYPE_LAST
} RS485_reply_type_t;

typedef struct {
	RS485_reply_type_t type;
	uint8_t expected_source_address;
	STRING_format_t format; // For value type.
	uint32_t timeout_ms;
} RS485_reply_input_t;

typedef struct {
	int32_t value; // For value type.
	uint8_t error_flag;
} RS485_reply_output_t;

typedef struct {
	volatile char_t buffer[RS485_BUFFER_SIZE_BYTES];
	volatile uint8_t size;
	volatile uint8_t line_end_flag;
	PARSER_context_t parser;
} RS485_reply_buffer_t;

typedef struct {
	RS485_mode_t mode;
	// Command buffer.
	char_t command[RS485_BUFFER_SIZE_BYTES];
	// Response buffers.
	RS485_reply_buffer_t reply[RS485_REPLY_BUFFER_DEPTH];
	uint8_t reply_write_idx;
	uint8_t reply_read_idx;
} RS485_context_t;

/*** RS485 local global variables ***/

static RS485_context_t rs485_ctx;

/*** RS485 local functions ***/

/* SPECIFIC MACRO USED IN RS485 SEND COMMAND FUNCTION.
 * @param character:	Character to add.
 * @return:				None.
 */
#define _RS485_add_char_to_reply(character) { \
	if (global_idx >= reply_size_bytes) { \
		status = RS485_ERROR_BUFFER_OVERFLOW; \
		goto errors; \
	} \
	reply[global_idx] = character; \
	global_idx++; \
}

/* BUILD RS485 COMMAND.
 * @param command:	Raw command to send.
 * @return:			Function execution status.
 */
static RS485_status_t _RS485_build_command(char_t* command) {
	// Local variables.
	RS485_status_t status = RS485_SUCCESS;
	uint8_t idx = 0;
	// Check parameter.
	if (command == NULL) {
		status = RS485_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Copy command into local buffer.
	while (command[idx] != STRING_CHAR_NULL) {
		rs485_ctx.command[idx] = command[idx];
		idx++;
	}
	// Add ending character.
	rs485_ctx.command[idx++] = RS485_FRAME_END;
	rs485_ctx.command[idx++] = STRING_CHAR_NULL;
errors:
	return status;
}

/* RESET RS485 REPLY BUFFER.
 * @param reply_index:	Reply index to reset.
 * @return:				None.
 */
static void _RS485_reset_reply(uint8_t reply_index) {
	// Flush buffer.
	rs485_ctx.reply[reply_index].size = 0;
	// Reset flag.
	rs485_ctx.reply[reply_index].line_end_flag = 0;
	// Reset parser.
	rs485_ctx.reply[reply_index].parser.buffer = (char_t*) rs485_ctx.reply[reply_index].buffer;
	rs485_ctx.reply[reply_index].parser.buffer_size = 0;
	rs485_ctx.reply[reply_index].parser.separator_idx = 0;
	rs485_ctx.reply[reply_index].parser.start_idx = 0;
}

/* RESET RS485 PARSER.
 * @param:	None.
 * @return:	None.
 */
static void _RS485_reset_replies(void) {
	// Local variabless.
	uint8_t rep_idx = 0;
	// Reset replys buffers.
	for (rep_idx=0 ; rep_idx<RS485_REPLY_BUFFER_DEPTH ; rep_idx++) {
		_RS485_reset_reply(rep_idx);
	}
	// Reset index and count.
	rs485_ctx.reply_write_idx = 0;
	rs485_ctx.reply_read_idx = 0;
}

/* WAIT FOR RECEIVING A VALUE.
 * @param reply_in_ptr:		Pointer to the reply input parameters.
 * @param reply_out_ptr:	Pointer to the reply output data.
 * @return status:			Function execution status.
 */
static RS485_status_t _RS485_wait_reply(RS485_reply_input_t* reply_in_ptr, RS485_reply_output_t* reply_out_ptr) {
	// Local variables.
	RS485_status_t status = RS485_SUCCESS;
	PARSER_status_t parser_status = PARSER_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint8_t idx = 0;
	uint32_t reply_time_ms = 0;
	uint32_t sequence_time_ms = 0;
	uint8_t reply_count = 0;
	// Check parameters.
	if ((reply_in_ptr == NULL) || (reply_out_ptr == NULL)) {
		status = RS485_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Reset output data.
	(reply_out_ptr -> value) = 0;
	(reply_out_ptr -> error_flag) = 0;
	// Main reception loop.
	while (1) {
		// Delay.
		lptim1_status = LPTIM1_delay_milliseconds(RS485_REPLY_PARSING_DELAY_MS, 0);
		LPTIM1_status_check(RS485_ERROR_BASE_LPTIM);
		reply_time_ms += RS485_REPLY_PARSING_DELAY_MS;
		sequence_time_ms += RS485_REPLY_PARSING_DELAY_MS;
		// Loop on all replys.
		for (idx=0 ; idx<RS485_REPLY_BUFFER_DEPTH ; idx++) {
			// Check line end flag.
			if (rs485_ctx.reply[idx].line_end_flag != 0) {
				// Increment parsing count.
				reply_count++;
				// Reset time and flag.
				reply_time_ms = 0;
				rs485_ctx.reply[idx].line_end_flag = 0;
				// Check mode.
				if (rs485_ctx.mode == RS485_MODE_ADDRESSED) {
					// Check source address.
					if (rs485_ctx.reply[idx].buffer[0] != (reply_in_ptr -> expected_source_address)) {
						status = RS485_ERROR_SOURCE_ADDRESS_MISMATCH;
						continue;
					}
					// Skip source address before parsing.
					rs485_ctx.reply[idx].parser.buffer = (char_t*) &(rs485_ctx.reply[idx].buffer[1]);
					rs485_ctx.reply[idx].parser.buffer_size = (rs485_ctx.reply[idx].size > 0) ? (rs485_ctx.reply[idx].size - 1) : 0;
				}
				else {
					// Update buffer length.
					rs485_ctx.reply[idx].parser.buffer_size = rs485_ctx.reply[idx].size;
				}
				// Parse reply.
				switch (reply_in_ptr -> type) {
				case RS485_REPLY_TYPE_RAW:
					// Do not parse.
					parser_status = PARSER_SUCCESS;
					break;
				case RS485_REPLY_TYPE_OK:
					// Compare to reference string.
					parser_status = PARSER_compare(&rs485_ctx.reply[idx].parser, PARSER_MODE_COMMAND, RS485_REPLY_OK);
					break;
				case RS485_REPLY_TYPE_VALUE:
					// Parse value.
					parser_status = PARSER_get_parameter(&rs485_ctx.reply[idx].parser, (reply_in_ptr -> format), STRING_CHAR_NULL, &(reply_out_ptr -> value));
					break;
				default:
					status = RS485_ERROR_REPLY_TYPE;
					goto errors;
				}
				// Check status.
				if (parser_status == PARSER_SUCCESS) {
					// Update status.
					status = RS485_SUCCESS;
					// In raw mode, let the function run until one of the 2 timeouts is reached.
					// In other modes, exit as soon as the value was successfully parsed.
					if ((reply_in_ptr -> type) != RS485_REPLY_TYPE_RAW) goto errors; // Not an error but to exit loop.
				}
				else {
					status = (RS485_ERROR_BASE_PARSER + parser_status);
				}
				// Check error.
				parser_status = PARSER_compare(&rs485_ctx.reply[idx].parser, PARSER_MODE_COMMAND, RS485_REPLY_ERROR);
				if (parser_status == PARSER_SUCCESS) {
					// Update output data.
					(reply_out_ptr -> error_flag) = 1;
					// Exit.
					status = RS485_SUCCESS;
					goto errors;
				}
			}
		}
		// Exit if timeout.
		if (reply_time_ms > (reply_in_ptr -> timeout_ms)) {
			// Set status to timeout if none reply has been received, otherwise the parser error code is returned.
			if (reply_count == 0) {
				status = RS485_ERROR_REPLY_TIMEOUT;
			}
			goto errors;
		}
		if (sequence_time_ms > RS485_SEQUENCE_TIMEOUT_MS) {
			// Set status to timeout in any case.
			status = RS485_ERROR_SEQUENCE_TIMEOUT;
			goto errors;
		}
	}
errors:
	return status;
}

/*** RS485 functions ***/

/* SET RS485 MODE.
 * @param mode:		Transmission mode (direct or addressed).
 * @return status:	Function execution status.
 */
RS485_status_t RS485_set_mode(RS485_mode_t mode) {
	// Local variables.
	RS485_status_t status = RS485_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	// Check parameter.
	if (mode >= RS485_MODE_LAST) {
		status = RS485_ERROR_MODE;
		goto errors;
	}
	// Configure LPUART.
	lpuart1_status = LPUART1_set_mode(mode);
	LPUART1_status_check(RS485_ERROR_BASE_LPUART);
	// Update context.
	rs485_ctx.mode = mode;
errors:
	return status;
}

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
	RS485_reply_input_t reply_in;
	RS485_reply_output_t reply_out;
	uint8_t node_address = 0;
	uint8_t node_list_idx = 0;
	// Check parameters.
	if ((nodes_list == NULL) || (number_of_nodes_found == NULL)) {
		status = RS485_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (node_list_size == 0) {
		status = RS485_ERROR_NULL_SIZE;
		goto errors;
	}
	// Reset result.
	(*number_of_nodes_found) = 0;
	// Build reply input common parameters.
	reply_in.format = STRING_FORMAT_HEXADECIMAL;
	reply_in.timeout_ms = RS485_REPLY_TIMEOUT_MS;
	// Loop on all addresses.
	for (node_address=0 ; node_address<=RS485_ADDRESS_LAST ; node_address++) {
		// Reset parser.
		_RS485_reset_replies();
		reply_in.type = RS485_REPLY_TYPE_OK;
		reply_in.expected_source_address = node_address;
		// Build command.
		_RS485_build_command("RS");
		// Send ping command.
		lpuart1_status = LPUART1_send_command(node_address, rs485_ctx.command);
		LPUART1_status_check(RS485_ERROR_BASE_LPUART);
		// Wait reply.
		LPUART1_enable_rx();
		status = _RS485_wait_reply(&reply_in, &reply_out);
		LPUART1_disable_rx();
		if (status == RS485_SUCCESS) {
			// Node found (even if an error was returned after ping command).
			(*number_of_nodes_found)++;
			// Store address and reset board ID.
			if (node_list_idx < node_list_size) {
				nodes_list[node_list_idx].address = node_address;
				nodes_list[node_list_idx].board_id = DINFOX_BOARD_ID_ERROR;
			}
			// Reset parser.
			_RS485_reset_replies();
			reply_in.type = RS485_REPLY_TYPE_VALUE;
			// Build command.
			_RS485_build_command("RS$R=01");
			// Get board ID.
			lpuart1_status = LPUART1_send_command(node_address, rs485_ctx.command);
			LPUART1_status_check(RS485_ERROR_BASE_LPUART);
			// Wait reply.
			LPUART1_enable_rx();
			status = _RS485_wait_reply(&reply_in, &reply_out);
			LPUART1_disable_rx();
			if ((status == RS485_SUCCESS) && (reply_out.error_flag == 0)) {
				// Update board ID.
				nodes_list[node_list_idx].board_id = (uint8_t) reply_out.value;
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
 * @param node_address:	Slave address.
 * @param command:		Command to send.
 * @return status:		Function execution status.
 */
RS485_status_t RS485_send_command(uint8_t node_address, char_t* command) {
	// Local variables.
	RS485_status_t status = RS485_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	RS485_reply_input_t reply_in;
	RS485_reply_output_t reply_out;
	uint8_t rep_idx = 0;
	// Check parameters.
	if (command == NULL) {
		status = RS485_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Build command.
	_RS485_build_command(command);
	// Build reply input common parameters.
	reply_in.type = RS485_REPLY_TYPE_RAW;
	reply_in.expected_source_address = node_address;
	reply_in.format = STRING_FORMAT_HEXADECIMAL;
	reply_in.timeout_ms = RS485_REPLY_TIMEOUT_MS;
	// Reset parser.
	_RS485_reset_replies();
	// Send command.
	lpuart1_status = LPUART1_send_command(node_address, rs485_ctx.command);
	LPUART1_status_check(RS485_ERROR_BASE_LPUART);
	// Wait reply.
	LPUART1_enable_rx();
	status = _RS485_wait_reply(&reply_in, &reply_out);
	LPUART1_disable_rx();
	if (status != RS485_SUCCESS) goto errors;
	// Replies loop.
	for (rep_idx=0 ; rep_idx<RS485_REPLY_BUFFER_DEPTH ; rep_idx++) {
		// Check reply size.
		if (rs485_ctx.reply[rep_idx].size > RS485_ADDRESS_SIZE_BYTES) {
			// Skip source address.
			AT_print_rs485_reply((char_t*) &(rs485_ctx.reply[rep_idx].buffer[RS485_FRAME_FIELD_INDEX_SOURCE_ADDRESS]));
		}
	}
errors:
	// Disable receiver.
	LPUART1_disable_rx();
	return status;
}

/* START CONTINUOUS LISTENING.
 * @param:	None.
 * @return:	None.
 */
void RS485_start_spy(void) {
	// Reset parser.
	_RS485_reset_replies();
	// Enable receiver.
	LPUART1_enable_rx();
}

/* STOP CONTINUOUS LISTENING.
 * @param:	None.
 * @return:	None.
 */
void RS485_stop_spy(void) {
	// Disable receiver.
	LPUART1_disable_rx();
}

/* CONTINUOUS LISTENING TASK.
 * @param:	None.
 * @return:	None.
 */
void RS485_spy_task(void) {
	// Check line end flag on current reply.
	if (rs485_ctx.reply[rs485_ctx.reply_read_idx].line_end_flag != 0) {
		// Print frame.
		AT_print_rs485_frame((char_t*) rs485_ctx.reply[rs485_ctx.reply_read_idx].buffer, rs485_ctx.reply[rs485_ctx.reply_read_idx].size);
		// Reset reply.
		_RS485_reset_reply(rs485_ctx.reply_read_idx);
		// Increment read index.
		rs485_ctx.reply_read_idx = (rs485_ctx.reply_read_idx + 1) % RS485_REPLY_BUFFER_DEPTH;
	}
}

/* CHECK IF AN RS485 FRAME IS AVAILABLE.
 * @param:					None.
 * @return frame_available:	0 if the reply buffer is empty, 1 otherwise.
 */
uint8_t RS485_is_frame_available(void) {
	// Local variables.
	uint8_t frame_available = (rs485_ctx.reply_read_idx == rs485_ctx.reply_write_idx) ? 0 : 1;
	return frame_available;
}

/* FILL RS485 BUFFER WITH A NEW BYTE (CALLED BY LPUART INTERRUPT).
 * @param rx_byte:	Incoming byte.
 * @return:			None.
 */
void RS485_fill_rx_buffer(uint8_t rx_byte) {
	// Read current index.
	uint8_t idx = rs485_ctx.reply[rs485_ctx.reply_write_idx].size;
	// Check ending characters.
	if (rx_byte == RS485_FRAME_END) {
		// Set flag on current buffer.
		rs485_ctx.reply[rs485_ctx.reply_write_idx].buffer[idx] = STRING_CHAR_NULL;
		rs485_ctx.reply[rs485_ctx.reply_write_idx].line_end_flag = 1;
		// Switch buffer.
		rs485_ctx.reply_write_idx = (rs485_ctx.reply_write_idx + 1) % RS485_REPLY_BUFFER_DEPTH;
	}
	else {
		// Store incoming byte.
		rs485_ctx.reply[rs485_ctx.reply_write_idx].buffer[idx] = rx_byte;
		// Manage index.
		idx = (idx + 1) % RS485_BUFFER_SIZE_BYTES;
		rs485_ctx.reply[rs485_ctx.reply_write_idx].size = idx;
	}
}
