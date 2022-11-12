/*
 * at.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#include "at.h"

#include "adc.h"
#include "config.h"
#include "dinfox.h"
#include "error.h"
#include "lptim.h"
#include "mapping.h"
#include "math.h"
#include "nvic.h"
#include "parser.h"
#include "rs485.h"
#include "string.h"
#include "types.h"
#include "usart.h"
#include "version.h"

/*** AT local macros ***/

// Common macros.
#define AT_COMMAND_LENGTH_MIN			2
#define AT_COMMAND_BUFFER_LENGTH		128
#define AT_RESPONSE_BUFFER_LENGTH		128
#define AT_STRING_VALUE_BUFFER_LENGTH	16
// Parameters separator.
#define AT_CHAR_SEPARATOR				','
// Responses.
#define AT_RESPONSE_END					"\r\n"
#define AT_RESPONSE_TAB					"     "
// RS485 nodes list size.
#define AT_RS485_NODES_LIST_LENGTH		16

/*** AT callbacks declaration ***/

static void _AT_print_ok(void);
static void _AT_print_command_list(void);
static void _AT_print_sw_version(void);
static void _AT_print_error_stack(void);
static void _AT_adc_callback(void);
static void _AT_scan_callback(void);
static void _AT_send_rs485_command_callback(void);

/*** AT local structures ***/

typedef struct {
	PARSER_mode_t mode;
	char_t* syntax;
	char_t* parameters;
	char_t* description;
	void (*callback)(void);
} AT_command_t;

typedef struct {
	// AT command buffer.
	volatile char_t command_buf[AT_COMMAND_BUFFER_LENGTH];
	volatile uint32_t command_buf_idx;
	volatile uint8_t line_end_flag;
	PARSER_context_t parser;
	char_t response_buf[AT_RESPONSE_BUFFER_LENGTH];
	uint32_t response_buf_idx;
} AT_context_t;

/*** AT local global variables ***/

static const AT_command_t AT_COMMAND_LIST[] = {
	{PARSER_MODE_COMMAND, "AT", STRING_NULL, "Ping command", _AT_print_ok},
	{PARSER_MODE_COMMAND, "AT?", STRING_NULL, "List all available AT commands", _AT_print_command_list},
	{PARSER_MODE_COMMAND, "AT$V?", STRING_NULL, "Get SW version", _AT_print_sw_version},
	{PARSER_MODE_COMMAND, "AT$ERROR?", STRING_NULL, "Read error stack", _AT_print_error_stack},
	{PARSER_MODE_COMMAND, "AT$ADC?", STRING_NULL, "Get ADC measurements", _AT_adc_callback},
	{PARSER_MODE_COMMAND, "AT$SCAN", STRING_NULL, "Scan all slaves connected to the RS485 bus", _AT_scan_callback},
	{PARSER_MODE_HEADER, "*", "node_address[hex],command[str]", "Send a command to a specific RS485 node", _AT_send_rs485_command_callback},
	{PARSER_MODE_HEADER, "*", "command[str]", "Send a command over RS485 bus without any address header", _AT_send_rs485_command_callback},
};
static AT_context_t at_ctx;

/*** AT local functions ***/

/* APPEND A STRING TO THE REPONSE BUFFER.
 * @param tx_string:	String to add.
 * @return:				None.
 */
static void _AT_response_add_string(char_t* tx_string) {
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		at_ctx.response_buf[at_ctx.response_buf_idx++] = *(tx_string++);
		// Manage rollover.
		if (at_ctx.response_buf_idx >= AT_RESPONSE_BUFFER_LENGTH) {
			at_ctx.response_buf_idx = 0;
		}
	}
}

/* APPEND A VALUE TO THE REPONSE BUFFER.
 * @param tx_value:		Value to add.
 * @param format:       Printing format.
 * @param print_prefix: Print base prefix is non zero.
 * @return:				None.
 */
static void _AT_response_add_value(int32_t tx_value, STRING_format_t format, uint8_t print_prefix) {
	// Local variables.
	char_t str_value[AT_STRING_VALUE_BUFFER_LENGTH];
	uint8_t idx = 0;
	// Reset string.
	for (idx=0 ; idx<AT_STRING_VALUE_BUFFER_LENGTH ; idx++) str_value[idx] = STRING_CHAR_NULL;
	// Convert value to string.
	STRING_value_to_string(tx_value, format, print_prefix, str_value);
	// Add string.
	_AT_response_add_string(str_value);
}

/* SEND AT REPONSE OVER AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_response_send(void) {
	// Local variables.
	uint32_t idx = 0;
	// Send response over UART.
	USART2_send_string(at_ctx.response_buf);
	// Flush response buffer.
	for (idx=0 ; idx<AT_RESPONSE_BUFFER_LENGTH ; idx++) at_ctx.response_buf[idx] = STRING_CHAR_NULL;
	at_ctx.response_buf_idx = 0;
}

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_ok(void) {
	_AT_response_add_string("OK");
	_AT_response_add_string(AT_RESPONSE_END);
	_AT_response_send();
}

/* PRINT AN ERROR THROUGH AT INTERFACE.
 * @param error_source:	8-bits error source.
 * @param error_code:	16-bits error code.
 * @return:				None.
 */
static void _AT_print_status(ERROR_t status) {
	_AT_response_add_string("ERROR ");
	if (status < 0x0100) {
		_AT_response_add_value(0, STRING_FORMAT_HEXADECIMAL, 1);
		_AT_response_add_value(status, STRING_FORMAT_HEXADECIMAL, 0);
	}
	else {
		_AT_response_add_value(status, STRING_FORMAT_HEXADECIMAL, 1);
	}
	_AT_response_add_string(AT_RESPONSE_END);
	_AT_response_send();
}

/* PRINT ALL SUPPORTED AT COMMANDS.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_command_list(void) {
	// Local variables.
	uint32_t idx = 0;
	// Commands loop.
	for (idx=0 ; idx<(sizeof(AT_COMMAND_LIST) / sizeof(AT_command_t)) ; idx++) {
		// Print syntax.
		_AT_response_add_string(AT_COMMAND_LIST[idx].syntax);
		// Print parameters.
		_AT_response_add_string(AT_COMMAND_LIST[idx].parameters);
		_AT_response_add_string(AT_RESPONSE_END);
		// Print description.
		_AT_response_add_string(AT_RESPONSE_TAB);
		_AT_response_add_string(AT_COMMAND_LIST[idx].description);
		_AT_response_add_string(AT_RESPONSE_END);
		_AT_response_send();
	}
	_AT_print_ok();
}

/* PRINT SW VERSION.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_sw_version(void) {
	_AT_response_add_string("SW");
	_AT_response_add_value(GIT_MAJOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_response_add_string(".");
	_AT_response_add_value(GIT_MINOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_response_add_string(".");
	_AT_response_add_value(GIT_COMMIT_INDEX, STRING_FORMAT_DECIMAL, 0);
	if (GIT_DIRTY_FLAG != 0) {
		_AT_response_add_string(".d");
	}
	_AT_response_add_string(" (");
	_AT_response_add_value(GIT_COMMIT_ID, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_response_add_string(")");
	_AT_response_add_string(AT_RESPONSE_END);
	_AT_response_send();
	_AT_print_ok();
}

/* PRINT ERROR STACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_error_stack(void) {
	// Local variables.
	ERROR_t error_stack[ERROR_STACK_DEPTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	uint32_t idx = 0;
	// Read stack.
	ERROR_stack_read(error_stack);
	// Print stack.
	_AT_response_add_string("[ ");
	for (idx=0 ; idx<ERROR_STACK_DEPTH ; idx++) {
		_AT_response_add_value((int32_t) error_stack[idx], STRING_FORMAT_HEXADECIMAL, 1);
		_AT_response_add_string(" ");
	}
	_AT_response_add_string("]");
	_AT_response_add_string(AT_RESPONSE_END);
	_AT_response_send();
	_AT_print_ok();
}

/* AT$ADC? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_adc_callback(void) {
	// Local variables.
	ADC_status_t adc1_status = ADC_SUCCESS;
	uint32_t voltage_mv = 0;
	int8_t tmcu_degrees = 0;
	// Trigger internal ADC conversions.
	_AT_response_add_string("ADC running...");
	_AT_response_add_string(AT_RESPONSE_END);
	_AT_response_send();
	adc1_status = ADC1_perform_measurements();
	ADC1_error_check_print();
	// Read and print data.
	// USB voltage.
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VUSB_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_response_add_string("Vusb=");
	_AT_response_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	// RS bus voltage.
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VRS_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_response_add_string("mV Vrs=");
	_AT_response_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	// MCU voltage.
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_response_add_string("mV Vmcu=");
	_AT_response_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	// MCU temperature.
	adc1_status = ADC1_get_tmcu(&tmcu_degrees);
	ADC1_error_check_print();
	_AT_response_add_string("mV Tmcu=");
	_AT_response_add_value((int32_t) tmcu_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_response_add_string("dC");
	_AT_response_add_string(AT_RESPONSE_END);
	_AT_response_send();
	_AT_print_ok();
errors:
	return;
}

/* AT$SCAN EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_scan_callback(void) {
	// Local variables.
	RS485_status_t rs485_status = RS485_SUCCESS;
	RS485_node_t node_list[AT_RS485_NODES_LIST_LENGTH];
	uint8_t number_of_nodes_found = 0;
	uint8_t idx = 0;
	// Check if TX is allowed.
	if (CONFIG_get_tx_mode() == CONFIG_TX_DISABLED) {
		_AT_print_status(ERROR_TX_DISABLED);
		goto errors;
	}
	// Perform bus scan.
	_AT_response_add_string("RS485 bus scan running...");
	_AT_response_add_string(AT_RESPONSE_END);
	_AT_response_send();
	rs485_status = RS485_scan_nodes(node_list, AT_RS485_NODES_LIST_LENGTH, &number_of_nodes_found);
	RS485_error_check_print();
	// Print result.
	_AT_response_add_value(number_of_nodes_found, STRING_FORMAT_DECIMAL, 0);
	_AT_response_add_string(" node(s) found");
	_AT_response_add_string(AT_RESPONSE_END);
	_AT_response_send();
	for (idx=0 ; idx<number_of_nodes_found ; idx++) {
		// Print address.
		_AT_response_add_value(node_list[idx].address, STRING_FORMAT_HEXADECIMAL, 1);
		_AT_response_add_string(" : ");
		// Print board type.
		if (node_list[idx].board_id >= DINFOX_BOARD_ID_LAST) {
			_AT_response_add_string("Unknown board ID (");
			_AT_response_add_value(node_list[idx].board_id, STRING_FORMAT_HEXADECIMAL, 1);
			_AT_response_add_string(")");
		}
		else {
			_AT_response_add_string((char_t*) DINFOX_BOARD_ID_NAME[node_list[idx].board_id]);
		}
		_AT_response_add_string(AT_RESPONSE_END);
		_AT_response_send();
	}
	_AT_print_ok();
errors:
	return;
}

/* RS485 COMMAND EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_send_rs485_command_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_SUCCESS;
	RS485_status_t rs485_status = RS485_SUCCESS;
	LPUART_mode_t lpuart_mode = LPUART_MODE_DIRECT;
	char_t rs485_command[AT_COMMAND_BUFFER_LENGTH];
	char_t rs485_response[AT_RESPONSE_BUFFER_LENGTH];
	int32_t node_address = 0;
	uint8_t idx = 0;
	// Check if TX is allowed.
	if (CONFIG_get_tx_mode() == CONFIG_TX_DISABLED) {
		_AT_print_status(ERROR_TX_DISABLED);
		goto errors;
	}
	// Try parsing node address.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_HEXADECIMAL, AT_CHAR_SEPARATOR, &node_address);
	// Check status to determine mode.
	lpuart_mode = (parser_status == PARSER_SUCCESS) ? LPUART_MODE_NODE : LPUART_MODE_DIRECT;
	// Reset RS485 buffers.
	for (idx=0 ; idx<AT_COMMAND_BUFFER_LENGTH ; idx++) rs485_command[idx] = STRING_CHAR_NULL;
	for (idx=0 ; idx<AT_RESPONSE_BUFFER_LENGTH ; idx++) rs485_response[idx] = STRING_CHAR_NULL;
	// Copy command.
	idx = 0;
	while (at_ctx.command_buf[at_ctx.parser.separator_idx + 1 + idx] != STRING_CHAR_NULL) {
		rs485_command[idx] = at_ctx.command_buf[at_ctx.parser.separator_idx + 1 + idx];
		idx++;
	}
	rs485_command[idx] = STRING_CHAR_CR;
	// RS485 address found.
	rs485_status = RS485_send_command(lpuart_mode, node_address, (char_t*) rs485_command, (char_t*) rs485_response, AT_RESPONSE_BUFFER_LENGTH);
	RS485_error_check_print();
	// Print response.
	_AT_response_add_string(rs485_response);
	_AT_response_add_string(AT_RESPONSE_END);
	_AT_response_send();
errors:
	return;
}

/* RESET AT PARSER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_reset_parser(void) {
	// Local variables.
	uint8_t idx = 0;
	// Reset buffers
	for (idx=0 ; idx<AT_COMMAND_BUFFER_LENGTH ; idx++) at_ctx.command_buf[idx] = STRING_CHAR_NULL;
	for (idx=0 ; idx<AT_RESPONSE_BUFFER_LENGTH ; idx++) at_ctx.response_buf[idx] = STRING_CHAR_NULL;
	// Reset parsing variables.
	at_ctx.command_buf_idx = 0;
	at_ctx.response_buf_idx = 0;
	at_ctx.line_end_flag = 0;
	at_ctx.parser.rx_buf = (char_t*) at_ctx.command_buf;
	at_ctx.parser.rx_buf_length = 0;
	at_ctx.parser.separator_idx = 0;
	at_ctx.parser.start_idx = 0;
}

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_decode(void) {
	// Local variables.
	uint8_t idx = 0;
	uint8_t decode_success = 0;
	// Empty or too short command.
	if (at_ctx.command_buf_idx < AT_COMMAND_LENGTH_MIN) {
		_AT_print_status(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND);
		goto errors;
	}
	// Update parser length.
	at_ctx.parser.rx_buf_length = at_ctx.command_buf_idx;
	// Loop on available commands.
	for (idx=0 ; idx<(sizeof(AT_COMMAND_LIST) / sizeof(AT_command_t)) ; idx++) {
		// Check type.
		if (PARSER_compare(&at_ctx.parser, AT_COMMAND_LIST[idx].mode, AT_COMMAND_LIST[idx].syntax) == PARSER_SUCCESS) {
			// Execute callback and exit.
			AT_COMMAND_LIST[idx].callback();
			decode_success = 1;
			break;
		}
	}
	if (decode_success == 0) {
		_AT_print_status(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
		goto errors;
	}
errors:
	_AT_reset_parser();
	return;
}

/*** AT functions ***/

/* INIT AT MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_init(void) {
	// Reset parser.
	_AT_reset_parser();
	// Enable LPUART.
	USART2_enable_interrupt();
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_task(void) {
	// Trigger decoding function if line end found.
	if (at_ctx.line_end_flag != 0) {
		USART2_disable_interrupt();
		_AT_decode();
		USART2_enable_interrupt();
	}
}

/* FILL AT COMMAND BUFFER WITH A NEW BYTE (CALLED BY USART INTERRUPT).
 * @param rx_byte:	Incoming byte.
 * @return:			None.
 */
void AT_fill_rx_buffer(uint8_t rx_byte) {
	// Append byte if line end flag is not allready set.
	if (at_ctx.line_end_flag == 0) {
		// Check ending characters.
		if ((rx_byte == STRING_CHAR_CR) || (rx_byte == STRING_CHAR_LF)) {
			at_ctx.command_buf[at_ctx.command_buf_idx] = STRING_CHAR_NULL;
			at_ctx.line_end_flag = 1;
		}
		else {
			// Store new byte.
			at_ctx.command_buf[at_ctx.command_buf_idx] = rx_byte;
			// Manage index.
			at_ctx.command_buf_idx++;
			if (at_ctx.command_buf_idx >= AT_COMMAND_BUFFER_LENGTH) {
				at_ctx.command_buf_idx = 0;
			}
		}
	}
}
