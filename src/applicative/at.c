/*
 * at.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#include <error.h>
#include "at.h"

#include "adc.h"
#include "flash_reg.h"
#include "lpuart.h"
#include "lptim.h"
#include "lvrm.h"
#include "mapping.h"
#include "math.h"
#include "nvic.h"
#include "parser.h"
#include "string.h"
#include "types.h"
#include "usart.h"

/*** AT local macros ***/

// Common macros.
#define AT_COMMAND_LENGTH_MIN			2
#define AT_COMMAND_BUFFER_LENGTH		128
#define AT_RESPONSE_BUFFER_LENGTH		128
#define AT_STRING_VALUE_BUFFER_LENGTH	16
// Parameters separator.
#define AT_CHAR_SEPARATOR				','
// Responses.
#define AT_RESPONSE_END					"\n"
#define AT_RESPONSE_TAB					"     "

/*** AT callbacks declaration ***/

static void _AT_print_ok(void);
static void _AT_print_command_list(void);
static void _AT_read_callback(void);
static void _AT_write_callback(void);

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
	volatile uint8_t command_buf[AT_COMMAND_BUFFER_LENGTH];
	volatile uint32_t command_buf_idx;
	volatile uint8_t line_end_flag;
	PARSER_context_t parser;
	char_t response_buf[AT_RESPONSE_BUFFER_LENGTH];
	uint32_t response_buf_idx;
} AT_context_t;

/*** AT local global variables ***/

static const AT_command_t AT_COMMAND_LIST[] = {
	{PARSER_MODE_COMMAND, "AT", "\0", "Ping command", _AT_print_ok},
	{PARSER_MODE_COMMAND, "AT?", "\0", "List all available AT commands", _AT_print_command_list},
	{PARSER_MODE_HEADER, "AT$R=", "address[dec]", "Read board register", _AT_read_callback},
	{PARSER_MODE_HEADER, "AT$W=", "address[dec]", "Write board register", _AT_write_callback},
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
}

/* AT$R EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_read_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_SUCCESS;
	int32_t register_address = 0;
	// Read address parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &register_address);
	PARSER_error_check();
	// Get data.
	switch (register_address) {
		// TODO
	default:
		goto errors;
		break;
	}
errors:
	return;
}

/* AT$W EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_write_callback(void) {
	// TODO
}

/* RESET AT PARSER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_reset_parser(void) {
	// Reset parsing variables.
	at_ctx.command_buf_idx = 0;
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
	uint32_t idx = 0;
	uint8_t decode_success = 0;
	// Empty or too short command.
	if (at_ctx.command_buf_idx < AT_COMMAND_LENGTH_MIN) {
		_AT_print_status(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND);
		goto errors;
	}
	// Update parser length.
	at_ctx.parser.rx_buf_length = (at_ctx.command_buf_idx - 1); // To ignore line end.
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
	// Init context.
	uint32_t idx = 0;
	for (idx=0 ; idx<AT_COMMAND_BUFFER_LENGTH ; idx++) at_ctx.command_buf[idx] = '\0';
	for (idx=0 ; idx<AT_RESPONSE_BUFFER_LENGTH ; idx++) at_ctx.response_buf[idx] = '\0';
	at_ctx.response_buf_idx = 0;
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
	// Append byte if LF flag is not allready set.
	if (at_ctx.line_end_flag == 0) {
		// Store new byte.
		at_ctx.command_buf[at_ctx.command_buf_idx] = rx_byte;
		// Manage index.
		at_ctx.command_buf_idx++;
		if (at_ctx.command_buf_idx >= AT_COMMAND_BUFFER_LENGTH) {
			at_ctx.command_buf_idx = 0;
		}
	}
	// Set LF flag to trigger decoding.
	if (rx_byte == STRING_CHAR_CR) {
		at_ctx.line_end_flag = 1;
	}
}
