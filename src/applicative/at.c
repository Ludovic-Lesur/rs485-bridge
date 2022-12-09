/*
 * at.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#include "at.h"

#include "adc.h"
#include "config.h"
#include "dim.h"
#include "dinfox.h"
#include "error.h"
#include "lptim.h"
#include "mapping.h"
#include "math.h"
#include "nvic.h"
#include "parser.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "rs485.h"
#include "rs485_common.h"
#include "string.h"
#include "types.h"
#include "usart.h"
#include "version.h"

/*** AT local macros ***/

// Commands.
#define AT_COMMAND_BUFFER_SIZE			128
// Parameters separator.
#define AT_CHAR_SEPARATOR				','
// Replies.
#define AT_REPLY_BUFFER_SIZE			128
#define AT_REPLY_END					"\r\n"
#define AT_REPLY_TAB					"     "
#define AT_STRING_VALUE_BUFFER_SIZE		16
// RS485 variables.
#define AT_RS485_COMMAND_HEADER			"*"
#define AT_RS485_NODES_LIST_SIZE		16

/*** AT callbacks declaration ***/

static void _AT_print_ok(void);
static void _AT_print_command_list(void);
static void _AT_print_sw_version(void);
static void _AT_print_error_stack(void);
static void _AT_adc_callback(void);
static void _AT_scan_callback(void);
static void _AT_read_callback(void);
static void _AT_write_callback(void);
static void _AT_send_rs485_command_callback(void);
static void _AT_spy_callback(void);

/*** AT local structures ***/

typedef struct {
	PARSER_mode_t mode;
	char_t* syntax;
	char_t* parameters;
	char_t* description;
	void (*callback)(void);
} AT_command_t;

typedef struct {
	// Command.
	volatile char_t command[AT_COMMAND_BUFFER_SIZE];
	volatile uint32_t command_size;
	volatile uint8_t line_end_flag;
	PARSER_context_t parser;
	// Replies.
	char_t reply[AT_REPLY_BUFFER_SIZE];
	uint32_t reply_size;
	// RS485.
	uint8_t spy_running;
	uint8_t address_parsing_enable;
} AT_context_t;

/*** AT local global variables ***/

static const AT_command_t AT_COMMAND_LIST[] = {
	{PARSER_MODE_COMMAND, "AT", STRING_NULL, "Ping command", _AT_print_ok},
	{PARSER_MODE_COMMAND, "AT?", STRING_NULL, "List all available AT commands", _AT_print_command_list},
	{PARSER_MODE_COMMAND, "AT$V?", STRING_NULL, "Get SW version", _AT_print_sw_version},
	{PARSER_MODE_COMMAND, "AT$ERROR?", STRING_NULL, "Read error stack", _AT_print_error_stack},
	{PARSER_MODE_COMMAND, "AT$RST", STRING_NULL, "Reset MCU", PWR_software_reset},
	{PARSER_MODE_COMMAND, "AT$ADC?", STRING_NULL, "Get ADC measurements", _AT_adc_callback},
	{PARSER_MODE_COMMAND, "AT$SCAN", STRING_NULL, "Scan all slaves connected to the RS485 bus", _AT_scan_callback},
	{PARSER_MODE_HEADER, "AT$R=", "address[hex]", "Read register", _AT_read_callback},
	{PARSER_MODE_HEADER, "AT$W=", "address[hex],value[hex]", "Write register",_AT_write_callback},
	{PARSER_MODE_HEADER, AT_RS485_COMMAND_HEADER, "node_address[hex],command[str]", "Send a command to a specific RS485 node", _AT_send_rs485_command_callback},
	{PARSER_MODE_HEADER, AT_RS485_COMMAND_HEADER, "command[str]", "Send a command over RS485 bus without any address header", _AT_send_rs485_command_callback},
	{PARSER_MODE_HEADER, "AT$SPY=", "enable[bit],address_parsing_enable[bit]", "Start or stop continuous RS485 bus listening", _AT_spy_callback},
};

static AT_context_t at_ctx;

/*** AT local functions ***/

/* GENERIC MACRO TO ADD A CHARACTER TO THE REPLY BUFFER.
 * @param character:	Character to add.
 * @return:				None.
 */
#define _AT_reply_add_char(character) { \
	at_ctx.reply[at_ctx.reply_size] = character; \
	at_ctx.reply_size = (at_ctx.reply_size + 1) % AT_REPLY_BUFFER_SIZE; \
}

/* APPEND A STRING TO THE REPONSE BUFFER.
 * @param tx_string:	String to add.
 * @return:				None.
 */
static void _AT_reply_add_string(char_t* tx_string) {
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		_AT_reply_add_char(*(tx_string++));
	}
}

/* APPEND A VALUE TO THE REPONSE BUFFER.
 * @param tx_value:		Value to add.
 * @param format:       Printing format.
 * @param print_prefix: Print base prefix is non zero.
 * @return:				None.
 */
static void _AT_reply_add_value(int32_t tx_value, STRING_format_t format, uint8_t print_prefix) {
	// Local variables.
	STRING_status_t string_status = STRING_SUCCESS;
	char_t str_value[AT_STRING_VALUE_BUFFER_SIZE];
	uint8_t idx = 0;
	// Reset string.
	for (idx=0 ; idx<AT_STRING_VALUE_BUFFER_SIZE ; idx++) str_value[idx] = STRING_CHAR_NULL;
	// Convert value to string.
	string_status = STRING_value_to_string(tx_value, format, print_prefix, str_value);
	STRING_error_check();
	// Add string.
	_AT_reply_add_string(str_value);
}

/* SEND AT REPONSE OVER AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_reply_send(void) {
	// Local variables.
	USART_status_t usart_status = USART_SUCCESS;
	// Add ending string.
	_AT_reply_add_string(AT_REPLY_END);
	_AT_reply_add_char(STRING_CHAR_NULL);
	// Send response over UART.
	usart_status = USART2_send_string(at_ctx.reply);
	USART_error_check();
	// Flush reply buffer.
	at_ctx.reply_size = 0;
}

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_ok(void) {
	_AT_reply_add_string("OK");
	_AT_reply_send();
}

/* PRINT AN ERROR THROUGH AT INTERFACE.
 * @param error:	Error code to print.
 * @return:			None.
 */
static void _AT_print_error(ERROR_t error) {
	// Add error to stack.
	ERROR_stack_add(error);
	// Print error.
	_AT_reply_add_string("ERROR_");
	if (error < 0x0100) {
		_AT_reply_add_value(0, STRING_FORMAT_HEXADECIMAL, 1);
		_AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 0);
	}
	else {
		_AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
	}
	_AT_reply_send();
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
		_AT_reply_add_string(AT_COMMAND_LIST[idx].syntax);
		// Print parameters.
		_AT_reply_add_string(AT_COMMAND_LIST[idx].parameters);
		_AT_reply_send();
		// Print description.
		_AT_reply_add_string(AT_REPLY_TAB);
		_AT_reply_add_string(AT_COMMAND_LIST[idx].description);
		_AT_reply_send();
	}
	_AT_print_ok();
}

/* PRINT SW VERSION.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_sw_version(void) {
	_AT_reply_add_string("SW");
	_AT_reply_add_value((int32_t) GIT_MAJOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(".");
	_AT_reply_add_value((int32_t) GIT_MINOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(".");
	_AT_reply_add_value((int32_t) GIT_COMMIT_INDEX, STRING_FORMAT_DECIMAL, 0);
	if (GIT_DIRTY_FLAG != 0) {
		_AT_reply_add_string(".d");
	}
	_AT_reply_add_string(" (");
	_AT_reply_add_value((int32_t) GIT_COMMIT_ID, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_reply_add_string(")");
	_AT_reply_send();
	_AT_print_ok();
}

/* PRINT ERROR STACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_error_stack(void) {
	// Local variables.
	ERROR_t error = SUCCESS;
	// Read stack.
	if (ERROR_stack_is_empty() != 0) {
		_AT_reply_add_string("Error stack empty");
	}
	else {
		// Unstack all errors.
		_AT_reply_add_string("[ ");
		do {
			error = ERROR_stack_read();
			if (error != SUCCESS) {
				_AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
				_AT_reply_add_string(" ");
			}
		}
		while (error != SUCCESS);
		_AT_reply_add_string("]");
	}
	_AT_reply_send();
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
	_AT_reply_add_string("ADC running...");
	_AT_reply_send();
	adc1_status = ADC1_perform_measurements();
	ADC1_error_check_print();
	// Read and print data.
	// USB voltage.
	_AT_reply_add_string("Vusb=");
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VUSB_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("mV");
	_AT_reply_send();
	// RS bus voltage.
	_AT_reply_add_string("Vrs=");
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VRS_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("mV");
	_AT_reply_send();
	// MCU voltage.
	_AT_reply_add_string("Vmcu=");
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("mV");
	_AT_reply_send();
	// MCU temperature.
	_AT_reply_add_string("Tmcu=");
	adc1_status = ADC1_get_tmcu(&tmcu_degrees);
	ADC1_error_check_print();
	_AT_reply_add_value((int32_t) tmcu_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("dC");
	_AT_reply_send();
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
	RS485_node_t node_list[AT_RS485_NODES_LIST_SIZE];
	uint8_t number_of_nodes_found = 0;
	uint8_t idx = 0;
	// Check if TX is allowed.
	if (CONFIG_get_tx_mode() == CONFIG_TX_DISABLED) {
		_AT_print_error(ERROR_TX_DISABLED);
		goto errors;
	}
	// Check if continuous listening is not running.
	if (at_ctx.spy_running != 0) {
		_AT_print_error(ERROR_BUSY_SPY_RUNNING);
		goto errors;
	}
	// Perform bus scan.
	_AT_reply_add_string("RS485 bus scan running...");
	_AT_reply_send();
	rs485_status = RS485_set_mode(RS485_MODE_ADDRESSED);
	RS485_error_check_print();
	rs485_status = RS485_scan_nodes(node_list, AT_RS485_NODES_LIST_SIZE, &number_of_nodes_found);
	RS485_error_check_print();
	// Print result.
	_AT_reply_add_value((int32_t) number_of_nodes_found, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(" node(s) found");
	_AT_reply_send();
	for (idx=0 ; idx<number_of_nodes_found ; idx++) {
		// Print address.
		_AT_reply_add_value(node_list[idx].address, STRING_FORMAT_HEXADECIMAL, 1);
		_AT_reply_add_string(" : ");
		// Print board type.
		if (node_list[idx].board_id == DINFOX_BOARD_ID_ERROR) {
			_AT_reply_add_string("Board ID error");
		}
		else {
			if (node_list[idx].board_id >= DINFOX_BOARD_ID_LAST) {
				_AT_reply_add_string("Unknown board ID (");
				_AT_reply_add_value((int32_t) (node_list[idx].board_id), STRING_FORMAT_HEXADECIMAL, 1);
				_AT_reply_add_string(")");
			}
			else {
				_AT_reply_add_string((char_t*) DINFOX_BOARD_ID_NAME[node_list[idx].board_id]);
			}
		}
		_AT_reply_send();
	}
	_AT_print_ok();
errors:
	return;
}

/* AT$R EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_read_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
	int32_t register_address = 0;
	uint8_t generic_u8 = 0;
	int8_t generic_s8 = 0;
	uint32_t generic_u32 = 0;
	// Read address parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_HEXADECIMAL, STRING_CHAR_NULL, &register_address);
	PARSER_error_check_print();
	// Get data.
	switch (register_address) {
	case DINFOX_REGISTER_RS485_ADDRESS:
		nvm_status = NVM_read_byte(NVM_ADDRESS_RS485_ADDRESS, &generic_u8);
		NVM_error_check_print();
		_AT_reply_add_value(generic_u8, STRING_FORMAT_HEXADECIMAL, 0);
		break;
	case DINFOX_REGISTER_BOARD_ID:
		_AT_reply_add_value(DINFOX_BOARD_ID_DIM, STRING_FORMAT_HEXADECIMAL, 0);
		break;
	case DINFOX_REGISTER_RESET:
		_AT_reply_add_value((((RCC -> CSR) >> 24) & 0xFF), STRING_FORMAT_HEXADECIMAL, 0);
		break;
	case DINFOX_REGISTER_SW_VERSION_MAJOR:
		_AT_reply_add_value(GIT_MAJOR_VERSION, STRING_FORMAT_DECIMAL, 0);
		break;
	case DINFOX_REGISTER_SW_VERSION_MINOR:
		_AT_reply_add_value(GIT_MINOR_VERSION, STRING_FORMAT_DECIMAL, 0);
		break;
	case DINFOX_REGISTER_SW_VERSION_COMMIT_INDEX:
		_AT_reply_add_value(GIT_COMMIT_INDEX, STRING_FORMAT_DECIMAL, 0);
		break;
	case DINFOX_REGISTER_SW_VERSION_COMMIT_ID:
		_AT_reply_add_value(GIT_COMMIT_ID, STRING_FORMAT_HEXADECIMAL, 0);
		break;
	case DINFOX_REGISTER_SW_VERSION_DIRTY_FLAG:
		_AT_reply_add_value(GIT_DIRTY_FLAG, STRING_FORMAT_BOOLEAN, 0);
		break;
	case DINFOX_REGISTER_ERROR_STACK:
		_AT_reply_add_value(ERROR_stack_read(), STRING_FORMAT_HEXADECIMAL, 0);
		break;
	case DINFOX_REGISTER_VMCU_MV:
	case DIM_REGISTER_VUSB_MV:
	case DIM_REGISTER_VRS_MV:
		// Perform analog measurements.
		adc1_status = ADC1_perform_measurements();
		ADC1_error_check_print();
		// Note: indexing only works if registers addresses are ordered in the same way as ADC data indexes.
		adc1_status = ADC1_get_data((register_address - DINFOX_REGISTER_VMCU_MV), &generic_u32);
		ADC1_error_check_print();
		_AT_reply_add_value((int32_t) generic_u32, STRING_FORMAT_DECIMAL, 0);
		break;
	case DINFOX_REGISTER_TMCU_DEGREES:
		// Perform analog measurements.
		adc1_status = ADC1_perform_measurements();
		ADC1_error_check_print();
		// Read temperature.
		adc1_status = ADC1_get_tmcu(&generic_s8);
		ADC1_error_check_print();
		_AT_reply_add_value((int32_t) generic_s8, STRING_FORMAT_DECIMAL, 0);
		break;
	default:
		_AT_print_error(ERROR_REGISTER_ADDRESS);
		goto errors;
	}
	// Send response.
	_AT_reply_send();
errors:
	return;
}

/* AT$W EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_write_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	int32_t register_value = 0;
	int32_t register_address = 0;
	// Read address parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_HEXADECIMAL, AT_CHAR_SEPARATOR, &register_address);
	PARSER_error_check_print();
	// Check address.
	if (register_address >= DIM_REGISTER_LAST) {
		_AT_print_error(ERROR_REGISTER_ADDRESS);
		goto errors;
	}
	// Write data.
	switch (register_address) {
	case DINFOX_REGISTER_RS485_ADDRESS:
		// Read new address.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_HEXADECIMAL, STRING_CHAR_NULL, &register_value);
		PARSER_error_check_print();
		// Check value.
		if (register_value > RS485_ADDRESS_LAST) {
			_AT_print_error(ERROR_RS485_ADDRESS);
			goto errors;
		}
		nvm_status = NVM_write_byte(NVM_ADDRESS_RS485_ADDRESS, (uint8_t) register_value);
		NVM_error_check_print();
		break;
	default:
		_AT_print_error(ERROR_REGISTER_READ_ONLY);
		goto errors;
	}
	// Operation completed.
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
	RS485_mode_t rs485_mode = RS485_MODE_DIRECT;
	int32_t node_address = 0;
	uint8_t command_offset = 0;
	// Check if TX is allowed.
	if (CONFIG_get_tx_mode() == CONFIG_TX_DISABLED) {
		_AT_print_error(ERROR_TX_DISABLED);
		goto errors;
	}
	// Check if continuous listening is not running.
	if (at_ctx.spy_running != 0) {
		_AT_print_error(ERROR_BUSY_SPY_RUNNING);
		goto errors;
	}
	// Try parsing node address.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_HEXADECIMAL, AT_CHAR_SEPARATOR, &node_address);
	// Check status to determine mode.
	if (parser_status == PARSER_SUCCESS) {
		// Addressed mode.
		rs485_mode = RS485_MODE_ADDRESSED;
		_AT_reply_add_string("Addressed mode");
		command_offset = at_ctx.parser.separator_idx + 1;
	}
	else {
		// Direct mode.
		rs485_mode = RS485_MODE_DIRECT;
		_AT_reply_add_string("Direct mode");
		command_offset = 1;
	}
	_AT_reply_send();
	// Set mode.
	rs485_status = RS485_set_mode(rs485_mode);
	RS485_error_check_print();
	// Send command.
	command_offset =
	rs485_status = RS485_send_command(node_address, (char_t*) &(at_ctx.command[command_offset]));
	RS485_error_check_print();
errors:
	return;
}

/* AT$SPY COMMAND EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_spy_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_SUCCESS;
	RS485_status_t rs485_status = RS485_SUCCESS;
	int32_t enable = 0;
	int32_t address_parsing_enable = 0;
	// Parse enable parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, &enable);
	PARSER_error_check_print();
	// Parser address parsing enable parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &address_parsing_enable);
	PARSER_error_check_print();
	// Check enable bit.
	if (enable == 0) {
		_AT_reply_add_string("Stopping continuous listening...");
		// Stop continuous listening.
		RS485_stop_spy();
		// Update flag.
		at_ctx.spy_running = 0;
	}
	else {
		_AT_reply_add_string("Starting continuous listening...");
		// Start continuous listening.
		rs485_status = RS485_set_mode(RS485_MODE_DIRECT);
		RS485_error_check_print();
		RS485_start_spy();
		// Update flag.
		at_ctx.spy_running = 1;
	}
	// Update mode.
	at_ctx.address_parsing_enable = (uint8_t) address_parsing_enable;
	// Send response.
	_AT_reply_send();
	_AT_print_ok();
errors:
	return;
}

/* RESET AT PARSER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_reset_parser(void) {
	// Flush buffers.
	at_ctx.command_size = 0;
	at_ctx.reply_size = 0;
	// Reset flag.
	at_ctx.line_end_flag = 0;
	// Reset parser.
	at_ctx.parser.buffer = (char_t*) at_ctx.command;
	at_ctx.parser.buffer_size = 0;
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
	// Update parser length.
	at_ctx.parser.buffer_size = at_ctx.command_size;
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
		_AT_print_error(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
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
	_AT_reset_parser();
	at_ctx.spy_running = 0;
	at_ctx.address_parsing_enable = 0;
	// Enable USART.
	USART2_enable_interrupt();
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_task(void) {
	// Trigger decoding function if line end found.
	if (at_ctx.line_end_flag != 0) {
		// Decode and execute command.
		USART2_disable_interrupt();
		_AT_decode();
		USART2_enable_interrupt();
	}
	// Perform spy task if needed.
	if (at_ctx.spy_running != 0) {
		// Polling task.
		while (RS485_is_frame_available() != 0) {
			// Get frame.
			RS485_spy_task();
		}
	}
}

/* PRINT AN RS485 REPLY OVER AT INTERFACE.
 * @param rs485_reply:	String to print.
 * @return:				None.
 */
void AT_print_rs485_reply(char_t* rs485_reply) {
	// Print reply.
	_AT_reply_add_string(AT_RS485_COMMAND_HEADER);
	_AT_reply_add_string(rs485_reply);
	_AT_reply_send();
}

/* DECODE AND PRINT AN RS485 FRAME.
 * @param rs485_frame:		Frame to print.
 * @param rs485_frame_size:	Size of the frame.
 * @return:					None.
 */
void AT_print_rs485_frame(char_t* rs485_frame, uint8_t rs485_frame_size) {
	// Local variables.
	uint8_t source_address = 0;
	uint8_t destination_address = 0;
	// Check parsing mode.
	if (at_ctx.address_parsing_enable == 0) {
		_AT_reply_add_string(rs485_frame);
	}
	else {
		// Check length.
		if (rs485_frame_size >= RS485_FRAME_FIELD_INDEX_DATA) {
			// Print source address.
			source_address = ((uint8_t) (rs485_frame[RS485_FRAME_FIELD_INDEX_SOURCE_ADDRESS])) & RS485_ADDRESS_MASK;
			_AT_reply_add_value((int32_t) source_address, STRING_FORMAT_HEXADECIMAL, 1);
			_AT_reply_add_string(" > ");
			// Print destination address.
			destination_address = ((uint8_t) (rs485_frame[RS485_FRAME_FIELD_INDEX_DESTINATION_ADDRESS])) & RS485_ADDRESS_MASK;
			_AT_reply_add_value((int32_t) destination_address, STRING_FORMAT_HEXADECIMAL, 1);
			_AT_reply_add_string(" : ");
			// Print command.
			_AT_reply_add_string((char_t*) &(rs485_frame[RS485_FRAME_FIELD_INDEX_DATA]));
		}
	}
	_AT_reply_send();
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
			at_ctx.command[at_ctx.command_size] = STRING_CHAR_NULL;
			at_ctx.line_end_flag = 1;
		}
		else {
			// Store new byte.
			at_ctx.command[at_ctx.command_size] = rx_byte;
			// Manage index.
			at_ctx.command_size = (at_ctx.command_size + 1) % AT_COMMAND_BUFFER_SIZE;
		}
	}
}
