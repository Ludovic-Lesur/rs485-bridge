/*
 * at.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#include "at_usb.h"

#include "at_bus.h"
#include "adc.h"
#include "config.h"
#include "common_reg.h"
#include "dinfox.h"
#include "error.h"
#include "lbus.h"
#include "lptim.h"
#include "mapping.h"
#include "math.h"
#include "node.h"
#include "nvic.h"
#include "parser.h"
#include "pwr.h"
#include "r4s8cr.h"
#include "rcc_reg.h"
#include "string.h"
#include "types.h"
#include "usart.h"
#include "version.h"

/*** AT local macros ***/

// Commands.
#define AT_USB_COMMAND_BUFFER_SIZE			128
// Parameters separator.
#define AT_USB_CHAR_SEPARATOR				','
// Replies.
#define AT_USB_REPLY_BUFFER_SIZE			128
#define AT_USB_REPLY_END					"\r\n"
#define AT_USB_REPLY_TAB					"     "
#define AT_USB_NODE_TRANSFER_HEADER			"*"
#define AT_USB_STRING_VALUE_BUFFER_SIZE		16
// DINFox boards name.
static const char_t* DINFOX_BOARD_ID_NAME[DINFOX_BOARD_ID_LAST] = {"LVRM", "BPSM", "DDRM", "UHFM", "GPSM", "SM", "DIM", "RRM", "DMM", "MPMCM", "R4S8CR"};

/*** AT callbacks declaration ***/

static void _AT_USB_print_ok(void);
static void _AT_USB_print_command_list(void);
static void _AT_USB_print_sw_version(void);
static void _AT_USB_print_error_stack(void);
static void _AT_USB_adc_callback(void);
static void _AT_USB_node_scan_callback(void);
static void _AT_USB_node_command_callback(void);
static void _AT_USB_node_get_protocol_callback(void);
static void _AT_USB_node_set_protocol_callback(void);

/*** AT local structures ***/

typedef struct {
	PARSER_mode_t mode;
	char_t* syntax;
	char_t* parameters;
	char_t* description;
	void (*callback)(void);
} AT_USB_command_t;

typedef struct {
	// Command.
	volatile char_t command[AT_USB_COMMAND_BUFFER_SIZE];
	volatile uint32_t command_size;
	volatile uint8_t line_end_flag;
	PARSER_context_t parser;
	// Replies.
	char_t reply[AT_USB_REPLY_BUFFER_SIZE];
	uint32_t reply_size;
	// Protocol.
	NODE_protocol_t node_protocol;
} AT_USB_context_t;

/*** AT local global variables ***/

static const AT_USB_command_t AT_USB_COMMAND_LIST[] = {
	{PARSER_MODE_COMMAND, "AT", STRING_NULL, "Ping command", _AT_USB_print_ok},
	{PARSER_MODE_COMMAND, "AT?", STRING_NULL, "List all available AT commands", _AT_USB_print_command_list},
	{PARSER_MODE_COMMAND, "AT$V?", STRING_NULL, "Get SW version", _AT_USB_print_sw_version},
	{PARSER_MODE_COMMAND, "AT$ERROR?", STRING_NULL, "Read error stack", _AT_USB_print_error_stack},
	{PARSER_MODE_COMMAND, "AT$RST", STRING_NULL, "Reset MCU", PWR_software_reset},
	{PARSER_MODE_COMMAND, "AT$ADC?", STRING_NULL, "Get ADC measurements", _AT_USB_adc_callback},
	{PARSER_MODE_COMMAND, "AT$SCAN", STRING_NULL, "Scan all slaves connected to the RS485 bus", _AT_USB_node_scan_callback},
	{PARSER_MODE_COMMAND, "AT$PR?", STRING_NULL, "Get current node protocol", _AT_USB_node_get_protocol_callback},
	{PARSER_MODE_HEADER, "AT$PR=", "protocol[dec]", "Set node protocol (0=AT_BUS, 1=R4S8CR)", _AT_USB_node_set_protocol_callback},
	{PARSER_MODE_HEADER, AT_USB_NODE_TRANSFER_HEADER, "node_addr[hex],command[str]", "Send node command", _AT_USB_node_command_callback},
};

static AT_USB_context_t at_usb_ctx;

/*** AT local functions ***/

/* GENERIC MACRO TO ADD A CHARACTER TO THE REPLY BUFFER.
 * @param character:	Character to add.
 * @return:				None.
 */
#define _AT_USB_reply_add_char(character) { \
	at_usb_ctx.reply[at_usb_ctx.reply_size] = character; \
	at_usb_ctx.reply_size = (at_usb_ctx.reply_size + 1) % AT_USB_REPLY_BUFFER_SIZE; \
}

/* APPEND A STRING TO THE REPONSE BUFFER.
 * @param tx_string:	String to add.
 * @return:				None.
 */
static void _AT_USB_reply_add_string(char_t* tx_string) {
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		_AT_USB_reply_add_char(*(tx_string++));
	}
}

/* APPEND A VALUE TO THE REPONSE BUFFER.
 * @param tx_value:		Value to add.
 * @param format:       Printing format.
 * @param print_prefix: Print base prefix is non zero.
 * @return:				None.
 */
static void _AT_USB_reply_add_value(int32_t tx_value, STRING_format_t format, uint8_t print_prefix) {
	// Local variables.
	STRING_status_t string_status = STRING_SUCCESS;
	char_t str_value[AT_USB_STRING_VALUE_BUFFER_SIZE];
	uint8_t idx = 0;
	// Reset string.
	for (idx=0 ; idx<AT_USB_STRING_VALUE_BUFFER_SIZE ; idx++) str_value[idx] = STRING_CHAR_NULL;
	// Convert value to string.
	string_status = STRING_value_to_string(tx_value, format, print_prefix, str_value);
	STRING_error_check();
	// Add string.
	_AT_USB_reply_add_string(str_value);
}

/* SEND AT REPONSE OVER AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_reply_send(void) {
	// Local variables.
	USART_status_t usart_status = USART_SUCCESS;
	// Add ending string.
	_AT_USB_reply_add_string(AT_USB_REPLY_END);
	_AT_USB_reply_add_char(STRING_CHAR_NULL);
	// Send response over UART.
	usart_status = USART2_send_string(at_usb_ctx.reply);
	USART_error_check();
	// Flush reply buffer.
	at_usb_ctx.reply_size = 0;
}

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_print_ok(void) {
	_AT_USB_reply_add_string("OK");
	_AT_USB_reply_send();
}

/* PRINT AN ERROR THROUGH AT INTERFACE.
 * @param error:	Error code to print.
 * @return:			None.
 */
static void _AT_USB_print_error(ERROR_t error) {
	// Add error to stack.
	ERROR_stack_add(error);
	// Print error.
	_AT_USB_reply_add_string("ERROR_");
	if (error < 0x0100) {
		_AT_USB_reply_add_value(0, STRING_FORMAT_HEXADECIMAL, 1);
		_AT_USB_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 0);
	}
	else {
		_AT_USB_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
	}
	_AT_USB_reply_send();
}

/* PRINT ALL SUPPORTED AT COMMANDS.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_print_command_list(void) {
	// Local variables.
	uint32_t idx = 0;
	// Commands loop.
	for (idx=0 ; idx<(sizeof(AT_USB_COMMAND_LIST) / sizeof(AT_USB_command_t)) ; idx++) {
		// Print syntax.
		_AT_USB_reply_add_string(AT_USB_COMMAND_LIST[idx].syntax);
		// Print parameters.
		_AT_USB_reply_add_string(AT_USB_COMMAND_LIST[idx].parameters);
		_AT_USB_reply_send();
		// Print description.
		_AT_USB_reply_add_string(AT_USB_REPLY_TAB);
		_AT_USB_reply_add_string(AT_USB_COMMAND_LIST[idx].description);
		_AT_USB_reply_send();
	}
	_AT_USB_print_ok();
}

/* PRINT SW VERSION.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_print_sw_version(void) {
	_AT_USB_reply_add_string("SW");
	_AT_USB_reply_add_value((int32_t) GIT_MAJOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_USB_reply_add_string(".");
	_AT_USB_reply_add_value((int32_t) GIT_MINOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_USB_reply_add_string(".");
	_AT_USB_reply_add_value((int32_t) GIT_COMMIT_INDEX, STRING_FORMAT_DECIMAL, 0);
	if (GIT_DIRTY_FLAG != 0) {
		_AT_USB_reply_add_string(".d");
	}
	_AT_USB_reply_add_string(" (");
	_AT_USB_reply_add_value((int32_t) GIT_COMMIT_ID, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_USB_reply_add_string(")");
	_AT_USB_reply_send();
	_AT_USB_print_ok();
}

/* PRINT ERROR STACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_print_error_stack(void) {
	// Local variables.
	ERROR_t error = SUCCESS;
	// Read stack.
	if (ERROR_stack_is_empty() != 0) {
		_AT_USB_reply_add_string("Error stack empty");
	}
	else {
		// Unstack all errors.
		_AT_USB_reply_add_string("[ ");
		do {
			error = ERROR_stack_read();
			if (error != SUCCESS) {
				_AT_USB_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
				_AT_USB_reply_add_string(" ");
			}
		}
		while (error != SUCCESS);
		_AT_USB_reply_add_string("]");
	}
	_AT_USB_reply_send();
	_AT_USB_print_ok();
}

/* AT$ADC? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_adc_callback(void) {
	// Local variables.
	ADC_status_t adc1_status = ADC_SUCCESS;
	uint32_t voltage_mv = 0;
	int8_t tmcu_degrees = 0;
	// Trigger internal ADC conversions.
	_AT_USB_reply_add_string("ADC running...");
	_AT_USB_reply_send();
	adc1_status = ADC1_perform_measurements();
	ADC1_error_check_print();
	// Read and print data.
	// USB voltage.
	_AT_USB_reply_add_string("Vusb=");
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VUSB_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_USB_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	_AT_USB_reply_add_string("mV");
	_AT_USB_reply_send();
	// RS bus voltage.
	_AT_USB_reply_add_string("Vrs=");
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VRS_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_USB_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	_AT_USB_reply_add_string("mV");
	_AT_USB_reply_send();
	// MCU voltage.
	_AT_USB_reply_add_string("Vmcu=");
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_USB_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	_AT_USB_reply_add_string("mV");
	_AT_USB_reply_send();
	// MCU temperature.
	_AT_USB_reply_add_string("Tmcu=");
	adc1_status = ADC1_get_tmcu(&tmcu_degrees);
	ADC1_error_check_print();
	_AT_USB_reply_add_value((int32_t) tmcu_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_USB_reply_add_string("dC");
	_AT_USB_reply_send();
	_AT_USB_print_ok();
errors:
	return;
}

/* AT$SCAN EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_node_scan_callback(void) {
	// Local variables.
	NODE_status_t node_status = NODE_SUCCESS;
	uint8_t idx = 0;
	// Perform bus scan.
	_AT_USB_reply_add_string("Nodes scan running...");
	_AT_USB_reply_send();
	node_status = NODE_scan();
	NODE_error_check_print();
	// Print list.
	_AT_USB_reply_add_value((int32_t) NODES_LIST.count, STRING_FORMAT_DECIMAL, 0);
	_AT_USB_reply_add_string(" node(s) found");
	_AT_USB_reply_send();
	for (idx=0 ; idx<NODES_LIST.count ; idx++) {
		// Print address.
		_AT_USB_reply_add_value(NODES_LIST.list[idx].address, STRING_FORMAT_HEXADECIMAL, 1);
		_AT_USB_reply_add_string(" : ");
		// Print board type.
		if (NODES_LIST.list[idx].board_id == DINFOX_BOARD_ID_ERROR) {
			_AT_USB_reply_add_string("Board ID error");
		}
		else {
			if (NODES_LIST.list[idx].board_id >= DINFOX_BOARD_ID_LAST) {
				_AT_USB_reply_add_string("Unknown board ID (");
				_AT_USB_reply_add_value((int32_t) (NODES_LIST.list[idx].board_id), STRING_FORMAT_HEXADECIMAL, 1);
				_AT_USB_reply_add_string(")");
			}
			else {
				_AT_USB_reply_add_string((char_t*) DINFOX_BOARD_ID_NAME[NODES_LIST.list[idx].board_id]);
			}
		}
		_AT_USB_reply_send();
	}
	_AT_USB_print_ok();
errors:
	return;
}

/* AT$PR? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_node_get_protocol_callback(void) {
	// Print value.
	_AT_USB_reply_add_value((int32_t) at_usb_ctx.node_protocol, STRING_FORMAT_DECIMAL, 0);
	// Print name.
	switch (at_usb_ctx.node_protocol) {
	case NODE_PROTOCOL_AT_BUS:
		_AT_USB_reply_add_string(" (AT_BUS)");
		break;
	case NODE_PROTOCOL_R4S8CR:
		_AT_USB_reply_add_string(" (R4S8CR)");
		break;
	default:
		_AT_USB_reply_add_string(" (unknown)");
	}
	_AT_USB_reply_send();
	_AT_USB_print_ok();
}

/* AT$PR EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_node_set_protocol_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_SUCCESS;
	NODE_status_t node_status = NODE_SUCCESS;
	int32_t protocol = NODE_PROTOCOL_AT_BUS;
	// Parse node address.
	parser_status = PARSER_get_parameter(&at_usb_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &protocol);
	PARSER_error_check_print();
	// Set protocol.
	node_status = NODE_set_protocol((NODE_protocol_t) protocol);
	NODE_error_check_print();
	// Update local variable.
	at_usb_ctx.node_protocol = protocol;
	_AT_USB_print_ok();
errors:
	return;
}

/* NODE SEND COMMAND EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_node_command_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_SUCCESS;
	NODE_status_t node_status = NODE_SUCCESS;
	NODE_command_parameters_t command_params;
	int32_t node_addr = 0;
	uint8_t command_offset = 0;
	// Check if TX is allowed.
	if (CONFIG_get_tx_mode() == CONFIG_TX_DISABLED) {
		_AT_USB_print_error(ERROR_TX_DISABLED);
		goto errors;
	}
	// Parse node address.
	parser_status = PARSER_get_parameter(&at_usb_ctx.parser, STRING_FORMAT_HEXADECIMAL, AT_USB_CHAR_SEPARATOR, &node_addr);
	PARSER_error_check_print();
	// Set command offset.
	command_offset = at_usb_ctx.parser.separator_idx + 1;
	// Update parameters.
	command_params.node_addr = (NODE_address_t) node_addr;
	command_params.command = (char_t*) &(at_usb_ctx.command[command_offset]);
	// Print node access.
	_AT_USB_reply_add_value(DINFOX_NODE_ADDRESS_DIM, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_USB_reply_add_string(" > ");
	_AT_USB_reply_add_value(command_params.node_addr, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_USB_reply_add_string(" : ");
	_AT_USB_reply_add_string(command_params.command);
	_AT_USB_reply_send();
	// Perform read operation.
	node_status = NODE_send_command(&command_params);
	NODE_error_check_print();
errors:
	return;
}

/* RESET AT PARSER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_reset_parser(void) {
	// Flush buffers.
	at_usb_ctx.command_size = 0;
	at_usb_ctx.reply_size = 0;
	// Reset flag.
	at_usb_ctx.line_end_flag = 0;
	// Reset parser.
	at_usb_ctx.parser.buffer = (char_t*) at_usb_ctx.command;
	at_usb_ctx.parser.buffer_size = 0;
	at_usb_ctx.parser.separator_idx = 0;
	at_usb_ctx.parser.start_idx = 0;
}

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_USB_decode(void) {
	// Local variables.
	uint8_t idx = 0;
	uint8_t decode_success = 0;
	// Update parser length.
	at_usb_ctx.parser.buffer_size = at_usb_ctx.command_size;
	// Loop on available commands.
	for (idx=0 ; idx<(sizeof(AT_USB_COMMAND_LIST) / sizeof(AT_USB_command_t)) ; idx++) {
		// Check type.
		if (PARSER_compare(&at_usb_ctx.parser, AT_USB_COMMAND_LIST[idx].mode, AT_USB_COMMAND_LIST[idx].syntax) == PARSER_SUCCESS) {
			// Execute callback and exit.
			AT_USB_COMMAND_LIST[idx].callback();
			decode_success = 1;
			break;
		}
	}
	if (decode_success == 0) {
		_AT_USB_print_error(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
		goto errors;
	}
errors:
	_AT_USB_reset_parser();
	return;
}

/*** AT functions ***/

/* INIT AT MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_USB_init(void) {
	// Init context.
	_AT_USB_reset_parser();
	at_usb_ctx.node_protocol = NODE_PROTOCOL_AT_BUS;
	// Start continuous listening in AT_BUS mode by default.
	NODE_set_protocol(NODE_PROTOCOL_AT_BUS);
	LPUART1_enable_rx();
	// Enable USART.
	USART2_enable_interrupt();
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_USB_task(void) {
	// Local variables.
	NODE_status_t node_status = NODE_SUCCESS;
	// Trigger decoding function if line end found.
	if (at_usb_ctx.line_end_flag != 0) {
		// Decode and execute command.
		USART2_disable_interrupt();
		_AT_USB_decode();
		USART2_enable_interrupt();
	}
	// Perform continuous listening task.
	node_status = AT_BUS_task();
	NODE_error_check_print();
	node_status = R4S8CR_task();
	NODE_error_check_print();
errors:
	return;
}

/* PRINT NODE FRAME OVER AT USB INTERFACE.
 * @param str:	String to print.
 * @return:		None.
 */
void AT_USB_print(char_t* str) {
	// Print string.
	_AT_USB_reply_add_string(str);
	_AT_USB_reply_send();
}

/* FILL AT COMMAND BUFFER WITH A NEW BYTE (CALLED BY USART INTERRUPT).
 * @param rx_byte:	Incoming byte.
 * @return:			None.
 */
void AT_USB_fill_rx_buffer(uint8_t rx_byte) {
	// Append byte if line end flag is not allready set.
	if (at_usb_ctx.line_end_flag == 0) {
		// Check ending characters.
		if ((rx_byte == STRING_CHAR_CR) || (rx_byte == STRING_CHAR_LF)) {
			at_usb_ctx.command[at_usb_ctx.command_size] = STRING_CHAR_NULL;
			at_usb_ctx.line_end_flag = 1;
		}
		else {
			// Store new byte.
			at_usb_ctx.command[at_usb_ctx.command_size] = rx_byte;
			// Manage index.
			at_usb_ctx.command_size = (at_usb_ctx.command_size + 1) % AT_USB_COMMAND_BUFFER_SIZE;
		}
	}
}

