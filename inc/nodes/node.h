/*
 * node.h
 *
 *  Created on: 18 Feb. 2023
 *      Author: Ludo
 */

#ifndef __NODE_H__
#define __NODE_H__

#include "lptim.h"
#include "lpuart.h"
#include "string.h"
#include "types.h"

/*** NODES macros ***/

#define NODES_LIST_SIZE_MAX		32

/*** NODE structures ***/

typedef enum {
	NODE_SUCCESS = 0,
	NODE_ERROR_NOT_SUPPORTED,
	NODE_ERROR_NULL_PARAMETER,
	NODE_ERROR_PROTOCOL,
	NODE_ERROR_NODE_ADDRESS,
	NODE_ERROR_REGISTER_ADDRESS,
	NODE_ERROR_REGISTER_FORMAT,
	NODE_ERROR_STRING_DATA_INDEX,
	NODE_ERROR_READ_TYPE,
	NODE_ERROR_ACCESS,
	NODE_ERROR_NONE_RADIO_MODULE,
	NODE_ERROR_SIGFOX_PAYLOAD_TYPE,
	NODE_ERROR_SIGFOX_PAYLOAD_EMPTY,
	NODE_ERROR_SIGFOX_LOOP,
	NODE_ERROR_SIGFOX_SEND,
	NODE_ERROR_DOWNLINK_NODE_ADDRESS,
	NODE_ERROR_DOWNLINK_BOARD_ID,
	NODE_ERROR_DOWNLINK_OPERATION_CODE,
	NODE_ERROR_ACTION_INDEX,
	NODE_ERROR_LBUS_MODE,
	NODE_ERROR_BASE_LPUART = 0x0100,
	NODE_ERROR_BASE_LPTIM = (NODE_ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
	NODE_ERROR_BASE_STRING = (NODE_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	NODE_ERROR_BASE_LAST = (NODE_ERROR_BASE_STRING + STRING_ERROR_BASE_LAST)
} NODE_status_t;

typedef uint8_t	NODE_address_t;

typedef enum {
	NODE_PROTOCOL_AT_BUS = 0,
	NODE_PROTOCOL_R4S8CR,
	NODE_PROTOCOL_LAST
} NODE_protocol_t;

typedef struct {
	NODE_address_t address;
	uint8_t board_id;
} NODE_t;

typedef struct {
	NODE_t list[NODES_LIST_SIZE_MAX];
	uint8_t count;
} NODE_list_t;

typedef enum {
	NODE_REPLY_TYPE_NONE = 0,
	NODE_REPLY_TYPE_RAW,
	NODE_REPLY_TYPE_OK,
	NODE_REPLY_TYPE_VALUE,
	NODE_REPLY_TYPE_BYTE_ARRAY,
	NODE_REPLY_TYPE_LAST
} NODE_reply_type_t;

typedef struct {
	NODE_address_t node_address;
	char_t* command;
} NODE_command_parameters_t;

typedef struct {
	NODE_reply_type_t type;
	STRING_format_t format; // Expected value format.
	uint32_t timeout_ms;
	// For byte array.
	uint8_t byte_array_size;
	uint8_t exact_length;
} NODE_reply_parameters_t;

typedef struct {
	NODE_address_t node_address;
	uint8_t register_address;
	uint32_t timeout_ms;
	STRING_format_t format; // Expected value format.
	NODE_reply_type_t type;
} NODE_read_parameters_t;

typedef struct {
	char_t* raw;
	int32_t value;
	uint8_t* byte_array;
	uint8_t extracted_length;
} NODE_read_data_t;

typedef struct {
	NODE_address_t node_address;
	uint8_t register_address;
	uint32_t timeout_ms;
	STRING_format_t format; // Register value format.
	int32_t value;
} NODE_write_parameters_t;

typedef union {
	struct {
		unsigned error_received : 1;
		unsigned parser_error : 1;
		unsigned reply_timeout : 1;
		unsigned sequence_timeout : 1;
	};
	uint8_t all;
} NODE_access_status_t;

/*** NODES global variables ***/

NODE_list_t NODES_LIST;

/*** NODE functions ***/

void NODE_init(void);
NODE_status_t NODE_set_protocol(NODE_protocol_t protocol);
NODE_status_t NODE_scan(void);
NODE_status_t NODE_send_command(NODE_command_parameters_t* command_params);

#define NODE_status_check(error_base) { if (node_status != NODE_SUCCESS) { status = error_base + node_status; goto errors; }}
#define NODE_error_check() { ERROR_status_check(node_status, NODE_SUCCESS, ERROR_BASE_NODE); }
#define NODE_error_check_print() { ERROR_status_check_print(node_status, NODE_SUCCESS, ERROR_BASE_NODE); }

#endif /* __NODE_H__ */
