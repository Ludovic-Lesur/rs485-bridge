/*
 * node.h
 *
 *  Created on: 18 Feb. 2023
 *      Author: Ludo
 */

#ifndef __NODE_H__
#define __NODE_H__

#include "dinfox.h"
#include "lptim.h"
#include "lpuart.h"
#include "node_common.h"
#include "string.h"
#include "types.h"

/*** NODES macros ***/

#define NODES_LIST_SIZE_MAX		32

/*** NODE structures ***/

typedef enum {
	NODE_SUCCESS = 0,
	NODE_ERROR_NULL_PARAMETER,
	NODE_ERROR_PROTOCOL,
	NODE_ERROR_NODE_ADDRESS,
	NODE_ERROR_REGISTER_ADDRESS,
	NODE_ERROR_REPLY_TYPE,
	NODE_ERROR_LBUS_MODE,
	NODE_ERROR_BASE_LPUART = 0x0100,
	NODE_ERROR_BASE_LPTIM = (NODE_ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
	NODE_ERROR_BASE_STRING = (NODE_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	NODE_ERROR_BASE_LAST = (NODE_ERROR_BASE_STRING + STRING_ERROR_BASE_LAST)
} NODE_status_t;

typedef struct {
	NODE_address_t address;
	uint8_t board_id;
} NODE_t;

typedef struct {
	NODE_t list[NODES_LIST_SIZE_MAX];
	uint8_t count;
} NODE_list_t;

/*** NODES global variables ***/

NODE_list_t NODES_LIST;

/*** NODE functions ***/

void NODE_init(void);

NODE_status_t NODE_set_protocol(NODE_protocol_t protocol);
NODE_protocol_t NODE_get_protocol(void);

NODE_status_t NODE_set_baud_rate(uint32_t baud_rate);
uint32_t NODE_get_baud_rate(void);

NODE_status_t NODE_scan(void);

NODE_status_t NODE_send_command(NODE_command_parameters_t* command_params);

#define NODE_status_check(error_base) { if (node_status != NODE_SUCCESS) { status = error_base + node_status; goto errors; }}
#define NODE_error_check() { ERROR_status_check(node_status, NODE_SUCCESS, ERROR_BASE_NODE); }
#define NODE_error_check_print() { ERROR_status_check_print(node_status, NODE_SUCCESS, ERROR_BASE_NODE); }

#endif /* __NODE_H__ */
