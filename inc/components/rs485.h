/*
 * rs485.h
 *
 *  Created on: 28 oct. 2022
 *      Author: Ludo
 */

#ifndef RS485_H
#define RS485_H

#include "lptim.h"
#include "lpuart.h"
#include "parser.h"

/*** RS485 macros ***/

#define RS485_ADDRESS_MASK	0x7F
#define RS485_ADDRESS_LAST	RS485_ADDRESS_MASK

/*** RS485 structures ***/

typedef enum {
	RS485_SUCCESS,
	RS485_ERROR_NULL_PARAMETER,
	RS485_ERROR_RESPONSE_TIMEOUT,
	RS485_ERROR_SOURCE_ADDRESS_MISMATCH,
	RS485_ERROR_BASE_LPUART = 0x0100,
	RS485_ERROR_BASE_LPTIM = (RS485_ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
	RS485_ERROR_BASE_PARSER = (RS485_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	RS485_ERROR_BASE_LAST = (RS485_ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST)
} RS485_status_t;

typedef struct {
	uint8_t address;
	uint8_t board_id;
} RS485_node_t;

/*** RS485 functions ***/

RS485_status_t RS485_scan_nodes(RS485_node_t* nodes_list, uint8_t node_list_size, uint8_t* number_of_nodes_found);
RS485_status_t RS485_send_command(LPUART_mode_t lpuart_mode, uint8_t node_address, char_t* command, char_t* response, uint8_t response_size_byte);
void RS485_fill_rx_buffer(uint8_t rx_byte);

#define RS485_status_check(error_base) { if (rs485_status != RS485_SUCCESS) { status = error_base + rs485_status; goto errors; }}
#define RS485_error_check() { ERROR_status_check(rs485_status, RS485_SUCCESS, ERROR_BASE_RS485); }
#define RS485_error_check_print() { ERROR_status_check_print(rs485_status, RS485_SUCCESS, ERROR_BASE_RS485); }

#endif /* RS485_H */
