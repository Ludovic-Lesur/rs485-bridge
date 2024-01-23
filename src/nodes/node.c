/*
 * node.c
 *
 *  Created on: 26 feb. 2023
 *      Author: Ludo
 */

#include "node.h"

#include "at_bus.h"
#include "at_usb.h"
#include "dinfox.h"
#include "lbus.h"
#include "node_common.h"
#include "r4s8cr.h"
#include "string.h"
#include "types.h"

/*** NODE local structures ***/

/*******************************************************************/
typedef struct {
	NODE_protocol_t protocol;
	uint32_t baud_rate;
	NODE_none_protocol_rx_irq_cb_t none_protocol_rx_irq_callback;
} NODE_context_t;

/*** NODE local global variables ***/

static NODE_context_t node_ctx;

/*** NODE local functions ***/

/*******************************************************************/
void _NODE_flush_list(void) {
	// Local variables.
	uint8_t idx = 0;
	// Reset node list.
	for (idx=0 ; idx<NODES_LIST_SIZE_MAX ; idx++) {
		NODES_LIST.list[idx].address = 0xFF;
		NODES_LIST.list[idx].board_id = DINFOX_BOARD_ID_ERROR;
	}
	NODES_LIST.count = 0;
}

/*** NODE functions ***/

/*******************************************************************/
NODE_status_t NODE_init(NODE_print_frame_cb_t print_callback, NODE_none_protocol_rx_irq_cb_t none_protocol_rx_irq_callback) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	// Init context.
	node_ctx.protocol = NODE_PROTOCOL_AT_BUS;
	node_ctx.baud_rate = 115200;
	node_ctx.none_protocol_rx_irq_callback = none_protocol_rx_irq_callback;
	// Reset node list.
	_NODE_flush_list();
	// Init interface layers.
	AT_BUS_init(print_callback);
	R4S8CR_init(print_callback);
	// Init and start LPUART.
	lpuart1_status = LPUART1_init();
	LPUART1_exit_error(NODE_ERROR_BASE_LPUART);
	LPUART1_enable_rx();
errors:
	return status;
}

/*******************************************************************/
NODE_status_t NODE_set_protocol(NODE_protocol_t protocol) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	LPUART_configuration_t lpuart_config;
	// Check parameter.
	if (protocol >= NODE_PROTOCOL_LAST) {
		status = NODE_ERROR_PROTOCOL;
		goto errors;
	}
	// Configure LPUART.
	switch (protocol) {
	case NODE_PROTOCOL_NONE:
		// Configure physical interface.
		lpuart_config.baud_rate = node_ctx.baud_rate;
		lpuart_config.rx_callback = node_ctx.none_protocol_rx_irq_callback;
		lpuart1_status = LPUART1_configure(&lpuart_config);
		LPUART1_exit_error(NODE_ERROR_BASE_LPUART);
		break;
	case NODE_PROTOCOL_AT_BUS:
		// Configure physical interface.
		status = LBUS_configure_phy();
		if (status != NODE_SUCCESS) goto errors;
		status = LBUS_set_mode(LBUS_MODE_RAW);
		if (status != NODE_SUCCESS) goto errors;
		break;
	case NODE_PROTOCOL_R4S8CR:
		status = R4S8CR_configure_phy();
		if (status != NODE_SUCCESS) goto errors;
		break;
	default:
		status = NODE_ERROR_PROTOCOL;
		break;
	}
	// Update local variable.
	node_ctx.protocol = protocol;
errors:
	return status;
}

/*******************************************************************/
NODE_protocol_t NODE_get_protocol(void) {
	return (node_ctx.protocol);
}

/*******************************************************************/
NODE_status_t NODE_set_baud_rate(uint32_t baud_rate) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	// Update local variable.
	node_ctx.baud_rate = baud_rate;
	return status;
}

/*******************************************************************/
uint32_t NODE_get_baud_rate(void) {
	return (node_ctx.baud_rate);
}

/*******************************************************************/
NODE_status_t NODE_scan(void) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	uint8_t nodes_count = 0;
	// Reset list.
	_NODE_flush_list();
	// Scan LBUS nodes.
	status = AT_BUS_scan(&(NODES_LIST.list[NODES_LIST.count]), (NODES_LIST_SIZE_MAX - NODES_LIST.count), &nodes_count);
	if (status != NODE_SUCCESS) goto errors;
	// Update count.
	NODES_LIST.count += nodes_count;
	// Scan R4S8CR nodes.
	status = R4S8CR_scan(&(NODES_LIST.list[NODES_LIST.count]), (NODES_LIST_SIZE_MAX - NODES_LIST.count), &nodes_count);
	if (status != NODE_SUCCESS) goto errors;
	// Update count.
	NODES_LIST.count += nodes_count;
errors:
	// Re-configure physical interface with previous protocol.
	NODE_set_protocol(node_ctx.protocol);
	return status;
}

/*******************************************************************/
NODE_status_t NODE_send_command(NODE_command_parameters_t* command_params) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	// Check parameters.
	if (command_params == NULL) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if ((command_params -> command) == NULL) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Send command with current protocol.
	switch (node_ctx.protocol) {
	case NODE_PROTOCOL_AT_BUS:
		status = AT_BUS_send_command(command_params);
		break;
	case NODE_PROTOCOL_R4S8CR:
		status = R4S8CR_send_command(command_params);
		break;
	default:
		status = NODE_ERROR_PROTOCOL;
		break;
	}
errors:
	return status;
}
