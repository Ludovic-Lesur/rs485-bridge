/*
 * node.c
 *
 *  Created on: Feb 26, 2023
 *      Author: ludo
 */

#include "node.h"

#include "at_bus.h"
#include "dinfox_types.h"
#include "lbus.h"
#include "r4s8cr.h"
#include "string.h"
#include "types.h"

/*** NODE local global variables ***/

static NODE_protocol_t node_protocol = NODE_PROTOCOL_AT_BUS;

/*** NODE local functions ***/

/* FLUSH NODES LIST.
 * @param:	None.
 */
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

/* INIT NODE LAYER.
 * @param:	None.
 * @return:	None.
 */
void NODE_init(void) {
	// Reset node list.
	_NODE_flush_list();
	// Init interface layers.
	AT_BUS_init();
}

/* SET PHYSICAL PROTOCOL.
 * @param protocol:	Protocol to use on RS485 bus.
 * @return status:	Function execution status.
 */
NODE_status_t NODE_set_protocol(NODE_protocol_t protocol) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	// Check parameter.
	if (protocol >= NODE_PROTOCOL_LAST) {
		status = NODE_ERROR_PROTOCOL;
		goto errors;
	}
	// Configure LPUART.
	switch (protocol) {
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
	node_protocol = protocol;
errors:
	return status;
}

/* SCAN ALL NODE ON BUS.
 * @param:			None.
 * @return status:	Function executions status.
 */
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
	NODE_set_protocol(node_protocol);
	return status;
}

/* SEND COMMAND OVER RS485 BUS.
 * @param command_params:	Pointer to the command parameters structure.
 * @return status:			Function execution status.
 */
NODE_status_t NODE_send_command(NODE_command_parameters_t* command_params) {
	// Local variables.
	NODE_status_t status = NODE_SUCCESS;
	NODE_reply_parameters_t unused_reply_params;
	NODE_read_data_t unused_read_data;
	NODE_access_status_t unused_access_status;
	// Check parameters.
	if (command_params == NULL) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if ((command_params -> command) == NULL) {
		status = NODE_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Do not wait for reply.
	unused_reply_params.type = NODE_REPLY_TYPE_NONE;
	// Send command with current protocol.
	switch (node_protocol) {
	case NODE_PROTOCOL_AT_BUS:
		status = AT_BUS_send_command(command_params, &unused_reply_params, &unused_read_data, &unused_access_status);
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
