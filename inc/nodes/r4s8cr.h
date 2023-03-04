/*
 * r4s8cr.h
 *
 *  Created on: 2 feb. 2023
 *      Author: Ludo
 */

#ifndef __R4S8CR_H__
#define __R4S8CR_H__

#include "node.h"

/*** R4S8CR macros ***/

#define R4S8CR_TIMEOUT_MS		100
#define R4S8CR_NODE_ADDRESS		0xFF

/*** R4S8CR structures ***/

typedef enum {
	R4S8CR_REGISTER_RELAY_1 = 0,
	R4S8CR_REGISTER_RELAY_2,
	R4S8CR_REGISTER_RELAY_3,
	R4S8CR_REGISTER_RELAY_4,
	R4S8CR_REGISTER_RELAY_5,
	R4S8CR_REGISTER_RELAY_6,
	R4S8CR_REGISTER_RELAY_7,
	R4S8CR_REGISTER_RELAY_8,
	R4S8CR_REGISTER_LAST,
} R4S8CR_register_address_t;

/*** R4S8CR functions ***/

NODE_status_t R4S8CR_configure_phy(void);
NODE_status_t R4S8CR_send_command(NODE_command_parameters_t* command_params);
NODE_status_t R4S8CR_scan(NODE_t* nodes_list, uint8_t nodes_list_size, uint8_t* nodes_count);
NODE_status_t R4S8CR_task(void);
void R4S8CR_fill_rx_buffer(uint8_t rx_byte);

#endif /* __R4S8CR_H__ */
