/*
 * lbus.h
 *
 *  Created on: 28 oct. 2022
 *      Author: Ludo
 */

#ifndef __LBUS_H__
#define __LBUS_H__

#include "mode.h"
#include "node.h"
#include "lpuart.h"
#include "types.h"

/*** LBUS macros ***/

#define LBUS_BAUD_RATE				1200
#define LBUS_ADDRESS_MASK			0x7F
#define LBUS_ADDRESS_LAST			LBUS_ADDRESS_MASK
#define LBUS_ADDRESS_SIZE_BYTES		1

/*** LBUS structures ***/

typedef enum {
	LBUS_MODE_RAW = 0,
	LBUS_MODE_DECODING,
	LBUS_MODE_LAST
} LBUS_mode_t;

typedef enum {
	LBUS_FRAME_FIELD_INDEX_DESTINATION_ADDRESS = 0,
	LBUS_FRAME_FIELD_INDEX_SOURCE_ADDRESS = (LBUS_FRAME_FIELD_INDEX_DESTINATION_ADDRESS + LBUS_ADDRESS_SIZE_BYTES),
	LBUS_FRAME_FIELD_INDEX_DATA = (LBUS_FRAME_FIELD_INDEX_SOURCE_ADDRESS + LBUS_ADDRESS_SIZE_BYTES)
} LBUS_frame_field_index_t;

/*** LBUS functions ***/

void LBUS_init(void);
NODE_status_t LBUS_configure_phy(void);
NODE_status_t LBUS_set_mode(LBUS_mode_t mode);
NODE_status_t LBUS_send(NODE_address_t destination_address, uint8_t* data, uint32_t data_size_bytes);
void LBUS_reset(void);
void LBUS_fill_rx_buffer(uint8_t rx_byte);

#define LBUS_status_check(error_base) { if (lbus_status != LBUS_SUCCESS) { status = error_base + lbus_status; goto errors; }}
#define LBUS_error_check() { ERROR_status_check(lbus_status, LBUS_SUCCESS, ERROR_BASE_LBUS); }
#define LBUS_error_check_print() { ERROR_status_check_print(lbus_status, LBUS_SUCCESS, ERROR_BASE_LBUS); }

#endif /* __LBUS_H__ */
