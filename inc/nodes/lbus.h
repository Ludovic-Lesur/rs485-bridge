/*
 * lbus.h
 *
 *  Created on: 28 oct. 2022
 *      Author: Ludo
 */

#ifndef __LBUS_H__
#define __LBUS_H__

#include "lpuart.h"
#include "node_common.h"
#include "types.h"

/*** LBUS macros ***/

#define LBUS_ADDRESS_MASK			0x7F
#define LBUS_ADDRESS_LAST			LBUS_ADDRESS_MASK
#define LBUS_ADDRESS_SIZE_BYTES		1

/*** LBUS structures ***/

/*!******************************************************************
 * \enum LBUS_status_t
 * \brief LBUS driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	LBUS_SUCCESS = 0,
	LBUS_ERROR_ADDRESS,
	LBUS_ERROR_MODE,
	// Low level drivers errors.
	LBUS_ERROR_BASE_LPUART = 0x0100,
	// Last base value.
	LBUS_ERROR_BASE_LAST = (LBUS_ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
} LBUS_status_t;

/*!******************************************************************
 * \enum LBUS_mode_t
 * \brief LBUS modes list.
 *******************************************************************/
typedef enum {
	LBUS_MODE_RAW = 0,
	LBUS_MODE_DECODING,
	LBUS_MODE_LAST
} LBUS_mode_t;

/*!******************************************************************
 * \enum LBUS_frame_field_index_t
 * \brief LBUS frame structure.
 *******************************************************************/
typedef enum {
	LBUS_FRAME_FIELD_INDEX_DESTINATION_ADDRESS = 0,
	LBUS_FRAME_FIELD_INDEX_SOURCE_ADDRESS = (LBUS_FRAME_FIELD_INDEX_DESTINATION_ADDRESS + LBUS_ADDRESS_SIZE_BYTES),
	LBUS_FRAME_FIELD_INDEX_DATA = (LBUS_FRAME_FIELD_INDEX_SOURCE_ADDRESS + LBUS_ADDRESS_SIZE_BYTES)
} LBUS_frame_field_index_t;

/*!******************************************************************
 * \fn LBUS_rx_irq_cb
 * \brief LBUS RX interrupt callback.
 *******************************************************************/
typedef void (*LBUS_rx_irq_cb)(uint8_t data);

/*** LBUS functions ***/

/*!******************************************************************
 * \fn void LBUS_init(LBUS_rx_irq_cb irq_callback)
 * \brief Init LBUS interface.
 * \param[in]  	irq_callback: Function to call on frame reception interrupt.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LBUS_init(LBUS_rx_irq_cb irq_callback);

/*!******************************************************************
 * \fn LBUS_status_t LBUS_configure_phy(void)
 * \brief Configure RS485 physical interface for LBUS transfer.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LBUS_status_t LBUS_configure_phy(void);

/*!******************************************************************
 * \fn LBUS_status_t LBUS_set_mode(LBUS_mode_t mode)
 * \brief Set LBUS transfer mode.
 * \param[in]  	mode: Mode to set.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
LBUS_status_t LBUS_set_mode(LBUS_mode_t mode);

/*!******************************************************************
 * \fn LBUS_status_t LBUS_send(NODE_address_t destination_address, uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over LBUS interface.
 * \param[in]	destination_address: RS485 address of the destination node.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LBUS_status_t LBUS_send(NODE_address_t destination_address, uint8_t* data, uint32_t data_size_bytes);

/*!******************************************************************
 * \fn void LBUS_reset(void)
 * \brief Reset LBUS interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LBUS_reset(void);

/*******************************************************************/
#define LBUS_exit_error(error_base) { if (lbus_status != LBUS_SUCCESS) { status = (error_base + lbus_status); goto errors; } }

/*******************************************************************/
#define LBUS_stack_error(void) { if (lbus_status != LBUS_SUCCESS) { ERROR_stack_add(ERROR_BASE_LBUS + lbus_status); } }

#endif /* __LBUS_H__ */
