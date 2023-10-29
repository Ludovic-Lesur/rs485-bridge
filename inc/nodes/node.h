/*
 * node.h
 *
 *  Created on: 18 Feb. 2023
 *      Author: Ludo
 */

#ifndef __NODE_H__
#define __NODE_H__

#include "lbus.h"
#include "lptim.h"
#include "lpuart.h"
#include "node_common.h"
#include "power.h"
#include "string.h"
#include "types.h"

/*** NODES macros ***/

#define NODES_LIST_SIZE_MAX		32

/*** NODE structures ***/

/*!******************************************************************
 * \enum NODE_status_t
 * \brief NODE driver error codes.
 *******************************************************************/
typedef enum {
	NODE_SUCCESS = 0,
	NODE_ERROR_NULL_PARAMETER,
	NODE_ERROR_PROTOCOL,
	NODE_ERROR_NODE_ADDRESS,
	NODE_ERROR_R4S8CR_ADDRESS,
	NODE_ERROR_REGISTER_ADDRESS,
	NODE_ERROR_REPLY_TYPE,
	NODE_ERROR_LBUS_MODE,
	NODE_ERROR_BASE_LPUART = 0x0100,
	NODE_ERROR_BASE_LPTIM = (NODE_ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
	NODE_ERROR_BASE_STRING = (NODE_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	NODE_ERROR_BASE_LBUS = (NODE_ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
	NODE_ERROR_BASE_LAST = (NODE_ERROR_BASE_LBUS + LBUS_ERROR_BASE_LAST)
} NODE_status_t;

/*!******************************************************************
 * \enum NODE_t
 * \brief Node descriptor.
 *******************************************************************/
typedef struct {
	NODE_address_t address;
	uint8_t board_id;
} NODE_t;

/*!******************************************************************
 * \enum NODE_list_t
 * \brief Node list type.
 *******************************************************************/
typedef struct {
	NODE_t list[NODES_LIST_SIZE_MAX];
	uint8_t count;
} NODE_list_t;

/*!******************************************************************
 * \fn NODE_print_frame_cb_t
 * \brief NODE frame print callback.
 *******************************************************************/
typedef void (*NODE_print_frame_cb_t)(char_t* str);

/*!******************************************************************
 * \fn NODE_none_protocol_rx_irq_cb_t
 * \brief NODE RX interrupt callback for none protocol mode.
 *******************************************************************/
typedef void (*NODE_none_protocol_rx_irq_cb_t)(uint8_t data);

/*** NODES global variables ***/

NODE_list_t NODES_LIST;

/*** NODE functions ***/

/*!******************************************************************
 * \fn void NODE_init(NODE_print_frame_cb_t print_callback, NODE_none_protocol_rx_irq_cb_t none_protocol_rx_irq_callback)
 * \brief Init NODE interface.
 * \param[in]  	print_callback: Function to call on frame reception.
 * \param[in]	none_protocol_rx_irq_callback: Function to call on RX interrupt in none protocol mode.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void NODE_init(NODE_print_frame_cb_t print_callback, NODE_none_protocol_rx_irq_cb_t none_protocol_rx_irq_callback);

/*!******************************************************************
 * \fn void NODE_status_t NODE_set_protocol(NODE_protocol_t protocol)
 * \brief Set node protocol.
 * \param[in]  	protocol: Protocol to decode.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t NODE_set_protocol(NODE_protocol_t protocol);

/*!******************************************************************
 * \fn NODE_protocol_t NODE_get_protocol(void)
 * \brief Get current node protocol.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Current node protocol.
 *******************************************************************/
NODE_protocol_t NODE_get_protocol(void);

/*!******************************************************************
 * \fn NODE_status_t NODE_set_baud_rate(uint32_t baud_rate)
 * \brief Set node baud rate.
 * \param[in]  	baud_rate: Baud rate to use on RS485 bus.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t NODE_set_baud_rate(uint32_t baud_rate);

/*!******************************************************************
 * \fn uint32_t NODE_get_baud_rate(void)
 * \brief Get current node baud rate.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Current baud rate used on RS485 bus.
 *******************************************************************/
uint32_t NODE_get_baud_rate(void);

/*!******************************************************************
 * \fn NODE_status_t NODE_scan(void)
 * \brief Scan all nodes connected to the RS485 bus.
 * \param[in]  	none
 * \param[out]	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t NODE_scan(void);

/*!******************************************************************
 * \fn NODE_status_t NODE_send_command(NODE_command_parameters_t* command_params)
 * \brief Send a command over node interface.
 * \param[in]  	command_params: Pointer to the command parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t NODE_send_command(NODE_command_parameters_t* command_params);

/*******************************************************************/
#define NODE_exit_error(error_base) { if (node_status != NODE_SUCCESS) { status = (error_base + node_status); goto errors; } }

/*******************************************************************/
#define NODE_stack_error(void) { if (node_status != NODE_SUCCESS) { ERROR_stack_add(ERROR_BASE_NODE + node_status); } }

/*******************************************************************/
#define NODE_stack_exit_error(error_code) { if (node_status != NODE_SUCCESS) { ERROR_stack_add(ERROR_BASE_NODE + node_status); status = error_code; goto errors; } }

#endif /* __NODE_H__ */
