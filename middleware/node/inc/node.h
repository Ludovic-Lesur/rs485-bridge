/*
 * node.h
 *
 *  Created on: 18 Feb. 2023
 *      Author: Ludo
 */

#ifndef __NODE_H__
#define __NODE_H__

#include "dim_flags.h"
#include "lpuart.h"
#include "strings.h"
#include "types.h"
#include "una.h"
#include "una_at.h"
#include "una_r4s8cr.h"

/*** NODE structures ***/

/*!******************************************************************
 * \enum NODE_status_t
 * \brief NODE driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    NODE_SUCCESS = 0,
    NODE_ERROR_NULL_PARAMETER,
    NODE_ERROR_PROTOCOL,
    // Low level drivers errors.
    NODE_ERROR_BASE_LPUART = 0x0100,
    NODE_ERROR_BASE_STRING = (NODE_ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
    NODE_ERROR_BASE_UNA_AT = (NODE_ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
    NODE_ERROR_BASE_UNA_R4S8CR = (NODE_ERROR_BASE_UNA_AT + UNA_AT_ERROR_BASE_LAST),
    // Last base value.
    NODE_ERROR_BASE_LAST = (NODE_ERROR_BASE_UNA_R4S8CR + UNA_R4S8CR_ERROR_BASE_LAST)
} NODE_status_t;

/*!******************************************************************
 * \enum NODE_protocol_t
 * \brief Node protocols list.
 *******************************************************************/
typedef enum {
    NODE_PROTOCOL_NONE = 0,
    NODE_PROTOCOL_UNA_AT,
    NODE_PROTOCOL_UNA_R4S8CR,
    NODE_PROTOCOL_LAST
} NODE_protocol_t;

/*!******************************************************************
 * \fn NODE_print_frame_cb_t
 * \brief NODE frame print callback.
 *******************************************************************/
typedef void (*NODE_print_frame_cb_t)(char_t* frame);

/*!******************************************************************
 * \fn NODE_none_protocol_rx_irq_cb_t
 * \brief NODE RX interrupt callback for none protocol mode.
 *******************************************************************/
typedef void (*NODE_none_protocol_rx_irq_cb_t)(uint8_t data);

/*** NODES global variables ***/

extern UNA_node_list_t NODES_LIST;

/*** NODE functions ***/

/*!******************************************************************
 * \fn NODE_status_t NODE_init(NODE_print_frame_cb_t print_frame_callback, NODE_none_protocol_rx_irq_cb_t none_protocol_rx_irq_callback)
 * \brief Init unified node interface.
 * \param[in]   print_frame_callback: Function to call to print a decoded frame.
 * \param[in]   none_protocol_rx_irq_callback: Function to call to print a raw byte in none protocol decoding mode.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_init(NODE_print_frame_cb_t print_frame_callback, NODE_none_protocol_rx_irq_cb_t none_protocol_rx_irq_callback);

/*!******************************************************************
 * \fn NODE_status_t NODE_de_init(void)
 * \brief Release unified node interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_de_init(void);

/*!******************************************************************
 * \fn NODE_status_t NODE_process(void)
 * \brief Process bus decoding.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_process(void);

/*!******************************************************************
 * \fn NODE_status_t NODE_set_protocol(NODE_protocol_t protocol, uint32_t baud_rate)
 * \brief Set node protocol.
 * \param[in]   protocol: Protocol to decode.
 * \param[in]   Bus baud rate.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_set_protocol(NODE_protocol_t protocol, uint32_t baud_rate);

/*!******************************************************************
 * \fn NODE_status_t NODE_get_protocol(NODE_protocol_t* protocol, uint32_t* baud_rate)
 * \brief Get current node protocol.
 * \param[in]   none
 * \param[out]  protocol: Pointer to the current decoding protocol.
 * \param[out]  baud_rate: Pointer to the current bus baud rate.
 * \retval      Current node protocol.
 *******************************************************************/
NODE_status_t NODE_get_protocol(NODE_protocol_t* protocol, uint32_t* baud_rate);

/*!******************************************************************
 * \fn NODE_status_t NODE_write_register(UNA_node_t* node, uint8_t reg_addr, uint32_t reg_value, uint32_t reg_mask, UNA_access_status_t* write_status)
 * \brief Write node register.
 * \param[in]   node: Pointer to the node to access.
 * \param[in]   reg_addr: Address of the register to write.
 * \param[in]   reg_value: Register value to write.
 * \param[in]   reg_mask: Writing operation mask.
 * \param[out]  write_status: Pointer to the writing operation status.
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_write_register(UNA_node_t* node, uint8_t reg_addr, uint32_t reg_value, uint32_t reg_mask, UNA_access_status_t* write_status);

/*!******************************************************************
 * \fn NODE_status_t NODE_read_register(UNA_node_address_t node_addr, uint8_t reg_addr, uint32_t* reg_value, UNA_access_status_t* read_status);
 * \brief Read node register.
 * \param[in]   node: Pointer to the node to access.
 * \param[in]   reg_addr: Address of the register to read.
 * \param[out]  reg_value: Pointer to the read register value.
 * \param[out]  read_status: Pointer to the read operation status.
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_read_register(UNA_node_t* node, uint8_t reg_addr, uint32_t* reg_value, UNA_access_status_t* read_status);

/*!******************************************************************
 * \fn NODE_status_t NODE_send_command(NODE_command_parameters_t* command_parameters)
 * \brief Send a command over node interface.
 * \param[in]   command_parameters: Pointer to the command parameters.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_send_command(UNA_command_parameters_t* command_parameters);

/*!******************************************************************
 * \fn NODE_status_t NODE_scan(void)
 * \brief Scan all nodes connected to the RS485 bus.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NODE_status_t NODE_scan(void);

/*******************************************************************/
#define NODE_exit_error(error_base) { if (node_status != NODE_SUCCESS) { status = (error_base + node_status); goto errors; } }

/*******************************************************************/
#define NODE_stack_error(void) { if (node_status != NODE_SUCCESS) { ERROR_stack_add(ERROR_BASE_NODE + node_status); } }

/*******************************************************************/
#define NODE_stack_exit_error(error_code) { if (node_status != NODE_SUCCESS) { ERROR_stack_add(ERROR_BASE_NODE + node_status); status = error_code; goto errors; } }

#endif /* __NODE_H__ */
