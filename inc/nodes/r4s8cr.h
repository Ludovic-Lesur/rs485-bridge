/*
 * r4s8cr.h
 *
 *  Created on: 2 feb. 2023
 *      Author: Ludo
 */

#ifndef __R4S8CR_H__
#define __R4S8CR_H__

#include "node.h"
#include "node_common.h"

/*** R4S8CR macros ***/

#define R4S8CR_READ_TIMEOUT_MS		200
#define R4S8CR_WRITE_TIMEOUT_MS		1000

/*** R4S8CR functions ***/

/*!******************************************************************
 * \fn void R4S8CR_init(NODE_print_frame_cb_t print_callback)
 * \brief Init R4S8CR interface.
 * \param[in]  	print_callback: Function to call on frame reception.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void R4S8CR_init(NODE_print_frame_cb_t print_callback);

/*!******************************************************************
 * \fn NODE_status_t R4S8CR_configure_phy(void)
 * \brief Configure RS485 physical interface for R4S8CR transfer.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t R4S8CR_configure_phy(void);

/*!******************************************************************
 * \fn NODE_status_t R4S8CR_send_command(NODE_command_parameters_t* command_params)
 * \brief Send a command over to R4S8CR node.
 * \param[in]  	command_params: Pointer to the command parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t R4S8CR_send_command(NODE_command_parameters_t* command_params);

/*!******************************************************************
 * \fn NODE_status_t R4S8CR_write_register(NODE_access_parameters_t* write_params, uint32_t reg_value, uint32_t reg_mask, NODE_access_status_t* write_status)
 * \brief Write R4S8CR node register.
 * \param[in]  	write_params: Pointer to the write operation parameters.
 * \param[in]	reg_value: Register value to write.
 * \param[in]	reg_mask: Writing operation mask.
 * \param[in]	access_error_stack: Stack node access status error is non-zero.
 * \param[out] 	write_status: Pointer to the writing operation status.
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t R4S8CR_write_register(NODE_access_parameters_t* write_params, uint32_t reg_value, uint32_t reg_mask, NODE_access_status_t* write_statu, uint8_t access_error_stacks);

/*!******************************************************************
 * \fn NODE_status_t R4S8CR_read_register(NODE_access_parameters_t* read_params, uint32_t* reg_value, NODE_access_status_t* read_status)
 * \brief Read R4S8CR node register.
 * \param[in]  	read_params: Pointer to the read operation parameters.
 * \param[in]	access_error_stack: Stack node access status error is non-zero.
 * \param[out]	reg_value: Pointer to the read register value.
 * \param[out] 	read_status: Pointer to the read operation status.
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t R4S8CR_read_register(NODE_access_parameters_t* read_params, uint32_t* reg_value, NODE_access_status_t* read_status, uint8_t access_error_stack);

/*!******************************************************************
 * \fn NODE_status_t R4S8CR_scan(NODE_t* nodes_list, uint8_t nodes_list_size, uint8_t* nodes_count)
 * \brief Scan all R4S8CR nodes connected to the RS485 bus.
 * \param[in]  	nodes_list_size: Maximum size of the node list.
 * \param[out]	nodes_list: Pointer to the list where to store the nodes.
 * \param[out] 	nodes_count: Pointer to the number of nodes detected.
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t R4S8CR_scan(NODE_t* nodes_list, uint8_t nodes_list_size, uint8_t* nodes_count);

/*!******************************************************************
 * \fn NODE_status_t R4S8CR_task(void)
 * \brief Main task of R4S8CR interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NODE_status_t R4S8CR_task(void);

#endif /* __R4S8CR_H__ */
