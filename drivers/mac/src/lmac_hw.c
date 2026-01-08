/*
 * lmac_hw.c
 *
 *  Created on: 27 nov. 2024
 *      Author: Ludo
 */

#include "lmac_hw.h"

#ifndef LMAC_DRIVER_DISABLE_FLAGS_FILE
#include "lmac_driver_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "mcu_mapping.h"
#include "lmac.h"
#include "nvic_priority.h"
#include "rtc.h"
#include "types.h"
#include "una.h"
#include "usart.h"

#ifndef LMAC_DRIVER_DISABLE

/*** LMAC HW functions ***/

/*******************************************************************/
LMAC_status_t LMAC_HW_init(uint32_t baud_rate, LMAC_rx_irq_cb_t rx_irq_callback, uint8_t* self_address) {
    // Local variables.
    LMAC_status_t status = LMAC_SUCCESS;
#ifdef DIM
    LPUART_status_t lpuart_status = LPUART_SUCCESS;
    LPUART_configuration_t lpuart_config;
#endif
#ifdef RS485_BRIDGE
    USART_status_t usart_status = USART_SUCCESS;
    USART_configuration_t usart_config;
#endif
    // Update self address.
    (*self_address) = UNA_NODE_ADDRESS_RS485_BRIDGE;
#ifdef DIM
    // Init LPUART.
    lpuart_config.baud_rate = baud_rate;
    lpuart_config.nvic_priority = NVIC_PRIORITY_RS485;
    lpuart_config.rxne_irq_callback = rx_irq_callback;
    lpuart_config.self_address = UNA_NODE_ADDRESS_RS485_BRIDGE;
    lpuart_config.rs485_mode = LPUART_RS485_MODE_ADDRESSED;
    lpuart_status = LPUART_init(&LPUART_GPIO_RS485, &lpuart_config);
    LPUART_exit_error(LMAC_ERROR_BASE_HW_INTERFACE);
#endif
#ifdef RS485_BRIDGE
    // Init USART.
    usart_config.clock = RCC_CLOCK_SYSTEM;
    usart_config.baud_rate = baud_rate;
    usart_config.auto_baud_rate_mode = USART_AUTO_BAUD_RATE_MODE_DISABLED;
    usart_config.parity = USART_PARITY_NONE;
    usart_config.nvic_priority = NVIC_PRIORITY_RS485;
    usart_config.rxne_irq_callback = rx_irq_callback;
    usart_config.cm_irq_callback = NULL;
    usart_config.match_character = 0;
    usart_config.rs485_mode = USART_RS485_MODE_ADDRESSED;
    usart_config.self_address = UNA_NODE_ADDRESS_RS485_BRIDGE;
    usart_status = USART_init(USART_INSTANCE_RS485, &USART_GPIO_RS485, &usart_config);
    USART_exit_error(NODE_ERROR_BASE_USART);
#endif
errors:
    return status;
}

/*******************************************************************/
LMAC_status_t LMAC_HW_de_init(void) {
    // Local variables.
    LMAC_status_t status = LMAC_SUCCESS;
#ifdef DIM
    LPUART_status_t lpuart_status = LPUART_SUCCESS;
#endif
#ifdef RS485_BRIDGE
    USART_status_t usart_status = USART_SUCCESS;
#endif
#ifdef DIM
    // Release LPUART.
    lpuart_status = LPUART_de_init(&LPUART_GPIO_RS485);
    LPUART_stack_error(ERROR_BASE_LMAC + LMAC_ERROR_BASE_HW_INTERFACE);
#endif
#ifdef RS485_BRIDGE
    // Release USART.
    usart_status = USART_de_init(USART_INSTANCE_RS485, &USART_GPIO_RS485);
    USART_stack_error(ERROR_BASE_LMAC + LMAC_ERROR_BASE_HW_INTERFACE);
#endif
    return status;
}

/*******************************************************************/
LMAC_status_t LMAC_HW_enable_rx(void) {
    // Local variables.
    LMAC_status_t status = LMAC_SUCCESS;
#ifdef DIM
    LPUART_status_t lpuart_status = LPUART_SUCCESS;
#endif
#ifdef RS485_BRIDGE
    USART_status_t usart_status = USART_SUCCESS;
#endif
    // Enable receiver.
#ifdef DIM
    lpuart_status = LPUART_enable_rx();
    LPUART_exit_error(LMAC_ERROR_BASE_HW_INTERFACE);
#endif
#ifdef RS485_BRIDGE
    usart_status = USART_enable_rx(USART_INSTANCE_RS485);
    USART_exit_error(LMAC_ERROR_BASE_HW_INTERFACE);
#endif
errors:
    return status;
}

/*******************************************************************/
LMAC_status_t LMAC_HW_disable_rx(void) {
    // Local variables.
    LMAC_status_t status = LMAC_SUCCESS;
#ifdef DIM
    LPUART_status_t lpuart_status = LPUART_SUCCESS;
#endif
#ifdef RS485_BRIDGE
    USART_status_t usart_status = USART_SUCCESS;
#endif
    // Disable receiver.
#ifdef DIM
    lpuart_status = LPUART_disable_rx();
    LPUART_exit_error(LMAC_ERROR_BASE_HW_INTERFACE);
#endif
#ifdef RS485_BRIDGE
    usart_status = USART_disable_rx(USART_INSTANCE_RS485);
    USART_exit_error(LMAC_ERROR_BASE_HW_INTERFACE);
#endif
errors:
    return status;
}

/*******************************************************************/
LMAC_status_t LMAC_HW_write(uint8_t* data, uint32_t data_size_bytes) {
    // Local variables.
    LMAC_status_t status = LMAC_SUCCESS;
#ifdef DIM
    LPUART_status_t lpuart_status = LPUART_SUCCESS;
#endif
#ifdef RS485_BRIDGE
    USART_status_t usart_status = USART_SUCCESS;
#endif
    // Write data.
#ifdef DIM
    lpuart_status = LPUART_write(data, data_size_bytes);
    LPUART_exit_error(LMAC_ERROR_BASE_HW_INTERFACE);
#endif
#ifdef RS485_BRIDGE
    usart_status = USART_write(USART_INSTANCE_RS485, data, data_size_bytes);
    USART_exit_error(LMAC_ERROR_BASE_HW_INTERFACE);
#endif
errors:
    return status;
}

/*******************************************************************/
void LMAC_HW_stack_error(LMAC_status_t status) {
    ERROR_stack_add(ERROR_BASE_LMAC + status);
}

#endif /* LMAC_DRIVER_DISABLE */
