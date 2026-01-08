/*
 * r4s8cr_hw.c
 *
 *  Created on: 23 dec. 2024
 *      Author: Ludo
 */

#include "r4s8cr_hw.h"

#ifndef R4S8CR_DRIVER_DISABLE_FLAGS_FILE
#include "r4s8cr_driver_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "lpuart.h"
#include "mcu_mapping.h"
#include "iwdg.h"
#include "nvic_priority.h"
#include "r4s8cr.h"
#include "types.h"
#include "una.h"

#ifndef R4S8CR_DRIVER_DISABLE

/*** R4S8CR HW functions ***/

/*******************************************************************/
R4S8CR_status_t R4S8CR_HW_init(R4S8CR_HW_configuration_t* configuration) {
    // Local variables.
    R4S8CR_status_t status = R4S8CR_SUCCESS;
#ifdef DIM
    LPUART_status_t lpuart_status = LPUART_SUCCESS;
    LPUART_configuration_t lpuart_config;
#endif
#ifdef RS485_BRIDGE
    USART_status_t usart_status = USART_SUCCESS;
    USART_configuration_t usart_config;
#endif
#ifdef DIM
    // Init LPUART.
    lpuart_config.baud_rate = (configuration->uart_baud_rate);
    lpuart_config.nvic_priority = NVIC_PRIORITY_RS485;
    lpuart_config.rxne_irq_callback = (configuration->rx_irq_callback);
    lpuart_config.self_address = UNA_NODE_ADDRESS_MASTER;
    lpuart_config.rs485_mode = LPUART_RS485_MODE_DIRECT;
    lpuart_status = LPUART_init(&LPUART_GPIO_RS485, &lpuart_config);
    LPUART_exit_error(R4S8CR_ERROR_BASE_RS485);
#endif
#ifdef RS485_BRIDGE
    // Init USART.
    usart_config.clock = RCC_CLOCK_SYSTEM;
    usart_config.baud_rate = (configuration->uart_baud_rate);
    usart_config.auto_baud_rate_mode = USART_AUTO_BAUD_RATE_MODE_DISABLED;
    usart_config.parity = USART_PARITY_NONE;
    usart_config.nvic_priority = NVIC_PRIORITY_RS485;
    usart_config.rxne_irq_callback = (configuration->rx_irq_callback);
    usart_config.cm_irq_callback = NULL;
    usart_config.match_character = 0;
    usart_config.rs485_mode = USART_RS485_MODE_DIRECT;
    usart_config.self_address = UNA_NODE_ADDRESS_RS485_BRIDGE;
    usart_status = USART_init(USART_INSTANCE_RS485, &USART_GPIO_RS485, &usart_config);
    USART_exit_error(NODE_ERROR_BASE_USART);
#endif
errors:
    return status;
}

/*******************************************************************/
R4S8CR_status_t R4S8CR_HW_de_init(void) {
    // Local variables.
    R4S8CR_status_t status = R4S8CR_SUCCESS;
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
R4S8CR_status_t R4S8CR_HW_enable_rx(void) {
    // Local variables.
    R4S8CR_status_t status = R4S8CR_SUCCESS;
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
R4S8CR_status_t R4S8CR_HW_disable_rx(void) {
    // Local variables.
    R4S8CR_status_t status = R4S8CR_SUCCESS;
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
R4S8CR_status_t R4S8CR_HW_write(uint8_t* data, uint32_t data_size_bytes) {
    // Local variables.
    R4S8CR_status_t status = R4S8CR_SUCCESS;
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
    return status;
}

/*******************************************************************/
R4S8CR_status_t R4S8CR_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    R4S8CR_status_t status = R4S8CR_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Reload watchdog.
    IWDG_reload();
    // Perform delay.
    lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    LPTIM_exit_error(R4S8CR_ERROR_BASE_DELAY);
errors:
    return status;
}

#endif /* R4S8CR_DRIVER_DISABLE */

