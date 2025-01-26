/*
 * node.c
 *
 *  Created on: 26 feb. 2023
 *      Author: Ludo
 */

#include "node.h"

#include "dim_flags.h"
#include "error.h"
#include "gpio_mapping.h"
#include "nvic_priority.h"
#include "string.h"
#include "types.h"
#include "una.h"
#include "una_at.h"
#include "una_r4s8cr.h"

/*** NODE local macros ***/

#define NODE_RX_BUFFER_DEPTH            16
#define NODE_RX_BUFFER_SIZE_BYTES       128

#define NODE_UNA_AT_FRAME_END_MARKER    STRING_CHAR_CR

#define NODE_UNA_R4S8CR_FRAME_SIZE      10

#define NODE_DEFAULT_TIMEOUT_MS         5000

/*** NODE local structures ***/

/*******************************************************************/
typedef struct {
    uint8_t buffer[NODE_RX_BUFFER_SIZE_BYTES];
    uint32_t size;
} NODE_rx_buffer_t;

/*******************************************************************/
typedef struct {
    NODE_print_frame_cb_t print_frame_callback;
    NODE_none_protocol_rx_irq_cb_t none_protocol_rx_irq_callback;
    NODE_rx_buffer_t rx_buffer[NODE_RX_BUFFER_DEPTH];
    uint8_t rx_buffer_write_index;
    uint8_t rx_buffer_read_index;
    NODE_protocol_t protocol;
    uint32_t baud_rate;
} NODE_context_t;

/*******************************************************************/
typedef NODE_status_t (*NODE_decode_frame_cb_t)(void);

/*** NODE local functions declaration ***/

#ifdef DIM_ENABLE_UNA_AT
static NODE_status_t _NODE_print_una_at_frame(void);
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
static NODE_status_t _NODE_print_una_r4s8cr_frame(void);
#endif

/*** NODES global variables ***/

UNA_node_list_t NODES_LIST;

/*** NODE local global variables ***/

static const NODE_decode_frame_cb_t NODE_DECODE_FRAME_PFN[NODE_PROTOCOL_LAST] = {
    NULL,
#ifdef DIM_ENABLE_UNA_AT
    &_NODE_print_una_at_frame,
#else
    NULL,
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    &_NODE_print_una_r4s8cr_frame,
#else
    NULL,
#endif
};

static NODE_context_t node_ctx;

/*** NODE local functions ***/

/*******************************************************************/
static void _NODE_flush_rx_buffer(uint8_t buffer_index) {
    // Local variables.
    uint8_t idx = 0;
    // Reset bytes and size.
    for (idx = 0; idx < NODE_RX_BUFFER_SIZE_BYTES; idx++) {
        node_ctx.rx_buffer[buffer_index].buffer[idx] = 0;
        node_ctx.rx_buffer[buffer_index].size = 0;
    }
}

/*******************************************************************/
static void _NODE_flush_rx_buffers(void) {
    // Local variables.
    uint8_t idx = 0;
    // Buffers loop.
    for (idx = 0; idx < NODE_RX_BUFFER_DEPTH; idx++) {
        _NODE_flush_rx_buffer(idx);
    }
    node_ctx.rx_buffer_write_index = 0;
    node_ctx.rx_buffer_read_index = 0;
}

#ifdef DIM_ENABLE_UNA_AT
/*******************************************************************/
static NODE_status_t _NODE_print_una_at_frame(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    STRING_status_t string_status = STRING_SUCCESS;
    NODE_rx_buffer_t* rx_buffer_ptr = &(node_ctx.rx_buffer[node_ctx.rx_buffer_read_index]);
    UNA_node_address_t source_address;
    UNA_node_address_t destination_address;
    char_t una_at_frame[NODE_RX_BUFFER_SIZE_BYTES];
    uint32_t una_at_frame_size = 0;
    uint8_t idx = 0;
    // Flush local frame.
    for (idx = 0; idx < NODE_RX_BUFFER_SIZE_BYTES; idx++) {
        una_at_frame[idx] = STRING_CHAR_NULL;
    }
    // Read addresses.
    destination_address = ((UNA_node_address_t) (rx_buffer_ptr->buffer[0])) & LMAC_ADDRESS_MASK;
    source_address = ((UNA_node_address_t) (rx_buffer_ptr->buffer[1])) & LMAC_ADDRESS_MASK;
    // Print source address.
    string_status = STRING_append_integer(una_at_frame, NODE_RX_BUFFER_SIZE_BYTES, source_address, STRING_FORMAT_HEXADECIMAL, 0, &una_at_frame_size);
    STRING_exit_error(NODE_ERROR_BASE_STRING);
    // Print symbol.
    string_status = STRING_append_string(una_at_frame, NODE_RX_BUFFER_SIZE_BYTES, " > ", &una_at_frame_size);
    STRING_exit_error(NODE_ERROR_BASE_STRING);
    // Print destination address.
    string_status = STRING_append_integer(una_at_frame, NODE_RX_BUFFER_SIZE_BYTES, destination_address, STRING_FORMAT_HEXADECIMAL, 0, &una_at_frame_size);
    STRING_exit_error(NODE_ERROR_BASE_STRING);
    // Print symbol.
    string_status = STRING_append_string(una_at_frame, NODE_RX_BUFFER_SIZE_BYTES, " : ", &una_at_frame_size);
    STRING_exit_error(NODE_ERROR_BASE_STRING);
    // Print command.
    string_status = STRING_append_string(una_at_frame, NODE_RX_BUFFER_SIZE_BYTES, (char_t*) &(rx_buffer_ptr->buffer[2]), &una_at_frame_size);
    STRING_exit_error(NODE_ERROR_BASE_STRING);
    // Print frame.
    if (node_ctx.print_frame_callback != NULL) {
        node_ctx.print_frame_callback(una_at_frame);
    }
errors:
    return status;
}
#endif

#ifdef DIM_ENABLE_UNA_R4S8CR
/*******************************************************************/
static NODE_status_t _NODE_print_una_r4s8cr_frame(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    STRING_status_t string_status = STRING_SUCCESS;
    NODE_rx_buffer_t* rx_buffer_ptr = &(node_ctx.rx_buffer[node_ctx.rx_buffer_read_index]);
    char_t una_r4s8cr_frame[NODE_RX_BUFFER_SIZE_BYTES];
    uint8_t idx = 0;
    // Flush local frame.
    for (idx = 0; idx < NODE_RX_BUFFER_SIZE_BYTES; idx++) {
        una_r4s8cr_frame[idx] = STRING_CHAR_NULL;
    }
    // Convert to ASCII.
    string_status = STRING_byte_array_to_hexadecimal_string((uint8_t*) (rx_buffer_ptr->buffer), (rx_buffer_ptr->size), 0, una_r4s8cr_frame);
    STRING_exit_error(NODE_ERROR_BASE_STRING);
    // Print frame.
    if (node_ctx.print_frame_callback != NULL) {
        node_ctx.print_frame_callback(una_r4s8cr_frame);
    }
errors:
    return status;
}
#endif

/*******************************************************************/
static void _NODE_rx_irq_callback(uint8_t data) {
    // Local variables.
#if ((defined DIM_ENABLE_UNA_AT) || (defined DIM_ENABLE_UNA_R4S8CR))
    NODE_rx_buffer_t* rx_buffer_ptr = &(node_ctx.rx_buffer[node_ctx.rx_buffer_write_index]);
#endif
    // Check protocol.
    switch (node_ctx.protocol) {
    case NODE_PROTOCOL_NONE:
        // Call the applicative callback directly.
        if (node_ctx.none_protocol_rx_irq_callback != NULL) {
            node_ctx.none_protocol_rx_irq_callback(data);
        }
        break;
#ifdef DIM_ENABLE_UNA_AT
    case NODE_PROTOCOL_UNA_AT:
        // Check ending marker.
        if (data == NODE_UNA_AT_FRAME_END_MARKER) {
            // Switch buffer.
            node_ctx.rx_buffer_write_index = (node_ctx.rx_buffer_write_index + 1) % NODE_RX_BUFFER_DEPTH;
        }
        else {
            rx_buffer_ptr->buffer[rx_buffer_ptr->size] = data;
            rx_buffer_ptr->size = (rx_buffer_ptr->size + 1) % NODE_RX_BUFFER_SIZE_BYTES;
        }
        break;
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    case NODE_PROTOCOL_UNA_R4S8CR:
        // Workaround to handle the LPUART TX/RX switching time issue.
        if ((rx_buffer_ptr->size == 0) && (data != 0xFF)) {
            // Manually add the R4S8CR header byte.
            rx_buffer_ptr->buffer[rx_buffer_ptr->size] = 0xFF;
            rx_buffer_ptr->size = (rx_buffer_ptr->size + 1) % NODE_RX_BUFFER_SIZE_BYTES;
        }
        // Fill buffer.
        rx_buffer_ptr->buffer[rx_buffer_ptr->size] = data;
        rx_buffer_ptr->size = (rx_buffer_ptr->size + 1) % NODE_RX_BUFFER_SIZE_BYTES;
        // Check frame size.
        if ((rx_buffer_ptr->size) >= NODE_UNA_R4S8CR_FRAME_SIZE) {
            // Switch buffer.
            node_ctx.rx_buffer_write_index = (node_ctx.rx_buffer_write_index + 1) % NODE_RX_BUFFER_DEPTH;
        }
        break;
#endif
    default:
        break;
    }
}

/*******************************************************************/
static NODE_status_t _NODE_start_decoding(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    LPUART_status_t lpuart_status = LPUART_SUCCESS;
    LPUART_configuration_t lpuart_config;
    // Init LPUART.
    lpuart_config.baud_rate = node_ctx.baud_rate;
    lpuart_config.nvic_priority = NVIC_PRIORITY_RS485;
    lpuart_config.rxne_callback = &_NODE_rx_irq_callback;
    lpuart_config.self_address = UNA_NODE_ADDRESS_DIM;
    lpuart_config.rx_mode = LPUART_RX_MODE_DIRECT;
    lpuart_status = LPUART_init(&GPIO_RS485_LPUART, &lpuart_config);
    LPUART_exit_error(NODE_ERROR_BASE_LPUART);
    // Start receiver.
    lpuart_status = LPUART_enable_rx();
    LPUART_exit_error(NODE_ERROR_BASE_LPUART);
errors:
    return status;
}

/*******************************************************************/
static NODE_status_t _NODE_stop_decoding(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    LPUART_status_t lpuart_status = LPUART_SUCCESS;
    // Stop receiver.
    lpuart_status = LPUART_disable_rx();
    LPUART_exit_error(NODE_ERROR_BASE_LPUART);
    // Release LPUART.
    lpuart_status = LPUART_de_init(&GPIO_RS485_LPUART);
    LPUART_exit_error(NODE_ERROR_BASE_LPUART);
errors:
    return status;
}

/*** NODE functions ***/

/*******************************************************************/
NODE_status_t NODE_init(NODE_print_frame_cb_t print_frame_callback, NODE_none_protocol_rx_irq_cb_t none_protocol_rx_irq_callback) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    // Reset node list and RX buffer.
    UNA_reset_node_list(&NODES_LIST);
    _NODE_flush_rx_buffers();
    // Register callbacks.
    node_ctx.print_frame_callback = print_frame_callback;
    node_ctx.none_protocol_rx_irq_callback = none_protocol_rx_irq_callback;
    // Default configuration.
#ifdef DIM_ENABLE_UNA_AT
    node_ctx.protocol = NODE_PROTOCOL_UNA_AT;
    node_ctx.baud_rate = 1200;
#else
    node_ctx.protocol = NODE_PROTOCOL_NONE;
    node_ctx.baud_rate = 9600;
#endif
    // Start reception in UNA_AT protocol mode by default.
    status = _NODE_start_decoding();
    if (status != NODE_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_de_init(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    // Stop reception.
    status = _NODE_stop_decoding();
    if (status != NODE_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_process(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    // Check write index.
    while (node_ctx.rx_buffer_read_index != node_ctx.rx_buffer_write_index) {
        // Execute decoding function.
        if (NODE_DECODE_FRAME_PFN[node_ctx.protocol] != NULL) {
            status = NODE_DECODE_FRAME_PFN[node_ctx.protocol]();
            if (status != NODE_SUCCESS) goto errors;
        }
        // Reset buffer.
        _NODE_flush_rx_buffer(node_ctx.rx_buffer_read_index);
        // Increment read index.
        node_ctx.rx_buffer_read_index = (node_ctx.rx_buffer_read_index + 1) % NODE_RX_BUFFER_DEPTH;
    }
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_set_protocol(NODE_protocol_t protocol, uint32_t baud_rate) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    // Check protocol.
    if (protocol >= NODE_PROTOCOL_LAST) {
        status = NODE_ERROR_PROTOCOL;
        goto errors;
    }
    // Flush buffer.
    _NODE_flush_rx_buffers();
    // Update context.
    node_ctx.protocol = protocol;
    node_ctx.baud_rate = baud_rate;
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_get_protocol(NODE_protocol_t* protocol, uint32_t* baud_rate) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    // Check parameters.
    if ((protocol == NULL) || (baud_rate == NULL)) {
        status = NODE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    (*protocol) = node_ctx.protocol;
    (*baud_rate) = node_ctx.baud_rate;
errors:
    return status;
}

/*******************************************************************/
NODE_status_t NODE_write_register(UNA_node_t* node, uint8_t reg_addr, uint32_t reg_value, uint32_t reg_mask, UNA_access_status_t* write_status) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
#ifdef DIM_ENABLE_UNA_AT
    UNA_AT_status_t una_at_status = UNA_AT_SUCCESS;
    UNA_AT_configuration_t una_at_config;
    uint8_t una_at_init = 0;
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    UNA_R4S8CR_status_t una_r4s8cr_status = UNA_R4S8CR_SUCCESS;
    uint8_t una_r4s8cr_init = 0;
#endif
    UNA_access_parameters_t write_params;
    // Check parameters.
    if ((node == NULL) || (write_status == NULL)) {
        status = NODE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Common write parameters.
    write_params.node_addr = (node -> address);
    write_params.reg_addr = reg_addr;
    write_params.reply_params.timeout_ms = NODE_DEFAULT_TIMEOUT_MS;
    write_params.reply_params.type = UNA_REPLY_TYPE_OK;
    // Stop reception.
    status = _NODE_stop_decoding();
    if (status != NODE_SUCCESS) goto errors;
    // Check protocol.
    switch (node_ctx.protocol) {
#ifdef DIM_ENABLE_UNA_AT
    case NODE_PROTOCOL_UNA_AT:
        // Write UNA AT node register.
        una_at_init = 1;
        una_at_config.baud_rate = node_ctx.baud_rate;
        una_at_status = UNA_AT_init(&una_at_config);
        UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
        una_at_status = UNA_AT_write_register(&write_params, reg_value, reg_mask, write_status);
        UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
        una_at_status = UNA_AT_de_init();
        UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
        una_at_init = 0;
        break;
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    case NODE_PROTOCOL_UNA_R4S8CR:
        // Write UNA R4S8CR node register.
        una_r4s8cr_init = 1;
        una_r4s8cr_status = UNA_R4S8CR_init();
        UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
        una_r4s8cr_status = UNA_R4S8CR_write_register(&write_params, reg_value, reg_mask, write_status);
        UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
        una_r4s8cr_status = UNA_R4S8CR_de_init();
        UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
        una_r4s8cr_init = 0;
        break;
#endif
    default:
        status = NODE_ERROR_PROTOCOL;
        goto errors;
    }
errors:
    // Force release in case of error.
#ifdef DIM_ENABLE_UNA_AT
    if (una_at_init != 0) {
        UNA_AT_de_init();
    }
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    if (una_r4s8cr_init != 0) {
        UNA_R4S8CR_de_init();
    }
#endif
    // Re-start reception.
    _NODE_start_decoding();
    return status;
}

/*******************************************************************/
NODE_status_t NODE_read_register(UNA_node_t* node, uint8_t reg_addr, uint32_t* reg_value, UNA_access_status_t* read_status) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
    UNA_access_parameters_t read_params;
#ifdef DIM_ENABLE_UNA_AT
    UNA_AT_status_t una_at_status = UNA_AT_SUCCESS;
    UNA_AT_configuration_t una_at_config;
    uint8_t una_at_init = 0;
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    UNA_R4S8CR_status_t una_r4s8cr_status = UNA_R4S8CR_SUCCESS;
    uint8_t una_r4s8cr_init = 0;
#endif
    // Check parameters.
    if ((node == NULL) || (read_status == NULL)) {
        status = NODE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Write parameters.
    read_params.node_addr = (node->address);
    read_params.reg_addr = reg_addr;
    read_params.reply_params.timeout_ms = NODE_DEFAULT_TIMEOUT_MS;
    read_params.reply_params.type = UNA_REPLY_TYPE_VALUE;
    // Stop reception.
    status = _NODE_stop_decoding();
    if (status != NODE_SUCCESS) goto errors;
    // Check protocol.
    switch (node_ctx.protocol) {
#ifdef DIM_ENABLE_UNA_AT
    case NODE_PROTOCOL_UNA_AT:
        // Write UNA AT node register.
        una_at_init = 1;
        una_at_config.baud_rate = node_ctx.baud_rate;
        una_at_status = UNA_AT_init(&una_at_config);
        UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
        una_at_status = UNA_AT_read_register(&read_params, reg_value, read_status);
        UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
        una_at_status = UNA_AT_de_init();
        UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
        una_at_init = 0;
        break;
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    case NODE_PROTOCOL_UNA_R4S8CR:
        // Write UNA R4S8CR node register.
        una_r4s8cr_init = 1;
        una_r4s8cr_status = UNA_R4S8CR_init();
        UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
        una_r4s8cr_status = UNA_R4S8CR_read_register(&read_params, reg_value, read_status);
        UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
        una_r4s8cr_status = UNA_R4S8CR_de_init();
        UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
        una_r4s8cr_init = 0;
        break;
#endif
    default:
        status = NODE_ERROR_PROTOCOL;
        goto errors;
    }
errors:
    // Force release in case of error.
#ifdef DIM_ENABLE_UNA_AT
    if (una_at_init != 0) {
        UNA_AT_de_init();
    }
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    if (una_r4s8cr_init != 0) {
        UNA_R4S8CR_de_init();
    }
#endif
    // Re-start reception.
    _NODE_start_decoding();
    return status;
}

/*******************************************************************/
NODE_status_t NODE_send_command(UNA_command_parameters_t* command_parameters) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
#ifdef DIM_ENABLE_UNA_AT
    UNA_AT_status_t una_at_status = UNA_AT_SUCCESS;
    UNA_AT_configuration_t una_at_config;
    uint8_t una_at_init = 0;
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    UNA_R4S8CR_status_t una_r4s8cr_status = UNA_R4S8CR_SUCCESS;
    uint8_t una_r4s8cr_init = 0;
#endif
    // Check parameters.
    if (command_parameters == NULL) {
        status = NODE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if ((command_parameters->command) == NULL) {
        status = NODE_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Stop reception.
    status = _NODE_stop_decoding();
    if (status != NODE_SUCCESS) goto errors;
    // Send command with current protocol.
    switch (node_ctx.protocol) {
#ifdef DIM_ENABLE_UNA_AT
    case NODE_PROTOCOL_UNA_AT:
        // Send command.
        una_at_init = 1;
        una_at_config.baud_rate = node_ctx.baud_rate;
        una_at_status = UNA_AT_init(&una_at_config);
        UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
        una_at_status = UNA_AT_send_command(command_parameters);
        UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
        una_at_status = UNA_AT_de_init();
        UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
        una_at_init = 0;
        break;
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    case NODE_PROTOCOL_UNA_R4S8CR:
        // Scan UNA R4S8CR nodes.
        una_r4s8cr_init = 1;
        una_r4s8cr_status = UNA_R4S8CR_init();
        UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
        una_r4s8cr_status = UNA_R4S8CR_send_command(command_parameters);
        UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
        una_r4s8cr_status = UNA_R4S8CR_de_init();
        UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
        una_r4s8cr_init = 0;
        break;
#endif
    default:
        status = NODE_ERROR_PROTOCOL;
        break;
    }
errors:
    // Force release in case of error.
#ifdef DIM_ENABLE_UNA_AT
    if (una_at_init != 0) {
        UNA_AT_de_init();
    }
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    if (una_r4s8cr_init != 0) {
        UNA_R4S8CR_de_init();
    }
#endif
    // Re-start reception.
    _NODE_start_decoding();
    return status;
}

/*******************************************************************/
NODE_status_t NODE_scan(void) {
    // Local variables.
    NODE_status_t status = NODE_SUCCESS;
#ifdef DIM_ENABLE_UNA_AT
    UNA_AT_status_t una_at_status = UNA_AT_SUCCESS;
    UNA_AT_configuration_t una_at_config;
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    UNA_R4S8CR_status_t una_r4s8cr_status = UNA_R4S8CR_SUCCESS;
#endif
    uint8_t nodes_count = 0;
    // Stop reception.
    status = _NODE_stop_decoding();
    if (status != NODE_SUCCESS) goto errors;
    // Reset list.
    UNA_reset_node_list(&NODES_LIST);
#ifdef DIM_ENABLE_UNA_AT
    // Scan UNA AT nodes.
    una_at_config.baud_rate = node_ctx.baud_rate;
    una_at_status = UNA_AT_init(&una_at_config);
    UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
    una_at_status = UNA_AT_scan(&(NODES_LIST.list[NODES_LIST.count]), (UNA_NODE_ADDRESS_LAST - NODES_LIST.count), &nodes_count);
    UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
    una_at_status = UNA_AT_de_init();
    UNA_AT_exit_error(NODE_ERROR_BASE_UNA_AT);
    // Update count.
    NODES_LIST.count += nodes_count;
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    // Scan UNA R4S8CR nodes.
    una_r4s8cr_status = UNA_R4S8CR_init();
    UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
    una_r4s8cr_status = UNA_R4S8CR_scan(&(NODES_LIST.list[NODES_LIST.count]), (UNA_NODE_ADDRESS_LAST - NODES_LIST.count), &nodes_count);
    UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
    una_r4s8cr_status = UNA_R4S8CR_de_init();
    UNA_R4S8CR_exit_error(NODE_ERROR_BASE_UNA_R4S8CR);
    // Update count.
    NODES_LIST.count += nodes_count;
#endif
    // Re-start reception.
    status = _NODE_start_decoding();
    if (status != NODE_SUCCESS) goto errors;
errors:
    return status;
}
