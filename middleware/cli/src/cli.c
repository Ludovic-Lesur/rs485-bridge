/*
 * cli.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#include "cli.h"

// Peripherals.
#include "iwdg.h"
#include "lptim.h"
#include "nvic_priority.h"
#include "nvm.h"
#include "nvm_address.h"
#include "pwr.h"
#include "rcc.h"
// Components.
#include "dip_switch.h"
// Utils.
#include "at.h"
#include "error.h"
#include "math.h"
#include "parser.h"
#include "string.h"
#include "terminal_hw.h"
#include "terminal_instance.h"
#include "types.h"
// Middleware.
#include "una.h"
#include "una_at.h"
#include "una_r4s8cr.h"
#include "node.h"
// Applicative.
#include "dim_flags.h"
#include "error_base.h"

/*** AT local macros ***/

#define CLI_CHAR_SEPARATOR  STRING_CHAR_COMMA

//#define CLI_COMMAND_RST
//#define CLI_COMMAND_RCC

/*** CLI local structures ***/

/*******************************************************************/
typedef struct {
    volatile uint8_t at_process_flag;
    PARSER_context_t* at_parser_ptr;
} CLI_context_t;

/*** CLI local functions declaration ***/

/*******************************************************************/
#ifdef CLI_COMMAND_RST
static AT_status_t _CLI_rst_callback(void);
#endif
#ifdef CLI_COMMAND_RCC
static AT_status_t _CLI_rcc_callback(void);
#endif
static AT_status_t _CLI_adc_callback(void);
/*******************************************************************/
static AT_status_t _CLI_node_scan_callback(void);
static AT_status_t _CLI_node_get_protocol_callback(void);
static AT_status_t _CLI_node_set_protocol_callback(void);
static AT_status_t _CLI_node_command_callback(void);

/*** CLI local global variables ***/

static const AT_command_t CLI_COMMANDS_LIST[] = {
#ifdef CLI_COMMAND_RST
    {
        .syntax = "$RST",
        .parameters = NULL,
        .description = "Reset MCU",
        .callback = &_CLI_rst_callback
    },
#endif
#ifdef CLI_COMMAND_RCC
    {
        .syntax = "$RCC?",
        .parameters = NULL,
        .description = "Get clocks frequency",
        .callback = &_CLI_rcc_callback
    },
#endif
    {
        .syntax = "$ADC?",
        .parameters = NULL,
        .description = "Read analog measurements",
        .callback = &_CLI_adc_callback
    },
    {
        .syntax = "$SCAN",
        .parameters = NULL,
        .description = "Scan all slaves connected to the RS485 bus",
        .callback = &_CLI_node_scan_callback
    },
    {
        .syntax = "$PR?",
        .parameters = NULL,
        .description = "Get current node protocol",
        .callback = &_CLI_node_get_protocol_callback
    },
    {
        .syntax = "$PR=",
        .parameters = "protocol[dec],baud_rate[dec]",
        .description = "Set node protocol (0=None, 1=UNA_AT, 2=UNA_R4S8CR) and bus baud rate",
        .callback = &_CLI_node_set_protocol_callback
    },
    {
        .syntax = "$NODE=",
        .parameters = "node_address[hex],command[str]",
        .description = "Send node command",
        .callback = &_CLI_node_command_callback
    }
};

static const char_t* CLI_NODE_PROTOCOL_NAME[NODE_PROTOCOL_LAST] = {
    "None",
#ifdef DIM_ENABLE_UNA_AT
    "UNA_AT",
#endif
#ifdef DIM_ENABLE_UNA_R4S8CR
    "UNA_RS48CR"
#endif
};

static CLI_context_t cli_ctx;

/*** CLI local functions ***/

/*******************************************************************/
#define _CLI_check_driver_status(driver_status, driver_success, driver_error_base) { \
    /* Check value */ \
    if (driver_status != driver_success) { \
        /* Stack error */ \
        ERROR_stack_add((ERROR_code_t) (driver_error_base + driver_status)); \
        /* Exit with execution error */ \
        status = AT_ERROR_COMMAND_EXECUTION; \
        goto errors; \
    } \
}

/*******************************************************************/
static void _CLI_at_process_callback(void) {
    // Set local flag.
    cli_ctx.at_process_flag = 1;
}

#ifdef CLI_COMMAND_RST
/*******************************************************************/
static AT_status_t _CLI_rst_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    // Execute command.
    PWR_software_reset();
    return status;
}
#endif

#ifdef CLI_COMMAND_RCC
/*******************************************************************/
static AT_status_t _CLI_rcc_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    RCC_status_t rcc_status = RCC_SUCCESS;
    char_t* rcc_clock_name[RCC_CLOCK_LAST - 1] = { "SYS", "HSI", "MSI", "HSE", "PLL", "LSI", "LSE" };
    uint8_t rcc_clock_index = 0;
    uint8_t clock_status = 0;
    uint32_t clock_frequency = 0;
    uint8_t idx = 0;
    // Calibrate clocks.
    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
    _CLI_check_driver_status(rcc_status, RCC_SUCCESS, ERROR_BASE_RCC);
    // Clocks loop.
    for (idx = 0; idx < (RCC_CLOCK_LAST - 1); idx++) {
        // Convert to RCC clock index.
        rcc_clock_index = (RCC_CLOCK_NONE + 1 + idx);
        // Read status.
        rcc_status = RCC_get_status(rcc_clock_index, &clock_status);
        _CLI_check_driver_status(rcc_status, RCC_SUCCESS, ERROR_BASE_RCC);
        // Read frequency.
        rcc_status = RCC_get_frequency_hz(rcc_clock_index, &clock_frequency);
        _CLI_check_driver_status(rcc_status, RCC_SUCCESS, ERROR_BASE_RCC);
        // Print data.
        AT_reply_add_string(rcc_clock_name[idx]);
        AT_reply_add_string((clock_status == 0) ? ": OFF " : ": ON  ");
        AT_reply_add_integer((int32_t) clock_frequency, STRING_FORMAT_DECIMAL, 0);
        AT_reply_add_string("Hz");
        AT_send_reply();
    }
errors:
    return status;
}
#endif

/*******************************************************************/
static AT_status_t _CLI_adc_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    POWER_status_t power_status = POWER_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    int32_t generic_s32 = 0;
    // Turn analog front-end on.
    power_status = POWER_enable(POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
    _CLI_check_driver_status(power_status, POWER_SUCCESS, ERROR_BASE_POWER);
    // MCU voltage.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VMCU_MV, &generic_s32);
    _CLI_check_driver_status(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG);
    AT_reply_add_string("Vmcu=");
    AT_reply_add_integer(generic_s32, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("mV");
    AT_send_reply();
    // MCU temperature.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_TMCU_DEGREES, &generic_s32);
    _CLI_check_driver_status(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG);
    AT_reply_add_string("Tmcu=");
    AT_reply_add_integer(generic_s32, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("dC");
    AT_send_reply();
    // Source voltage.
    AT_reply_add_string("Vrs=");
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VRS_MV, &generic_s32);
    _CLI_check_driver_status(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG);
    AT_reply_add_integer(generic_s32, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("mV");
    AT_send_reply();
    // Supercap voltage.
    AT_reply_add_string("Vusb=");
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VUSB_MV, &generic_s32);
    _CLI_check_driver_status(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG);
    AT_reply_add_integer(generic_s32, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("mV");
    AT_send_reply();
    // Turn analog front-end off.
    power_status = POWER_disable(POWER_DOMAIN_ANALOG);
    _CLI_check_driver_status(power_status, POWER_SUCCESS, ERROR_BASE_POWER);
    goto end;
errors:
    POWER_disable(POWER_DOMAIN_ANALOG);
end:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_node_scan_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    uint8_t idx = 0;
    // Check if TX is allowed.
    if (DIP_SWITCH_get_tx_mode() == DIP_SWITCH_TX_MODE_DISABLED) {
        status = AT_ERROR_COMMAND_EXECUTION;
        goto errors;
    }
    // Init screen.
    AT_reply_add_string("Nodes scan running...");
    AT_send_reply();
    // Perform bus scan.
    node_status = NODE_scan();
    _CLI_check_driver_status(node_status, NODE_SUCCESS, ERROR_BASE_NODE);
    // Print list.
    AT_reply_add_integer((int32_t) NODES_LIST.count, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string(" node(s) found");
    AT_send_reply();
    for (idx = 0; idx < NODES_LIST.count; idx++) {
        // Print address.
        AT_reply_add_integer(NODES_LIST.list[idx].address, STRING_FORMAT_HEXADECIMAL, 1);
        AT_reply_add_string(" : ");
        // Print board type.
        if (NODES_LIST.list[idx].board_id == UNA_BOARD_ID_ERROR) {
            AT_reply_add_string("Board ID error");
        }
        else {
            if (NODES_LIST.list[idx].board_id >= UNA_BOARD_ID_LAST) {
                AT_reply_add_string("Unknown board ID (");
                AT_reply_add_integer((int32_t) (NODES_LIST.list[idx].board_id), STRING_FORMAT_HEXADECIMAL, 1);
                AT_reply_add_string(")");
            }
            else {
                AT_reply_add_string((char_t*) UNA_BOARD_NAME[NODES_LIST.list[idx].board_id]);
            }
        }
        AT_send_reply();
    }
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_node_get_protocol_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    NODE_protocol_t protocol = NODE_PROTOCOL_NONE;
    uint32_t baud_rate = 0;
    // Read protocol.
    node_status = NODE_get_protocol(&protocol, &baud_rate);
    _CLI_check_driver_status(node_status, NODE_SUCCESS, ERROR_BASE_NODE);
    // Print value.
    AT_reply_add_integer((int32_t) protocol, STRING_FORMAT_DECIMAL, 0);
    // Print name.
    AT_reply_add_string(" (");
    AT_reply_add_string((char_t*) CLI_NODE_PROTOCOL_NAME[protocol]);
    // Print baud rate.
    AT_reply_add_string(") @");
    AT_reply_add_integer((int32_t) baud_rate, STRING_FORMAT_DECIMAL, 0);
    AT_reply_add_string("bauds");
    // Send reply.
    AT_send_reply();
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_node_set_protocol_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    int32_t protocol = NODE_PROTOCOL_NONE;
    int32_t baud_rate = 0;
    // Parse protocol.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, CLI_CHAR_SEPARATOR, &protocol);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Parse baud rate
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &baud_rate);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Set protocol.
    node_status = NODE_set_protocol((NODE_protocol_t) protocol, (uint32_t) baud_rate);
    _CLI_check_driver_status(node_status, NODE_SUCCESS, ERROR_BASE_NODE);
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_node_command_callback(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    PARSER_status_t parser_status = PARSER_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    UNA_command_parameters_t command_params;
    int32_t node_addr = 0;
    uint8_t command_offset = 0;
    // Check if TX is allowed.
    if (DIP_SWITCH_get_tx_mode() == DIP_SWITCH_TX_MODE_DISABLED) {
        status = AT_ERROR_COMMAND_EXECUTION;
        goto errors;
    }
    // Parse node address.
    parser_status = PARSER_get_parameter(cli_ctx.at_parser_ptr, STRING_FORMAT_HEXADECIMAL, CLI_CHAR_SEPARATOR, &node_addr);
    PARSER_exit_error(AT_ERROR_BASE_PARSER);
    // Set command offset.
    command_offset = ((cli_ctx.at_parser_ptr)->separator_index) + 1;
    // Update parameters.
    command_params.node_addr = (UNA_node_address_t) node_addr;
    command_params.command = (char_t*) &((cli_ctx.at_parser_ptr)->buffer[command_offset]);
    // Print node access.
    AT_reply_add_integer(UNA_NODE_ADDRESS_DIM, STRING_FORMAT_HEXADECIMAL, 1);
    AT_reply_add_string(" > ");
    AT_reply_add_integer(command_params.node_addr, STRING_FORMAT_HEXADECIMAL, 1);
    AT_reply_add_string(" : ");
    AT_reply_add_string(command_params.command);
    AT_send_reply();
    // Perform read operation.
    node_status = NODE_send_command(&command_params);
    _CLI_check_driver_status(node_status, NODE_SUCCESS, ERROR_BASE_NODE);
errors:
    return status;
}

/*******************************************************************/
static void _CLI_node_print_frame_callback(char_t* frame) {
    AT_reply_add_string(frame);
    AT_send_reply();
}

/*******************************************************************/
static void _CLI_node_none_protocol_rx_irq_callback(uint8_t data) {
    // Local variables.
    uint8_t raw_byte = data;
    // Call terminal directly.
    TERMINAL_HW_write(TERMINAL_INSTANCE_CLI, &raw_byte, 1);
}

/*** AT functions ***/

/*******************************************************************/
CLI_status_t CLI_init(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    AT_configuration_t at_config;
    uint8_t idx = 0;
    // Init context.
    cli_ctx.at_process_flag = 0;
    cli_ctx.at_parser_ptr = NULL;
    // Init AT driver.
    at_config.terminal_instance = TERMINAL_INSTANCE_CLI;
    at_config.process_callback = &_CLI_at_process_callback;
    at_status = AT_init(&at_config, &(cli_ctx.at_parser_ptr));
    AT_exit_error(CLI_ERROR_BASE_AT);
    // Register commands.
    for (idx = 0; idx < (sizeof(CLI_COMMANDS_LIST) / sizeof(AT_command_t)); idx++) {
        at_status = AT_register_command(&(CLI_COMMANDS_LIST[idx]));
        AT_exit_error(CLI_ERROR_BASE_AT);
    }
    // Init node layer.
    node_status = NODE_init(&_CLI_node_print_frame_callback, &_CLI_node_none_protocol_rx_irq_callback);
    NODE_exit_error(CLI_ERROR_BASE_NODE);
errors:
    return status;
}

/*******************************************************************/
CLI_status_t CLI_de_init(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    uint8_t idx = 0;
    // Unregister commands.
    for (idx = 0; idx < (sizeof(CLI_COMMANDS_LIST) / sizeof(AT_command_t)); idx++) {
        at_status = AT_unregister_command(&(CLI_COMMANDS_LIST[idx]));
        AT_exit_error(CLI_ERROR_BASE_AT);
    }
    // Release AT driver.
    at_status = AT_de_init();
    AT_exit_error(CLI_ERROR_BASE_AT);
    // Release node layer.
    node_status = NODE_de_init();
    NODE_exit_error(CLI_ERROR_BASE_NODE);
errors:
    return status;
}

/*******************************************************************/
CLI_status_t CLI_process(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    // Check process flag.
    if (cli_ctx.at_process_flag != 0) {
        // Clear flag.
        cli_ctx.at_process_flag = 0;
        // Process AT driver.
        at_status = AT_process();
        AT_exit_error(CLI_ERROR_BASE_AT);
    }
    // Process bus decoding.
    node_status = NODE_process();
    NODE_exit_error(CLI_ERROR_BASE_NODE);
errors:
    return status;
}
