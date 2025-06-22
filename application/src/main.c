/*
 * main.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

// Peripherals.
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
#include "types.h"
// Utils.
#include "error.h"
// Components.
#include "led.h"
// Middleware.
#include "cli.h"
#include "power.h"
// Applicative.
#include "error_base.h"

/*** MAIN local functions ***/

/*******************************************************************/
static void RS485_BRIDGE_init_hw(void) {
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    RTC_status_t rtc_status = RTC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    CLI_status_t cli_status = CLI_SUCCESS;
#ifndef RS485_BRIDGE_MODE_DEBUG
    IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
#ifdef RS485_BRIDGE
    RCC_pll_configuration_t pll_config;
    LED_status_t led_status = LED_SUCCESS;
#endif
    // Init error stack
    ERROR_stack_init();
    // Init memory.
    NVIC_init();
    // Init power module and clock tree.
    PWR_init();
    rcc_status = RCC_init(NVIC_PRIORITY_CLOCK);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init GPIOs.
    GPIO_init();
    POWER_init();
    EXTI_init();
#ifndef RS485_BRIDGE_MODE_DEBUG
    // Start independent watchdog.
    iwdg_status = IWDG_init();
    IWDG_stack_error(ERROR_BASE_IWDG);
    IWDG_reload();
#endif
#ifdef DIM
    // High speed oscillator.
    rcc_status = RCC_switch_to_hsi();
    RCC_stack_error(ERROR_BASE_RCC);
#endif
    // Init RTC.
    rtc_status = RTC_init(NULL, NVIC_PRIORITY_RTC);
    RTC_stack_error(ERROR_BASE_RTC);
    // Init delay timer.
    lptim_status = LPTIM_init(NVIC_PRIORITY_DELAY);
    LPTIM_stack_error(ERROR_BASE_LPTIM);
#ifdef RS485_BRIDGE
    // Turn TCXO on.
    POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_TCXO, LPTIM_DELAY_MODE_SLEEP);
    // Switch to PLL (system clock 120MHz, ADC clock 8MHz).
    pll_config.source = RCC_CLOCK_HSE;
    pll_config.hse_mode = RCC_HSE_MODE_BYPASS;
    pll_config.m = 2;
    pll_config.r = RCC_PLL_RQ_2;
#ifdef RS485_BRIDGE_MODE_LOW_BAUD_RATE
    // System clock 60MHz, ADC clock 8MHz.
    pll_config.n = 15;
    pll_config.p = 15;
    pll_config.q = RCC_PLL_RQ_4;
#else
    // System clock 120MHz, ADC clock 8MHz.
    pll_config.n = 30;
    pll_config.p = 30;
    pll_config.q = RCC_PLL_RQ_8;
#endif
    rcc_status = RCC_switch_to_pll(&pll_config);
    RCC_stack_error(ERROR_BASE_RCC);
#endif
    // Calibrate clocks.
    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
    RCC_stack_error(ERROR_BASE_RCC);
#ifdef RS485_BRIDGE
    led_status = LED_init();
    LED_stack_error(ERROR_BASE_LED);
#endif
    // Init AT interface.
    cli_status = CLI_init();
    CLI_stack_error(ERROR_BASE_CLI);
}

/*** MAIN function ***/

/*******************************************************************/
int main(void) {
    // Local variables.
    CLI_status_t cli_status = CLI_SUCCESS;
    // Init board.
    RS485_BRIDGE_init_hw();
    // Main loop.
    while (1) {
        // Enter sleep mode.
        IWDG_reload();
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        IWDG_reload();
        // Wake-up.
        cli_status = CLI_process();
        CLI_stack_error(ERROR_BASE_CLI);
    }
}
