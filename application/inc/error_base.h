/*
 * error.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_BASE_H__
#define __ERROR_BASE_H__

// Peripherals.
#include "iwdg.h"
#include "lptim.h"
#include "rcc.h"
#include "rtc.h"
// MAC.
#include "lmac.h"
// Utils.
#include "error.h"
#include "terminal.h"
// Components.
#include "led.h"
#include "r4s8cr.h"
// Middleware.
#include "analog.h"
#include "cli.h"
#include "power.h"
// Nodes.
#include "node.h"

/*!******************************************************************
 * \enum ERROR_base_t
 * \brief Board error bases.
 *******************************************************************/
typedef enum {
    SUCCESS = 0,
    // Peripherals.
    ERROR_BASE_IWDG = ERROR_BASE_STEP,
    ERROR_BASE_LPTIM = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
    ERROR_BASE_RCC = (ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
    ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
    // MAC.
    ERROR_BASE_LMAC = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
    // Utils.
    ERROR_BASE_TERMINAL = (ERROR_BASE_LMAC + LMAC_ERROR_BASE_LAST),
    // Components.
    ERROR_BASE_LED = (ERROR_BASE_TERMINAL + TERMINAL_ERROR_BASE_LAST),
    ERROR_BASE_R4S8CR = (ERROR_BASE_LED + LED_ERROR_BASE_LAST),
    // Middleware.
    ERROR_BASE_ANALOG = (ERROR_BASE_R4S8CR + R4S8CR_ERROR_BASE_LAST),
    ERROR_BASE_CLI = (ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_LAST),
    ERROR_BASE_POWER = (ERROR_BASE_CLI + CLI_ERROR_BASE_LAST),
    // Nodes.
    ERROR_BASE_NODE = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
    // Last index.
    ERROR_BASE_LAST = (ERROR_BASE_NODE + NODE_ERROR_BASE_LAST)
} ERROR_base_t;

#endif /* __ERROR_BASE_H__ */
