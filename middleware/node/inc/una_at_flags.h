/*
 * una_at_flags.h
 *
 *  Created on: 07 dec. 2024
 *      Author: Ludo
 */

#include "lptim.h"
#include "terminal_instance.h"

#ifndef __UNA_AT_FLAGS_H__
#define __UNA_AT_FLAGS_H__

/*** UNA AT compilation flags ***/

#define UNA_AT_DELAY_ERROR_BASE_LAST            LPTIM_ERROR_BASE_LAST

#define UNA_AT_TERMINAL_INSTANCE                TERMINAL_INSTANCE_LMAC

#define UNA_AT_MODE_MASTER
//#define UNA_AT_MODE_SLAVE

#ifdef UNA_AT_MODE_MASTER

#define UNA_AT_NODE_ACCESS_RETRY_MAX            1
#define UNA_AT_SCAN_REGISTER_ADDRESS            0x00
#define UNA_AT_SCAN_REGISTER_MASK_NODE_ADDRESS  0x0000007F
#define UNA_AT_SCAN_REGISTER_MASK_BOARD_ID      0x0000FF00
#define UNA_AT_SCAN_REGISTER_TIMEOUT_MS         200

#endif /* UNA_AT_MODE_MASTER */

#ifdef UNA_AT_MODE_SLAVE

//#define UNA_AT_CUSTOM_COMMANDS

#endif /* UNA_AT_MODE_SLAVE */

#endif /* __UNA_AT_FLAGS_H__ */
