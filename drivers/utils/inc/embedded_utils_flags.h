/*
 * embedded_utils_flags.h
 *
 *  Created on: 15 aug. 2024
 *      Author: Ludo
 */

#ifndef __EMBEDDED_UTILS_FLAGS_H__
#define __EMBEDDED_UTILS_FLAGS_H__

#include "lmac.h"
#include "version.h"

/*** Embedded utility functions compilation flags ***/

#define EMBEDDED_UTILS_HW_INTERFACE_ERROR_BASE_LAST     LMAC_ERROR_BASE_LAST

#ifdef DIM
#define EMBEDDED_UTILS_AT_BAUD_RATE                     9600
#endif
#ifdef RS485_BRIDGE
#define EMBEDDED_UTILS_AT_BAUD_RATE                     115200
#endif
#define EMBEDDED_UTILS_AT_REPLY_END                     "\r\n"
#define EMBEDDED_UTILS_AT_FORCE_OK
#define EMBEDDED_UTILS_AT_INTERNAL_COMMANDS_ENABLE
#define EMBEDDED_UTILS_AT_COMMANDS_LIST_SIZE            16
#define EMBEDDED_UTILS_AT_BUFFER_SIZE                   128
#ifdef EMBEDDED_UTILS_AT_INTERNAL_COMMANDS_ENABLE
#ifdef DIM
#define EMBEDDED_UTILS_AT_BOARD_NAME                    "dim"
#endif
#ifdef RS485_BRIDGE
#define EMBEDDED_UTILS_AT_BOARD_NAME                    "rs485-bridge"
#endif
#ifdef HW1_0
#define EMBEDDED_UTILS_AT_HW_VERSION_MAJOR              1
#define EMBEDDED_UTILS_AT_HW_VERSION_MINOR              0
#endif
#ifdef HW1_1
#define EMBEDDED_UTILS_AT_HW_VERSION_MAJOR              1
#define EMBEDDED_UTILS_AT_HW_VERSION_MINOR              1
#endif
#define EMBEDDED_UTILS_AT_SW_VERSION_MAJOR              GIT_MAJOR_VERSION
#define EMBEDDED_UTILS_AT_SW_VERSION_MINOR              GIT_MINOR_VERSION
#define EMBEDDED_UTILS_AT_SW_VERSION_INDEX              GIT_COMMIT_INDEX
#define EMBEDDED_UTILS_AT_SW_VERSION_DIRTY_FLAG         GIT_DIRTY_FLAG
#define EMBEDDED_UTILS_AT_SW_VERSION_ID                 GIT_COMMIT_ID
#endif

#define EMBEDDED_UTILS_ERROR_STACK_DEPTH                32
#define EMBEDDED_UTILS_ERROR_STACK_SUCCESS_VALUE        0
//#define EMBEDDED_UTILS_ERROR_STACK_SIGFOX

#define EMBEDDED_UTILS_MATH_PRECISION                   0
//#define EMBEDDED_UTILS_MATH_COS_TABLE
//#define EMBEDDED_UTILS_MATH_SIN_TABLE
//#define EMBEDDED_UTILS_MATH_ATAN2

#define EMBEDDED_UTILS_STRING_HEXADECIMAL_UPPER_CASE

#define EMBEDDED_UTILS_TERMINAL_INSTANCES_NUMBER        2
#define EMBEDDED_UTILS_TERMINAL_BUFFER_SIZE             128
#define EMBEDDED_UTILS_TERMINAL_MODE_BUS

#endif /* __EMBEDDED_UTILS_FLAGS_H__ */
