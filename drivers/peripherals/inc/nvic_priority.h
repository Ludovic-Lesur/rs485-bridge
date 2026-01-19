/*
 * nvic_priority.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __NVIC_PRIORITY_H__
#define __NVIC_PRIORITY_H__

/*!******************************************************************
 * \enum NVIC_priority_list_t
 * \brief NVIC interrupt priorities list.
 *******************************************************************/
typedef enum {
    // Common.
    NVIC_PRIORITY_CLOCK = 0,
    NVIC_PRIORITY_CLOCK_CALIBRATION = 1,
    NVIC_PRIORITY_DELAY = 2,
    NVIC_PRIORITY_RTC = 3,
#ifdef RS485_BRIDGE
    // RS485 interface.
    NVIC_PRIORITY_RS485 = 4,
    NVIC_PRIORITY_TIM_NODE_UNA_R4S8CR = 5,
    NVIC_PRIORITY_TIM_NODE_AUTO_BAUD_RATE_REQUEST = 6,
    // AT interface.
    NVIC_PRIORITY_CLI = 7
#else
    // RS485 interface.
    NVIC_PRIORITY_RS485 = 0,
    NVIC_PRIORITY_TIM_NODE_UNA_R4S8CR = 1,
    NVIC_PRIORITY_TIM_NODE_AUTO_BAUD_RATE_REQUEST = 2,
    // AT interface.
    NVIC_PRIORITY_CLI = 3
#endif
} NVIC_priority_list_t;

#endif /* __NVIC_PRIORITY_H__ */
