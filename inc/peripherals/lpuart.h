/*
 * lpuart.h
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#ifndef __LPUART_H__
#define __LPUART_H__

#include "lptim.h"

/*** LPUART structures ***/

typedef enum {
	LPUART_SUCCESS = 0,
	LPUART_ERROR_TX_TIMEOUT,
	LPUART_ERROR_BASE_LPTIM = 0x0100,
	LPUART_ERROR_BASE_LAST = (LPUART_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} LPUART_status_t;

/*** LPUART functions ***/

void LPUART1_init(void);
void LPUART1_enable_rx(void);
void LPUART1_disable_rx(void);
void LPUART1_send_string(char* tx_string);

#endif /* __LPUART_H__ */
