/*
 * at.h
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#ifndef __AT_H__
#define __AT_H__

#include "types.h"

/*** AT functions ***/

void AT_init(void);
void AT_task(void);
void AT_fill_rx_buffer(uint8_t rx_byte);

#endif /* __AT_H__ */
