/*
 * at_usb.h
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#ifndef __AT_USB_H__
#define __AT_USB_H__

#include "types.h"

/*** AT functions ***/

void AT_USB_init(void);
void AT_USB_task(void);
void AT_USB_print(char_t* str);

void AT_USB_fill_none_protocol_buffer(uint8_t rx_byte);
void AT_USB_fill_rx_buffer(uint8_t rx_byte);

#endif /* __AT_USB_H__ */
