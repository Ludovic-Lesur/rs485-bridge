/*
 * config.h
 *
 *  Created on: 30 oct. 2022
 *      Author: Ludo
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*** CONFIG structures ***/

typedef enum {
	CONFIG_TX_ENABLED,
	CONFIG_TX_DISABLED,
	CONFIG_TX_LAST
} CONFIG_tx_mode_t;

typedef enum {
	CONFIG_RS485_ADDRESS_ENABLED,
	CONFIG_RS485_ADDRESS_DISABLED,
	CONFIG_RS485_ADDRESS_LAST
} CONFIG_rs485_address_mode_t;

/*** CONFIG functions ***/

void CONFIG_init(void);
CONFIG_tx_mode_t CONFIG_get_tx_mode(void);
CONFIG_rs485_address_mode_t CONFIG_get_rs485_address_mode(void);

#endif /* __CONFIG_H__ */
