/*
 * dim.h
 *
 *  Created on: 09 dec. 2022
 *      Author: Ludo
 */

#ifndef __DIM_H__
#define __DIM_H__

#include "dinfox.h"

typedef enum {
	DIM_REGISTER_VUSB_MV = DINFOX_REGISTER_LAST,
	DIM_REGISTER_VRS_MV,
	DIM_REGISTER_RS485_MODE,
	DIM_REGISTER_LAST,
} UHFM_register_address_t;

#endif /* __DIM_H__ */
