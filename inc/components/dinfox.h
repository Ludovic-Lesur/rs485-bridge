/*
 * dinfox.h
 *
 *  Created on: 12 nov 2022
 *      Author: Ludo
 */

#ifndef __DINFOX_H__
#define __DINFOX_H__

#include "types.h"

/*** DINFOX boards identifier ***/

typedef enum {
	DINFOX_BOARD_ID_LVRM = 0,
	DINFOX_BOARD_ID_BPSM,
	DINFOX_BOARD_ID_DDRM,
	DINFOX_BOARD_ID_UHFM,
	DINFOX_BOARD_ID_GPSM,
	DINFOX_BOARD_ID_SM,
	DINFOX_BOARD_ID_DIM,
	DINFOX_BOARD_ID_RRM,
	DINFOX_BOARD_ID_LAST
} DINFOX_board_id_t;

static const char_t DINFOX_BOARD_ID_NAME[][DINFOX_BOARD_ID_LAST] = {
	"LVRM",
	"BPSM",
	"DDRM",
	"UHFM",
	"GPSM",
	"SM",
	"DIM",
	"RRM"
};

#endif /* __DINFOX_H__ */
