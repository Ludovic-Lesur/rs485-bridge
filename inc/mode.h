/*
 * mode.h
 *
 *  Created on: 25 oct. 2022
 *      Author: Ludo
 */

#ifndef __MODE_H__
#define __MODE_H__

//#define RSM		// RS485 mode with address check.
#define ATM	// AT command mode without address check.


/*** Debug mode ***/

#define DEBUG		// Keep programming pins and disable watchdog.

/*** Error management ***/

#if (defined RSM && defined ATM)
#error "Only 1 mode must be selected."
#endif

#endif /* __MODE_H__ */
