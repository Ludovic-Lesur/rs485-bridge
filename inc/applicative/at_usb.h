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

/*!******************************************************************
 * \fn void AT_USB_init(void
 * \brief Init AT USB interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_USB_init(void);

/*!******************************************************************
 * \fn void AT_USB_task(void)
 * \brief Main task of AT USB interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_USB_task(void);

#endif /* __AT_USB_H__ */
