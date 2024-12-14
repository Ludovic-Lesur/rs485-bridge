/*
 * dip_switch.h
 *
 *  Created on: 30 oct. 2022
 *      Author: Ludo
 */

#ifndef __DIP_SWITCH_H__
#define __DIP_SWITCH_H__

/*** DIP SWITCH structures ***/

/*!******************************************************************
 * \enum DIP_SWITCH_tx_mode_t
 * \brief Board TX configuration.
 *******************************************************************/
typedef enum {
    DIP_SWITCH_TX_MODE_DISABLED = 0,
    DIP_SWITCH_TX_MODE_ENABLED,
    DIP_SWITCH_TX_LAST
} DIP_SWITCH_tx_mode_t;

/*** DIP SWITCH functions ***/

/*!******************************************************************
 * \fn DIP_SWITCH_tx_mode_t DIP_SWITCH_get_tx_mode(void)
 * \brief Get TX mode DIP switch configuration.
 * \param[in]   none
 * \param[out]  none
 * \retval      Current TX mode.
 *******************************************************************/
DIP_SWITCH_tx_mode_t DIP_SWITCH_get_tx_mode(void);

#endif /* __DIP_SWITCH_H__ */
