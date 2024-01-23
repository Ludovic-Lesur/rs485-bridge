/*
 * tim.h
 *
 *  Created on: 23 jan. 2024
 *      Author: Ludo
 */

#ifndef __TIM_H__
#define __TIM_H__

#include "rcc.h"
#include "types.h"

/*** TIM structures ***/

/*!******************************************************************
 * \enum TIM_status_t
 * \brief TIM driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	TIM_SUCCESS = 0,
	TIM_ERROR_NULL_PARAMETER,
	TIM_ERROR_CAPTURE_TIMEOUT,
	// Low level drivers errors.
	TIM_ERROR_BASE_RCC = 0x0100,
	// Last base value.
	TIM_ERROR_BASE_LAST = (TIM_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST)
} TIM_status_t;

/*** TIM functions ***/

/*!******************************************************************
 * \fn void TIM21_init(void)
 * \brief Init TIM21 peripheral for internal oscillators frequency measurement.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM21_init(void);

/*!******************************************************************
 * \fn void TIM21_de_init(void)
 * \brief Release TIM21 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM21_de_init(void);

/*!******************************************************************
 * \fn TIM_status_t TIM21_mco_capture(uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count)
 * \brief Perform MCO clock capture.
 * \param[in]  	none
 * \param[out] 	ref_clock_pulse_count: Pointer to the number of pulses of the timer reference clock during the capture.
 * \param[out]	mco_pulse_count: Pointer to the number of pulses of the MCO clock during the capture.
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM21_mco_capture(uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count);

/*******************************************************************/
#define TIM21_exit_error(error_base) { if (tim21_status != TIM_SUCCESS) { status = (error_base + tim21_status); goto errors; } }

/*******************************************************************/
#define TIM21_stack_error(void) { if (tim21_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM21 + tim21_status); } }

/*******************************************************************/
#define TIM21_stack_exit_error(error_code) { if (tim21_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM21 + tim21_status); status = error_code; goto errors; } }

#endif /* __TIM_H__ */
