/*
 * analog.c
 *
 *  Created on: 12 aug. 2024
 *      Author: Ludo
 */

#include "analog.h"

#include "adc.h"
#include "error.h"
#include "error_base.h"
#include "mcu_mapping.h"
#include "types.h"

/*** ANALOG local macros ***/

#define ANALOG_VMCU_MV_DEFAULT          3000
#define ANALOG_TMCU_DEGREES_DEFAULT     25

#define ANALOG_VRS_DIVIDER_RATIO        10
#define ANALOG_VUSB_DIVIDER_RATIO       2

#define ANALOG_ERROR_VALUE              0xFFFF

/*** ANALOG local structures ***/

/*******************************************************************/
typedef struct {
    int32_t vmcu_mv;
} ANALOG_context_t;

/*** ANALOG local global variables ***/

static ANALOG_context_t analog_ctx = {
    .vmcu_mv = ANALOG_VMCU_MV_DEFAULT,
};

/*** ANALOG functions ***/

/*******************************************************************/
ANALOG_status_t ANALOG_init(void) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
#ifdef RS485_BRIDGE
    ADC_SGL_configuration_t adc_config;
#endif
    // Init context.
    analog_ctx.vmcu_mv = ANALOG_VMCU_MV_DEFAULT;
    // Init internal ADC.
#ifdef DIM
    adc_status = ADC_init(&ADC_GPIO);
    ADC_exit_error(ANALOG_ERROR_BASE_ADC);
#endif
#ifdef RS485_BRIDGE
    adc_config.clock = ADC_CLOCK_PLL;
    adc_config.clock_prescaler = ADC_CLOCK_PRESCALER_NONE;
    adc_status = ADC_SGL_init(ADC_INSTANCE_VMCU_TMCU, NULL, &adc_config);
    ADC_exit_error(ANALOG_ERROR_BASE_ADC);
    adc_status = ADC_SGL_init(ADC_INSTANCE_GPIO, &ADC_GPIO, &adc_config);
    ADC_exit_error(ANALOG_ERROR_BASE_ADC);
#endif
errors:
    return status;
}

/*******************************************************************/
ANALOG_status_t ANALOG_de_init(void) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    // Release internal ADC.
#ifdef DIM
    adc_status = ADC_de_init();
    ADC_stack_error(ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_ADC);
#endif
#ifdef RS485_BRIDGE
    adc_status = ADC_SGL_de_init(ADC_INSTANCE_VMCU_TMCU);
    ADC_stack_error(ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_ADC);
    adc_status = ADC_SGL_de_init(ADC_INSTANCE_GPIO);
    ADC_stack_error(ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_ADC);
#endif
    return status;
}

/*******************************************************************/
ANALOG_status_t ANALOG_convert_channel(ANALOG_channel_t channel, int32_t* analog_data) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    int32_t adc_data_12bits = 0;
    // Check parameter.
    if (analog_data == NULL) {
        status = ANALOG_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check channel.
    switch (channel) {
    case ANALOG_CHANNEL_VMCU_MV:
        // MCU voltage.
#ifdef DIM
        adc_status = ADC_convert_channel(ADC_CHANNEL_VREFINT, &adc_data_12bits);
#endif
#ifdef RS485_BRIDGE
        adc_status = ADC_SGL_convert_channel(ADC_INSTANCE_VMCU_TMCU, ADC_CHANNEL_VREFINT, &adc_data_12bits);
#endif
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to mV.
        adc_status = ADC_compute_vmcu(adc_data_12bits, ADC_get_vrefint_voltage_mv(), analog_data);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Update local value for temperature computation.
        analog_ctx.vmcu_mv = (*analog_data);
        break;
    case ANALOG_CHANNEL_TMCU_DEGREES:
        // MCU temperature.
#ifdef DIM
        adc_status = ADC_convert_channel(ADC_CHANNEL_VREFINT, &adc_data_12bits);
#endif
#ifdef RS485_BRIDGE
        adc_status = ADC_SGL_convert_channel(ADC_INSTANCE_VMCU_TMCU, ADC_CHANNEL_TEMPERATURE_SENSOR, &adc_data_12bits);
#endif
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to degrees.
        adc_status = ADC_compute_tmcu(analog_ctx.vmcu_mv, adc_data_12bits, analog_data);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        break;
    case ANALOG_CHANNEL_VRS_MV:
        // RS485 bus voltage.
#ifdef DIM
        adc_status = ADC_convert_channel(ADC_CHANNEL_VRS, &adc_data_12bits);
#endif
#ifdef RS485_BRIDGE
        adc_status = ADC_SGL_convert_channel(ADC_INSTANCE_GPIO, ADC_CHANNEL_VRS, &adc_data_12bits);
#endif
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to mV.
        (*analog_data) = (adc_data_12bits * analog_ctx.vmcu_mv * ANALOG_VRS_DIVIDER_RATIO) / (ADC_FULL_SCALE);
        break;
    case ANALOG_CHANNEL_VUSB_MV:
        // USB voltage.
#ifdef DIM
        adc_status = ADC_convert_channel(ADC_CHANNEL_VUSB, &adc_data_12bits);
#endif
#ifdef RS485_BRIDGE
        adc_status = ADC_SGL_convert_channel(ADC_INSTANCE_GPIO, ADC_CHANNEL_VUSB, &adc_data_12bits);
#endif
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to mV.
        (*analog_data) = (adc_data_12bits * analog_ctx.vmcu_mv * ANALOG_VUSB_DIVIDER_RATIO) / (ADC_FULL_SCALE);
        break;
    default:
        status = ANALOG_ERROR_CHANNEL;
        goto errors;
    }
errors:
    return status;
}
