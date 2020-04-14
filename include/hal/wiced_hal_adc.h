/*
 * Copyright 2014, Cypress Semiconductor
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */

/** @file
*
* List of parameters and defined functions needed to access the
* Analog-to-Digital Converter (ADC) driver.
*
*/

#ifndef __WICED_ADC_H__
#define __WICED_ADC_H__


/**  \addtogroup AdcDriver
* \ingroup HardwareDrivers
*/
/*! @{ */
/**
* Defines a driver to facilitate interfacing with the Analog-to-Digital
* Converter (ADC) driver. Use this driver to measure a DC voltage via
* a GPIO pin. Voltage measurement values are reported in millivolts (mV).
*
* Note that the ADC channels do not correspond directly to the GPIO pin
* numbering convention (e.g. ADC channel 0 is tied to GPIO P18, etc). Please
* reference the User Documentation for more information regarding your
* specific platform.
*
*/

/******************************************************************************
*** Parameters.
***
*** The following parameters are used to configure the driver or define
*** return status. They are not modifiable.
******************************************************************************/

/// supported ADC input channel selection
typedef enum ADC_INPUT_CHANNEL_SEL {
    ADC_INPUT_P18           =   0x0,    //WICED_P18
    ADC_INPUT_P17           =   0x1,    //WICED_P17
    ADC_INPUT_P16           =   0x2,    //WICED_P16
    ADC_INPUT_P15           =   0x3,    //WICED_P15
    ADC_INPUT_P14           =   0x4,    //WICED_P14
    ADC_INPUT_P13           =   0x5,    //WICED_P13
    ADC_INPUT_P12           =   0x6,    //WICED_P12
    ADC_INPUT_P11           =   0x7,    //WICED_P11
    ADC_INPUT_P10           =   0x8,    //WICED_P10
    ADC_INPUT_P9            =   0x9,    //WICED_P9
    ADC_INPUT_P8            =   0xA,    //WICED_P8
    ADC_INPUT_P1            =   0xB,    //WICED_P1
    ADC_INPUT_P0            =   0xC,    //WICED_P0
    ADC_INPUT_VDD_CORE      =   0xD,
    ADC_INPUT_ADC_BGREF     =   0xE,
    ADC_INPUT_ADC_REFGND    =   0xF,
    ADC_INPUT_P19           =   0x1F,   //WICED_P19
    ADC_INPUT_CHANNEL_MASK  =   0x1f,
}ADC_INPUT_CHANNEL_SEL;

/// ADC power type
typedef enum ADC_POWER_TYPE
{
    ADC_POWER_DOWN          = 1,
    ADC_POWER_UP            = 0,
}ADC_POWER_TYPE;

/// input  voltage range selection
typedef enum ADC_INPUT_RANGE_SEL
{
    ADC_RANGE_0_3P6V = 0,
    ADC_RANGE_0_1P8V = 1,
}ADC_INPUT_RANGE_SEL;


/******************************************************************************
*** Function prototypes and defines.
******************************************************************************/

///////////////////////////////////////////////////////////////////////////////
/// Initialize the ADC hardware to its default state.
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void wiced_hal_adc_init(void);


///////////////////////////////////////////////////////////////////////////////
/// Power up or power down the ADC hardware.
///
/// \param pwr - ADC_POWER_UP(0) or ADC_POWER_DOWN(1).
///
/// \return The previous power state.
///////////////////////////////////////////////////////////////////////////////
ADC_POWER_TYPE wiced_hal_adc_set_power(ADC_POWER_TYPE pwr);


///////////////////////////////////////////////////////////////////////////////
/// Calibrate and set the input voltage range expected for the ADC hardware.
///
/// \param range_idx - 0 to 3.6v(0) or 0 to 1.8v(1).
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void wiced_hal_adc_set_input_range(ADC_INPUT_RANGE_SEL rangeIdx);


///////////////////////////////////////////////////////////////////////////////
/// Read the raw ADC register value for the given channel. The value returned
/// here is direct from the register.
///
/// \param channel - The input channel that corresponds to a GPIO pin.
///
/// \return Raw digital value read from the hardware register.
///////////////////////////////////////////////////////////////////////////////
int16_t wiced_hal_adc_read_raw_sample( ADC_INPUT_CHANNEL_SEL channel);


///////////////////////////////////////////////////////////////////////////////
/// Read the ADC voltage value for the given channel. The value returned here
/// is converted to a voltage value from the register.
///
/// \param channel - The input channel that corresponds to a GPIO pin.
///
/// \return Converted digital voltage value.
///////////////////////////////////////////////////////////////////////////////
uint32_t wiced_hal_adc_read_voltage(ADC_INPUT_CHANNEL_SEL channel);


/* @} */

#endif // __WICED_ADC_H__
