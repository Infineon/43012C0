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
 *WICED BT App HAL Common Utilities
 */
#ifndef _WICED_BT_APP_HAL_COMMON_H_
#define _WICED_BT_APP_HAL_COMMON_H_

#include "bt_types.h"
#include "wiced.h"

//#define WICED_BT_TRACE_ENABLE

/******************************************************************************
 *                                                          Constants
 ******************************************************************************/

/* Number of GPIOs that can be configured using the wiced bt app hal interface */
#define GPIO_NUM_MAX        16

/* GPIO flag bit (this is different than definition that used in gpio driver source code (GPIO_INPUT_ENABLE, etc) */
#define GPIO_INPUT          0x0000
#define GPIO_OUTPUT         0x0001

/******************************************************************************
 *                                                        Type Definitions
 ******************************************************************************/

/******************************************************************************
 *                                                          Function Prototypes
 ******************************************************************************/

/**  Wiced BT App Common HAL Initialization
 *
 * @return   None
 */
void wiced_bt_app_hal_init ( void );

/**  Blinks the LED
 *
 *@param[in]    on_ms             : LED on duration in milli seconds
 *
  *@param[in]    off_ms             : LED off duration in milliseconds
 *
  *@param[in]    num_of_blinks             : LED number of blinks
 *
 * @return   None
 */
void wiced_bt_app_hal_led_blink(uint16_t on_ms, uint16_t off_ms, uint8_t num_of_blinks );

/**  Turns the LED On
 *
 * @return   None
 */
void wiced_bt_app_hal_led_on( void );

/**  Turns the LED Off
 *
 * @return   None
 */
void wiced_bt_app_hal_led_off( void );

/**  Turns OFF Flash
 *
 * @return   WICED_TRUE if Success, else WICED_FALSE
 */
wiced_bool_t wiced_bt_app_hal_power_down_flash( void );

#endif //_WICED_BT_APP_HAL_COMMON_H_
