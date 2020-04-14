/*
 * Cypress Semiconductor Proprietary and Confidential. © 2016 Cypress Semiconductor.
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */

/** @file
 *
 * Defines the utilities for configuring the I2S
 */

#ifndef _WICED_HAL_I2S_H_
#define _WICED_HAL_I2S_H_

/**
 * Function         wiced_hal_i2s_select_pads
 *
 * Selects the I2S clock, word select, data out and data in, LHL pins for the I2S peripheral to use.
 *
 *
 * @param[in]    i2s_clk           : I2S Clock Pin
 * @param[in]    i2s_ws           : I2S Word Select Pin
 * @param[in]    i2s_do           : I2S Data Out Pin
 * @param[in]    i2s_di            : I2S Data In Pin
 *
 * @return     : None
 */
void wiced_hal_i2s_select_pads( uint8_t i2s_clk, uint8_t i2s_ws,
                            uint8_t i2s_do, uint8_t i2s_di );
#endif // _WICED_HAL_I2S_H_
