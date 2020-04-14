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
* List of parameters and defined functions needed to utilize the
* watchdog.
*
*/

#ifndef __WICED_WATCHDOG_RESET_H__
#define __WICED_WATCHDOG_RESET_H__

/**  \addtogroup WatchdogInterface
*    \ingroup HardwareDrivers
*
* Defines a driver for the watchdog interface. This driver manages the
* hardware watchdog countdown timer. When enabled, the watchdog timer will generate
* an interrupt when the timer counts down to zero, then will reload the counter.
* If the counter counts down to zero again, the hardware performs a device reset.
* The lowest priority "idle" thread restarts the watchdog counter preiodically,
* and other operations that may keep the system busy call wiced_hal_wdog_restart to
* extend the watchdog countdown.
*/
/*! @{ */

/******************************************************************************
*** Function prototypes.
******************************************************************************/

///////////////////////////////////////////////////////////////////////////////
/// Disable the system watchdog. Useful for debugging when the watchdog would
/// interfere and reset the system when not desired (e.g. when the system
/// is connected to a debugger, etc).
///
/// NOTE: This is for debugging purposes only. Hence, there is no complementary
/// "Enable Watchdog" function. Please do *not* use this function in production
/// applications.
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
#define wiced_hal_watchdog_disable wiced_hal_wdog_disable
void wiced_hal_watchdog_disable(void);

///////////////////////////////////////////////////////////////////////////////
/// Execute a soft reset of the system.
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
#define wiced_hal_watchdog_reset_system wiced_hal_wdog_reset_system
void wiced_hal_watchdog_reset_system(void);

///////////////////////////////////////////////////////////////////////////////
/// Restart the watchdog (restart the watchdog's internal timer). Used to
/// manually "pet" the watchdog when certain processes might otherwise cause
/// the watchdog to trigger and reset the system.
/// The default timeout for watchdog reset is two seconds.
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
#define wiced_hal_watchdog_restart wiced_hal_wdog_restart
void wiced_hal_watchdog_restart(void);

///////////////////////////////////////////////////////////////////////////////
///// API to configure additional delay between consecutive dump events
/////
///// \param delayus delay in micro seconds
/////
///// \return none
/////////////////////////////////////////////////////////////////////////////////
#define wiced_hal_watchdog_coredump_set_uart_delay wiced_hal_wdog_coredump_set_uart_delay
void wiced_hal_watchdog_coredump_set_uart_delay(uint16_t delayus);

///////////////////////////////////////////////////////////////////////////////
///// API to check whether reset happened due to Watchdog timer expiry
/////
///// \return TRUE - if reset is due to Watchdog, FALSE otherwise
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_watchdog_get_reset_reason(void);

///////////////////////////////////////////////////////////////////////////////
///// API to configure gpio and its active level to be set on WDT expiry
/////
///// \param [in] gpio BT GPIO to be asserted before watchdog reset(only WICED_GPIO_00 - WICED_GPIO_15 are valid).
///// \param [in] active_level WICED_GPIO_ACTIVE_LOW/WICED_GPIO_ACTIVE_HIGH
/////
///// \return WICED_SUCCESS - if GPIO is configured sucessfully, else WICED_BADARG.
/////////////////////////////////////////////////////////////////////////////////
wiced_result_t wiced_watchdog_configure_reset_gpio(wiced_bt_gpio_numbers_t gpio, wiced_bool_t active_level);

///////////////////////////////////////////////////////////////////////////////
///// API to add application required delay on WDT expiry after asserting HOST_WAKE/configured GPIO
/////
///// \param [in] delay_ms delay in ms (default delay is 0ms).
/////
///// \return none
/////////////////////////////////////////////////////////////////////////////////
void wiced_watchdog_configure_reset_delay(uint32_t delay_ms);
#endif // __WICED_WATCHDOG_RESET_H__
