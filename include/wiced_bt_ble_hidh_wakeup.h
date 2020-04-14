/*
 *  Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 *  Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 *  This software, including source code, documentation and related
 *  materials ("Software"), is owned by Cypress Semiconductor Corporation
 *  or one of its subsidiaries ("Cypress") and is protected by and subject to
 *  worldwide patent protection (United States and foreign),
 *  United States copyright laws and international treaty provisions.
 *  Therefore, you may use this Software only as provided in the license
 *  agreement accompanying the software package from which you
 *  obtained this Software ("EULA").
 *  If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 *  non-transferable license to copy, modify, and compile the Software
 *  source code solely for use in connection with Cypress's
 *  integrated circuit products. Any reproduction, modification, translation,
 *  compilation, or representation of this Software except as specified
 *  above is prohibited without the express written permission of Cypress.
 * 
 *  Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 *  reserves the right to make changes to the Software without notice. Cypress
 *  does not assume any liability arising out of the application or use of the
 *  Software or any product or circuit described in the Software. Cypress does
 *  not authorize its products for use in any products where a malfunction or
 *  failure of the Cypress product may reasonably be expected to result in
 *  significant property damage, injury or death ("High Risk Product"). By
 *  including Cypress's product in a High Risk Product, the manufacturer
 *  of such system or application assumes all risk of such use and in doing
 *  so agrees to indemnify Cypress against all liability.
 */

#pragma once

#include "wiced_bt_ble_hidh.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup  wiced_bt_ble_hidh_wakeup_api_functions   Human Interface Device Host WakeUp
 * @ingroup     wiced_bt_ble_hidh_wakeup_api_functions
 *
 * The Human Interface Device Host WakeUp library of the WICED SDK provide a simple method
 * for an application to integrate HIDH WakeUp functionality.
 * This library is typically used to WakeUp the Host (MCU) upon reception of a specific HID Report
 *
 * @{
*/

/**
 * @brief BLE HIDH WakeUp Pattern Maximum Length.
 */
#define WICED_BT_BLE_HIDH_WAKEUP_PATTERN_LEN_MAX    10

/**
 * @brief BLE HIDH WakeUp Pattern Maximum Number.
 */
#define WICED_BT_BLE_HIDH_WAKEUP_PATTERN_NB_MAX     1

/**
 * @brief BLE HID Device WakeUp Commands
 *
 */
/*  */
typedef enum
{
    WICED_BT_BLE_HIDH_WAKEUP_PATTERN_CMD_ADD = 1,
    WICED_BT_BLE_HIDH_WAKEUP_PATTERN_CMD_DEL,
    WICED_BT_BLE_HIDH_WAKEUP_PATTERN_CMD_LIST,
} wiced_bt_ble_hidh_wakeup_pattern_cmd_t;



/** @} wiced_bt_ble_hidh_wakeup_api_functions */

#ifdef __cplusplus
}
#endif
