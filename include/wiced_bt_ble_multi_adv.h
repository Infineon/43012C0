/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/**************************************************************************//**
* \file <wiced_bt_ble_multi_adv.h>
*
* Definitions for multi advertisemnt APIs
*
*/
#ifndef MULTI_ADV_H
#define MULTI_ADV_H

/**
 * \addtogroup  wiced_bt_ble BLE Host Stack Management
 * \ingroup     wiced_bt_ble
 * \{
 *
 * BLE (Bluetooth Low Energy) Functions.
 *
 * \defgroup group_ble_macro Macro
 * \defgroup group_ble_data_structures Data Structures
 * \defgroup group_ble_enums Enumerated Types
 * \defgroup group_ble_functions Functions
 * \{
 *     \defgroup group_ble_functions_adv Advertising
 *     \defgroup group_ble_functions_scan Scanning
 *     \defgroup group_ble_functions_bg Background Connections
 *     \defgroup group_ble_functions_sec Security
 *     \defgroup group_ble_functions_multi Multi Advertising
 *     \defgroup group_ble_functions_ctrl LE Controller Settings
 * \}
 *
 */

/******************************************************************************
 * Macro definitions                                                          *
 ******************************************************************************/

/**
* \addtogroup group_ble_macros
* \{
*/
 /******************************************************************************/
/* Maximum number of advertisment instances*/
#define MULTI_ADV_MAX_NUM_INSTANCES                 16              /**< this does not include instance 0 (regular adv). */

 /* The power table for multi ADV Tx Power levels
    Min   : -21 dBm     #define BTM_BLE_ADV_TX_POWER_MIN        0
    Low   : -15 dBm     #define BTM_BLE_ADV_TX_POWER_LOW        1
    Mid   :  -7 dBm     #define BTM_BLE_ADV_TX_POWER_MID        2
    Upper :   1 dBm     #define BTM_BLE_ADV_TX_POWER_UPPER      3
    Max   :   9 dBm     #define BTM_BLE_ADV_TX_POWER_MAX        4
 */
#define MULTI_ADV_TX_POWER_MIN                      0
#define MULTI_ADV_TX_POWER_MAX                      4


/** \} group_ble_macros */


/**
* \addtogroup group_ble_enums
* \{
*/

/** Multi-advertisement start/stop */
enum wiced_bt_ble_multi_advert_start_e
{
    MULTI_ADVERT_STOP                               = 0x00,                 /**< Stop Multi-adverstisment */
    MULTI_ADVERT_START                              = 0x01                  /**< Start Multi-adverstisment */
};

/** Multi-advertisement type */
enum wiced_bt_ble_multi_advert_type_e
{
    MULTI_ADVERT_CONNECTABLE_UNDIRECT_EVENT         = 0x00,
    MULTI_ADVERT_CONNECTABLE_DIRECT_EVENT           = 0x01,
    MULTI_ADVERT_DISCOVERABLE_EVENT                 = 0x02,
    MULTI_ADVERT_NONCONNECTABLE_EVENT               = 0x03,
    MULTI_ADVERT_LOW_DUTY_CYCLE_DIRECT_EVENT        = 0x04
};
typedef uint8_t wiced_bt_ble_multi_advert_type_t;    /**< BLE advertisement type (see #wiced_bt_ble_multi_advert_type_e) */

/** Multi-advertisement Filtering policy */
enum wiced_bt_ble_multi_advert_filtering_policy_e
{
    MULTI_ADVERT_FILTER_POLICY_WHITE_LIST_NOT_USED                    = 0x00,   // white list not used
    MULTI_ADVERT_WHITE_LIST_POLICY_ADV_ALLOW_UNKNOWN_CONNECTION       = 0x01,   // white list for scan request
    MULTI_ADVERT_WHITE_LIST_POLICY_ADV_ALLOW_UNKNOWN_SCANNING         = 0x02,   // white list for connection request
    MULTI_ADVERT_FILTER_POLICY_WHITE_LIST_USED_FOR_ALL                = 0x03
};
typedef uint8_t wiced_bt_ble_multi_advert_filtering_policy_t;    /**< BLE advertisement filtering policy (see #wiced_bt_ble_multi_advert_filtering_policy_e) */

/** \} group_ble_enums */



/*******************************************************************************
* Function Name: wiced_start_multi_advertisements
****************************************************************************//**
*
* Enable or disable advertisements of a specific instance. Prior to calling this
* function, the instance should be instantiated and populated using the APIs
* below. At a minimum, \ref wiced_set_multi_advertisement_params and
* \ref wiced_set_multi_advertisement_data should be used prior.
*
* \param[in] advertising_enable          MULTI_ADVERT_START or MULTI_ADVERT_STOP
* \param[in] adv_instance                1 to MULTI_ADV_MAX_NUM_INSTANCES
*
* \return
*  - BTM_CMD_STARTED
*  - BTM_SUCCESS
*  - BTM_NO_RESOURCES
*
*******************************************************************************/
wiced_result_t wiced_start_multi_advertisements( uint8_t advertising_enable, uint8_t adv_instance );

 /*******************************************************************************
 * Function Name: wiced_set_multi_advertisement_params
 ****************************************************************************//**
 *
 * Sets the advertising parameters of a specific advertising instance. The
 * parameters are populated into a struct \ref wiced_bt_ble_multi_adv_params_t.
 * The parameters must be set according to the constraints placed by the BT
 * spec, otherwise the parameters will be rejected with error code
 * BTM_ILLEGAL_VALUE.
 *
 * \code
 * {
 *     wiced_bt_ble_multi_adv_params_t params =
 *     {
 *         .adv_int_min = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,
 *         .adv_int_max = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,
 *         .adv_type = MULTI_ADVERT_NONCONNECTABLE_EVENT,
 *         .channel_map = BTM_BLE_DEFAULT_ADVERT_CHNL_MAP,
 *         .adv_filter_policy = MULTI_ADVERT_FILTER_POLICY_WHITE_LIST_NOT_USED,
 *         .adv_tx_power = MULTI_ADV_TX_POWER_MAX,
 *         .peer_addr_type = BLE_ADDR_PUBLIC, //valid only for directed type
 *         .peer_bd_addr = NULL, //valid only for directed type
 *         .own_addr_type = BLE_ADDR_RANDOM
 *     };
 *
 *     uint8_t local_bda[BD_ADDR_LEN] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
 *     memcpy(params.own_bd_addr, local_bda, BD_ADDR_LEN);
 *
 *     wiced_set_multi_advertisement_params(adv_instance, &params);
 * }
 * \endcode
 *
 * \param[in] adv_instance                  1 to MULTI_ADV_MAX_NUM_INSTANCES
 * \param[in] p_param                       \ref wiced_bt_ble_multi_adv_params_t
 *
 * \return
 *  - BTM_ILLEGAL_VALUE element of p_param is out of bounds
 *  - BTM_CMD_STARTED
 *  - BTM_SUCCESS
 *  - BTM_NO_RESOURCES
 *
 *******************************************************************************/
wiced_result_t wiced_set_multi_advertisement_params( uint16_t advertising_interval_min, uint16_t advertising_interval_max,
                    wiced_bt_ble_multi_advert_type_t advertising_type, wiced_bt_ble_address_type_t own_address_type,
                    wiced_bt_device_address_t ownAddr, wiced_bt_ble_address_type_t peerAddrType, wiced_bt_device_address_t peerAddr,
                    wiced_bt_ble_advert_chnl_map_t advertising_channel_map, wiced_bt_ble_multi_advert_filtering_policy_t advertising_filter_policy,
                    uint8_t adv_instance, int8_t transmit_power );

/*******************************************************************************
* Function Name: wiced_set_multi_advertisement_data
****************************************************************************//**
*
* Refer to \ref wiced_bt_ble_set_raw_advertisement_data. These two functions
* are functionally equivalent other than the fact that this API can specify
* a multi ADV instance for which to set the advertisement data.
*
* \param[in] p_data                             raw adv data
* \param[in] data_len                           num bytes of p_data (max 31)
* \param[in] adv_instance                       1 to MULTI_ADV_MAX_NUM_INSTANCES
*
* \return
*  - BTM_ILLEGAL_VALUE data_len too long or p_data null
*  - BTM_CMD_STARTED success
*  - BTM_SUCCESS success
*  - BTM_NO_RESOURCES transport buffer allocation failed
*
*******************************************************************************/
wiced_result_t wiced_set_multi_advertisement_data( uint8_t * p_data, uint8_t data_len, uint8_t adv_instance );

wiced_result_t wiced_set_multi_advertisement_scan_response_data(uint8_t * p_data, uint8_t data_len, uint8_t adv_instance);

wiced_bool_t wiced_bt_notify_multi_advertisement_packet_transmissions(uint8_t adv_instance, void(*clientCallback)(uint32_t),
    uint32_t advanceNoticeInMicroSeconds);

#endif
