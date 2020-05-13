/*
 * Copyright 2015, Cypress Semiconductor
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */

/** @file
 *
 * Bluetooth Synchronous Connection Oriented Channel Application Programming Interface
 *
 */
#pragma once

#include "wiced.h"
/******************************************************
 *              Constants
 ******************************************************/
#define WICED_BT_SCO_CONNECTION_ACCEPT              0x00
#define WICED_BT_SCO_CONNECTION_REJECT_RESOURCES    0x0D
#define WICED_BT_SCO_CONNECTION_REJECT_SECURITY     0x0E
#define WICED_BT_SCO_CONNECTION_REJECT_DEVICE       0x0F

#define WICED_INVALID_SCO_INDEX           0xFFFF
#define WICED_SCO_LINK_ALL_PKT_MASK       0x003F
#define WICED_SCO_LINK_ONLY_MASK          0x0007
#define WICED_SCO_PKT_TYPES_MASK_HV3      0x0004
#define WICED_SCO_PKT_TYPES_MASK_EV3      0x0008
#define WICED_SCO_PKT_TYPES_MASK_EV4      0x0010
#define WICED_SCO_PKT_TYPES_MASK_EV5      0x0020
#define WICED_SCO_PKT_TYPES_MASK_NO_2_EV3 0x0040
#define WICED_SCO_PKT_TYPES_MASK_NO_3_EV3 0x0080
#define WICED_SCO_PKT_TYPES_MASK_NO_2_EV5 0x0100
#define WICED_SCO_PKT_TYPES_MASK_NO_3_EV5 0x0200

/** SCO route path */
typedef enum
{
    WICED_BT_SCO_OVER_PCM = 0,   /**< [DEFAULT] PCM data config for routing over I2S/PCM interface */
}wiced_bt_sco_route_path_t;

/******************************************************
 *              Type Definitions
 ******************************************************/
#define  WICED_BT_SCO_DATA_CB_GET_LENGTH(ltch_len)   ((ltch_len>>8)&0xff)	/**< SCO data callback length */

/** Subset for the enhanced setup/accept synchronous connection paramters. See BT 4.1 or later HCI spec for details */
typedef struct
{
    uint16_t max_latency;                   /**< Maximum latency (0x4-0xFFFE in msecs) */
    uint16_t packet_types;                  /**< Packet Types */
    uint8_t retrans_effort;                 /**< 0x00-0x02, 0xFF don't care */
    wiced_bool_t use_wbs;                   /**< True to use wide band, False to use narrow band */

} wiced_bt_sco_params_t;

/** SCO path config */
typedef struct
{
    wiced_bt_sco_route_path_t    path;           /**< sco routing path  0:pcm/i2s; 1: app*/
}wiced_bt_voice_path_setup_t;

/**
 *  @addtogroup  sco_api_functions       Synchronous Connection Oriented (SCO) Channel
 *  @ingroup     wicedbt
 *
 *  SCO Functions
 *
 *  @{
 */

/******************************************************
 *              Function Declarations
 ******************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Function         wiced_bt_sco_create_as_initiator
 *
 *                  Creates a synchronous connection oriented connection as initiator.
 *
 *  @param[in]  bd_addr                 : Peer bd_addr
 *  @param[in]  p_params                : Pointer to the SCO parameter structure
 *  @param[out] p_sco_index             : SCO index returned
 *
 *  @return     <b> WICED_BT_UNKNOWN_ADDR </b>      : Create connection failed, ACL connection is not up
 *              <b> WICED_BT_BUSY </b>              : Create connection failed, another SCO is being
 *                                                    conncted to the same BD address
 *              <b> WICED_BT_WRONG_MODE </b>        : Create connection failed, wrong mode
 *              <b> WICED_BT_NO_RESOURCES </b>      : Create connection failed, max SCO limit has been
 *                                                    reached
 *              <b> WICED_BT_PENDING </b>            : Create connection successfully, "p_sco_index" is returned
 */
wiced_bt_dev_status_t wiced_bt_sco_create_as_initiator (wiced_bt_device_address_t bd_addr,
                                                        uint16_t *p_sco_index,
                                                        wiced_bt_sco_params_t *p_params);

/**
 * Function         wiced_bt_sco_create_as_acceptor
 *
 *                  Creates a synchronous connection oriented connection as acceptor.
 *
 *  @param[out] p_sco_index             : SCO index returned
 *
 *  @return     <b> WICED_BT_UNKNOWN_ADDR </b>      : Create connection failed, ACL connection is not up or
 *                                                    address is invalid
 *              <b> WICED_BT_BUSY </b>              : Create connection failed, a SCO connection is already
 *                                                    conncted to the same BD address
 *              <b> WICED_BT_WRONG_MODE </b>        : Create connection failed, link in park mode or
 *                                                    automatic un-park is not supported
 *              <b> WICED_BT_NO_RESOURCES </b>      : Create connection failed, max SCO limit has been
 *                                                    reached
 *              <b> WICED_BT_PENDING </b>            : Create connection successfully, "p_sco_index" is returned
 */
wiced_bt_dev_status_t wiced_bt_sco_create_as_acceptor (uint16_t *p_sco_index);

/**
 * Function         wiced_bt_sco_create_as_acceptor
 *
 *                  Creates a synchronous connection oriented connection as acceptor with specifi
 *                  Audio Gateway.
 *  @param[in]  ag_addr                 : Target Audio Gateway's address
 *  @param[out] p_sco_index             : SCO index returned
 *
 *  @return     <b> WICED_BT_UNKNOWN_ADDR </b>      : Create connection failed, ACL connection is not up or
 *                                                    address is invalid
 *              <b> WICED_BT_BUSY </b>              : Create connection failed, a SCO connection is already
 *                                                    conncted to the same BD address
 *              <b> WICED_BT_WRONG_MODE </b>        : Create connection failed, link in park mode or
 *                                                    automatic un-park is not supported
 *              <b> WICED_BT_NO_RESOURCES </b>      : Create connection failed, max SCO limit has been
 *                                                    reached
 *              <b> BTM_CMD_STARTED </b>            : Create connection successfully, "p_sco_index" is returned
 */
wiced_bt_dev_status_t wiced_bt_sco_create_as_acceptor_with_specific_ag(wiced_bt_device_address_t ag_addr,
                                                                       uint16_t *p_sco_index);

/**
 * Function         wiced_bt_sco_remove
 *
 *                  Removes a specific synchronous connection oriented connection.
 *
 *  @param[in]  sco_index           : SCO index to remove
 *
 *  @return     <b> WICED_BT_UNKNOWN_ADDR </b>      : Remove connection failed, invalid SCO index
 *              <b> WICED_BT_NO_RESOURCES </b>      : Remove connection failed, no resource
 *              <b> WICED_BT_SUCCESS </b>           : Remove connection successfully, device is still
 *                                                    listening for incoming connection
 *              <b> WICED_BT_PENDING </b>            : Remove connection successfully
 */
wiced_bt_dev_status_t wiced_bt_sco_remove (uint16_t sco_index);

/**
 * Function         wiced_bt_sco_accept_connection
 *
 *                  Called to handle (e)SCO connection request event (wiced_bt_sco_connect_request_event).
 *
 *
 *  @param[in]  sco_index           : SCO index
 *
 *  @param[in]  HCI status code     : WICED_BT_SCO_CONNECTION_ACCEPT              0x00
 *                                    WICED_BT_SCO_CONNECTION_REJECT_RESOURCES    0x0D
 *                                    WICED_BT_SCO_CONNECTION_REJECT_SECURITY     0x0E
 *                                    WICED_BT_SCO_CONNECTION_REJECT_DEVICE       0x0F
 *  @param[in]  p_params            : Pointer to the SCO parameter structure
 *
 */
void wiced_bt_sco_accept_connection (uint16_t sco_index, uint8_t hci_status,
                                     wiced_bt_sco_params_t *p_params);


/**
 * Function         wiced_bt_setup_voice_path
 *
 *                  Configure the SCO routing path.
 *
 *  @param[in]  pData                 : To setup SCO routing path
 *
 * @return
 *              WICED_BT_SUCCESS    : Config success.
 *              WICED_BT_PENDING    : Command sent. Waiting for command complete event.
 *              WICED_BT_BUSY       : Command not sent. Waiting for command complete event for prior command.
 */
wiced_bt_dev_status_t wiced_bt_setup_voice_path(wiced_bt_voice_path_setup_t *pData);
#define wiced_bt_sco_setup_voice_path wiced_bt_setup_voice_path

/**
 * Function         wiced_bt_sco_turn_off_pcm_clock
 *
 *                  The wiced_voice_path.a was required to include before we link this function.
 *                  To turn off the PCM/I2S hardware clock
 *                  This function needs to be called after the application turns off (or mutes audio data in/out of) the codec
 *
 *  @return                     : None
 */

void  wiced_bt_sco_turn_off_pcm_clock( void );


#ifdef __cplusplus
}
#endif
/** @} wicedbt_sco */
