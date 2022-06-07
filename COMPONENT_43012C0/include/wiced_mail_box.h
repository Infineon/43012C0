/*
* Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
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

/** @file
 *
 * @defgroup wiced_mail_box AIROC MAIL Box feature
 * @ingroup wicedsys
 * @{
 */

#pragma once

/**
 * Mailbox feature enables apps to use GCI interface for communication between
 * Bluetooth and WiFi cores. Mail-box can be used to send/receive 32bits of data at a time.
 */

/** Mailbox Rx Callback prototype */
typedef void (*wiced_mail_box_rx_cback_t)( uint32_t data );

/** Mailbox Tx Complete Callback prototype */
typedef void (*wiced_mail_box_tx_complete_t)( uint32_t data );

/**
 *  \brief Function to receive callback upon receiving data from WLAN core
 *
 *  \param [in] mbox_rx_cb call back function to be invoked upon receiving data from WLAN core.
 *  \return none
 */
void wiced_mail_box_register_data_rx_cback( wiced_mail_box_rx_cback_t mbox_rx_cb );

/**
 *  \brief Function to receive callback upon data transmission to WLAN core
 *
 *  \param [in] wiced_mail_box_tx_complete_t call back function to be invoked upon receiving ACK from WLAN core.
 *  \return none
 */
void wiced_mail_box_register_data_tx_complete_cback( wiced_mail_box_tx_complete_t mbox_tx_complete_cb );

/**
 *  \brief Function to send data to WLAN core
 *
 *  \param [in] data 4 byte data to be sent to WLAN core
 *  \return TRUE if successful, FALSE otherwise
 */
wiced_bool_t wiced_mail_box_write_data(uint32_t data);
/** @} */
