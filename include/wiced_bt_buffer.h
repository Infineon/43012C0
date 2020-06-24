/*
 *  Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
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

#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup  wiced_bt_buffer_api_functions   Buffer Management
 * @ingroup     wicedbt
 *
 * The Buffer management library of the WICED SDK.
 *
 * This library is used to allow Applications & Libraries to allocate & free buffers (from a
 * commom pool)
 *
 * @{
*/

/**
 * @brief BLE BATTC Maximum BATTC Devices.
 */
#define WICED_BT_BUFFER_POOL_MAX                    3

/**
 * @brief Configuration of one buffer pool
 * The application pass a table of buffer pool configuration (up to WICED_BT_BUFFER_POOL_MAX)
 * at initialization time.
 * Later, application and libraries will allocate/free buffers from these pools.
 */
typedef struct
{
    uint32_t size;                              /**< Size of every buffer in a pool. */
    uint32_t number;                            /**< Number of buffer in a pool. */
} wiced_bt_buffer_pool_cfg_t;

/**
 *
 * Function         wiced_bt_buffer_init
 *
 * @param[in]       buffer_pool_cfg: Table of buffer pool configuration
 * @param[in]       nb_buffer_pool: Number of element in the Table of buffer pool configuration
 *
 *                  This function is called to initialize the buffer management library.
 *                  This function must be called, once, before any other BT Buffer functions.
 *                  The buffer pool table must be sorted by increasing size.
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_buffer_init(const wiced_bt_buffer_pool_cfg_t *p_buffer_pool_cfg,
        uint8_t nb_buffer_pool);

/**
 *
 * Function         wiced_bt_buffer_alloc
 *
 * @param[in]       size: Size of the buffer to allocate
 *
 *                  This function is called to allocate a buffer from the buffer pools.
 *
 * @return          pointer on the allocated buffer or NULL if out of memory
 *
 */
void *wiced_bt_buffer_alloc(uint32_t size);

/**
 *
 * Function         wiced_bt_buffer_free
 *
 * @param[in]       p_buffer: buffer to free
 *
 *                  This function is called to free a buffer (previously allocated with
 *                  wiced_bt_buffer_alloc)
 *
 * @return          None
 *
 */
void wiced_bt_buffer_free(void *p_buffer);
