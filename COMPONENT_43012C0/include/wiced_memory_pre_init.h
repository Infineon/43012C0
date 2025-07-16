/*
 * Copyright 2020-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
 * support memory allocation tuning
 *
 */
#ifndef __WICED_MEMORY_PRE_INIT_H__
#define __WICED_MEMORY_PRE_INIT_H__

/* Usage:
 * The following describes how to use the memory allocation tuning API:
 *
 * Step 1:
 *      In application add this header file:
 *      #include "wiced_memory_pre_init.h"
 *
 * Step 2:
 *    In application add the following data structure. This will replace the
 *    same named structure with default settings in spar_setup.c:
 *    Example:
 *    This structure defines values for ACL data buffers passed between device and host.
 *    WICED_CONFIG_ACL_POOLS_t ACL_pool_config =
 *    {
 *        .host_claim_host_to_device_count = WICED_MEM_PRE_INIT_IGNORE,
 *        .host_to_device_count = 8,
 *        .device_to_host_count = 12
 *    };
 *
 *    This structure defines values for LE data buffers passed between device and host.
 *    WICED_CONFIG_ACL_POOLS_t LE_pool_config =
 *    {
 *        .host_claim_host_to_device_count = 8,
 *        .host_to_device_count = 8,
 *        .device_to_host_count = 8
 *    };
 *
 *    This structure sets the size, number, and low threshold of general memory pools.
 *    WICED_CONFIG_DYNAMIC_MEMORY_t gen_pool_config =
 *    {
 *        .num_pools = 5,
 *        .pools[0] = {16, 32, 3},
 *        .pools[1] = {32, 36, 2},
 *        .pools[2] = {96, 8, 1},
 *        .pools[3] = {268, 8, 1},
 *        .pools[4] = {572, 2, 0}
 *    };
 *
 *    The tuning structure had individual parameters and pointers to the structures
 *    perviously listed. Use WICED_MEM_PRE_INIT_IGNORE to leave settings at default value.
 *    Use a NULL pointer to leave pool configurations at default value.
 *    WICED_MEM_PRE_INIT_CONTROL g_mem_pre_init =
 *    {
 *        .max_ble_connections = 2,
 *        .max_peripheral_piconet = 2,
 *        .max_resolving_list = 8,
 *        .onfound_list_len = 0,
 *        .max_multi_adv_instances = WICED_MEM_PRE_INIT_IGNORE,
 *        .adv_filter_size = 0,
 *        .max_bt_connections = 5,
 *        .disable_coex_fix = 1,
 *        .p_ACL_pool_config = &ACL_pool_config,
 *        .p_LE_pool_config = &LE_pool_config,
 *        .p_gen_pool_config = &gen_pool_config
 *    };
 *
 * Step 3:
 *      See the "AIROC CYW20xxx and CYW43012 Application RAM" memory tuning app note at
 *      https://infineon.github.io/btsdk-docs/BT-SDK/AIROC-CYW20XXX-and-CYW43012-Application-RAM.pdf for
 *      more details on the configuration settings.
 */

/**
 * Deprecated in favor of wiced_memory_pre_init_ex.
 * Description:
 *      set pre-init memory allocation parameters.
 *      call this from spar_crt_init function to set parameters prior to allocations
 *      Note: Application don't need to call this function.
 *            Application only need to provide the global variables as following and tuning the value.
 *
 *            uint8_t g_wiced_memory_pre_init_enable = 1;
 *            uint8_t g_wiced_memory_pre_init_max_ble_connections = 7;
 *            uint8_t g_wiced_memory_pre_init_num_ble_rl = 60;
 *
 * Input:
 *      g_wiced_memory_pre_init_enable:
 *          Enable or disable this memory tuning.
 *
 *      g_wiced_memory_pre_init_max_ble_connections:
 *          This is for BLE connections which device could support
 *          Default value is 15
 *          Max value is 15, user is able to tuning this value form 1 to 15.
 *
 *      g_wiced_memory_pre_init_num_ble_rl
 *          The Resolving List is used by the Link Layer to resolve
 *          Resolvable Private Addresses used by advertisers, scanners or initiators.
 *          This allows the Host to configure the Link Layer to act on a request
 *          without awakening the Host.
 *          Default value is 128
 *          Max value is 128, user is able to tuning this value from 1 to 128.
 *
 * Output:
 *      None
 * Return:
 *      None
 */
void wiced_memory_pre_init(uint32_t enable, uint32_t max_ble_connections, uint32_t num_ble_whitlelist);

/**
 * @brief Memory tuning configuration for ACL data buffer transfer between host and device.
 *
 */
typedef struct
{
    //! The total number of HCI ACL Data Packets that can be stored in the data buffers of the Controller.
    UINT8 host_claim_host_to_device_count;

    //! The number of buffers for host to device ACL traffic.
    UINT8 host_to_device_count;

    //! The number of buffers for device to host ACL traffic.
    UINT8 device_to_host_count;
} WICED_CONFIG_ACL_POOLS_t;

/**
 * @brief Memory tuning configuration for LE data buffer transfer between host and device.
 *
 */
typedef struct
{
    //! The size of each block in this pool.
    UINT16 size;

    //! The number of blocks in this pool.
    UINT8 count;

    //! The number of blocks in this pool that are reserved for dynamic_memory_AllocateOrDie calls.
    //! This number of reserved blocks cannot be consumed by calls to
    //! dynamic_memory_AllocateOrReturnNULL, which will return NULL if the block count is below the
    //! die_reserve threshold.
    UINT8 die_reserve;
} WICED_CONFIG_DYNAMIC_MEMORY_POOL_t;

/**
 * @brief Memory tuning configuration for dynamic memory general use pools.
 *
 */
typedef struct
{
    //! The number of pools that are to be created from the general pools.  The default value is
    //! DYNAMIC_MEMORY_NUM_POOLS, but we reserve an extra pool control block, in case we need to add
    //! a block size category from configuration data.  Unless we need to add a new block size
    //! category pool, config data (.ags, .cgx) should probably just use DYNAMIC_MEMORY_NUM_POOLS
    //! as a named value for this field.
    UINT8 num_pools;

    //$ DEFINE num_pools: DYNAMIC_MEMORY_NUM_POOLS

    //! Info on the size, count, and blocks reserved for dynamic_memory_AllocateOrDie in each pool.
    WICED_CONFIG_DYNAMIC_MEMORY_POOL_t pools[DYNAMIC_MEMORY_NUM_POOLS+1];
} WICED_CONFIG_DYNAMIC_MEMORY_t;

/**
 * use this define value to indicate no change to parameter
 */
#define WICED_MEM_PRE_INIT_IGNORE (0xff)

/**
 * structure containing paramters to pass for memory pre-init
 */
typedef struct tag_mem_pre_init_control
{
    uint8_t max_ble_connections;
    uint8_t max_peripheral_piconet; /* use to reduce bt connections */
    uint8_t max_resolving_list;
    uint8_t onfound_list_len;
    uint8_t max_multi_adv_instances;
    uint8_t adv_filter_size;
    uint8_t max_bt_connections;
    uint8_t disable_coex_fix;
    WICED_CONFIG_ACL_POOLS_t *p_ACL_pool_config;
    WICED_CONFIG_ACL_POOLS_t *p_LE_pool_config;
    WICED_CONFIG_DYNAMIC_MEMORY_t *p_gen_pool_config;
} WICED_MEM_PRE_INIT_CONTROL;

/**
 * set pre-init memory allocation parameters
 * call this from spar_crt_init function to set parameters prior to allocations
 */
void wiced_memory_pre_init_ex(WICED_MEM_PRE_INIT_CONTROL *mem_pre_init);

#endif
