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
 * Wiced Memory Management Interface
 */

#pragma once

#include "wiced_bt_types.h"
#include "wiced_gki.h"

/** wiced buffer pool */
typedef struct wiced_pool_t wiced_bt_buffer_pool_t;

#pragma pack(1)

/** wiced bt dynamic buffer statistics */
typedef PACKED struct
{
    uint8_t     pool_id;                    /**< pool id */
    uint16_t    pool_size;                  /**< pool buffer size */
    uint16_t    current_allocated_count;    /**< number of  buffers currently allocated */
    uint16_t    max_allocated_count;        /**< maximum number of buffers allocated at any time */
    uint16_t    total_count;                /**< total number of buffers */
}wiced_bt_buffer_statistics_t;

#pragma pack()

/******************************************************************************
* Function Name: wiced_memory_permanent_allocate
***************************************************************************//**
*
* Allocates memory for permanent usage.
*
* \param size
* The size of the memory to be allocated
*
* \return
* The pointer to the allocated memory on success
* NULL on failure
*
******************************************************************************/
void* wiced_memory_permanent_allocate( uint32_t size );

/**
 * Function         wiced_memory_get_free_bytes
 *
 *                  Returns the number of free bytes of RAM left
 *
 * @return          the number of free bytes of RAM left
 */
uint32_t wiced_memory_get_free_bytes( void );

/**
 * Function         wiced_memory_set_application_thread_stack_size
 *
 * Update the stack size of the application thread (MPAF)
 *
 * NOTE : This API will work only when invoked from SPAR_CRT_SETUP()
 *
 * @param[in]       new_stack_size - required size of the stack.
 *
 * @return          TRUE on success else FALSE
 */
wiced_bool_t wiced_memory_set_application_thread_stack_size(uint16_t new_stack_size);

/**
 * Function         wiced_bt_create_pool
 *
 *                  Creates a private pool dedicated for the application usage.
 *
 * @param[in]       buffer_size           :size of the buffers in the pool
 *
  * @param[in]       buffer_cnt           :number of buffers in the pool
 *
 * @return         pointer to the created pool on success
 *                     NULL on failure
 */

wiced_bt_buffer_pool_t* wiced_bt_create_pool( uint32_t buffer_size, uint32_t buffer_cnt );

/**
 * Function         wiced_bt_get_buffer_from_pool
 *
 *                  Allocates a buffer from the private pool. Pass the pool pointer to get the buffer from the desired pool
 *
 * @param[in]       p_pool           : pool pointer
 *
 * @return         the pointer to the buffer
 *                     NULL on failure
 */
void* wiced_bt_get_buffer_from_pool( wiced_bt_buffer_pool_t* p_pool );

/**
 * Function         wiced_bt_get_buffer_count
 *
 *                  To get the number of buffers available in the pool
 *
 * @param[in]       p_pool           : pool pointer
 *
 * @return         the number of buffers available in the pool
 */
uint32_t wiced_bt_get_buffer_count( wiced_bt_buffer_pool_t* p_pool );

/**
 * Function         wiced_bt_get_buffer
 *
 *                  Allocates a buffer from the public pools. Public pools shall be specified
 *                  using wiced_bt_cfg_buf_pool_t from the application.
 *
 * @param[in]       buffer_size           : size of the buffer
 *
 * @return         the pointer to the buffer
 *                     NULL on failure
 */
void* wiced_bt_get_buffer( uint16_t buffer_size );

/**
 * Function         wiced_bt_free_buffer
 *
 *                  Frees the buffer
 *
 * @param[in]       p_buf           : pointer to the start of the buffer to be freed
 *
 * @return         None
 */
void wiced_bt_free_buffer( void* p_buf );

/**
 * Function         wiced_bt_get_buffer_size
 *
 *                  Gets the buffer size
 *
 * @param[in]       p_buf           : pointer to the start of the buffer
 *
 * @return        the buffer size
 */
uint32_t wiced_bt_get_buffer_size( void* p_buf );

/**
 * Function         wiced_bt_get_buffer_usage
 *
 * Dumps dynamic buffer usage (see #wiced_bt_buffer_statistics_t),from the last start of the system.
 *
 * @param[in]       p_buffer_stat - pointer buffer to fill statistics.
 * @param[in]       size - size of the memory to get the statistics.
 *
 * @return          WICED_BT_SUCCESS on success else error
 */
wiced_result_t wiced_bt_get_buffer_usage ( wiced_bt_buffer_statistics_t *p_buffer_stat, uint16_t size );
