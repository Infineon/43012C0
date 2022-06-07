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
*Defines the interfaces for Buffer , Timer and Event Management Services
*/
#ifndef _WICED_GKI_H_
#define _WICED_GKI_H_

#include "wiced.h"

#if 0
/*
 * Function prototype for the timer call backs.
 */
typedef void ( *wiced_timer_callback_t )( uint32_t cb_params );

/** Forward typedefinition of the wiced transport buffer pool */
typedef struct _wiced_trans_buffer_pool_t wiced_trans_buffer_pool_t;

typedef struct wiced_pool_t wiced_bt_buffer_pool_t;
#endif

/*****************************************************************************
**                                                 Function Declarations
*****************************************************************************/
/**
 * Function         wiced_bt_did_stack_overflow
 *
 * Checks if the application thread stack overflowed at some point
 *
 * @return    TRUE : on stack overflow;
 *            FALSE : if no stack overflow
 *
 */
uint8_t wiced_bt_did_stack_overflow(void);

/**
 * Function         wiced_bt_stack_check_init
 *
 * Prepares the stack to allow the app to check for stack overflow.
 *
 */
void wiced_bt_stack_check_init(void);
#if 0
/**
 * Function         wiced_trans_create_buffer_pool
 *
 * Creates a buffer pool
 *
 * * @param[in]     buffer_size         : Size of each buffer in pool
 * * @param[in]     buffer_count        : Number of buffers in the pool
 *
 * @return     :  pointer to the buffer pool on success, NULL on failure
 *
 */
wiced_trans_buffer_pool_t * wiced_trans_create_buffer_pool( int32_t buffer_size, int32_t buffer_count );

/**
 * Function         wiced_trans_destroy_buffer_pool
 *
 * destoryes specified buffer pool
 *
 * * @param[in]     p_pool               : pointer to the pool
 * * @param[in]     buffer_size         : Size of each buffer in pool
 * * @param[in]     buffer_count        : Number of buffers in the pool
 *
 * @return     :  None
 *
 */
void wiced_trans_destroy_buffer_pool( wiced_trans_buffer_pool_t *p_pool, int32_t buffer_size, int32_t buffer_count );

/**
 * Function         wiced_trans_buffer_alloc
 *
 * Returns a buffer from the pool
 *
 * * @param[in]    p_pool           : Pointer to buffer pool returned from wiced_trans_create_buffer_pool
 *
 * @return     : pointer to the buffer from the pool on success, NULL on failure
 */
void* wiced_trans_buffer_alloc( wiced_trans_buffer_pool_t * p_pool );

/**
 * Function         wiced_trans_get_buffer_size
 *
 * Returns the size of buffer of the pool
 *
 * * @param[in]    p_pool           : Pointer to buffer pool returned from wiced_trans_create_buffer_pool
 *
 * @return     : size of the buffers of the pool
 */
uint16_t wiced_trans_get_buffer_size ( wiced_trans_buffer_pool_t *p_pool );

/**
 * Function         wiced_trans_get_available_buffers
 *
 * To get the number of buffers available in the pool
 *
 * * @param[in]    p_pool           : Pointer to buffer pool created using wiced_trans_create_buffer_pool
 *
 * @return     : the number of buffers available in the pool
 */
uint32_t wiced_trans_get_available_buffers( wiced_trans_buffer_pool_t *p_pool );

/**
 * Function         wiced_trans_create_event_buffer_pool
 *
 * Creates a buffer pool for sending the event data over uart to the host
 *
 * * @param[in]     buffer_size           : Size of each buffer in pool
 * * @param[in]     buffer_count          : Number of buffers in the pool
 *
 * @return     :  pointer to the buffer pool on success, NULL on failure
 */
wiced_trans_buffer_pool_t * wiced_trans_create_event_buffer_pool( int32_t buffer_size, int32_t buffer_count );

/**
 * Function         wiced_trans_event_buffer_alloc
 *
 * Returns a buffer from the event buffer pool
 *
 * * @param[in]    p_pool           : Pointer to buffer pool returned from wiced_trans_create_event_buffer_pool
 *
 * @return     : pointer to the buffer from the pool on success, NULL on failure
 */
void* wiced_trans_event_buffer_alloc( wiced_trans_buffer_pool_t * p_pool );

/**
 * Function         wiced_trans_get_event_buffer_size
 *
 * Returns the size of buffer of the pool
 *
 * * @param[in]    p_pool           : Pointer to buffer pool returned from wiced_trans_create_event_buffer_pool
 *
 * @return     : size of the buffers of the pool
 */
uint16_t wiced_trans_get_event_buffer_size ( wiced_trans_buffer_pool_t *p_pool );

/** Send the event to the host over the hci uart. This function creates the custom hci header internally.
 * Note - This function should be used only for buffer allocated using wiced_trans_event_buffer_alloc
 *
 *@param[in]    code                 :Class code and command opcode
 *@param[in]    p_buf                :Pointer to the event payload
 *@param[in]    length               :Event payload length
 *
 * @return   wiced_result_t
 */
wiced_result_t wiced_trans_send_event_buffer_data ( uint16_t code, uint8_t* p_buf, uint16_t length );
#endif

/**
 * Function         wiced_bt_ble_get_available_tx_buffers
 *
 * Used to get the available number of ble tx buffers
 *
 * Return           the available number of ble tx buffers
 */
uint32_t wiced_bt_ble_get_available_tx_buffers( void );

void wdog_generate_hw_reset(void);//TODO: #define wdog_generate_hw_reset wiced_hal_wdog_reset_system
uint32_t wiced_get_free_memory(void);//TODO: #define wiced_get_free_memory wiced_memory_get_free_bytes

#endif //_WICED_GKI_H_
