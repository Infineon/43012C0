/*
 * Copyright 2016-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Bluetooth A2DP Application Programming Interface
 *
 */
#pragma once

#include "wiced_bt_sdp.h"
/**
 *
 * @addtogroup    wicedbt_av_a2d_helper      A2DP Helper Functions
 * @ingroup       wicedbt_a2dp
 *
 * Advanced Audio Distribution Profile
 *
 * @{
 */
/*****************************************************************************
**  constants
*****************************************************************************/

/* Profile supported features */
#define A2D_SUPF_PLAYER     0x0001
#define A2D_SUPF_MIC        0x0002
#define A2D_SUPF_TUNER      0x0004
#define A2D_SUPF_MIXER      0x0008

#define A2D_SUPF_HEADPHONE  0x0001
#define A2D_SUPF_SPEAKER    0x0002
#define A2D_SUPF_RECORDER   0x0004
#define A2D_SUPF_AMP        0x0008

/* AV Media Types */
#define A2D_MEDIA_TYPE_AUDIO    0x00    /* audio media type + RFA */
#define A2D_MEDIA_TYPE_VIDEO    0x10    /* video media type + RFA */
#define A2D_MEDIA_TYPE_MULTI    0x20    /* multimedia media type + RFA */

/* AV Media Codec Type (Audio Codec ID) */
#define A2D_MEDIA_CT_SBC        0x00    /* SBC media codec type */
#define A2D_MEDIA_CT_M12        0x01    /* MPEG-1, 2 Audio media codec type */
#define A2D_MEDIA_CT_M24        0x02    /* MPEG-2, 4 AAC media codec type */
#define A2D_MEDIA_CT_ATRAC      0x04    /* ATRAC family media codec type */
#define A2D_MEDIA_CT_VEND       0xFF    /* Vendor specific */
#define A2D_MEDIA_CT_APTX       A2D_MEDIA_CT_VEND    /* APTX media codec type */


/**
 * @anchor A2D_STATUS
 * @name A2DP status codes.
 * @{
 */

#define A2D_SUCCESS           0     /**< Success */
#define A2D_FAIL              0x0A  /**< Failed */
#define A2D_BUSY              0x0B  /**< wiced_bt_a2d_find_service is already in progress */
#define A2D_INVALID_PARAMS    0x0C  /**< bad parameters */
#define A2D_WRONG_CODEC       0x0D  /**< wrong codec info */
#define A2D_BAD_CODEC_TYPE    0xC1  /**< Media Codec Type is not valid  */
#define A2D_NS_CODEC_TYPE     0xC2  /**< Media Codec Type is not supported */
#define A2D_BAD_SAMP_FREQ     0xC3  /**< Sampling Frequency is not valid or multiple values have been selected  */
#define A2D_NS_SAMP_FREQ      0xC4  /**< Sampling Frequency is not supported  */
#define A2D_BAD_CH_MODE       0xC5  /**< Channel Mode is not valid or multiple values have been selected  */
#define A2D_NS_CH_MODE        0xC6  /**< Channel Mode is not supported */
#define A2D_BAD_SUBBANDS      0xC7  /**< None or multiple values have been selected for Number of Subbands */
#define A2D_NS_SUBBANDS       0xC8  /**< Number of Subbands is not supported */
#define A2D_BAD_ALLOC_MTHD    0xC9  /**< None or multiple values have been selected for Allocation Method */
#define A2D_NS_ALLOC_MTHD     0xCA  /**< Allocation Method is not supported */
#define A2D_BAD_MIN_BITPOOL   0xCB  /**< Minimum Bitpool Value is not valid */
#define A2D_NS_MIN_BITPOOL    0xCC  /**< Minimum Bitpool Value is not supported */
#define A2D_BAD_MAX_BITPOOL   0xCD  /**< Maximum Bitpool Value is not valid */
#define A2D_NS_MAX_BITPOOL    0xCE  /**< Maximum Bitpool Value is not supported */
#define A2D_BAD_LAYER         0xCF  /**< None or multiple values have been selected for Layer */
#define A2D_NS_LAYER          0xD0  /**< Layer is not supported */
#define A2D_NS_CRC            0xD1  /**< CRC is not supported */
#define A2D_NS_MPF            0xD2  /**< MPF-2 is not supported */
#define A2D_NS_VBR            0xD3  /**< VBR is not supported */
#define A2D_BAD_BIT_RATE      0xD4  /**< None or multiple values have been selected for Bit Rate */
#define A2D_NS_BIT_RATE       0xD5  /**< Bit Rate is not supported */
#define A2D_BAD_OBJ_TYPE      0xD6  /**< Either 1) Object type is not valid (b3-b0) or 2) None or multiple values have been selected for Object Type */
#define A2D_NS_OBJ_TYPE       0xD7  /**< Object type is not supported */
#define A2D_BAD_CHANNEL       0xD8  /**< None or multiple values have been selected for Channels */
#define A2D_NS_CHANNEL        0xD9  /**< Channels is not supported */
#define A2D_BAD_BLOCK_LEN     0xDD  /**< None or multiple values have been selected for Block Length */
#define A2D_BAD_CP_TYPE       0xE0  /**< The requested CP Type is not supported. */
#define A2D_BAD_CP_FORMAT     0xE1  /**< The format of Content Protection Service Capability/Content Protection Scheme Dependent Data is not correct. */

#define A2D_SUCCESS_ONLY      0xF1  /** < CY Specific - Set Status to Success but not to set to route locally. */
typedef uint8_t wiced_bt_a2d_status_t;

/** @} A2D_STATUS */

/* the return values from wiced_bt_a2d_bits_set() */
#define A2D_SET_ONE_BIT         1   /* one and only one bit is set */
#define A2D_SET_ZERO_BIT        0   /* all bits clear */
#define A2D_SET_MULTL_BIT       2   /* multiple bits are set */

/*****************************************************************************
**  type definitions
*****************************************************************************/
/**< SDP record information from the server to initialize discovery database */
typedef struct
{
    uint32_t                       db_len;             /**< Length, in bytes, of the discovery database */
    uint16_t                       num_attr;           /**< The number of attributes in p_attrs */
    wiced_bt_sdp_discovery_db_t    *p_db;              /**< Pointer to the discovery database */
    uint16_t                       *p_attrs;           /**< Attributes filter, Set by A2DP API filter if NULL, else input is taken as filter*/
} wiced_bt_a2d_sdp_db_params_t;

/**< This data type is used in tA2D_FIND_CBACK to report the result of the SDP discovery process. */
typedef struct
{
    uint16_t  service_len;         /**< Length, in bytes, of the service name */
    uint16_t  provider_len;        /**< Length, in bytes, of the provider name */
    char *    p_service_name;      /**< Pointer to the service name, service_len parameter to be used to copy the string*/
    char *    p_provider_name;     /**< Pointer to the provider name,service_len parameter to be used to copy the string */
    uint16_t  features;            /**< Profile supported features */
    uint16_t  avdt_version;        /**< AVDTP protocol version */
} wiced_bt_a2d_service_t;

/**
 * A2D Find Service Discovery callback
 *
 * Callback for A2D Find Service Discovery Records
 * Registered using wiced_bt_a2d_find_service()
 * The callback function will be executed when
 * service discovery is complete.
 *
 * @param found                  : Record found status
 * @param p_service            : Contains result of SDP discovery process
 *
 * @return Status of event handling
*/

/**< This is the callback to notify the result of the SDP discovery process. */
typedef void (wiced_bt_a2d_find_cback)(wiced_bool_t found, wiced_bt_a2d_service_t* p_service);

/** @} wicedbt_av_a2d_helper */

/*****************************************************************************
**  external function declarations
*****************************************************************************/

/**
 * @addtogroup  wicedbt_a2dp        Advanced Audio (A2DP)
 * @ingroup     wicedbt_av
 *
 * Advanced Audio Distribution Profile
 *
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif
/*
 *
 * Function         wiced_bt_a2d_set_trace_level
 *
 *                  Sets the trace level for A2D. If 0xff is passed, the
 *                  current trace level is returned.
 *
 * @param[in]       new_level:  The level to set the A2D tracing to:
 *                              0xff-returns the current setting.
 *                              0-turns off tracing.
 *                              >= 1-Errors.
 *                              >= 2-Warnings.
 *                              >= 3-APIs.
 *                              >= 4-Events.
 *                              >= 5-Debug.
 *
 * @return          The new trace level or current trace level if
 *                  the input parameter is 0xff.
 *
 */
uint8_t wiced_bt_a2d_set_trace_level (uint8_t new_level);

/**
 * Function         wiced_bt_a2d_bits_set
 *
 *                  Check the number of bits set in a given mask (used to parse stream configuration masks)
 *
 * @param[in]       mask :  mask to check
 *
 * @return          A2D_SET_ONE_BIT, if one and only one bit is set
 *                  A2D_SET_ZERO_BIT, if all bits clear
 *                  A2D_SET_MULTL_BIT, if multiple bits are set
 */
uint8_t wiced_bt_a2d_bits_set(uint8_t mask);

/**
 * Function     wiced_bt_a2d_find_service
 *
 *              Performs Service Discovery and fetches SRC/SINK SDP Record Info.
 *              Information is returned for the first service record found on the
 *              server that matches the service UUID.There can only be one outstanding
 *              call to wiced_bt_a2d_find_service() at a time; the application must wait
 *              for the callback before it makes another call to the function.
 *
 *
 * @param[in]   service_uuid : Indicates SRC or SNK.
 * @param[in]   bd_addr       : BD address of the peer device.
 * @param[in]   p_db            : Pointer to the information to initialize the discovery database.
 * @param[in]   p_cback        : Pointer to the wiced_bt_a2d_find_service() callback function.
 *
 * @return @link wiced_bt_a2d_status_t @endlink
 *
 */
wiced_bt_a2d_status_t wiced_bt_a2d_find_service( uint16_t service_uuid, BD_ADDR bd_addr, wiced_bt_a2d_sdp_db_params_t *p_db, wiced_bt_a2d_find_cback *p_cback );

#ifdef __cplusplus
}
#endif

/** @} wicedbt_a2dp */
