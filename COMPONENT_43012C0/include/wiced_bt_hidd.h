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
 * Human Interface Device Profile (HID) Device over BR/EDR
 *
 */
#pragma once

#include "wiced.h"
#include "wiced_bt_dev.h"

/******************************************************
 *              Constants
 ******************************************************/


/** HID status codes */
enum wiced_bt_hidd_status_e
{
    WICED_BT_HIDD_SUCCESS,                      /**< Success */
    WICED_BT_HIDD_ERR_NOT_REGISTERED,           /**< Not registered */
    WICED_BT_HIDD_ERR_ALREADY_REGISTERED,       /**< Alreadu registered */
    WICED_BT_HIDD_ERR_NO_RESOURCES,             /**< No resources */
    WICED_BT_HIDD_ERR_NO_CONNECTION,            /**< Not connection */
    WICED_BT_HIDD_ERR_INVALID_PARAM,            /**< Invalid parameter */
    WICED_BT_HIDD_ERR_UNSUPPORTED,              /**< Not supported */
    WICED_BT_HIDD_ERR_UNKNOWN_COMMAND,          /**< Unknown command */
    WICED_BT_HIDD_ERR_CONGESTED,                /**< Congested */
    WICED_BT_HIDD_ERR_CONN_IN_PROCESS,          /**< Connection in process */
    WICED_BT_HIDD_ERR_ALREADY_CONN,             /**< Already connected */
    WICED_BT_HIDD_ERR_DISCONNECTING,            /**< Disconnecting is process */
    WICED_BT_HIDD_ERR_SET_CONNECTABLE_FAIL,     /**< Set connectable failiure */

    /* Device specific error codes */
    WICED_BT_HIDD_ERR_HOST_UNKNOWN,             /**< Host unknown */
    WICED_BT_HIDD_ERR_L2CAP_FAILED,             /**< L2CAP failed */
    WICED_BT_HIDD_ERR_AUTH_FAILED,              /**< Authentication failed */
    WICED_BT_HIDD_ERR_SDP_BUSY,                 /**< SDP busy */
    WICED_BT_HIDD_ERR_GATT,                     /**< GATT */

    WICED_BT_HIDD_ERR_INVALID = 0xFF            /**< Invalid */
};
typedef uint8_t wiced_bt_hidd_status_t;         /**< HIDD status codes (see #wiced_bt_hidd_status_e) */


/* Define the HID transaction types */
#define HID_TRANS_HANDSHAKE     (0)
#define HID_TRANS_CONTROL       (1)
#define HID_TRANS_GET_REPORT    (4)
#define HID_TRANS_SET_REPORT    (5)
#define HID_TRANS_GET_PROTOCOL  (6)
#define HID_TRANS_SET_PROTOCOL  (7)
#define HID_TRANS_GET_IDLE      (8)
#define HID_TRANS_SET_IDLE      (9)
#define HID_TRANS_DATA          (10)
#define HID_TRANS_DATAC         (11)

#define HID_GET_TRANS_FROM_HDR(x) ((x >> 4) & 0x0f)
#define HID_GET_PARAM_FROM_HDR(x) (x & 0x0f)
#ifndef HID_BUILD_HDR
#define HID_BUILD_HDR(t,p)  (uint8_t)((t << 4) | (p & 0x0f))
#endif
#define HID_HDR_LEN (1)

/* Parameters for Handshake */
#define HID_PAR_HANDSHAKE_RSP_SUCCESS               (0)
#define HID_PAR_HANDSHAKE_RSP_NOT_READY             (1)
#define HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID    (2)
#define HID_PAR_HANDSHAKE_RSP_ERR_UNSUPPORTED_REQ   (3)
#define HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM     (4)
#define HID_PAR_HANDSHAKE_RSP_ERR_UNKNOWN           (14)
#define HID_PAR_HANDSHAKE_RSP_ERR_FATAL             (15)


/* Parameters for Control */
#define HID_PAR_CONTROL_NOP                         (0)
#define HID_PAR_CONTROL_HARD_RESET                  (1)
#define HID_PAR_CONTROL_SOFT_RESET                  (2)
#define HID_PAR_CONTROL_SUSPEND                     (3)
#define HID_PAR_CONTROL_EXIT_SUSPEND                (4)
#define HID_PAR_CONTROL_VIRTUAL_CABLE_UNPLUG        (5)


/* Different report types in get, set, data */
#define HID_PAR_REP_TYPE_MASK                       (0x03)
#define HID_PAR_REP_TYPE_OTHER                      (0x00)
#define HID_PAR_REP_TYPE_INPUT                      (0x01)
#define HID_PAR_REP_TYPE_OUTPUT                     (0x02)
#define HID_PAR_REP_TYPE_FEATURE                    (0x03)

/* Parameters for Get Report */

/* Buffer size in two bytes after Report ID */
#define HID_PAR_GET_REP_BUFSIZE_FOLLOWS             (0x08)


/* Parameters for Protocol Type */
#define HID_PAR_PROTOCOL_MASK                       (0x01)
#define HID_PAR_PROTOCOL_REPORT                     (0x01)
#define HID_PAR_PROTOCOL_BOOT_MODE                  (0x00)

#define HID_PAR_REP_TYPE_MASK                       (0x03)

/* Descriptor types in the SDP record */
#define HID_SDP_DESCRIPTOR_REPORT                   (0x22)
#define HID_SDP_DESCRIPTOR_PHYSICAL                 (0x23)


/******************************************************
 *              Type Definitions
 ******************************************************/
/** HIDD QoS configuration */
typedef struct
{
    wiced_bt_flow_spec_t    ctrl_ch;    /**< Control channel */
    wiced_bt_flow_spec_t    int_ch;     /**< Interrupt */
    wiced_bt_flow_spec_t    hci;        /**< HCI */
} wiced_bt_hidd_qos_info_t;             /**< HIDD QOS info */

typedef struct wiced_bt_rep_data
{
    uint8_t rep_type;                   /**< Report type */
    uint8_t rep_id;                     /**< Report ID */
} wiced_bt_hidd_get_rep_data_t;         /**< HIDD get report data */

/** HID-Device Callback Events */
enum wiced_bt_hidd_cback_event_e
{
    WICED_BT_HIDD_EVT_OPEN,         /**< Connected to host with Interrupt and Control  Data = 1 if Virtual Cable
                                         Channels in OPEN state. pdata = Host BD-Addr.*/

    WICED_BT_HIDD_EVT_CLOSE,        /**< Connection with host is closed.            Data=Reason Code. */
    WICED_BT_HIDD_EVT_RETRYING,     /**< Lost connection is being re-connected.     Data=Retrial number */
    WICED_BT_HIDD_EVT_MODE_CHG,     /**< Device changed power mode.                 Data=new power mode */
    WICED_BT_HIDD_EVT_PM_FAILED,    /**< Device power mode change failed */
    WICED_BT_HIDD_EVT_CONTROL,      /**< Host sent HID_CONTROL                      Data=Control Operation */
    WICED_BT_HIDD_EVT_GET_REPORT,   /**< Host sent GET_REPORT                       Data=Length pdata=structure
                                         having details of get-report.*/
    WICED_BT_HIDD_EVT_SET_REPORT,   /**< Host sent SET_REPORT                       Data=Length pdata=details.*/
    WICED_BT_HIDD_EVT_GET_PROTO,    /**< Host sent GET_PROTOCOL                     Data=NA*/
    WICED_BT_HIDD_EVT_SET_PROTO,    /**< Host sent SET_PROTOCOL                     Data=1 for Report, 0 for Boot*/
    WICED_BT_HIDD_EVT_GET_IDLE,     /**< Host sent GET_IDLE                         Data=NA */
    WICED_BT_HIDD_EVT_SET_IDLE,     /**< Host sent SET_IDLE                         Data=Idle Rate */
    WICED_BT_HIDD_EVT_DATA,
    WICED_BT_HIDD_EVT_DATC,

    WICED_BT_HIDD_EVT_L2CAP_CONGEST /**< L2CAP channel congested */
};
typedef uint8_t wiced_bt_hidd_cback_event_t;   /**< HIDD events (see #wiced_bt_hidd_cback_event_e)*/

enum wiced_bt_hidd_st_e
{
    WICED_BT_HIDD_BUSY_CONN_ST,         /**< Busy state */
    WICED_BT_HIDD_IDLE_CONN_ST,         /**< Idle state */
    WICED_BT_HIDD_SUSP_CONN_ST          /**< Suspension state */
};
typedef uint8_t wiced_bt_hidd_st_t;     /**< HIDD state (see #wiced_bt_hidd_st_e) */

/** Incoming data */
typedef struct
{
    uint8_t         *p_data;            /**< Pointer to incoming data */
    uint16_t        len;                /**< data length */
} wiced_bt_hidd_data_t;

/** Data types for HIDD event callback */
typedef union {
    wiced_bt_device_address_t           host_bdaddr;    /**< Host bd address */
    wiced_bt_hidd_data_t                data;           /**< Incoming data */
    wiced_bt_hidd_get_rep_data_t        get_rep;        /**< Get report data */
    uint8_t                             pm_err_code;    /**< Power mode error code */
    uint16_t                            pm_interval;    /**< Power mode interval */
} wiced_bt_hidd_event_data_t;                           /**< HIDD callback data */

/**
 * HIDD callback
 *
 * Callback for Human Interface Device Profile Device (HIDD)
 *
 * @param[in] event         : Callback event (see #
 * @param[in] data          : Integer data corresponding to the event
 * @param[in] p_data        : Data associated with the event
 *
 * @return void
 */

typedef void (wiced_bt_hidd_callback_t) (wiced_bt_hidd_cback_event_t  event, uint32_t data, wiced_bt_hidd_event_data_t *p_event_data );

typedef struct
{
    wiced_bt_device_address_t   host_addr;      /**< Host bd address */
    wiced_bt_hidd_qos_info_t    *p_qos_info;    /**< Qos info */
    wiced_bt_hidd_callback_t    *p_app_cback;   /**< callback function */
} wiced_bt_hidd_reg_info_t;                     /**< HIDD registration info */

typedef struct wiced_bt_hidd_pwr_md
{
    uint16_t            max;        /**< Max interval */
    uint16_t            min;        /**< Min interval */
    uint16_t            attempt;    /**< Number of attempt */
    uint16_t            timeout;    /**< Timeout */
    uint8_t             mode;       /**< Power mode */
} wiced_bt_hidd_pm_pwr_md_t;        /**< HIDD power mode */


/**
 *  @addtogroup  hidd_api_functions      HIDD over BR/EDR
 *  @ingroup     wicedbt
 *
 *  HIDD Functions
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
 * Function         wiced_bt_hidd_register
 *
 *                  Called at startup to register necessary info for HIDD.
 *
 *  @param[in]      p_reg_info  : Registration info
 *  @param[out]     None
 *
 *  @return         status code (see #wiced_bt_hidd_status_t)
 */
wiced_bt_hidd_status_t wiced_bt_hidd_register(wiced_bt_hidd_reg_info_t *p_reg_info);

/**
 * Function         wiced_bt_hidd_deregister
 *
 *                  Remove HIDD service records and deregister L2CAP channel.
 *
 *  @param[in]      None
 *  @param[out]     None
 *
 *  @return         status code (see #wiced_bt_hidd_status_t)
 */
wiced_bt_hidd_status_t wiced_bt_hidd_deregister(void);

/**
 * Function         wiced_bt_hidd_connect
 *
 *                  Initiates a connection to the host.
 *
 *  @param[in]      None
 *  @param[out]     None
 *
 *  @return         status code (see #wiced_bt_hidd_status_t)
 */
wiced_bt_hidd_status_t wiced_bt_hidd_connect(void);

/**
 * Function         wiced_bt_hidd_disconnect
 *
 *                  Disconnects from the host.
 *
 *  @param[in]      None
 *  @param[out]     None
 *
 *  @return         status code (see #wiced_bt_hidd_status_t)
 */
wiced_bt_hidd_status_t wiced_bt_hidd_disconnect(void);

/**
 * Function         wiced_bt_hidd_hand_shake
 *
 *                  Sends HAND-SHAKE to host.
 *
 *  @param[in]      res_code : Result code
 *  @param[out]     None
 *
 *  @return         status code (see #wiced_bt_hidd_status_t)
 */
wiced_bt_hidd_status_t wiced_bt_hidd_hand_shake(uint8_t res_code);

/**
 * Function         wiced_bt_hidd_virtual_unplug
 *
 *                  Sends VIRTUAL-UNPLUG to host.
 *
 *  @param[in]      None
 *  @param[out]     None
 *
 *  @return         status code (see #wiced_bt_hidd_status_t)
 */
wiced_bt_hidd_status_t wiced_bt_hidd_virtual_unplug(void);

/**
 * Function         wiced_bt_hidd_send_data
 *
 *                  Sends input reports to host.
 *
 *  @param[in]      control_ch  : True if control block, False otherwise
 *  @param[in]      rep_type    : Report type
 *  @param[in]      p_data      : Report data
 *  @param[in]      data_len    : Data length
 *
 *  @return         status code (see #wiced_bt_hidd_status_t)
 */
wiced_bt_hidd_status_t wiced_bt_hidd_send_data(wiced_bool_t control_ch, uint8_t rep_type,
                                               uint8_t *p_data, uint16_t data_len);

#ifdef WICED_BT_HID_POWER_MANAGEMENT_SUPPORTED
/**
 * Function         wiced_bt_hidd_set_power_mgmt_params
 *
 *                  Changes power mgmt parameters.
 *
 *  @param[in]      conn_substate   : Connection substate
 *  @param[in]      pm_params       : Power management paramters
 *  @param[out]     None
 *
 *  @return         status code (see #wiced_bt_hidd_status_t)
 */
wiced_bt_hidd_status_t wiced_bt_hidd_set_power_mgmt_params(uint8_t conn_substate,
                                                           wiced_bt_hidd_pm_pwr_md_t pm_params);
# endif // WICED_BT_HID_POWER_MANAGEMENT_SUPPORTED

#ifdef __cplusplus
}
#endif

/** @} wicedbt_hidd */
