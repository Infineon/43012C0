/*
 * Copyright 2014, Cypress Semiconductor
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */
 /** @file
 *
 *  Defines functions for using the WICED MPAF Framework
 */
#ifndef WICED_BT_APP_H
#define WICED_BT_APP_H

//typedef uint32_t (*WICED_BT_GET_TIME_TO_SLEEP_FP)(void);
//typedef MPAF_STATUS (*WICED_BT_STACK_EVENT_HANDLER_CB)( const BTHCI_EVENT_HDR_t* event );

/*****************************************************************************
 *                                              Function Definitions
 ****************************************************************************/
//BOOL32 wiced_bt_app_memInit(void);
//void wiced_bt_app_install( MPAF_APP_ID app_id );
//MPAF_APP_ID wiced_get_app_id ( void );
//MPAF_SAP_HANDLE wiced_bt_app_get_mpaf_pf_handle( void );
//extern void wiced_bt_InstallIdleThreadCSAPoll(MPAF_CORE_VOID_CB callback);
//extern void wiced_bt_InstallGetTimeToSleep(WICED_BT_GET_TIME_TO_SLEEP_FP callback);


/*
* Enum type received on callabck
*/
enum
{
    /// Ready to send out an adv in the next few mS. App can change ADV data if required.
    /// Typically invoked about 2.5mS before te ADV. If there are other higher priority
    /// tasks or other events in the app thread event queue, this will be delayed.
    /// Notification is best effort.
    WICED_BT_ADV_NOTIFICATION_READY,

    /// Just completed transmitting an ADV packet.
    WICED_BT_ADV_NOTIFICATION_DONE
};

// TODO: Move this to appropriate file in wiced_bt.
// This function is to add just the "wiced_bt_" prefix to the function.
/*******************************************************************************
* Function: wiced_bt_notifyAdvPacketTransmissions
*
* Abstract: Allows the application to register a callback that will be invoked
*           just before an ADV is packet is about to be sent out and again,
*           immediately after.
*
* Input : clientCallback - Pointer to a function that will be invoked in application
*                          thread context with WICED_BT_ADV_NOTIFICATION_READY for
                           before ADV and WICED_BT_ADV_NOTIFICATION_DONE after ADV
                           packet is complete.
*         advanceNoticeInMicroSeconds - Number of microseconds before the ADV the
*                          notification is to be sent. Will be rounded down to
*                          the nearest 1.25mS. Has to be an even multiple of 625uS.
* Output: TRUE if feature available, FALSE otherwise.
*
*******************************************************************************/
wiced_bool_t wiced_bt_notifyAdvPacketTransmissions(void (*clientCallback)(uint8_t),
                                             uint32_t advanceNoticeInMicroSeconds);

#endif /* WICED_BT_APP_H*/
