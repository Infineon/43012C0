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

/****************************************************************************
**
**  Name        bt_types.h
**
**  Function    this file contains definitions that are shared between
**              units in the Bluetooth system such as events.
**
**
****************************************************************************/

#ifndef BT_TYPES_H
#define BT_TYPES_H

#include "data_types.h"

#ifdef _WIN32
#ifdef BLUESTACK_TESTER
    #include "bte_stack_entry.h"
#endif
#endif

/* READ WELL !!
**
** This section defines global events. These are events that cross layers.
** Any event that passes between layers MUST be one of these events. Tasks
** can use their own events internally, but a FUNDAMENTAL design issue is
** that global events MUST be one of these events defined below.
**
** The convention used is the the event name contains the layer that the
** event is going to.
*/
#define BT_EVT_MASK                 0xFF00
#define BT_SUB_EVT_MASK             0x00FF
                                                /* To Bluetooth Upper Layers        */
                                                /************************************/
#define BT_EVT_TO_BTU_L2C_EVT       0x0900      /* L2CAP event */
#define BT_EVT_TO_BTU_HCI_EVT       0x1000      /* HCI Event                        */
#define BT_EVT_TO_BTU_HCI_BR_EDR_EVT (0x0000 | BT_EVT_TO_BTU_HCI_EVT)      /* event from BR/EDR controller */

#define BT_EVT_TO_BTU_HCI_ACL       0x1100      /* ACL Data from HCI                */
#define BT_EVT_TO_BTU_HCI_SCO       0x1200      /* SCO Data from HCI                */
#define BT_EVT_TO_BTU_HCIT_ERR      0x1300      /* HCI Transport Error              */

#define BT_EVT_TO_BTU_SP_EVT        0x1400      /* Serial Port Event                */
#define BT_EVT_TO_BTU_SP_DATA       0x1500      /* Serial Port Data                 */

#define BT_EVT_TO_BTU_HCI_CMD       0x1600      /* HCI command from upper layer     */


#define BT_EVT_TO_BTU_L2C_SEG_XMIT  0x1900      /* L2CAP segment(s) transmitted     */

#define BT_EVT_PROXY_INCOMING_MSG   0x1A00      /* BlueStackTester event: incoming message from target */

#define BT_EVT_BTSIM                0x1B00      /* Insight BTSIM event */
#define BT_EVT_BTISE                0x1C00      /* Insight Script Engine event */

                                                /* To LM                            */
                                                /************************************/
#define BT_EVT_TO_LM_HCI_CMD        0x2000      /* HCI Command                      */
#define BT_EVT_TO_LM_HCI_ACL        0x2100      /* HCI ACL Data                     */
#define BT_EVT_TO_LM_HCI_SCO        0x2200      /* HCI SCO Data                     */
#define BT_EVT_TO_LM_HCIT_ERR       0x2300      /* HCI Transport Error              */
#define BT_EVT_TO_LM_LC_EVT         0x2400      /* LC event                         */
#define BT_EVT_TO_LM_LC_LMP         0x2500      /* LC Received LMP command frame    */
#define BT_EVT_TO_LM_LC_ACL         0x2600      /* LC Received ACL data             */
#define BT_EVT_TO_LM_LC_SCO         0x2700      /* LC Received SCO data  (not used) */
#define BT_EVT_TO_LM_LC_ACL_TX      0x2800      /* LMP data transmit complete       */
#define BT_EVT_TO_LM_LC_LMPC_TX     0x2900      /* LMP Command transmit complete    */
#define BT_EVT_TO_LM_LOCAL_ACL_LB   0x2a00      /* Data to be locally loopbacked    */
#define BT_EVT_TO_LM_HCI_ACL_ACK    0x2b00      /* HCI ACL Data ack      (not used) */
#define BT_EVT_TO_LM_DIAG           0x2c00      /* LM Diagnostics commands          */


#define BT_EVT_TO_BTM_CMDS          0x2f00
#define BT_EVT_TO_BTM_PM_MDCHG_EVT (0x0001 | BT_EVT_TO_BTM_CMDS)

#define BT_EVT_TO_OBX_CL_MSG        0x3100
#define BT_EVT_TO_OBX_SR_MSG        0x3200

/* Obex Over L2CAP */
#define BT_EVT_TO_OBX_CL_L2C_MSG    0x3400
#define BT_EVT_TO_OBX_SR_L2C_MSG    0x3500

/* ftp events */
#define BT_EVT_TO_FTP_SRVR_CMDS     0x3600
#define BT_EVT_TO_FTP_CLNT_CMDS     0x3700

#define BT_EVT_TO_BTU_SAP           0x3800       /* SIM Access Profile events */

/* opp events */
#define BT_EVT_TO_OPP_SRVR_CMDS     0x3900
#define BT_EVT_TO_OPP_CLNT_CMDS     0x3a00

/* gap events */
#define BT_EVT_TO_GAP_MSG           0x3b00

/* start timer */
#define BT_EVT_TO_START_TIMER       0x3c00

/* stop timer */
#define BT_EVT_TO_STOP_TIMER        0x3d00

/* start quick timer */
#define BT_EVT_TO_START_QUICK_TIMER 0x3e00


/* for NFC                          */
                                                /************************************/
#define BT_EVT_TO_NFC_NCI           0x4000      /* NCI Command, Notification or Data*/
#define BT_EVT_TO_NFC_NCI_VS        0x4200      /* Vendor specific message */
#define BT_EVT_TO_NFC_MSGS          0x4300      /* messages between NFC and NCI task */
#define BT_EVT_TO_LLCP_MSGS         0x4400      /* messages for LLCP */

#define BT_EVT_TO_NFCCSIM_NCI       0x4a00      /* events to NFCC simulation (NCI packets) */

/* HCISU Events */

#define BT_EVT_HCISU                0x5000

#define BT_EVT_TO_HCISU_LP_APP_SLEEPING_EVT     (0x0005 | BT_EVT_HCISU)
#define BT_EVT_TO_HCISU_LP_ALLOW_BT_SLEEP_EVT   (0x0006 | BT_EVT_HCISU)
#define BT_EVT_TO_HCISU_LP_WAKEUP_HOST_EVT      (0x0007 | BT_EVT_HCISU)
#define BT_EVT_TO_HCISU_LP_RCV_H4IBSS_EVT       (0x0008 | BT_EVT_HCISU)
#define BT_EVT_TO_HCISU_H5_RESET_EVT            (0x0009 | BT_EVT_HCISU)
#define BT_EVT_HCISU_START_QUICK_TIMER          (0x000a | BT_EVT_HCISU)

/* HSP Events */

#define BT_EVT_BTU_HSP2             0x6000

#define BT_EVT_TO_BTU_HSP2_EVT     (0x0001 | BT_EVT_BTU_HSP2)

/* BPP Events */
#define BT_EVT_TO_BPP_PR_CMDS       0x6100      /* Printer Events */
#define BT_EVT_TO_BPP_SND_CMDS      0x6200      /* BPP Sender Events */

/* BIP Events */
#define BT_EVT_TO_BIP_CMDS          0x6300

#define BT_EVT_BTU_HFP              0x8000
#define BT_EVT_TO_BTU_HFP_EVT      (0x0001 | BT_EVT_BTU_HFP)

#define BT_EVT_BTU_IPC_EVT          0x9000
#define BT_EVT_BTU_IPC_LOGMSG_EVT  (0x0000 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_ACL_EVT     (0x0001 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_BTU_EVT     (0x0002 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_L2C_EVT     (0x0003 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_L2C_MSG_EVT (0x0004 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_BTM_EVT     (0x0005 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_AVDT_EVT    (0x0006 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_SLIP_EVT    (0x0007 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_MGMT_EVT    (0x0008 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_BTTRC_EVT   (0x0009 | BT_EVT_BTU_IPC_EVT)
#define BT_EVT_BTU_IPC_BURST_EVT   (0x000A | BT_EVT_BTU_IPC_EVT)

/* Define the header of each buffer used in the Bluetooth stack.
*/
typedef struct
{
    UINT16          event;
    UINT16          len;
    UINT16          offset;
    UINT16          layer_specific;
} BT_HDR;

#define BT_HDR_SIZE (sizeof (BT_HDR))

#define BT_PSM_SDP                      0x0001 /* Service Discovery Protocol) */
#define BT_PSM_RFCOMM                   0x0003 /* RFCOMM with TS 07.10 */
#define BT_PSM_TCS                      0x0005 /* (deprecated) */
#define BT_PSM_CTP                      0x0007 /* (deprecated) */
#define BT_PSM_BNEP                     0x000F /* Bluetooth Network Encapsulation Protocol */
#define BT_PSM_HIDC                     0x0011 /* Human Interface Device (Control) */
#define BT_PSM_HIDI                     0x0013 /* Human Interface Device (Interrupt) */
#define BT_PSM_UPNP                     0x0015 /* Universal PlugNPlay */
#define BT_PSM_AVCTP                    0x0017 /* Audio/Video Control Transport Protocol */
#define BT_PSM_AVDTP                    0x0019 /* Audio/Video Distribution Transport Protocol */
#define BT_PSM_AVCTP_13                 0x001B /* Advanced Control - Browsing */
#define BT_PSM_UDI_CP                   0x001D /* Unrestricted Digital Information C-Plane  */
#define BT_PSM_ATT                      0x001F /* Attribute Protocol  */
#define BT_PSM_3DS                      0x0021 /* 3D sync */
#define BT_LE_PSM_IPSP                  0x0023 /* Internet Protocol Support (IPv6/6LoWPAN) */


/* These macros extract the HCI opcodes from a buffer
*/
#define HCI_GET_CMD_HDR_OPCODE(p)    (UINT16)((*((UINT8 *)((p) + 1) + p->offset) + \
                                              (*((UINT8 *)((p) + 1) + p->offset + 1) << 8)))
#define HCI_GET_CMD_HDR_PARAM_LEN(p) (UINT8)  (*((UINT8 *)((p) + 1) + p->offset + 2))

#define HCI_GET_EVT_HDR_OPCODE(p)    (UINT8)(*((UINT8 *)((p) + 1) + p->offset))
#define HCI_GET_EVT_HDR_PARAM_LEN(p) (UINT8)  (*((UINT8 *)((p) + 1) + p->offset + 1))


/********************************************************************************
** Macros to get and put bytes to and from a stream (Little Endian format).
*/
#define UINT128_TO_STREAM(p, u128) {*(p)++ = (UINT8)(u128);       *(p)++ = (UINT8)((u128) >> 8);*(p)++ = (UINT8)((u128) >> 16); *(p)++ = (UINT8)((u128) >> 24); \
                                    *(p)++ = (UINT8)((u128) >> 32); *(p)++ = (UINT8)((u128) >> 40);*(p)++ = (UINT8)((u128) >> 48); *(p)++ = (UINT8)((u128) >> 56); \
                                    *(p)++ = (UINT8)((u128) >> 64); *(p)++ = (UINT8)((u128) >> 72);*(p)++ = (UINT8)((u128) >> 80); *(p)++ = (UINT8)((u128) >> 88); \
                                    *(p)++ = (UINT8)((u128) >> 96); *(p)++ = (UINT8)((u128) >> 104);*(p)++ = (UINT8)((u128) >> 112); *(p)++ = (UINT8)((u128) >> 120); }
#define UINT64_TO_STREAM(p, u64) {*(p)++ = (UINT8)(u64);       *(p)++ = (UINT8)((u64) >> 8);*(p)++ = (UINT8)((u64) >> 16); *(p)++ = (UINT8)((u64) >> 24); \
                                    *(p)++ = (UINT8)((u64) >> 32); *(p)++ = (UINT8)((u64) >> 40);*(p)++ = (UINT8)((u64) >> 48); *(p)++ = (UINT8)((u64) >> 56);}
#define UINT40_TO_STREAM(p, u40) {*(p)++ = (UINT8)(u40); *(p)++ = (UINT8)((u40) >> 8); *(p)++ = (UINT8)((u40) >> 16); *(p)++ = (UINT8)((u40) >> 24); \
                                  *(p)++ = (UINT8)((u40) >> 32);}
#define UINT48_TO_STREAM(p, u48) {*(p)++ = (UINT8)(u48); *(p)++ = (UINT8)((u48) >> 8); *(p)++ = (UINT8)((u48) >> 16); *(p)++ = (UINT8)((u48) >> 24); *(p)++ = (UINT8)((u48) >> 32); *(p)++ = (UINT8)((u48) >> 40);}
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (UINT8)(u32); *(p)++ = (UINT8)((u32) >> 8); *(p)++ = (UINT8)((u32) >> 16); *(p)++ = (UINT8)((u32) >> 24);}
#define UINT24_TO_STREAM(p, u24) {*(p)++ = (UINT8)(u24); *(p)++ = (UINT8)((u24) >> 8); *(p)++ = (UINT8)((u24) >> 16);}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (UINT8)(u16); *(p)++ = (UINT8)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (UINT8)(u8);}
#define INT8_TO_STREAM(p, u8)    {*(p)++ = (INT8)(u8);}
#define ARRAY32_TO_STREAM(p, a)  {register int ijk; for (ijk = 0; ijk < 32;           ijk++) *(p)++ = (UINT8) a[31 - ijk];}
#define ARRAY16_TO_STREAM(p, a)  {register int ijk; for (ijk = 0; ijk < 16;           ijk++) *(p)++ = (UINT8) a[15 - ijk];}
#define ARRAY8_TO_STREAM(p, a)   {register int ijk; for (ijk = 0; ijk < 8;            ijk++) *(p)++ = (UINT8) a[7 - ijk];}
#define BDADDR_TO_STREAM(p, a)   {register int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (UINT8) a[BD_ADDR_LEN - 1 - ijk];}
#define LAP_TO_STREAM(p, a)      {register int ijk; for (ijk = 0; ijk < LAP_LEN;      ijk++) *(p)++ = (UINT8) a[LAP_LEN - 1 - ijk];}
#define DEVCLASS_TO_STREAM(p, a) {register int ijk; for (ijk = 0; ijk < DEV_CLASS_LEN;ijk++) *(p)++ = (UINT8) a[DEV_CLASS_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {register int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (UINT8) a[ijk];}
#define REVERSE_ARRAY_TO_STREAM(p, a, len)  {register int ijk; for (ijk = 0; ijk < len; ijk++) *(p)++ = (UINT8) a[len - 1 - ijk];}

#define STREAM_TO_UINT8(u8, p)   {u8 = (UINT8)(*(p)); (p) += 1;}
#define STREAM_TO_UINT16(u16, p) {u16 = ((UINT16)(*(p)) + (((UINT16)(*((p) + 1))) << 8)); (p) += 2;}
#define STREAM_TO_UINT24(u32, p) {u32 = (((UINT32)(*(p))) + ((((UINT32)(*((p) + 1)))) << 8) + ((((UINT32)(*((p) + 2)))) << 16) ); (p) += 3;}
#define STREAM_TO_UINT32(u32, p) {u32 = (((UINT32)(*(p))) + ((((UINT32)(*((p) + 1)))) << 8) + ((((UINT32)(*((p) + 2)))) << 16) + ((((UINT32)(*((p) + 3)))) << 24)); (p) += 4;}
#define STREAM_TO_UINT40(u40, p) {u40 = (((UINT64)(*(p))) + ((((UINT64)(*((p) + 1)))) << 8) + ((((UINT64)(*((p) + 2)))) << 16) + ((((UINT64)(*((p) + 3)))) << 24) + ((((UINT64)(*((p) + 4)))) << 32)); (p) += 5;}
#define STREAM_TO_BDADDR(a, p)   {register int ijk; register UINT8 *pbda = (UINT8 *)a + BD_ADDR_LEN - 1; for (ijk = 0; ijk < BD_ADDR_LEN; ijk++) *pbda-- = *p++;}
#define STREAM_TO_ARRAY32(a, p)  {register int ijk; register UINT8 *_pa = (UINT8 *)a + 31; for (ijk = 0; ijk < 32; ijk++) *_pa-- = *p++;}
#define STREAM_TO_ARRAY16(a, p)  {register int ijk; register UINT8 *_pa = (UINT8 *)a + 15; for (ijk = 0; ijk < 16; ijk++) *_pa-- = *p++;}
#define STREAM_TO_ARRAY8(a, p)   {register int ijk; register UINT8 *_pa = (UINT8 *)a + 7; for (ijk = 0; ijk < 8; ijk++) *_pa-- = *p++;}
#define STREAM_TO_DEVCLASS(a, p) {register int ijk; register UINT8 *_pa = (UINT8 *)a + DEV_CLASS_LEN - 1; for (ijk = 0; ijk < DEV_CLASS_LEN; ijk++) *_pa-- = *p++;}
#define STREAM_TO_LAP(a, p)      {register int ijk; register UINT8 *plap = (UINT8 *)a + LAP_LEN - 1; for (ijk = 0; ijk < LAP_LEN; ijk++) *plap-- = *p++;}
#define STREAM_TO_ARRAY(a, p, len) {register int ijk; for (ijk = 0; ijk < len; ijk++) ((UINT8 *) a)[ijk] = *p++;}
#define REVERSE_STREAM_TO_ARRAY(a, p, len) {register int ijk; register UINT8 *_pa = (UINT8 *)a + len - 1; for (ijk = 0; ijk < len; ijk++) *_pa-- = *p++;}

/********************************************************************************
** Macros to get and put bytes to and from a field (Little Endian format).
** These are the same as to stream, except the pointer is not incremented.
*/
#define UINT32_TO_FIELD(p, u32) {*(UINT8 *)(p) = (UINT8)(u32); *((UINT8 *)(p)+1) = (UINT8)((u32) >> 8); *((UINT8 *)(p)+2) = (UINT8)((u32) >> 16); *((UINT8 *)(p)+3) = (UINT8)((u32) >> 24);}
#define UINT24_TO_FIELD(p, u24) {*(UINT8 *)(p) = (UINT8)(u24); *((UINT8 *)(p)+1) = (UINT8)((u24) >> 8); *((UINT8 *)(p)+2) = (UINT8)((u24) >> 16);}
#define UINT16_TO_FIELD(p, u16) {*(UINT8 *)(p) = (UINT8)(u16); *((UINT8 *)(p)+1) = (UINT8)((u16) >> 8);}
#define UINT8_TO_FIELD(p, u8)   {*(UINT8 *)(p) = (UINT8)(u8);}


/********************************************************************************
** Macros to get and put bytes to and from a stream (Big Endian format)
*/
#define UINT128_TO_BE_STREAM(p, u128) {*(p)++ = (UINT8)(u128)>> 120;       *(p)++ = (UINT8)((u128) >> 112);*(p)++ = (UINT8)((u128) >> 104); *(p)++ = (UINT8)((u128) >> 96); \
                                    *(p)++ = (UINT8)((u128) >> 88); *(p)++ = (UINT8)((u128) >> 80);*(p)++ = (UINT8)((u128) >> 72); *(p)++ = (UINT8)((u128) >> 64); \
                                    *(p)++ = (UINT8)((u128) >> 56); *(p)++ = (UINT8)((u128) >> 48);*(p)++ = (UINT8)((u128) >> 40); *(p)++ = (UINT8)((u128) >> 32); \
                                    *(p)++ = (UINT8)((u128) >> 24); *(p)++ = (UINT8)((u128) >> 16);*(p)++ = (UINT8)((u128) >> 8); *(p)++ = (UINT8)(u128); }
#define UINT64_TO_BE_STREAM(p, u64) {*(p)++ = (UINT8)((u64) >> 56); *(p)++ = (UINT8)((u64) >> 48);*(p)++ = (UINT8)((u64) >> 40); *(p)++ = (UINT8)((u64) >> 32); \
                                     *(p)++ = (UINT8)((u64) >> 24); *(p)++ = (UINT8)((u64) >> 16);*(p)++ = (UINT8)((u64) >> 8); *(p)++ = (UINT8)(u64); }
#define UINT32_TO_BE_STREAM(p, u32) {*(p)++ = (UINT8)((u32) >> 24);  *(p)++ = (UINT8)((u32) >> 16); *(p)++ = (UINT8)((u32) >> 8); *(p)++ = (UINT8)(u32); }
#define UINT24_TO_BE_STREAM(p, u24) {*(p)++ = (UINT8)((u24) >> 16); *(p)++ = (UINT8)((u24) >> 8); *(p)++ = (UINT8)(u24);}
#define UINT16_TO_BE_STREAM(p, u16) {*(p)++ = (UINT8)((u16) >> 8); *(p)++ = (UINT8)(u16);}
#define UINT8_TO_BE_STREAM(p, u8)   {*(p)++ = (UINT8)(u8);}
#define ARRAY_TO_BE_STREAM(p, a, len) {register int ijk; for (ijk = 0; ijk < len; ijk++) *(p)++ = (UINT8) a[ijk];}

#define BE_STREAM_TO_UINT8(u8, p)   {u8 = (UINT8)(*(p)); (p) += 1;}
#define BE_STREAM_TO_UINT16(u16, p) {u16 = (UINT16)(((UINT16)(*(p)) << 8) + (UINT16)(*((p) + 1))); (p) += 2;}
#define BE_STREAM_TO_UINT24(u32, p) {u32 = (((UINT32)(*((p) + 2))) + ((UINT32)(*((p) + 1)) << 8) + ((UINT32)(*(p)) << 16)); (p) += 3;}
#define BE_STREAM_TO_UINT32(u32, p) {u32 = ((UINT32)(*((p) + 3)) + ((UINT32)(*((p) + 2)) << 8) + ((UINT32)(*((p) + 1)) << 16) + ((UINT32)(*(p)) << 24)); (p) += 4;}
#define BE_STREAM_TO_UINT64(u64, p) {u64 = ((UINT64)(*((p) + 7)) + ((UINT64)(*((p) + 6)) << 8) + ((UINT64)(*((p) + 5)) << 16) + ((UINT64)(*((p) + 4)) << 24) + \
                                           ((UINT64)(*((p) + 3)) << 32) + ((UINT64)(*((p) + 2)) << 40) + ((UINT64)(*((p) + 1)) << 48) + ((UINT64)(*(p)) << 56)); (p) += 8;}
#define BE_STREAM_TO_ARRAY(p, a, len) {register int ijk; for (ijk = 0; ijk < len; ijk++) ((UINT8 *) a)[ijk] = *p++;}


/********************************************************************************
** Macros to get and put bytes to and from a field (Big Endian format).
** These are the same as to stream, except the pointer is not incremented.
*/
#define UINT32_TO_BE_FIELD(p, u32) {*(UINT8 *)(p) = (UINT8)((u32) >> 24);  *((UINT8 *)(p)+1) = (UINT8)((u32) >> 16); *((UINT8 *)(p)+2) = (UINT8)((u32) >> 8); *((UINT8 *)(p)+3) = (UINT8)(u32); }
#define UINT24_TO_BE_FIELD(p, u24) {*(UINT8 *)(p) = (UINT8)((u24) >> 16); *((UINT8 *)(p)+1) = (UINT8)((u24) >> 8); *((UINT8 *)(p)+2) = (UINT8)(u24);}
#define UINT16_TO_BE_FIELD(p, u16) {*(UINT8 *)(p) = (UINT8)((u16) >> 8); *((UINT8 *)(p)+1) = (UINT8)(u16);}
#define UINT8_TO_BE_FIELD(p, u8)   {*(UINT8 *)(p) = (UINT8)(u8);}


/* Common Bluetooth field definitions */
#define BD_ADDR_LEN     6                   /* Device address length */
typedef UINT8 BD_ADDR[BD_ADDR_LEN];         /* Device address */
typedef UINT8 *BD_ADDR_PTR;                 /* Pointer to Device Address */

#define BT_OCTET8_LEN    8
typedef UINT8 BT_OCTET8[BT_OCTET8_LEN];   /* octet array: size 8 */

#define LINK_KEY_LEN    16
typedef UINT8 LINK_KEY[LINK_KEY_LEN];       /* Link Key */

#define BT_OCTET16_LEN    16
typedef UINT8 BT_OCTET16[BT_OCTET16_LEN];   /* octet array: size 16 */

#define BT_OCTET32_LEN    32
typedef UINT8 BT_OCTET32[BT_OCTET32_LEN];   /* octet array: size 32 */

#define PIN_CODE_LEN    16
typedef UINT8 PIN_CODE[PIN_CODE_LEN];       /* Pin Code (upto 128 bits) MSB is 0 */
typedef UINT8 *PIN_CODE_PTR;                /* Pointer to Pin Code */

#define DEV_CLASS_LEN   3
typedef UINT8 DEV_CLASS[DEV_CLASS_LEN];     /* Device class */
typedef UINT8 *DEV_CLASS_PTR;               /* Pointer to Device class */

#define EXT_INQ_RESP_LEN   3
typedef UINT8 EXT_INQ_RESP[EXT_INQ_RESP_LEN];/* Extended Inquiry Response */
typedef UINT8 *EXT_INQ_RESP_PTR;             /* Pointer to Extended Inquiry Response */

#define BD_NAME_LEN     248
typedef UINT8 BD_NAME[BD_NAME_LEN + 1];         /* Device name */
typedef UINT8 *BD_NAME_PTR;                 /* Pointer to Device name */

#define BD_FEATURES_LEN 8
typedef UINT8 BD_FEATURES[BD_FEATURES_LEN]; /* LMP features supported by device */

#define BT_EVENT_MASK_LEN  8
typedef UINT8 BT_EVENT_MASK[BT_EVENT_MASK_LEN];   /* Event Mask */

#define LAP_LEN         3
typedef UINT8 LAP[LAP_LEN];                 /* IAC as passed to Inquiry (LAP) */
typedef UINT8 INQ_LAP[LAP_LEN];             /* IAC as passed to Inquiry (LAP) */

#define RAND_NUM_LEN    16
typedef UINT8 RAND_NUM[RAND_NUM_LEN];

#define ACO_LEN         12
typedef UINT8 ACO[ACO_LEN];                 /* Authenticated ciphering offset */

#define COF_LEN         12
typedef UINT8 COF[COF_LEN];                 /* ciphering offset number */

typedef struct {
    UINT8               qos_flags;          /* TBD */
    UINT8               service_type;       /* see below */
    UINT32              token_rate;         /* bytes/second */
    UINT32              token_bucket_size;  /* bytes */
    UINT32              peak_bandwidth;     /* bytes/second */
    UINT32              latency;            /* microseconds */
    UINT32              delay_variation;    /* microseconds */
} FLOW_SPEC;

/* Values for service_type */
#define NO_TRAFFIC      0
#define BEST_EFFORT     1
#define GUARANTEED      2

/* Service class of the CoD */
#define SERV_CLASS_NETWORKING               (1 << 1)
#define SERV_CLASS_RENDERING                (1 << 2)
#define SERV_CLASS_CAPTURING                (1 << 3)
#define SERV_CLASS_OBJECT_TRANSFER          (1 << 4)
#define SERV_CLASS_OBJECT_AUDIO             (1 << 5)
#define SERV_CLASS_OBJECT_TELEPHONY         (1 << 6)
#define SERV_CLASS_OBJECT_INFORMATION       (1 << 7)

/* Second byte */
#define SERV_CLASS_LIMITED_DISC_MODE        (0x20)

/* Field size definitions. Note that byte lengths are rounded up. */
#define ACCESS_CODE_BIT_LEN             72
#define ACCESS_CODE_BYTE_LEN            9
#define SHORTENED_ACCESS_CODE_BIT_LEN   68

typedef UINT8 ACCESS_CODE[ACCESS_CODE_BYTE_LEN];

#define SYNTH_TX                1           /* want synth code to TRANSMIT at this freq */
#define SYNTH_RX                2           /* want synth code to RECEIVE at this freq */

#define SYNC_REPS 1             /* repeats of sync word transmitted to start of burst */

/* Bluetooth CLK27 */
#define BT_CLK27            (2 << 26)

/* Bluetooth CLK12 is 1.28 sec */
#define BT_CLK12_TO_MS(x)    ((x) * 1280)
#define BT_MS_TO_CLK12(x)    ((x) / 1280)
#define BT_CLK12_TO_SLOTS(x) ((x) << 11)

/* Bluetooth CLK is 0.625 msec */
#define BT_CLK_TO_MS(x)      (((x) * 5 + 3) / 8)
#define BT_MS_TO_CLK(x)      (((x) * 8 + 2) / 5)

#define BT_CLK_TO_MICROSECS(x)  (((x) * 5000 + 3) / 8)
#define BT_MICROSECS_TO_CLK(x)  (((x) * 8 + 2499) / 5000)

/* Maximum UUID size - 16 bytes, and structure to hold any type of UUID. */
#define MAX_UUID_SIZE              16
typedef struct
{
#define LEN_UUID_16     2
#define LEN_UUID_32     4
#define LEN_UUID_128    16

    UINT16          len;

    union
    {
        UINT16      uuid16;
        UINT32      uuid32;
        UINT8       uuid128[MAX_UUID_SIZE];
    } uu;

} tBT_UUID;

#define BT_EIR_FLAGS_TYPE                   0x01
#define BT_EIR_MORE_16BITS_UUID_TYPE        0x02
#define BT_EIR_COMPLETE_16BITS_UUID_TYPE    0x03
#define BT_EIR_MORE_32BITS_UUID_TYPE        0x04
#define BT_EIR_COMPLETE_32BITS_UUID_TYPE    0x05
#define BT_EIR_MORE_128BITS_UUID_TYPE       0x06
#define BT_EIR_COMPLETE_128BITS_UUID_TYPE   0x07
#define BT_EIR_SHORTENED_LOCAL_NAME_TYPE    0x08
#define BT_EIR_COMPLETE_LOCAL_NAME_TYPE     0x09
#define BT_EIR_TX_POWER_LEVEL_TYPE          0x0A
#define BT_EIR_OOB_BD_ADDR_TYPE             0x0C
#define BT_EIR_OOB_COD_TYPE                 0x0D
#define BT_EIR_OOB_SSP_HASH_C_TYPE          0x0E
#define BT_EIR_OOB_SSP_RAND_R_TYPE          0x0F
#define BT_EIR_OOB_SSP_HASH_C_256_TYPE      0x1D
#define BT_EIR_OOB_SSP_RAND_R_256_TYPE      0x1E
#define BT_EIR_3D_SYNC_TYPE                 0x3D
#define BT_EIR_MANUFACTURER_SPECIFIC_TYPE   0xFF

#define BT_OOB_COD_SIZE            3
#define BT_OOB_HASH_C_SIZE         16
#define BT_OOB_RAND_R_SIZE         16
#define BT_OOB_HASH_C_256_SIZE     16
#define BT_OOB_RAND_R_256_SIZE     16

/* Broadcom proprietary UUIDs and reserved PSMs
**
** The lowest 4 bytes byte of the UUID or GUID depends on the feature. Typically,
** the value of those bytes will be the PSM or SCN, but it is up to the features.
*/
#define BRCM_PROPRIETARY_UUID_BASE  0xDA, 0x23, 0x41, 0x02, 0xA3, 0xBB, 0xC1, 0x71, 0xBA, 0x09, 0x6f, 0x21
#define BRCM_PROPRIETARY_GUID_BASE  0xda23, 0x4102, 0xa3, 0xbb, 0xc1, 0x71, 0xba, 0x09, 0x6f, 0x21

/* We will not allocate a PSM in the reserved range to 3rd party apps
*/
#define BRCM_RESERVED_PSM_START	    0x5AE1
#define BRCM_RESERVED_PSM_END	    0x5AFF

#define BRCM_UTILITY_SERVICE_PSM    0x5AE1
#define BRCM_MATCHER_PSM            0x5AE3

/* Connection statistics
*/

/* Structure to hold connection stats */
#ifndef BT_CONN_STATS_DEFINED
#define BT_CONN_STATS_DEFINED

/* These bits are used in the bIsConnected field */
#define BT_CONNECTED_USING_BREDR   1

typedef struct
{
    UINT32   is_connected;
    INT32    rssi;
    UINT32   bytes_sent;
    UINT32   bytes_rcvd;
    UINT32   duration;
} tBT_CONN_STATS;

#endif

/*****************************************************************************
**                          Low Energy definitions
**
** Address types
*/
#define BLE_ADDR_PUBLIC         0x00
#define BLE_ADDR_RANDOM         0x01
#define BLE_ADDR_PUBLIC_ID      0x02
#define BLE_ADDR_RANDOM_ID      0x03
typedef UINT8 tBLE_ADDR_TYPE;
#define BLE_ADDR_TYPE_MASK      (BLE_ADDR_RANDOM | BLE_ADDR_PUBLIC)


#define BT_TRANSPORT_BR_EDR    1
#define BT_TRANSPORT_LE        2
typedef UINT8 tBT_TRANSPORT;

#define BLE_ADDR_IS_STATIC(x)   ((x[0] & 0xC0) == 0xC0)

typedef struct
{
    tBLE_ADDR_TYPE      type;
    BD_ADDR             bda;
} tBLE_BD_ADDR;

/* Device Types
*/
#define BT_DEVICE_TYPE_BREDR   0x01
#define BT_DEVICE_TYPE_BLE     0x02
#define BT_DEVICE_TYPE_DUMO    0x03
typedef UINT8 tBT_DEVICE_TYPE;
/*****************************************************************************/


/* Define trace levels */
#define BT_TRACE_LEVEL_NONE    0          /* No trace messages to be generated    */
#define BT_TRACE_LEVEL_ERROR   1          /* Error condition trace messages       */
#define BT_TRACE_LEVEL_WARNING 2          /* Warning condition trace messages     */
#define BT_TRACE_LEVEL_API     3          /* API traces                           */
#define BT_TRACE_LEVEL_EVENT   4          /* Debug messages for events            */
#define BT_TRACE_LEVEL_DEBUG   5          /* Full debug messages                  */

#define MAX_TRACE_LEVEL        5


/* Define New Trace Type Definition */
/* TRACE_CTRL_TYPE                  0x^^000000*/
#define TRACE_CTRL_MASK             0xff000000
#define TRACE_GET_CTRL(x)           ((((UINT32)(x)) & TRACE_CTRL_MASK) >> 24)

#define TRACE_CTRL_GENERAL          0x00000000
#define TRACE_CTRL_STR_RESOURCE     0x01000000
#define TRACE_CTRL_SEQ_FLOW         0x02000000
#define TRACE_CTRL_MAX_NUM          3

/* LAYER SPECIFIC                   0x00^^0000*/
#define TRACE_LAYER_MASK            0x00ff0000
#define TRACE_GET_LAYER(x)          ((((UINT32)(x)) & TRACE_LAYER_MASK) >> 16)

#define TRACE_LAYER_NONE            0x00000000
#define TRACE_LAYER_USB             0x00010000
#define TRACE_LAYER_SERIAL          0x00020000
#define TRACE_LAYER_SOCKET          0x00030000
#define TRACE_LAYER_RS232           0x00040000
#define TRACE_LAYER_TRANS_MAX_NUM   5
#define TRACE_LAYER_TRANS_ALL       0x007f0000
#define TRACE_LAYER_LC              0x00050000
#define TRACE_LAYER_LM              0x00060000
#define TRACE_LAYER_HCI             0x00070000
#define TRACE_LAYER_L2CAP           0x00080000
#define TRACE_LAYER_RFCOMM          0x00090000
#define TRACE_LAYER_SDP             0x000a0000
#define TRACE_LAYER_OBEX            0x000b0000
#define TRACE_LAYER_BTM             0x000c0000
#define TRACE_LAYER_GAP             0x000d0000
#define TRACE_LAYER_DUN             0x000e0000
#define TRACE_LAYER_GOEP            0x000f0000
#define TRACE_LAYER_HSP2            0x00100000
#define TRACE_LAYER_SPP             0x00110000
#define TRACE_LAYER_BPP             0x00120000
#define TRACE_LAYER_FTP             0x00130000
#define TRACE_LAYER_OPP             0x00140000
#define TRACE_LAYER_BTU             0x00150000
#define TRACE_LAYER_GKI             0x00160000 /*it's overwritten in nfc_types.h*/
#define TRACE_LAYER_BNEP            0x00170000
#define TRACE_LAYER_PAN             0x00180000
#define TRACE_LAYER_HFP             0x00190000
#define TRACE_LAYER_HID             0x001a0000
#define TRACE_LAYER_BIP             0x001b0000
#define TRACE_LAYER_AVP             0x001c0000
#define TRACE_LAYER_A2D             0x001d0000
#define TRACE_LAYER_SAP             0x001e0000
#define TRACE_LAYER_MCA             0x001f0000
#define TRACE_LAYER_ATT             0x00200000
#define TRACE_LAYER_SMP             0x00210000
#define TRACE_LAYER_NFC             0x00220000
#define TRACE_LAYER_NCI             0x00230000 /*it's overwritten in nfc_types.h*/
#define TRACE_LAYER_LLCP            0x00240000
#define TRACE_LAYER_NDEF            0x00250000
#define TRACE_LAYER_RW              0x00260000
#define TRACE_LAYER_CE              0x00270000
#define TRACE_LAYER_P2P             0x00280000
#define TRACE_LAYER_SNEP            0x00290000
#define TRACE_LAYER_CHO             0x002a0000
#define TRACE_LAYER_NFA             0x002b0000
#define TRACE_LAYER_HAL             0x002c0000 /*it's overwritten in nfc_types.h*/
#define TRACE_LAYER_MAX_NUM         0x002d


/* TRACE_ORIGINATOR                 0x0000^^00*/
#define TRACE_ORG_MASK              0x0000ff00
#define TRACE_GET_ORG(x)            ((((UINT32)(x)) & TRACE_ORG_MASK) >> 8)

#define TRACE_ORG_STACK             0x00000000
#define TRACE_ORG_HCI_TRANS         0x00000100
#define TRACE_ORG_PROTO_DISP        0x00000200
#define TRACE_ORG_RPC               0x00000300
#define TRACE_ORG_GKI               0x00000400
#define TRACE_ORG_APPL              0x00000500
#define TRACE_ORG_SCR_WRAPPER       0x00000600
#define TRACE_ORG_SCR_ENGINE        0x00000700
#define TRACE_ORG_USER_SCR          0x00000800
#define TRACE_ORG_TESTER            0x00000900
#define TRACE_ORG_MAX_NUM           10          /* 32-bit mask; must be < 32 */
#define TRACE_LITE_ORG_MAX_NUM		6
#define TRACE_ORG_ALL               0x03ff
#define TRACE_ORG_RPC_TRANS         0x04

#define TRACE_ORG_REG               0x00000909
#define TRACE_ORG_REG_SUCCESS       0x0000090a

/* TRACE_TYPE                       0x000000^^*/
#define TRACE_TYPE_MASK             0x000000ff
#define TRACE_GET_TYPE(x)           (((UINT32)(x)) & TRACE_TYPE_MASK)

#define TRACE_TYPE_ERROR            0x00000000
#define TRACE_TYPE_WARNING          0x00000001
#define TRACE_TYPE_API              0x00000002
#define TRACE_TYPE_EVENT            0x00000003
#define TRACE_TYPE_DEBUG            0x00000004
#define TRACE_TYPE_STACK_ONLY_MAX   TRACE_TYPE_DEBUG
#define TRACE_TYPE_TX               0x00000005
#define TRACE_TYPE_RX               0x00000006
#define TRACE_TYPE_DEBUG_ASSERT     0x00000007
#define TRACE_TYPE_GENERIC          0x00000008
#define TRACE_TYPE_REG              0x00000009
#define TRACE_TYPE_REG_SUCCESS      0x0000000a
#define TRACE_TYPE_CMD_TX           0x0000000b
#define TRACE_TYPE_EVT_TX           0x0000000c
#define TRACE_TYPE_ACL_TX           0x0000000d
#define TRACE_TYPE_CMD_RX           0x0000000e
#define TRACE_TYPE_EVT_RX           0x0000000f
#define TRACE_TYPE_ACL_RX           0x00000010
#define TRACE_TYPE_TARGET_TRACE     0x00000011
#define TRACE_TYPE_SCO_TX           0x00000012
#define TRACE_TYPE_SCO_RX           0x00000013


#define TRACE_TYPE_MAX_NUM          20
#define TRACE_TYPE_ALL              0xffff

/* Define color for script type */
#define SCR_COLOR_DEFAULT       0
#define SCR_COLOR_TYPE_COMMENT  1
#define SCR_COLOR_TYPE_COMMAND  2
#define SCR_COLOR_TYPE_EVENT    3
#define SCR_COLOR_TYPE_SELECT   4

/* Define protocol trace flag values */
#define SCR_PROTO_TRACE_HCI_SUMMARY 0x00000001
#define SCR_PROTO_TRACE_HCI_DATA    0x00000002
#define SCR_PROTO_TRACE_L2CAP       0x00000004
#define SCR_PROTO_TRACE_RFCOMM      0x00000008
#define SCR_PROTO_TRACE_SDP         0x00000010
#define SCR_PROTO_TRACE_OBEX        0x00000040
#define SCR_PROTO_TRACE_OAPP        0x00000080 /* OBEX Application Profile */
#define SCR_PROTO_TRACE_BNEP        0x00000200
#define SCR_PROTO_TRACE_AVP         0x00000400
#define SCR_PROTO_TRACE_MCA         0x00000800
#define SCR_PROTO_TRACE_ATT         0x00001000
#define SCR_PROTO_TRACE_SMP         0x00002000
#define SCR_PROTO_TRACE_NCI         0x00004000
#define SCR_PROTO_TRACE_LLCP        0x00008000
#define SCR_PROTO_TRACE_NDEF        0x00010000
#define SCR_PROTO_TRACE_RW          0x00020000
#define SCR_PROTO_TRACE_CE          0x00040000
#define SCR_PROTO_TRACE_SNEP        0x00080000
#define SCR_PROTO_TRACE_CHO         0x00100000
#define SCR_PROTO_TRACE_ALL         0x001fffff
#define SCR_PROTO_TRACE_HCI_LOGGING_VSE 0x0800 /* Brcm vs event for logmsg and protocol traces */

#define MAX_SCRIPT_TYPE             5

/* Define PSMs HID uses */
#define HID_PSM_CONTROL         BT_PSM_HIDC
#define HID_PSM_INTERRUPT       BT_PSM_HIDI

#if defined(UCD_HID_INCLUDED) && (UCD_HID_INCLUDED == TRUE)
#define UCD_PSM_MIN     0x8001
#define UCD_PSM_MAX     0x8003
#define UCD_PSM_HID_CTRL 0x8001
#define UCD_PSM_HID_INTR 0x8003
#endif

#if defined(HIDH_UCD_INCLUDED) && (HIDH_UCD_INCLUDED == TRUE)
#define HID_UCD_PSM_CONTROL 0x8001
#define HID_UCD_PSM_INTERRUPT 0x8003
#endif

/* Define a function for logging */
typedef void (BT_LOG_FUNC) (int trace_type, const char *fmt_str, ...);

#endif
