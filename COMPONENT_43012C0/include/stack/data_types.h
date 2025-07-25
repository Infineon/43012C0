/*
 * Copyright 2016-2025, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include "brcm_fw_types.h"

#ifndef NULL
#define NULL     0
#endif

#ifndef FALSE
#define FALSE  0
#endif

typedef unsigned char   UINT8;
typedef unsigned short  UINT16;

//typedef unsigned long   UINT32;

typedef unsigned long long int UINT64;
//typedef signed   long   INT32;
typedef signed   char   INT8;
typedef signed   short  INT16;
//typedef unsigned char   BOOLEAN;

typedef UINT32          UINTPTR;
typedef UINT32          TIME_STAMP;

#undef BOOLEAN
#define BOOLEAN BOOL32


#ifndef TRUE
#define TRUE   (!FALSE)
#endif

typedef unsigned char   UBYTE;

#ifdef __arm
    #ifdef BYTE_PACKED
        #define PACKED  BYTE_PACKED
    #else
    #define PACKED
    #endif
#else //__arm
#ifndef PACKED
#define PACKED
#endif

#ifndef INLINE
#define INLINE
#endif
#endif // __arm

#ifndef PACKED_X
    #ifdef WICED_X
        /* For WICED-X, define structure alignment (make it same as embedded) */
        #define PACKED_X  __attribute__((packed))
    #else
        #define PACKED_X
    #endif
#endif


#ifndef BIG_ENDIAN
#define BIG_ENDIAN FALSE
#endif

#define UINT16_LOW_BYTE(x)      ((x) & 0xff)
#define UINT16_HI_BYTE(x)       ((x) >> 8)

/* MACRO definitions for safe string functions */
/* Replace standard string functions with safe functions if available */
#define BCM_STRCAT_S(x1,x2,x3)      strcat((x1),(x3))
#define BCM_STRNCAT_S(x1,x2,x3,x4)  strncat((x1),(x3),(x4))
#define BCM_STRCPY_S(x1,x2,x3)      strcpy((x1),(x3))
#define BCM_STRNCPY_S(x1,x2,x3,x4)  strncpy((x1),(x3),(x4))
#define BCM_SPRINTF_S(x1,x2,x3,x4)  sprintf((x1),(x3),(x4))
#define BCM_VSPRINTF_S(x1,x2,x3,x4) vsprintf((x1),(x3),(x4))

/* Timer list entry callback type
*/
//typedef void (TIMER_CBACK)(void *p_tle);
#ifndef TIMER_PARAM_TYPE
#if defined  _WIN32 || defined WICEDX || defined __ANDROID__ || defined __APPLE__
#define TIMER_PARAM_TYPE    void *
#else
#define TIMER_PARAM_TYPE    uint32_t
#endif
#endif

#endif
