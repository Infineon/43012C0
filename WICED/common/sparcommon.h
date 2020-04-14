/*******************************************************************************
 * All common definitions for this SPAR
 *******************************************************************************/

#ifndef _SPAR_COMMON_H_
#define _SPAR_COMMON_H_

#include "wiced.h"

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C extern
#endif

EXTERN_C void application_start( void );

extern void (*wiced_bt_app_pre_init)(void );

#define APPLICATION_START() __attribute__((section(".app_init_code"))) \
void application_start( void )

// TODO: Other technologies.

#endif
