/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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
/*
********************************************************************
*    File Name: spar_setup.c
*
*           The Stackable Patch and Application Runtime
*
*    Abstract: C-runtime setup of this SPAR tier
*
********************************************************************
*/

#include "spar_utils.h"
#include "sparcommon.h"
#include "string.h"
#include "bt_types.h"

/*****************************************************************
*   External definitions
*
*****************************************************************/
extern uint8_t* g_dynamic_memory_MinAddress;
extern uint8_t* g_aon_memory_manager_MinAddress;
extern void install_libs(void);
extern void application_start( void );
extern void wiced_platform_init(void);

//redefined here by refer to 20719B1.
//#define micro_bcsIsNormalModeTransition()     boot_isWarmboot()
__attribute__((section(".app_init_code"))) void application_start_internal( void )
{
    wiced_platform_init();
    application_start();
}

/*****************************************************************
 *   Function: spar_setup()
 *
 *   Abstract: Process the information in .secinfo, copying and
 *   clearing sections as needed.
 *
 *
 *****************************************************************/
#ifndef __GNUC__
#pragma arm section code = "spar_setup"
void SPAR_CRT_SETUP(void)
{
    typedef struct
    {
        uint32_t source;
        uint32_t target;
        uint32_t len;
    } armlink_copy_secinfo_t;

    extern void *_tx_initialize_unused_memory;
    extern uint32_t Region$$Table$$Base;
    extern uint32_t Region$$Table$$Limit;

    extern uint32_t Image$$SPAR_DRAM_ZI_AREA$$ZI$$Base;
    extern uint32_t Image$$SPAR_DRAM_ZI_AREA$$ZI$$Length;
    extern uint32_t Image$$first_free_section_in_spar_NV_RAM$$Base;

    armlink_copy_secinfo_t *cpysecinfo;

    // Get the section info base of this spar slice
    uint32_t cpysecinfobase = (uint32_t)&Region$$Table$$Base;
    uint32_t cpysecinfolim = (uint32_t)&Region$$Table$$Limit;

    uint32_t clrsecbase = (uint32_t)&Image$$SPAR_DRAM_ZI_AREA$$ZI$$Base;
    uint32_t clrseclen = (uint32_t)&Image$$SPAR_DRAM_ZI_AREA$$ZI$$Length;
    uint32_t endofspar = (uint32_t)&Image$$first_free_section_in_spar_NV_RAM$$Base;

    // Here we ought to assert that we're linked against the right
    // image, before we call memcpy/memset in ROM/Flash...

    if(cpysecinfobase != cpysecinfolim)
    {
        // Section info length is not zero
        // which means that there is RW data
        cpysecinfo = (armlink_copy_secinfo_t *)cpysecinfobase;
        memcpy((void *)cpysecinfo->target, (void *)cpysecinfo->source, cpysecinfo->len);
    }

    // Clear ZI section
    if(clrseclen != 0)
        memset((void *)clrsecbase, 0x00, clrseclen);

    // And move avail memory above this spar if required
    // Note that if there are other spars will be placed with minimum
    // alignment (because of the linker option to IRAM_SPAR_BEGIN) and itself
    // is responsible for moving the avail mem ptr.
    g_dynamic_memory_MinAddress = (char *)endofspar;

	// Install included libraries and patches if any
	install_libs();

    // Setup the application start function.
	wiced_bt_set_app_start_function(application_start_internal);
}
#pragma arm section code

#else

/* WEAK application pre_init function. Will be called if not defined anywhere else */
__attribute__((weak))
void wiced_memory_pre_init(uint32_t enable)
{

}

uint8_t g_wiced_memory_pre_init_enable __attribute__((weak)) = 0;

__attribute__ ((section(".spar_setup")))
void SPAR_CRT_SETUP(void)
{
    extern void* spar_iram_bss_begin;
    extern unsigned spar_iram_data_length, spar_iram_bss_length;
    extern void* spar_irom_data_begin, *spar_iram_data_begin, *spar_iram_end, *aon_iram_end;

    // Initialize initialized data if .data length is non-zero and it needs to be copied from NV to RAM.

        memcpy((void*)&spar_iram_data_begin, (void*)&spar_irom_data_begin, (uint32_t)&spar_iram_data_length);

    // // Clear the ZI section
    if((uint32_t)&spar_iram_bss_length != 0)
    {
        memset((void*)&spar_iram_bss_begin, 0x00, (uint32_t)&spar_iram_bss_length);
    }

    // And move avail memory above this spar if required
    // Note that if there are other spars will be placed with minimum
    // alignment (because of the linker option to IRAM_SPAR_BEGIN) and itself
    // is responsible for moving the avail mem ptr.
    g_dynamic_memory_MinAddress = (uint8_t *)(((uint32_t)&spar_iram_end + 32) & 0xFFFFFFF0);
#if 0 // TODO:
    if( !micro_bcsIsNormalModeTransition() )
    {
        g_aon_memory_manager_MinAddress = (char*)(&aon_iram_end);
    }
#endif
    // default attach point for jtag if ENABLE_DEBUG
    SETUP_APP_FOR_DEBUG_IF_DEBUG_ENABLED();
    BUSY_WAIT_TILL_MANUAL_CONTINUE_IF_DEBUG_ENABLED();

    // Install included libraries and patches if any
    install_libs();

    // Setup the application start function.
    wiced_bt_app_pre_init = application_start_internal;

    // Call Memory pre-initialization function (either the weak or the real (lib) one)
    #if ENABLE_MEMORY_PRE_INIT
    g_wiced_memory_pre_init_enable = 1;
    #endif
    wiced_memory_pre_init(g_wiced_memory_pre_init_enable);
}

#endif
