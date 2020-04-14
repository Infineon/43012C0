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
 * This file provided functionality to provide debug traces
 *
 */


#ifndef __TFP_PRINTF__
#define __TFP_PRINTF__

#include <stdarg.h>

int wiced_printf(char * buf, int len, ...);


#endif
