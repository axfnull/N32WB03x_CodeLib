/**
 ****************************************************************************************
 *
 * @file rvds/compiler.h
 *
 * @brief Definitions of compiler specific directives.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _COMPILER_H_
#define _COMPILER_H_

#ifdef __ARMCC_VERSION
#elif defined(__ICCARM__)
#elif defined (__GNUC__)
#else
#error "File only included with RVDS!"
#endif // __ARMCC_VERSION
#include "core_cm0.h"

/// define the static keyword for this compiler
#define __STATIC static

/// define the force inlining attribute for this compiler
//#define __INLINE __forceinline static

/// define the IRQ handler attribute for this compiler
#define __IRQ __irq

/// define the BLE IRQ handler attribute for this compiler
#define __BTIRQ

/// define the BLE IRQ handler attribute for this compiler
#define __BLEIRQ

/// define the FIQ handler attribute for this compiler
#define __FIQ __irq

/// define size of an empty array (used to declare structure with an array size not defined)
#define __ARRAY_EMPTY

/// Put a variable in a memory maintained during deep sleep
#define __LOWPOWER_SAVED

#endif // _COMPILER_H_
