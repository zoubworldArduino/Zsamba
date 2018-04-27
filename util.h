/*
 * Copyright (c) 2015 Arduino LLC.  All right reserved.
 * Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
 * Copyright (c) 2017 MattairTech LLC. All right reserved.
 * Copyright (C) 2014, ChaN, all right reserved.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _UTIL_H_
#define _UTIL_H_

#include "sam.h"
#include <stdbool.h>
//#include "board_definitions/board_definitions.h"

extern unsigned int s_fcpu_hz;
#define SYSTICK_NUMBER_CYCLE 1
#ifdef __ICCARM__
/*! \name Compiler Keywords
 *
 * Port of some keywords from GCC to IAR Embedded Workbench.
 */
//! @{
#define __asm__              asm
#define __inline__           inline
#define __volatile__
//! @}
#endif

#define TRUE (1==1)
#define FALSE (1==0)

// TODO: Variable bootloader sizes
#if (SAMD11)
  #define APP_START 0x00001000
#else
  #define APP_START 0x00002000
#endif
#ifdef __ICCARM__
#pragma location = "__OUT_SEGMENT"
#pragma object_attribute = __root
extern __no_init const uint32_t   __sketch_vectors_ptr; // Exported value from linker script
#else
extern uint32_t __sketch_vectors_ptr; // Exported value from linker script
#endif
#define SCB_AIRCR_VECTKEY_Val   0x05FA

void flashErase (uint32_t startAddress);
void flashWrite (uint32_t startAddress, uint32_t * buffer, uint32_t * ptr_data);
void pinMux (uint32_t pinmux);
void systemReset (void);
void pinConfig (uint8_t port, uint8_t pin, uint8_t config);
bool isPinActive (uint8_t port, uint8_t pin, uint8_t config);
void delayUs (unsigned int delay);
void waitForSync (void);


#define INPUT                   (0x0)
#define OUTPUT                  (0x1)
#define INPUT_PULLUP            (0x2)
#define INPUT_PULLDOWN          (0x3)
#define OUTPUT_LOW              (0x4)
#define OUTPUT_HIGH             (0x5)

#define PINMUX_UNUSED          0xFFFFFFFF

#define LED_POLARITY_LOW_ON             0
#define LED_POLARITY_HIGH_ON            1
#define PIN_POLARITY_ACTIVE_LOW         0
#define PIN_POLARITY_ACTIVE_HIGH        1
#define PIN_POLARITY_USBCDC_LOW         0
#define PIN_POLARITY_USBCDC_HIGH        1

#define USB_VID_HIGH   0x23
#define USB_VID_LOW    0x41
#define USB_PID_HIGH   0x00
#define USB_PID_LOW    0x4D
#endif
