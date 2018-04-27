/*
 * Copyright (c) 2015 Arduino LLC.  All right reserved.
 * Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
 * Copyright (c) 2017 MattairTech LLC. All right reserved.
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


#include "Arduino.h"
#include "util.h"
#ifdef __ICCARM__
#pragma location = "__OUT_SEGMENT"
#pragma object_attribute = __root
__no_init const uint32_t   __sketch_vectors_ptr; // Exported value from linker script

#else
extern uint32_t __sketch_vectors_ptr; // Exported value from linker script
#endif
#if defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
#pragma location="CODE32"
#pragma object_attribute=__root
#endif
/*
 * The SAMD21 has a default 1MHz clock @ reset.
 * SAML21 and SAMC21 have a default 4MHz clock @ reset.
 * It is switched to 48MHz in board_init.c
 */
#if (SAMD21 || SAMD20 || SAMD11)
  #define CLOCK_DEFAULT 1000000ul
#elif (SAML21 || SAMC21 || SAMC20)
  #define CLOCK_DEFAULT 4000000ul
#endif
//uint32_t SystemCoreClock=CLOCK_DEFAULT;

void flashErase (uint32_t startAddress)
{
  // Syntax: X[ADDR]#
  // Erase the flash memory starting from ADDR to the end of flash.

  // Note: the flash memory is erased in ROWS, that is in block of 4 pages.
  //       Even if the starting address is the last byte of a ROW the entire
  //       ROW is erased anyway.

  uint32_t dst_addr = startAddress; // starting address

  while (dst_addr < FLASH_SIZE)
  {
    // Execute "ER" Erase Row
    NVMCTRL->ADDR.reg = dst_addr / 2;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    while (NVMCTRL->INTFLAG.bit.READY == 0)
      ;
    dst_addr += FLASH_PAGE_SIZE * 4; // Skip a ROW
  }
}

void flashWrite(uint32_t numBytes, uint32_t * buffer, uint32_t * ptr_data)
{
  // This command writes the content of a buffer in SRAM into flash memory.

  // Syntax: Y[ADDR],0#
  // Set the starting address of the SRAM buffer.

  // Syntax: Y[ROM_ADDR],[SIZE]#
  // Write the first SIZE bytes from the SRAM buffer (previously set) into
  // flash memory starting from address ROM_ADDR

  // Write to flash
  uint32_t size = numBytes/4;
  uint32_t *src_addr = buffer;
  uint32_t *dst_addr = ptr_data;

  // Do writes in pages
  while (size)
  {
    // Execute "PBC" Page Buffer Clear
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
    while (NVMCTRL->INTFLAG.bit.READY == 0)
      ;

    // Fill page buffer
    uint32_t i;
    for (i=0; i<(FLASH_PAGE_SIZE/4) && i<size; i++)
    {
      dst_addr[i] = src_addr[i];
    }

    // Execute "WP" Write Page
    //NVMCTRL->ADDR.reg = ((uint32_t)dst_addr) / 2;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
    while (NVMCTRL->INTFLAG.bit.READY == 0)
      ;

    // Advance to next page
    dst_addr += i;
    src_addr += i;
    size     -= i;
  }
}

void pinMux (uint32_t pinmux)
{
  uint32_t port;
  uint32_t pin;

  if (pinmux != PINMUX_UNUSED)
  {
    /* Mask 6th bit in pin number to check whether it is greater than 32 i.e., PORTB pin */
    port = (pinmux & 0x200000) >> 21;
    pin = (pinmux >> 16);
    PORT->Group[port].PINCFG[(pin - (port*32))].bit.PMUXEN = 1;
    PORT->Group[port].PMUX[(pin - (port*32))/2].reg &= ~(0xF << (4 * (pin & 0x01u)));
    PORT->Group[port].PMUX[(pin - (port*32))/2].reg |= (pinmux & 0xFF) << (4 * (pin & 0x01u));
  }
}

void pinConfig (uint8_t port, uint8_t pin, uint8_t config)
{
  uint8_t pinCfg = PORT_PINCFG_INEN;

  if (config == INPUT_PULLUP || config == INPUT_PULLDOWN  || config == INPUT) {
    PORT->Group[port].DIRCLR.reg = (1 << pin);
    if (config == INPUT_PULLUP) {
      pinCfg |= PORT_PINCFG_PULLEN;
      PORT->Group[port].OUTSET.reg = (1 << pin);
    } else if (config == INPUT_PULLDOWN) {
      pinCfg |= PORT_PINCFG_PULLEN;
      PORT->Group[port].OUTCLR.reg = (1 << pin);
    }
  } else {
    if (config == OUTPUT_HIGH) {
      PORT->Group[port].OUTSET.reg = (1 << pin);
    } else if (config == OUTPUT_LOW) {
      PORT->Group[port].OUTCLR.reg = (1 << pin);
    }
    PORT->Group[port].DIRSET.reg = (1 << pin);
  }

  PORT->Group[port].PINCFG[pin].reg = (uint8_t)pinCfg;
}

bool isPinActive (uint8_t port, uint8_t pin, uint8_t config)
{
  uint32_t pinState = ((PORT->Group[port].IN.reg) & (1 << pin));
  if (config == PIN_POLARITY_ACTIVE_LOW) {
    if (!pinState) return(true);
  } else {
    if (pinState) return(true);
  }
  return(false);
}

#define Get_sys_count()     ( (SysTick->VAL)  & SysTick_VAL_CURRENT_Msk       )

void delayUs (unsigned int  delay)
{
   SysTick->CTRL  |= SysTick_CTRL_CLKSOURCE_Msk |
                    
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
   
  // The SAMD21 has a default 1MHz clock @ reset.
   // SAML21 and SAMC21 have a default 4MHz clock @ reset.
   // It is switched to 48MHz in board_init.c
   //
  volatile uint32_t numLoops;

signed int delta=(((long)delay*(long)SystemCoreClock)/(long)1000000);
/*
signed int now=Get_sys_count();
signed int time=now;
time-=delta;

while(time<0)
{
	
	while(now>(SysTick->LOAD/2))
          now=Get_sys_count();
	while(now<(SysTick->LOAD-50))
          now=Get_sys_count();// wait an over flow	
	time+=SysTick->LOAD;
}
while(now>time)
  now=Get_sys_count();// wait an over flow	
*/
	
  if (SystemCoreClock == VARIANT_MCK) {
    numLoops = (12 * delay);
  } else {
#if (SAMD21 || SAMD11)
    numLoops = (delay >> 2);
#elif (SAML21 || SAMC21 || SAMC20)
    numLoops = delay;
#endif
  }

  for (volatile uint32_t i=0; i < numLoops; i++) // 10ms 
    // force compiler to not optimize this... 
     __asm__ __volatile__("nop");
}

/*
uint32_t us2CyFactor = 0;
unsigned int s_fcpu_hz=VARIANT_MCK;
//initialize time base




void delay_init(unsigned long fcpu_hz){
  s_fcpu_hz = fcpu_hz;
  SysTick->LOAD= SysTick_LOAD_RELOAD_Msk;
  SysTick->CTRL    = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; //! ENABLE Enables the counter &   CLKSOURCE Indicates the clock source: 1 processor clock.
  SysTick->VAL=0;

//SGU    us2CyFactor=   ((unsigned long long)fcpu_hz*1024)  / 1000000 /SYSTICK_NUMBER_CYCLE ;
  us2CyFactor = fcpu_hz;
  us2CyFactor /= 1000;
  us2CyFactor *= 1024;
  us2CyFactor /= 1000;
  us2CyFactor /= SYSTICK_NUMBER_CYCLE;
}
// delay to wait (error about +100s at 10Mhz   and can't manage delay bealow 300ms at 50Mhz; 100ms at 150Mhz

void delayMs(unsigned long delay){
   // Assert(delay < 64);
    
   while ( (cpu_ms_2_cy_fast( delay ) >SysTick_LOAD_RELOAD_Msk))
   {
      cpu_delay_ms(50, s_fcpu_hz);delay-=50;
   }
   cpu_delay_ms(delay, s_fcpu_hz);
}

// delay to wait (error about +100s at 10Mhz    and can't manage delay below 100s

void delayUs(unsigned long delay){
   // Assert(delay <= 0xFFFF);
   cpu_delay_us(delay, s_fcpu_hz);

}


void delay_cy(unsigned long delay){
  cpu_delay_cy(delay);
}

*/
void systemReset (void)
{
  /* Request a system reset */
  SCB->AIRCR = (uint32_t)((1 << SCB_AIRCR_SYSRESETREQ_Pos) | (SCB_AIRCR_VECTKEY_Val << SCB_AIRCR_VECTKEY_Pos));

  while(1);
}
//cpp version
void waitForSync (void)
{
  #if (SAMD)
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
  #elif (SAML21 || SAMC21 || SAMC20)
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );
  #endif
}
