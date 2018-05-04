/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "sam_ba_monitor.h"
#include "sam_ba_uart.h"
#include "sam_ba_wire.h"
#include "sam_ba_cdc.h"
#include "spi.h"

const char RomBOOT_Version[] = SAM_BA_VERSION;
#if (defined ARDUINO_EXTENDED_CAPABILITIES) && (ARDUINO_EXTENDED_CAPABILITIES == 1)
const char RomBOOT_ExtendedCapabilities[] = "[Arduino:XYZ]";
#endif






/* The pointer to the interface object use by the monitor */
t_monitor_if * ptr_monitor_if;

/* b_terminal_mode mode (ascii) or hex mode */
#if defined(TERMINAL_MODE_ENABLED)
volatile bool b_terminal_mode = false;
#endif
volatile bool b_sam_ba_interface_usart = false;
volatile bool b_sam_ba_interface_wire = false;

/* Pulse generation counters to keep track of the time remaining for each pulse type */
#define TX_RX_LED_PULSE_PERIOD 100
#if defined(BOARD_LEDTX_PORT)
volatile uint16_t txLEDPulse = 0; // time remaining for Tx LED pulse
#endif
#if defined(BOARD_LEDRX_PORT)
volatile uint16_t rxLEDPulse = 0; // time remaining for Rx LED pulse
#endif

void sam_ba_monitor_init(uint8_t com_interface)
{
#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  //Selects the requested interface for future actions
  if (com_interface == SAM_BA_INTERFACE_USART)
  {
    ptr_monitor_if = (t_monitor_if*) &uart_if;
    b_sam_ba_interface_usart = true;
  }
#endif
  #if SAM_BA_INTERFACE == SAM_BA_WIRE_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  //Selects the requested interface for future actions
  if (com_interface == SAM_BA_INTERFACE_WIRE)
  {
    ptr_monitor_if = (t_monitor_if*) &wire_if;
    b_sam_ba_interface_wire = true;
  }
#endif
#if defined(USBCON)
  #if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    if (com_interface == SAM_BA_INTERFACE_USBCDC)
    {
      ptr_monitor_if = (t_monitor_if*) &usbcdc_if;
    }
  #endif
#endif
}

/*
 * Central SAM-BA monitor putdata function using the board LEDs
 */
static uint32_t sam_ba_putdata(t_monitor_if* pInterface, void const* data, uint32_t length)
{
	uint32_t result ;

	result=pInterface->putdata(data, length);
#if defined(BOARD_LEDTX_PORT)

	LEDTX_on();
	txLEDPulse = TX_RX_LED_PULSE_PERIOD;
#endif

	return result;
}

/*
 * Central SAM-BA monitor getdata function using the board LEDs
 */
static uint32_t sam_ba_getdata(t_monitor_if* pInterface, void* data, uint32_t length)
{
	uint32_t result ;

	result=pInterface->getdata(data, length);
#if defined(BOARD_LEDRX_PORT)

	if (result)
	{
		LEDRX_on();
		rxLEDPulse = TX_RX_LED_PULSE_PERIOD;
	}
#endif

	return result;
}

/*
 * Central SAM-BA monitor putdata function using the board LEDs
 */
static uint32_t sam_ba_putdata_xmd(t_monitor_if* pInterface, void const* data, uint32_t length)
{
	uint32_t result ;

	result=pInterface->putdata_xmd(data, length);
#if defined(BOARD_LEDTX_PORT)

	LEDTX_on();
	txLEDPulse = TX_RX_LED_PULSE_PERIOD;
#endif

	return result;
}

/*
 * Central SAM-BA monitor getdata function using the board LEDs
 */
static uint32_t sam_ba_getdata_xmd(t_monitor_if* pInterface, void* data, uint32_t length)
{
	uint32_t result ;

	result=pInterface->getdata_xmd(data, length);
#if defined(BOARD_LEDRX_PORT)

	if (result)
	{
		LEDRX_on();
		rxLEDPulse = TX_RX_LED_PULSE_PERIOD;
	}
#endif

	return result;
}

/**
 * \brief This function allows data emission by USART
 *
 * \param *data  Data pointer
 * \param length Length of the data
 */
void sam_ba_putdata_term(uint8_t* data, uint32_t length)
{
  #if defined(TERMINAL_MODE_ENABLED)
  uint8_t temp, buf[12], *data_ascii;
  uint32_t i, int_value;

  if (b_terminal_mode)
  {
    if (length == 4)
      int_value = *(uint32_t *) data;
    else if (length == 2)
      int_value = *(uint16_t *) data;
    else
      int_value = *(uint8_t *) data;

    data_ascii = buf + 2;
    data_ascii += length * 2 - 1;

    for (i = 0; i < length * 2; i++)
    {
      temp = (uint8_t) (int_value & 0xf);

      if (temp <= 0x9)
        *data_ascii = temp | 0x30;
      else
        *data_ascii = temp + 0x37;

      int_value >>= 4;
      data_ascii--;
    }
    buf[0] = '0';
    buf[1] = 'x';
    buf[length * 2 + 2] = '\n';
    buf[length * 2 + 3] = '\r';
    sam_ba_putdata(ptr_monitor_if, buf, length * 2 + 4);
  }
  else
#endif
    sam_ba_putdata(ptr_monitor_if, data, length);
  return;
}

volatile uint32_t sp;
void call_applet(uint32_t address)
{
  uint32_t app_start_address;

  __disable_irq();

  sp = __get_MSP();

  /* Rebase the Stack Pointer */
  __set_MSP(*(uint32_t *) address);

  /* Load the Reset Handler address of the application */
  app_start_address = *(uint32_t *)(address + 4);

  /* Jump to application Reset Handler in the application */
  asm("bx %0"::"r"(app_start_address));
}

uint32_t current_number;
uint32_t  length;
uint8_t command, *ptr_data, *ptr, data[SIZEBUFMAX];
uint8_t j;
uint32_t u32tmp;


// Prints a 32-bit integer in hex.
#if ((SAM_BA_INTERFACE != SAM_BA_NONE) && ((defined(ARDUINO_EXTENDED_CAPABILITIES)) && (ARDUINO_EXTENDED_CAPABILITIES == 1)))
static void put_uint32(uint32_t n)
{
  char buff[8];
  int i;
  for (i=0; i<8; i++)
  {
    int d = n & 0XF;
    n = (n >> 4);

    buff[7-i] = d > 9 ? 'A' + d - 10 : '0' + d;
  }
  sam_ba_putdata( ptr_monitor_if, buff, 8);
}
#endif

static void sam_ba_monitor_loop(void)
{
  length = sam_ba_getdata(ptr_monitor_if, data, SIZEBUFMAX);
  ptr = data;
  int i;
  for (i = 0; i < length; i++, ptr++)
  {
    if (*ptr == 0xff) continue;

    if (*ptr == '#')
    {
      #if defined(TERMINAL_MODE_ENABLED)
      if (b_terminal_mode)
      {
        sam_ba_putdata(ptr_monitor_if, "\n\r", 2);
      }
      #endif
      if (command == 'S')
      {
        //Check if some data are remaining in the "data" buffer
        if(length>i)
        {
          //Move current indexes to next avail data (currently ptr points to "#")
          ptr++;
          i++;

          //We need to add first the remaining data of the current buffer already read from usb
          //read a maximum of "current_number" bytes
          if ((length-i) < current_number)
          {
            u32tmp=(length-i);
          }
          else
          {
            u32tmp=current_number;
          }

          memcpy(ptr_data, ptr, u32tmp);
          i += u32tmp;
          ptr += u32tmp;
          j = u32tmp;
        }
        //update i with the data read from the buffer
        i--;
        ptr--;
        //Do we expect more data ?
        if(j<current_number)
          sam_ba_getdata_xmd(ptr_monitor_if, ptr_data, current_number-j);

        __asm("nop");
      }
      else if (command == 'R')
      {
        sam_ba_putdata_xmd(ptr_monitor_if, ptr_data, current_number);
      }
      else if (command == 'O')
      {
        *ptr_data = (char) current_number;
      }
      else if (command == 'H')
      {
        *((uint16_t *) ptr_data) = (uint16_t) current_number;
      }
      else if (command == 'W')
      {
        *((int *) ptr_data) = current_number;
      }
      else if (command == 'o')
      {
        sam_ba_putdata_term(ptr_data, 1);
      }
      else if (command == 'h')
      {
        current_number = *((uint16_t *) ptr_data);
        sam_ba_putdata_term((uint8_t*) &current_number, 2);
      }
      else if (command == 'w')
      {
        current_number = *((uint32_t *) ptr_data);
        sam_ba_putdata_term((uint8_t*) &current_number, 4);
      }
      else if (command == 'G')
      {
        call_applet(current_number);
        /* Rebase the Stack Pointer */
        __set_MSP(sp);
        __enable_irq();
        if (b_sam_ba_interface_usart|b_sam_ba_interface_wire) {
          ptr_monitor_if->put_c(0x6);
        }
      }
      #if defined(TERMINAL_MODE_ENABLED)
      /*
      gateway
      */
      else if (command == 'g')
      {
        b_terminal_mode = 1;
 // start a gateway from current interface to the one specified
        // It's useful to quickly check if a transfer has been done
        // successfully.

        // Syntax: g[0xmmssaabb]#
        // Returns: g#
        // aa: type of com : wire0  usart1 usart2 SPI4 ....
        // bb :index of feature( PCOM index )
        // ss : slave address in wire
        // mm : master address
        uint32_t start_addr = (uint32_t)ptr_data;
        uint32_t oXaabb = current_number;


        // Send response
        sam_ba_putdata( ptr_monitor_if, "g", 1);
        sam_ba_putdata(ptr_monitor_if, "#\n\r", 2);
        while(1)
        {
          switch((oXaabb>>8) &0xFF)
          {
            case 0:
              P_COM[oXaabb &0xFF].wire.begin(((oXaabb>>24) &0xFF)); 
              // join i2c bus with address #
  
              while(1)
              {
              if (P_COM[oXaabb &0xFF].wire.available())
                ptr_monitor_if->put_c(P_COM[oXaabb &0xFF].wire.read());
              if( ptr_monitor_if->is_rx_ready())
              {
                
                P_COM[oXaabb &0xFF].wire.beginTransmission(((oXaabb>>16) &0xFF)); // transmit to device #4
                P_COM[oXaabb &0xFF].wire.write(ptr_monitor_if->get_c());
                P_COM[oXaabb &0xFF].wire.endTransmission();    // stop transmitting
            }
              }
            break;
            
            case 1:
                P_COM[oXaabb &0xFF].serial.begin(115200);
              while(1)
              {
                if (P_COM[oXaabb &0xFF].serial.available())
                  ptr_monitor_if->put_c(P_COM[oXaabb &0xFF].serial.read());
                if( ptr_monitor_if->is_rx_ready())
                  P_COM[oXaabb &0xFF].serial.write(ptr_monitor_if->get_c());
              }
            break;
            
            case 2:
                P_COM[oXaabb &0xFF].serial2.begin(115200);
              while(1)
              {
              if (P_COM[oXaabb &0xFF].serial2.available())
                ptr_monitor_if->put_c(P_COM[oXaabb &0xFF].serial2.read());
              if( ptr_monitor_if->is_rx_ready())
                P_COM[oXaabb &0xFF].serial2.write(ptr_monitor_if->get_c());
              }
            break;
            
            case 3:
              P_COM[oXaabb &0xFF].spi.begin();
              while(1)
              {/*
              if (P_COM[oXaabb & 0xFF].spi.transfer())
                ptr_monitor_if->put_c(P_COM[oXaabb &0xFF].spi.read());*/
              if( ptr_monitor_if->is_rx_ready())
                ptr_monitor_if->put_c(P_COM[oXaabb & 0xFF].spi.transfer(ptr_monitor_if->get_c()));
              
              }
            break;
            
          
            
          }
        }
      }
      
      else if (command == 'T')
      {
        b_terminal_mode = 1;
        sam_ba_putdata(ptr_monitor_if, "\n\r", 2);
      }
      else if (command == 'N')
      {
        if (b_terminal_mode == 0)
        {
          sam_ba_putdata( ptr_monitor_if, "\n\r", 2);
        }
        b_terminal_mode = 0;
      }
      #endif
      else if (command == 'V')
      {
        sam_ba_putdata( ptr_monitor_if, "v", 1);
        sam_ba_putdata( ptr_monitor_if, (uint8_t *) RomBOOT_Version, strlen(RomBOOT_Version));
#if (defined ARDUINO_EXTENDED_CAPABILITIES) && (ARDUINO_EXTENDED_CAPABILITIES == 1)
        sam_ba_putdata( ptr_monitor_if, " ", 1);
        sam_ba_putdata( ptr_monitor_if, (uint8_t *) RomBOOT_ExtendedCapabilities, strlen(RomBOOT_ExtendedCapabilities));
#endif
        sam_ba_putdata( ptr_monitor_if, " ", 1);
        ptr = (uint8_t*) &(__DATE__);
        i = 0;
        while (*ptr++ != '\0')
          i++;
        sam_ba_putdata( ptr_monitor_if, (uint8_t *) &(__DATE__), i);
        sam_ba_putdata( ptr_monitor_if, " ", 1);
        i = 0;
        ptr = (uint8_t*) &(__TIME__);
        while (*ptr++ != '\0')
          i++;
        sam_ba_putdata( ptr_monitor_if, (uint8_t *) &(__TIME__), i);
        sam_ba_putdata( ptr_monitor_if, "\n\r", 2);
      }
#if (defined ARDUINO_EXTENDED_CAPABILITIES) && (ARDUINO_EXTENDED_CAPABILITIES == 1)
      else if (command == 'X')
      {
        // Syntax: X[ADDR]#
        // Erase the flash memory starting from ADDR to the end of flash.

        // Note: the flash memory is erased in ROWS, that is in block of 4 pages.
        //       Even if the starting address is the last byte of a ROW the entire
        //       ROW is erased anyway.

        flashErase(current_number);

        // Notify command completed
        sam_ba_putdata( ptr_monitor_if, "X\n\r", 3);
      }
      else if (command == 'Y')
      {
        // This command writes the content of a buffer in SRAM into flash memory.

        // Syntax: Y[ADDR],0#
        // Set the starting address of the SRAM buffer.

        // Syntax: Y[ROM_ADDR],[SIZE]#
        // Write the first SIZE bytes from the SRAM buffer (previously set) into
        // flash memory starting from address ROM_ADDR

        static uint32_t *src_buff_addr = NULL;

        if (current_number == 0)
        {
          // Set buffer address
          src_buff_addr = (uint32_t*)ptr_data;
        }
        else
        {
          flashWrite(current_number, src_buff_addr, (uint32_t*)ptr_data);
        }

        // Notify command completed
        sam_ba_putdata( ptr_monitor_if, "Y\n\r", 3);
      }
      else if (command == 'Z')
      {
        // This command calculate CRC for a given area of memory.
        // It's useful to quickly check if a transfer has been done
        // successfully.

        // Syntax: Z[START_ADDR],[SIZE]#
        // Returns: Z[CRC]#

        uint8_t *data = (uint8_t *)ptr_data;
        uint32_t size = current_number;
        uint16_t crc = 0;
        uint32_t i = 0;
        for (i=0; i<size; i++)
          crc = uart_add_crc(*data++, crc);

        // Send response
        sam_ba_putdata( ptr_monitor_if, "Z", 1);
        put_uint32(crc);
        sam_ba_putdata( ptr_monitor_if, "#\n\r", 3);
      }
#endif

      command = 'z';
      current_number = 0;
      #if defined(TERMINAL_MODE_ENABLED)

      if (b_terminal_mode)
      {
        sam_ba_putdata( ptr_monitor_if, ">", 1);
      }
      #endif
    }
    else
    {
      if (('0' <= *ptr) && (*ptr <= '9'))
      {
        current_number = (current_number << 4) | (*ptr - '0');
      }
      else if (('A' <= *ptr) && (*ptr <= 'F'))
      {
        current_number = (current_number << 4) | (*ptr - 'A' + 0xa);
      }
      else if (('a' <= *ptr) && (*ptr <= 'f'))
      {
        current_number = (current_number << 4) | (*ptr - 'a' + 0xa);
      }
      else if (*ptr == ',')
      {
        ptr_data = (uint8_t *) current_number;
        current_number = 0;
      }
      else
      {
        command = *ptr;
        current_number = 0;
      }
    }
  }
}

void sam_ba_monitor_sys_tick(void)
{
	/* Check whether the TX or RX LED one-shot period has elapsed.  if so, turn off the LED */
  #if defined(BOARD_LEDTX_PORT)
	if (txLEDPulse && !(--txLEDPulse))
		LEDTX_off();
  #endif
  #if defined(BOARD_LEDRX_PORT)
	if (rxLEDPulse && !(--rxLEDPulse))
		LEDRX_off();
  #endif
}

/**
 * \brief This function starts the SAM-BA monitor.
 */
void sam_ba_monitor_run(void)
{

  ptr_data = NULL;
  command = 'z';
  while (1)
  {
    sam_ba_monitor_loop();
  }
}
