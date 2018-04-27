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

#include <sam.h>

#include "sam_ba_monitor.h"
#include "sam_ba_uart.h"

#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
/* Initialize structures with function pointers from supported interfaces */
t_monitor_if uart_if =
{
  .put_c =       uart_putc,
  .get_c =       uart_getc,
  .is_rx_ready = uart_is_rx_ready,
  .putdata =     uart_putdata,
  .getdata =     uart_getdata,
  .putdata_xmd = uart_putdata_xmd,
  .getdata_xmd = uart_getdata_xmd
};
#endif
Uart * serial;
void uart_setup(Uart &Myserial)
{
  serial=&Myserial;
}

/* Local reference to current Usart instance in use with this driver */
//struct usart_module usart_sam_ba;

/* Variable to let the main task select the appropriate communication interface */
volatile uint8_t uart_b_sharp_received;

/* RX and TX Buffers + rw pointers for each buffer */
volatile uint8_t buffer_rx_usart[USART_BUFFER_SIZE];

volatile uint8_t uart_idx_rx_read;
volatile uint8_t uart_idx_rx_write;

volatile uint8_t uart_buffer_tx_usart[USART_BUFFER_SIZE];

volatile uint8_t uart_idx_tx_read;
volatile uint8_t uart_idx_tx_write;

/* Test for timeout in AT91F_GetChar */
uint8_t uart_error_timeout;
uint16_t uart_size_of_data;
uint8_t uart_mode_of_transfer;


//#define Serial ?????

/**
 * \brief Open the given USART
 */
void uart_open(unsigned int fBaudSpeed)
{
	serial->begin(fBaudSpeed);
	uart_getc();//wait until 1st exchange
	uart_putc('B');
	uart_putc('O');
	uart_putc('O');
	uart_putc('T');
	uart_error_timeout = 0;
}
/**
 * \brief Close communication line
 */
void uart_close(void)
{
	serial->end();	
}

/**
 * \brief Puts a byte on usart line
 * The type int is used to support printf redirection from compiler LIB.
 *
 * \param value      Value to put
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
int uart_putc(int value)
{	
	serial->write( (uint8_t)value);
	return 1;
}

int uart_getc(void)
{
	uint16_t retval;
	//Wait until input buffer is filled
	while(!(uart_is_rx_ready()));
	retval = serial->read();
	//usart_read_wait(&usart_sam_ba, &retval);
	return (int)retval;

}

int uart_sharp_received(void)
{
	if (uart_is_rx_ready())
  {
		if (uart_getc() == SHARP_CHARACTER)
			return (true);
	}
	return (false);
}

bool uart_is_rx_ready(void)
{
	return (serial->available());
}



int uart_readc(void)
{
	int retval;
	retval = buffer_rx_usart[uart_idx_rx_read];
	uart_idx_rx_read = (uart_idx_rx_read + 1) & (USART_BUFFER_SIZE - 1);
	return (retval);
}

//Send given data (polling)
uint32_t uart_putdata(void const* data, uint32_t length)
{
	uint32_t i;
	uint8_t* ptrdata;
	ptrdata = (uint8_t*) data;
	for (i = 0; i < length; i++)
  {
		uart_putc(*ptrdata);
		ptrdata++;
	}
	return (i);
}

//Get data from comm. device
uint32_t uart_getdata(void* data, uint32_t length)
{
	uint8_t* ptrdata;
	ptrdata = (uint8_t*) data;
	*ptrdata = uart_getc();
	return (1);
}

static const uint16_t crc16Table[256]=
{
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

//*----------------------------------------------------------------------------
//* \brief Compute the CRC
//*----------------------------------------------------------------------------
unsigned short uart_add_crc(char ptr, unsigned short crc)
{
	return (crc << 8) ^ crc16Table[((crc >> 8) ^ ptr) & 0xff];
}

//*----------------------------------------------------------------------------
//* \brief
//*----------------------------------------------------------------------------
static uint16_t getbytes(uint8_t *ptr_data, uint16_t length)
{
	uint16_t crc = 0;
	uint16_t cpt;
	uint8_t c;

	for (cpt = 0; cpt < length; ++cpt)
  {
		c = uart_getc();
		if (uart_error_timeout)
			return 1;
		crc = uart_add_crc(c, crc);
		//crc = (crc << 8) ^ xcrc16tab[(crc>>8) ^ c];
		if (uart_size_of_data || uart_mode_of_transfer)
    {
			*ptr_data++ = c;
			if (length == PKTLEN_128)
				uart_size_of_data--;
		}
	}

	return crc;
}

//*----------------------------------------------------------------------------
//* \brief Used by Xup to send packets.
//*----------------------------------------------------------------------------
static int putPacket(uint8_t *tmppkt, uint8_t sno)
{
	uint32_t i;
	uint16_t chksm;
	uint8_t data;

	chksm = 0;

	uart_putc(SOH);

	uart_putc(sno);
	uart_putc((uint8_t) ~(sno));

	for (i = 0; i < PKTLEN_128; i++)
  {
		if (uart_size_of_data || uart_mode_of_transfer)
    {
			data = *tmppkt++;
			uart_size_of_data--;
		}
    else
			data = 0x00;

		uart_putc(data);

		//chksm = (chksm<<8) ^ xcrc16tab[(chksm>>8)^data];
		chksm = uart_add_crc(data, chksm);
	}

	/* An "endian independent way to extract the CRC bytes. */
	uart_putc((uint8_t) (chksm >> 8));
	uart_putc((uint8_t) chksm);

	return (uart_getc()); /* Wait for ack */
}

//*----------------------------------------------------------------------------
//* \brief Called when a transfer from target to host is being made (considered
//*        an upload).
//*----------------------------------------------------------------------------
//Send given data (polling) using xmodem (if necessary)
uint32_t uart_putdata_xmd(void const* data, uint32_t length)
{
	uint8_t c, sno = 1;
	uint8_t done;
	uint8_t * ptr_data = (uint8_t *) data;
	uart_error_timeout = 0;
	if (!length)
		uart_mode_of_transfer = 1;
	else
  {
		uart_size_of_data = length;
		uart_mode_of_transfer = 0;
	}

	if (length & (PKTLEN_128 - 1))
  {
		length += PKTLEN_128;
		length &= ~(PKTLEN_128 - 1);
	}

	/* Startup synchronization... */
	/* Wait to receive a NAK or 'C' from receiver. */
	done = 0;
	while (!done) {
		c = (uint8_t) uart_getc();
		if (uart_error_timeout)
    { // Test for timeout in uart_getc
			uart_error_timeout = 0;
			c = (uint8_t) uart_getc();
			if (uart_error_timeout)
      {
				uart_error_timeout = 0;
				return (0);
			}
		}
		switch (c)
    {
      case NAK:
        done = 1;
        // ("CSM");
			break;
      case 'C':
        done = 1;
        // ("CRC");
			break;
      case 'q': /* ELS addition, not part of XMODEM spec. */
        return (0);
      default:
			break;
		}
	}

	done = 0;
	sno = 1;
	while (!done)
  {
		c = (uint8_t) putPacket((uint8_t *) ptr_data, sno);
		if (uart_error_timeout)
    { // Test for timeout in uart_getc
			uart_error_timeout = 0;
			return (0);
		}
		switch (c)
    {
      case ACK:
        ++sno;
        length -= PKTLEN_128;
        ptr_data += PKTLEN_128;
        // ("A");
			break;

      case NAK:
        // ("N");
			break;

      case CAN:
      case EOT:
      default:
        done = 0;
			break;
		}

		if (!length)
    {
			uart_putc(EOT);
			uart_getc(); /* Flush the ACK */
			break;
		}
		// ("!");
	}

	uart_mode_of_transfer = 0;
	// ("Xup_done.");
	return (1);
	//    return(0);
}

/*----------------------------------------------------------------------------
 * \brief Used by uart_getdata_xmd to retrieve packets.
 */
static uint8_t uart_getPacket(uint8_t *ptr_data, uint8_t sno)
{
	uint8_t seq[2];
	uint16_t crc, xcrc;

	getbytes(seq, 2);
	xcrc = getbytes(ptr_data, PKTLEN_128);
	if (uart_error_timeout)
		return (false);

	/* An "endian independent way to combine the CRC bytes. */
	crc = (uint16_t) uart_getc() << 8;
	crc += (uint16_t) uart_getc();

	if (uart_error_timeout == 1)
		return (false);

	if ((crc != xcrc) || (seq[0] != sno) || (seq[1] != (uint8_t) (~sno)))
  {
		uart_putc(CAN);
		return (false);
	}

	uart_putc(ACK);
	return (true);
}

//*----------------------------------------------------------------------------
//* \brief Called when a transfer from host to target is being made (considered
//*        an download).
//*----------------------------------------------------------------------------
//Get data from comm. device using xmodem (if necessary)
uint32_t uart_getdata_xmd(void* data, uint32_t length)
{
	uint32_t timeout;
	char c;
	uint8_t * ptr_data = (uint8_t *) data;
	uint32_t b_run, nbr_of_timeout = 100;
	uint8_t sno = 0x01;
	uint32_t data_transfered = 0;

	//Copied from legacy source code ... might need some tweaking
	uint32_t loops_per_second = VARIANT_MCK/60;

	uart_error_timeout = 0;

	if (length == 0)
		uart_mode_of_transfer = 1;
	else
  {
		uart_size_of_data = length;
		uart_mode_of_transfer = 0;
	}

	/* Startup synchronization... */
	/* Continuously send NAK or 'C' until sender responds. */
	// ("Xdown");
	while (1)
  {
		uart_putc('C');
		timeout = loops_per_second;
		while (!(uart_is_rx_ready()) && timeout)
			timeout--;
		if (timeout)
			break;

		if (!(--nbr_of_timeout))
			return (0);
//            return -1;
	}

	b_run = true;
	// ("Got response");
	while (b_run != false)
  {
		c = (char) uart_getc();
		if (uart_error_timeout)
    { // Test for timeout in uart_getc
			uart_error_timeout = 0;
			return (0);
//            return (-1);
		}
		switch (c)
    {
      case SOH: /* 128-byte incoming packet */
        // ("O");
        b_run = uart_getPacket(ptr_data, sno);
        if (uart_error_timeout)
        { // Test for timeout in uart_getc
          uart_error_timeout = 0;
          return (0);
  //                return (-1);
        }
        if (b_run == true)
        {
          ++sno;
          ptr_data += PKTLEN_128;
          data_transfered += PKTLEN_128;
        }
			break;
      case EOT: // ("E");
        uart_putc(ACK);
        b_run = false;
			break;
      case CAN: // ("C");
      case ESC: /* "X" User-invoked abort */
      default:
        b_run = false;
			break;
		}
		// ("!");
	}
	uart_mode_of_transfer = 0;
	return (true);
//    return(b_run);
}

