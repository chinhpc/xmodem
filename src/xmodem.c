/*
 * Copyright 2001-2021 Georges Menie (www.menie.org)
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* this code needs standard functions memcpy() and memset()
   and input/output functions _inbyte() and _outbyte().

   the prototypes of the input/output functions are:
     int _inbyte(unsigned short timeout); // msec timeout
     void _outbyte(int c);

 */

#define PC_SIDE
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h> /* calloc, exit, free */
#ifdef PC_SIDE
#include <fcntl.h>   /* File Control Definitions           */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <termios.h> /* POSIX Terminal Control Definitions */
#endif

#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x18
#define CTRLZ 0x1A

#define DLY_1S 1000
#define MAXRETRANS 25

#ifdef PC_SIDE
#define XMODEM_DEV_FILE fd
#define XMODEM_RECV_BYTE(c, dev)	SERIAL_RECV(c, dev)
#define XMODEM_SEND_BYTE(c, dev)	SERIAL_SEND(c, dev)
#define OS_SLEEP(ms)	usleep(ms*1000)
static int fd;
#define MAX_FILE_SIZE 10485760
#else
#define XMODEM_DEV_FILE USART1
#define XMODEM_RECV_BYTE(c, dev)	UART_RECV(c, dev)
#define XMODEM_SEND_BYTE(c, dev)	UART_SEND(c, dev)
#define OS_SLEEP(ms)	vTaskDelay(ms / portTICK_PERIOD_MS)
#define MAX_FILE_SIZE 65536
#endif

#ifdef PC_SIDE
static int SERIAL_RECV(uint8_t* c, int fd)
{
	if (read(fd,c,1) == 1)
		return 1;
	else
		return -1;
}
static void SERIAL_SEND(uint8_t c, int fd)
{
	write(fd,&c,1);
}
#else
static void UART_SEND(uint8_t c, int32_t dev)
{
	usart_send_blocking(dev, c);
}
static int UART_RECV(uint8_t* c, int32_t dev)
{
	if((USART_ISR(dev) & USART_ISR_RXNE) == 0)
		return (-1);
	*c = (uint8_t)usart_recv(dev);
	return 1;
}
#endif

static void _outbyte(uint8_t c)
{
	XMODEM_SEND_BYTE(c, XMODEM_DEV_FILE);
}

static int _inbyte(int msec)
{
	uint8_t c=0;
	int i=3;
	while(i>0)
	{
		if(XMODEM_RECV_BYTE(&c, XMODEM_DEV_FILE) == 1)
			return c;
		OS_SLEEP(1);
		i++;
		if(i>(msec+3))
		{
			break;
		}
	}
	return (-1);
}

static unsigned short crc16_ccitt(const unsigned char *buf, int sz)
{
	unsigned short crc = 0;
	while (--sz >= 0) {
		int i;
		crc ^= (unsigned short) *buf++ << 8;
		for (i = 0; i < 8; i++)
			if (crc & 0x8000)
				crc = crc << 1 ^ 0x1021;
			else
				crc <<= 1;
	}
	return crc;
}

static int check(int crc, const unsigned char *buf, int sz)
{
	if (crc) {
		unsigned short crc = crc16_ccitt(buf, sz);
		unsigned short tcrc = (buf[sz]<<8)+buf[sz+1];
		if (crc == tcrc)
			return 1;
	}
	else {
		int i;
		unsigned char cks = 0;
		for (i = 0; i < sz; ++i) {
			cks += buf[i];
		}
		if (cks == buf[sz])
		return 1;
	}
	return 0;
}

static void flushinput(void)
{
	while (_inbyte(((DLY_1S)*3)>>1) >= 0)
		;
}

int xmodemReceive(unsigned char *dest, int destsz)
{
	unsigned char xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
	unsigned char *p;
	int bufsz, crc = 0;
	unsigned char trychar = 'C';
	unsigned char packetno = 1;
	int i, c, len = 0;
	int retry, retrans = MAXRETRANS;

	for(;;) {
		for( retry = 0; retry < 16; ++retry) {
			if (trychar) _outbyte(trychar);
			if ((c = _inbyte((DLY_1S)<<1)) >= 0) {
				switch (c) {
				case SOH:
					bufsz = 128;
					goto start_recv;
				case STX:
					bufsz = 1024;
					goto start_recv;
				case EOT:
					flushinput();
					_outbyte(ACK);
					return len; /* normal end */
				case CAN:
					if ((c = _inbyte(DLY_1S)) == CAN) {
						flushinput();
						_outbyte(ACK);
						return -1; /* canceled by remote */
					}
					break;
				default:
					break;
				}
			}
		}
		if (trychar == 'C') { trychar = NAK; continue; }
		flushinput();
		_outbyte(CAN);
		_outbyte(CAN);
		_outbyte(CAN);
		return -2; /* sync error */

	start_recv:
		if (trychar == 'C') crc = 1;
		trychar = 0;
		p = xbuff;
		*p++ = c;
		for (i = 0;  i < (bufsz+(crc?1:0)+3); ++i) {
			if ((c = _inbyte(DLY_1S)) < 0) goto reject;
			*p++ = c;
		}

		if (xbuff[1] == (unsigned char)(~xbuff[2]) && 
			(xbuff[1] == packetno || xbuff[1] == (unsigned char)packetno-1) &&
			check(crc, &xbuff[3], bufsz)) {
			if (xbuff[1] == packetno)	{
				register int count = destsz - len;
				if (count > bufsz) count = bufsz;
				if (count > 0) {
					memcpy (&dest[len], &xbuff[3], count);
					len += count;
				}
				++packetno;
				retrans = MAXRETRANS+1;
			}
			if (--retrans <= 0) {
				flushinput();
				_outbyte(CAN);
				_outbyte(CAN);
				_outbyte(CAN);
				return -3; /* too many retry error */
			}
			_outbyte(ACK);
			continue;
		}
	reject:
		flushinput();
		_outbyte(NAK);
	}
}

int xmodemTransmit(unsigned char *src, int srcsz)
{
	unsigned char xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
	int bufsz, crc = -1;
	unsigned char packetno = 1;
	int i, c, len = 0;
	int retry;

	for(;;) {
		for( retry = 0; retry < 16; ++retry) {
			if ((c = _inbyte((DLY_1S)<<1)) >= 0) {
				switch (c) {
				case 'C':
					crc = 1;
					goto start_trans;
				case NAK:
					crc = 0;
					goto start_trans;
				case CAN:
					if ((c = _inbyte(DLY_1S)) == CAN) {
						_outbyte(ACK);
						flushinput();
						return -1; /* canceled by remote */
					}
					break;
				default:
					break;
				}
			}
		}
		_outbyte(CAN);
		_outbyte(CAN);
		_outbyte(CAN);
		flushinput();
		return -2; /* no sync */

		for(;;) {
		start_trans:
			xbuff[0] = SOH; bufsz = 128;
			xbuff[1] = packetno;
			xbuff[2] = ~packetno;
			c = srcsz - len;
			if (c > bufsz) c = bufsz;
			if (c >= 0) {
				memset (&xbuff[3], 0, bufsz);
				if (c == 0) {
					xbuff[3] = CTRLZ;
				}
				else {
					memcpy (&xbuff[3], &src[len], c);
					if (c < bufsz) xbuff[3+c] = CTRLZ;
				}
				if (crc) {
					unsigned short ccrc = crc16_ccitt(&xbuff[3], bufsz);
					xbuff[bufsz+3] = (ccrc>>8) & 0xFF;
					xbuff[bufsz+4] = ccrc & 0xFF;
				}
				else {
					unsigned char ccks = 0;
					for (i = 3; i < bufsz+3; ++i) {
						ccks += xbuff[i];
					}
					xbuff[bufsz+3] = ccks;
				}
				for (retry = 0; retry < MAXRETRANS; ++retry) {
					for (i = 0; i < bufsz+4+(crc?1:0); ++i) {
						_outbyte(xbuff[i]);
					}
					if ((c = _inbyte(DLY_1S)) >= 0 ) {
						switch (c) {
						case ACK:
							++packetno;
							len += bufsz;
							goto start_trans;
						case CAN:
							if ((c = _inbyte(DLY_1S)) == CAN) {
								_outbyte(ACK);
								flushinput();
								return -1; /* canceled by remote */
							}
							break;
						case NAK:
						default:
							break;
						}
					}
				}
				_outbyte(CAN);
				_outbyte(CAN);
				_outbyte(CAN);
				flushinput();
				return -4; /* xmit error */
			}
			else {
				for (retry = 0; retry < 10; ++retry) {
					_outbyte(EOT);
					if ((c = _inbyte((DLY_1S)<<1)) == ACK) break;
				}
				flushinput();
				return (c == ACK)?len:-5;
			}
		}
	}
}

#ifdef TEST_XMODEM_RECEIVE
int main(int argc, char** argv)
{
#ifdef PC_SIDE
	if(argc != 3)
		return 0;
	XMODEM_DEV_FILE = open(argv[1],O_RDWR | O_NOCTTY | O_NDELAY);
	if(XMODEM_DEV_FILE == -1)						/* Error Checking */
		printf("\n  Error! in Opening file\n");
	else
		printf("\n  file Opened Successfully\n");
#if 1
		/*---------- Setting the Attributes of the serial port using termios structure --------- */

		struct termios SerialPortSettings;	/* Create the structure                          */

		tcgetattr(XMODEM_DEV_FILE, &SerialPortSettings);	/* Get the current attributes of the Serial port */

		/* Setting the Baud rate */
		cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 115200                       */
		cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 115200                       */

		/* 8N1 Mode */
		SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
		SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
		SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
		SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

		SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 

		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
		SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

		SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

		/* Setting Time outs */
		SerialPortSettings.c_cc[VMIN] = 1; /* Read at least 10 characters */
		SerialPortSettings.c_cc[VTIME] = 1; /* Wait indefinetly   */

		if((tcsetattr(XMODEM_DEV_FILE,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
			printf("\n  ERROR ! in Setting attributes\n");
		else
			printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none\n\n");
#endif
#endif
	int st;
	char* data = (char *)calloc(MAX_FILE_SIZE, sizeof(uint8_t));

	printf ("Send data using the xmodem protocol from your terminal emulator now...\n");
	/* the following should be changed for your environment:
	   0x30000 is the download address,
	   65536 is the maximum size to be written at this address
	 */
	st = xmodemReceive((unsigned char *)data, MAX_FILE_SIZE);
	if (st < 0) {
		printf ("Xmodem receive error: status: %d\n", st);
	}
	else {
		printf ("Xmodem successfully received %d bytes\n", st);
#ifdef PC_SIDE
		FILE *fp = fopen(argv[2], "wb");
		if (fp == NULL) {
			perror("Error opening file");
			return 1;
		}
		int fsize = st;
		while(data[fsize - 1] == CTRLZ) fsize--;
		size_t written = fwrite(data, sizeof(char), fsize, fp);
		printf("Number of elements written: %zu\n", written);
		fclose(fp);
#else
		printf ("Received: %s\n", data);
#endif
	}
	close(XMODEM_DEV_FILE);/* Close the Serial port */
	free(data);
	return 0;
}
#endif
#ifdef TEST_XMODEM_SEND
int main(int argc, char** argv)
{
#ifdef PC_SIDE
	if(argc != 2)
		return 0;
	XMODEM_DEV_FILE = open(argv[1],O_RDWR | O_NOCTTY | O_NDELAY);
	if(XMODEM_DEV_FILE == -1)						/* Error Checking */
		printf("\n  Error! in Opening file\n");
	else
		printf("\n  file Opened Successfully\n");
#if 1
		/*---------- Setting the Attributes of the serial port using termios structure --------- */

		struct termios SerialPortSettings;	/* Create the structure                          */

		tcgetattr(XMODEM_DEV_FILE, &SerialPortSettings);	/* Get the current attributes of the Serial port */

		cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 115200                       */
		cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 115200                       */

		SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
		SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
		SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
		SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

		SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 

		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
		SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

		SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

		if((tcsetattr(XMODEM_DEV_FILE,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
			printf("\n  ERROR ! in Setting attributes\n");
		else
			printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none\n\n");
#endif
#endif
	int st;
	char* data = (char *)calloc(MAX_FILE_SIZE, sizeof(uint8_t));
	strcpy(data, "Hello World ...\n");

	printf ("Prepare your terminal emulator to send data now...\n");
	/* the following should be changed for your environment:
	   0x30000 is the download address,
	   12000 is the maximum size to be send from this address
	 */
	st = xmodemTransmit((unsigned char *)data, 12000);
	if (st < 0) {
		printf ("Xmodem transmit error: status: %d\n", st);
	}
	else {
		printf ("Xmodem successfully transmitted %d bytes\n", st);
	}
	close(XMODEM_DEV_FILE);/* Close the Serial port */
	free(data);
	return 0;
}
#endif
