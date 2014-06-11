//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <cassert>
#include <cstring>
#include <iostream>

#ifndef WIN32
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#ifdef __QNX__
#include <sys/select.h>
#endif // __QNX__
#include <sys/stat.h>
#ifdef __APPLE__
#include <termios.h>
#endif // __APPLE__
#include <sys/types.h>
#endif // WIN32

#include "ComException.h"
#include "Serial.h"
#include "TimeoutException.h"

#ifdef __QNX__
#define CRTSCTS (IHFLOW | OHFLOW) 
#endif // __QNX__

namespace rl
{
	namespace hal
	{
		Serial::Serial(
			const ::std::string& filename,
			const BaudRate& baudRate,
			const DataBits& dataBits,
			const FlowControl& flowControl,
			const Parity& parity,
			const StopBits& stopBits
		) :
			Com(),
			baudRate(baudRate),
			current(),
			dataBits(dataBits),
			fd(0),
			filename(filename),
			flowControl(flowControl),
			parity(parity),
			restore(),
			stopBits(stopBits)
		{
#ifndef WIN32
			cfmakeraw(&this->current);
#endif // WIN32
			this->setBaudRate(this->baudRate);
			this->setDataBits(this->dataBits);
			this->setFlowControl(this->flowControl);
			this->setParity(this->parity);
			this->setStopBits(this->stopBits);
		}
		
		Serial::~Serial()
		{
			if (this->isConnected())
			{
				this->close();
			}
		}
		
		void
		Serial::changeParameters()
		{
#ifdef WIN32
			if (0 == SetCommState(this->fd, &this->current))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			if (-1 == tcsetattr(this->fd, TCSANOW, &this->current))
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
		}
		
		void
		Serial::close()
		{
			assert(this->isConnected());
			
#ifdef WIN32
			if (0 == SetCommState(this->fd, &this->restore))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
			
			this->flush(true, true);
			
			if (0 == CloseHandle(this->fd))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			if (-1 == tcsetattr(this->fd, TCSANOW, &this->restore))
			{
				throw ComException(strerror(errno));
			}
			
			this->flush(true, true);
			
			if (-1 == ::close(this->fd))
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			this->setConnected(false);
		}
		
		void
		Serial::doBreak(const bool& doOn)
		{
#ifdef WIN32
			if (0 == EscapeCommFunction(this->fd, doOn ? SETBREAK : CLRBREAK))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			if (-1 == ioctl(this->fd, doOn ? TIOCSBRK : TIOCCBRK))
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
		}
		
		void
		Serial::doDtr(const bool& doOn)
		{
#ifdef WIN32
			if (0 == EscapeCommFunction(this->fd, doOn ? SETDTR : CLRDTR))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			if (-1 == ioctl(this->fd, doOn ? TIOCMBIS : TIOCMBIC, TIOCM_DTR))
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
		}
		
		void
		Serial::doModemStatus(bool& ctsOn, bool& dsrOn, bool& riOn, bool& dcdOn)
		{
#ifdef WIN32
			DWORD status;
			
			if (0 == GetCommModemStatus(this->fd, &status))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
			
			ctsOn = status & MS_CTS_ON ? true : false;
			dsrOn = status & MS_DSR_ON ? true : false;
			riOn = status & MS_RING_ON ? true : false;
			dcdOn = status & MS_RLSD_ON ? true : false;
#else // WIN32
			int status = 0;
			
			if (-1 == ioctl(this->fd, TIOCMGET, status))
			{
				throw ComException(strerror(errno));
			}
			
			ctsOn = status & TIOCM_CTS;
			dsrOn = status & TIOCM_DSR;
			riOn = status & TIOCM_RI;
			dcdOn = status & TIOCM_CD;
#endif // WIN32
		}
		
		void
		Serial::doRts(const bool& doOn)
		{
#ifdef WIN32
			if (0 == EscapeCommFunction(this->fd, doOn ? SETRTS : CLRRTS))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			if (-1 == ioctl(this->fd, doOn ? TIOCMBIS : TIOCMBIC, TIOCM_RTS))
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
		}
		
		void
		Serial::flush(const bool& read, const bool& write)
		{
			assert(this->isConnected());
			
#ifdef WIN32
			if (read && write)
			{
				if (0 == PurgeComm(this->fd, PURGE_RXCLEAR | PURGE_TXCLEAR))
				{
					LPTSTR buffer = NULL;
					FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
					ComException e(NULL != buffer ? buffer : "ComException");
					LocalFree(buffer);
					throw e;
				}
			}
			else if (read)
			{
				if (0 == PurgeComm(this->fd, PURGE_RXCLEAR))
				{
					LPTSTR buffer = NULL;
					FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
					ComException e(NULL != buffer ? buffer : "ComException");
					LocalFree(buffer);
					throw e;
				}
			}
			else if (write)
			{
				if (0 == PurgeComm(this->fd, PURGE_TXCLEAR))
				{
					LPTSTR buffer = NULL;
					FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
					ComException e(NULL != buffer ? buffer : "ComException");
					LocalFree(buffer);
					throw e;
				}
			}
#else // WIN32
			if (read && write)
			{
				if (-1 == tcflush(this->fd, TCIOFLUSH))
				{
					throw ComException(strerror(errno));
				}
			}
			else if (read)
			{
				if (-1 == tcflush(this->fd, TCIFLUSH))
				{
					throw ComException(strerror(errno));
				}
			}
			else if (write)
			{
				if (-1 == tcflush(this->fd, TCOFLUSH))
				{
					throw ComException(strerror(errno));
				}
			}
#endif // WIN32
		}
		
		Serial::BaudRate
		Serial::getBaudRate() const
		{
			return this->baudRate;
		}
		
		Serial::DataBits
		Serial::getDataBits() const
		{
			return this->dataBits;
		}
		
		::std::string
		Serial::getFilename() const
		{
			return this->filename;
		}
		
		Serial::FlowControl
		Serial::getFlowControl() const
		{
			return this->flowControl;
		}
		
		Serial::Parity
		Serial::getParity() const
		{
			return this->parity;
		}
		
		Serial::StopBits
		Serial::getStopBits() const
		{
			return this->stopBits;
		}
		
		void
		Serial::open()
		{
			assert(!this->isConnected());
			
#ifdef WIN32
			this->fd = ::CreateFile(this->filename.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
			
			if (INVALID_HANDLE_VALUE == this->fd)
			{
				this->fd = 0;
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
			
			this->setConnected(true);
			
			if (0 == GetCommState(this->fd, &this->restore))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
			
			this->flush(true, true);
			
			if (0 == SetCommState(this->fd, &this->current))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			this->fd = ::open(this->filename.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
			
			if (-1 == this->fd)
			{
				throw ComException(strerror(errno));
			}
			
			this->setConnected(true);
			
			if (-1 == tcgetattr(this->fd, &this->restore))
			{ 
				throw ComException(strerror(errno));
			}
			
			this->flush(true, true);
			
			this->current.c_cflag |= CREAD | CLOCAL;
			
			if (-1 == tcsetattr(this->fd, TCSANOW, &this->current))
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
		}
		
		::std::size_t
		Serial::read(void* buf, const ::std::size_t& count)
		{
			assert(this->isConnected());
			
			memset(buf, 0, count);
			
#ifdef WIN32
			DWORD numbytes;
			
			if (0 == ReadFile(this->fd, buf, count, &numbytes, NULL))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			ssize_t numbytes = ::read(this->fd, buf, count);
			
			if (-1 == numbytes)
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			return numbytes;
		}
		
		::std::size_t
		Serial::select(const bool& read, const bool& write, const ::rl::math::Real& timeout)
		{
#ifdef WIN32
			assert(false);
			return 0;
#else
			::rl::math::Real tmp = timeout;
			
			struct timeval tv;
			
			tv.tv_sec = static_cast< long int >(tmp);
			tmp -= tv.tv_sec;
			tv.tv_usec = static_cast< long int >(tmp * 1000000.0f);
			
			fd_set readfds;
			FD_ZERO(&readfds);
			FD_SET(this->fd, &readfds);
			
			fd_set writefds;
			FD_ZERO(&writefds);
			FD_SET(this->fd, &writefds);
			
			ssize_t numdescriptors;
			
			if (read && write)
			{
				numdescriptors = ::select(this->fd + 1, &readfds, &writefds, NULL, &tv);
			}
			else if (read)
			{
				numdescriptors = ::select(this->fd + 1, &readfds, NULL, NULL, &tv);
			}
			else if (write)
			{
				numdescriptors = ::select(this->fd + 1, NULL, &writefds, NULL, &tv);
			}
			else
			{
				numdescriptors = ::select(this->fd + 1, NULL, NULL, NULL, &tv);
			}
			
			if (0 == numdescriptors)
			{
				throw TimeoutException();
			}
			else if (-1 == numdescriptors)
			{
				throw ComException(strerror(errno));
			}
			
			return numdescriptors;
#endif // WIN32
		}
		
		void
		Serial::setBaudRate(const BaudRate& baudRate)
		{
#ifdef WIN32
			switch (baudRate)
			{
			case BAUDRATE_110BPS:
				this->current.BaudRate = CBR_110;
				break;
			case BAUDRATE_300BPS:
				this->current.BaudRate = CBR_300;
				break;
			case BAUDRATE_600BPS:
				this->current.BaudRate = CBR_600;
				break;
			case BAUDRATE_1200BPS:
				this->current.BaudRate = CBR_1200;
				break;
			case BAUDRATE_2400BPS:
				this->current.BaudRate = CBR_2400;
				break;
			case BAUDRATE_4800BPS:
				this->current.BaudRate = CBR_4800;
				break;
			case BAUDRATE_9600BPS:
				this->current.BaudRate = CBR_9600;
				break;
			case BAUDRATE_14400BPS:
				this->current.BaudRate = CBR_14400;
				break;
			case BAUDRATE_19200BPS:
				this->current.BaudRate = CBR_19200;
				break;
			case BAUDRATE_38400BPS:
				this->current.BaudRate = CBR_38400;
				break;
			case BAUDRATE_57600BPS:
				this->current.BaudRate = CBR_57600;
				break;
			case BAUDRATE_115200BPS:
				this->current.BaudRate = CBR_115200;
				break;
			case BAUDRATE_128000BPS:
				this->current.BaudRate = CBR_128000;
				break;
			case BAUDRATE_256000BPS:
				this->current.BaudRate = CBR_256000;
				break;
			default:
				break;
			}
#else // WIN32
			speed_t speed = B110;
			
			switch (baudRate)
			{
			case BAUDRATE_110BPS:
				speed = B110;
				break;
			case BAUDRATE_300BPS:
				speed = B300;
				break;
			case BAUDRATE_600BPS:
				speed = B600;
				break;
			case BAUDRATE_1200BPS:
				speed = B1200;
				break;
			case BAUDRATE_2400BPS:
				speed = B2400;
				break;
			case BAUDRATE_4800BPS:
				speed = B4800;
				break;
			case BAUDRATE_9600BPS:
				speed = B9600;
				break;
			case BAUDRATE_19200BPS:
				speed = B19200;
				break;
			case BAUDRATE_38400BPS:
				speed = B38400;
				break;
			case BAUDRATE_57600BPS:
				speed = B57600;
				break;
			case BAUDRATE_115200BPS:
				speed = B115200;
				break;
#ifndef __QNX__
			case BAUDRATE_230400BPS:
				speed = B230400;
				break;
#ifndef __APPLE__
			case BAUDRATE_460800BPS:
				speed = B460800;
				break;
			case BAUDRATE_500000BPS:
				speed = B500000;
				break;
			case BAUDRATE_576000BPS:
				speed = B576000;
				break;
			case BAUDRATE_921600BPS:
				speed = B921600;
				break;
			case BAUDRATE_1000000BPS:
				speed = B1000000;
				break;
			case BAUDRATE_1152000BPS:
				speed = B1152000;
				break;
			case BAUDRATE_1500000BPS:
				speed = B1500000;
				break;
			case BAUDRATE_2000000BPS:
				speed = B2000000;
				break;
			case BAUDRATE_2500000BPS:
				speed = B2500000;
				break;
			case BAUDRATE_3000000BPS:
				speed = B3000000;
				break;
			case BAUDRATE_3500000BPS:
				speed = B3500000;
				break;
			case BAUDRATE_4000000BPS:
				speed = B4000000;
				break;
#endif // __APPLE__
#endif // __QNX__
			default:
				break;
			}
			
			if (-1 == cfsetispeed(&this->current, speed))
			{
				throw ComException(strerror(errno));
			}
			
			if (-1 == cfsetospeed(&this->current, speed))
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			this->baudRate = baudRate;
		}
		
		void
		Serial::setDataBits(const DataBits& dataBits)
		{
#ifdef WIN32
			switch (dataBits)
			{
			case DATABITS_5BITS:
				this->current.ByteSize = 5;
				break;
			case DATABITS_6BITS:
				this->current.ByteSize = 6;
				break;
			case DATABITS_7BITS:
				this->current.ByteSize = 7;
				break;
			case DATABITS_8BITS:
				this->current.ByteSize = 8;
				break;
			default:
				break;
			}
#else // WIN32
			switch (dataBits)
			{
			case DATABITS_5BITS:
				this->current.c_cflag &= ~(CS6 | CS7 | CS8);
				this->current.c_cflag |= CS5;
				this->current.c_iflag |= ISTRIP;
				break;
			case DATABITS_6BITS:
				this->current.c_cflag &= ~(CS5 | CS7 | CS8);
				this->current.c_cflag |= CS6;
				this->current.c_iflag |= ISTRIP;
				break;
			case DATABITS_7BITS:
				this->current.c_cflag &= ~(CS5 | CS6 | CS8);
				this->current.c_cflag |= CS7;
				this->current.c_iflag |= ISTRIP;
				break;
			case DATABITS_8BITS:
				this->current.c_cflag &= ~(CS5 | CS6 | CS7);
				this->current.c_cflag |= CS8;
				this->current.c_iflag &= ~ISTRIP;
				break;
			default:
				break;
			}
#endif // WIN32
			
			this->dataBits = dataBits;
		}
		
		void
		Serial::setFilename(const ::std::string& filename)
		{
			this->filename = filename;
		}
		
		void
		Serial::setFlowControl(const FlowControl& flowControl)
		{
#ifdef WIN32
			switch (flowControl)
			{
			case FLOWCONTROL_OFF:
				this->current.fOutxCtsFlow = false;
				this->current.fOutxDsrFlow = false;
				this->current.fDtrControl = DTR_CONTROL_DISABLE;
				this->current.fOutX = false;
				this->current.fInX = false;
				this->current.fRtsControl = RTS_CONTROL_DISABLE;
				this->current.XoffChar = 0x00;
				this->current.XonChar = 0x00;
				break;
			case FLOWCONTROL_RTSCTS:
				this->current.fOutxCtsFlow = true;
				this->current.fOutxDsrFlow = true;
				this->current.fDtrControl = DTR_CONTROL_HANDSHAKE;
				this->current.fOutX = false;
				this->current.fInX = false;
				this->current.fRtsControl = RTS_CONTROL_HANDSHAKE;
				this->current.XoffChar = 0x00;
				this->current.XonChar = 0x00;
				break;
			case FLOWCONTROL_XONXOFF:
				this->current.fOutxCtsFlow = false;
				this->current.fOutxDsrFlow = false;
				this->current.fDtrControl = DTR_CONTROL_DISABLE;
				this->current.fOutX = true;
				this->current.fInX = true;
				this->current.fRtsControl = RTS_CONTROL_DISABLE;
				this->current.XoffChar = 0x13;
				this->current.XonChar = 0x11;
				break;
			default:
				break;
			}
#else // WIN32
			switch (flowControl)
			{
			case FLOWCONTROL_OFF:
				this->current.c_cflag &= ~CRTSCTS;
				this->current.c_iflag &= ~(IXANY | IXOFF | IXON);
				this->current.c_cc[VSTART] = 0x00;
				this->current.c_cc[VSTOP] = 0x00;
				break;
			case FLOWCONTROL_RTSCTS:
				this->current.c_cflag |= CRTSCTS;
				this->current.c_iflag &= ~(IXANY | IXOFF | IXON);
				this->current.c_cc[VSTART] = 0x00;
				this->current.c_cc[VSTOP] = 0x00;
				break;
			case FLOWCONTROL_XONXOFF:
				this->current.c_cflag &= ~CRTSCTS;
				this->current.c_iflag |= IXANY | IXOFF | IXON;
				this->current.c_cc[VSTART] = 0x11;
				this->current.c_cc[VSTOP] = 0x13;
				break;
			default:
				break;
			}
#endif // WIN32
			
			this->flowControl = flowControl;
		}
		
		void
		Serial::setParity(const Parity& parity)
		{
#ifdef WIN32
			switch (parity)
			{
			case PARITY_EVENPARITY:
				this->current.Parity = EVENPARITY;
				break;
			case PARITY_NOPARITY:
				this->current.Parity = NOPARITY;
				break;
			case PARITY_ODDPARITY:
				this->current.Parity = ODDPARITY;
				break;
			default:
				break;
			}
#else // WIN32
			switch (parity)
			{
			case PARITY_EVENPARITY:
				this->current.c_cflag &= ~PARODD;
				this->current.c_cflag |= PARENB;
				this->current.c_iflag &= ~IGNPAR;
				this->current.c_iflag |= INPCK;
				break;
			case PARITY_NOPARITY:
				this->current.c_cflag &= ~(PARENB | PARODD);
				this->current.c_iflag &= ~INPCK;
				this->current.c_iflag |= IGNPAR;
				break;
			case PARITY_ODDPARITY:
				this->current.c_cflag |= PARENB | PARODD;
				this->current.c_iflag &= ~IGNPAR;
				this->current.c_iflag |= INPCK;
				break;
			default:
				break;
			}
#endif // WIN32
			
			this->parity = parity;
		}
		
		void
		Serial::setStopBits(const StopBits& stopBits)
		{
#ifdef WIN32
			switch (stopBits)
			{
			case STOPBITS_1BIT:
				this->current.StopBits = ONESTOPBIT;
				break;
			case STOPBITS_2BITS:
				this->current.StopBits = TWOSTOPBITS;
				break;
			default:
				break;
			}
#else // WIN32
			switch (stopBits)
			{
			case STOPBITS_1BIT:
				this->current.c_cflag &= ~CSTOPB;
				break;
			case STOPBITS_2BITS:
				this->current.c_cflag |= CSTOPB;
				break;
			default:
				break;
			}
#endif // WIN32
			
			this->stopBits = stopBits;
		}
		
		::std::size_t
		Serial::write(const void* buf, const ::std::size_t& count)
		{
			assert(this->isConnected());
			
#ifdef WIN32
			DWORD numbytes;
			
			if (0 == WriteFile(this->fd, buf, count, &numbytes, NULL))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, GetLastError(), 0, buffer, 0, NULL);
				ComException e(NULL != buffer ? buffer : "ComException");
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			ssize_t numbytes = ::write(this->fd, buf, count);
			
			if (-1 == numbytes)
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			return numbytes;
		}
	}
}
