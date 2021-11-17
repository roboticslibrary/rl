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

#ifdef WIN32
#include <windows.h>
#else // WIN32
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#ifdef __QNX__
#include <sys/select.h>
#endif // __QNX__
#include <sys/stat.h>
#include <sys/types.h>
#endif // WIN32

#include <cassert>
#include <cstring>
#include <iostream>
#include <rl/std/memory.h>

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
		constexpr Serial::BaudRate Serial::BAUDRATE_110BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_300BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_600BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_1200BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_2400BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_4800BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_9600BPS;
#ifdef WIN32
		constexpr Serial::BaudRate Serial::BAUDRATE_14400BPS;
#endif // WIN32
		constexpr Serial::BaudRate Serial::BAUDRATE_19200BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_38400BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_57600BPS;
#ifdef __QNX__
		constexpr Serial::BaudRate Serial::BAUDRATE_115200BPS;
#else // __QNX__
		constexpr Serial::BaudRate Serial::BAUDRATE_115200BPS;
#endif // __QNX__
#ifdef WIN32
		constexpr Serial::BaudRate Serial::BAUDRATE_128000BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_256000BPS;
#else // WIN32
#ifndef __QNX__
		constexpr Serial::BaudRate Serial::BAUDRATE_230400BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_460800BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_500000BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_576000BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_921600BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_1000000BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_1152000BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_1500000BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_2000000BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_2500000BPS;
#ifdef __CYGWIN__
		constexpr Serial::BaudRate Serial::BAUDRATE_3000000BPS;
#else // __CYGWIN__
		constexpr Serial::BaudRate Serial::BAUDRATE_3000000BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_3500000BPS;
		constexpr Serial::BaudRate Serial::BAUDRATE_4000000BPS;
#endif // __CYGWIN__
#endif // __QNX__
#endif // WIN32
		
		constexpr Serial::DataBits Serial::DATABITS_5BITS;
		constexpr Serial::DataBits Serial::DATABITS_6BITS;
		constexpr Serial::DataBits Serial::DATABITS_7BITS;
		constexpr Serial::DataBits Serial::DATABITS_8BITS;
		
		constexpr Serial::FlowControl Serial::FLOWCONTROL_OFF;
		constexpr Serial::FlowControl Serial::FLOWCONTROL_RTSCTS;
		constexpr Serial::FlowControl Serial::FLOWCONTROL_XONXOFF;
		
		constexpr Serial::Parity Serial::PARITY_EVENPARITY;
		constexpr Serial::Parity Serial::PARITY_NOPARITY;
		constexpr Serial::Parity Serial::PARITY_ODDPARITY;
		
		constexpr Serial::StopBits Serial::STOPBITS_1BIT;
		constexpr Serial::StopBits Serial::STOPBITS_2BITS;
		
		struct Serial::Impl
		{
#ifdef WIN32
			DCB current;
			
			HANDLE fd;
			
			DCB restore;
#else // WIN32
			struct termios current;
			
			int fd;
			
			struct termios restore;
#endif // WIN32
		};
		
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
			dataBits(dataBits),
			filename(filename),
#ifdef WIN32
			flags(GENERIC_READ | GENERIC_WRITE),
#else // WIN32
			flags(O_RDWR | O_NOCTTY),
#endif // WIN32
			flowControl(flowControl),
			impl(::rl::std14::make_unique<Impl>()),
			parity(parity),
			stopBits(stopBits)
		{
#ifndef WIN32
			::cfmakeraw(&this->impl->current);
#endif // WIN32
			this->setBaudRate(this->baudRate);
			this->setDataBits(this->dataBits);
			this->setFlowControl(this->flowControl);
			this->setParity(this->parity);
			this->setStopBits(this->stopBits);
		}
		
		Serial::Serial(
			const ::std::string& filename,
			const BaudRate& baudRate,
			const DataBits& dataBits,
			const FlowControl& flowControl,
			const Parity& parity,
			const StopBits& stopBits,
			const int& flags
		) :
			Com(),
			baudRate(baudRate),
			dataBits(dataBits),
			filename(filename),
			flags(flags),
			flowControl(flowControl),
			impl(::rl::std14::make_unique<Impl>()),
			parity(parity),
			stopBits(stopBits)
		{
#ifndef WIN32
			::cfmakeraw(&this->impl->current);
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
			if (0 == ::SetCommState(this->impl->fd, &this->impl->current))
			{
				throw ComException(::GetLastError());
			}
#else // WIN32
			if (-1 == ::tcsetattr(this->impl->fd, TCSANOW, &this->impl->current))
			{
				throw ComException(errno);
			}
#endif // WIN32
		}
		
		void
		Serial::close()
		{
			assert(this->isConnected());
			
#ifdef WIN32
			if (0 == ::SetCommState(this->impl->fd, &this->impl->restore))
			{
				throw ComException(::GetLastError());
			}
			
			this->flush(true, true);
			
			if (0 == ::CloseHandle(this->impl->fd))
			{
				throw ComException(::GetLastError());
			}
#else // WIN32
			if (-1 == ::tcsetattr(this->impl->fd, TCSANOW, &this->impl->restore))
			{
				throw ComException(errno);
			}
			
			this->flush(true, true);
			
			if (-1 == ::close(this->impl->fd))
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			this->setConnected(false);
		}
		
		void
		Serial::doBreak(const bool& doOn)
		{
#ifdef WIN32
			if (0 == ::EscapeCommFunction(this->impl->fd, doOn ? SETBREAK : CLRBREAK))
			{
				throw ComException(::GetLastError());
			}
#else // WIN32
			if (-1 == ::ioctl(this->impl->fd, doOn ? TIOCSBRK : TIOCCBRK))
			{
				throw ComException(errno);
			}
#endif // WIN32
		}
		
		void
		Serial::doDtr(const bool& doOn)
		{
#ifdef WIN32
			if (0 == ::EscapeCommFunction(this->impl->fd, doOn ? SETDTR : CLRDTR))
			{
				throw ComException(::GetLastError());
			}
#else // WIN32
			if (-1 == ::ioctl(this->impl->fd, doOn ? TIOCMBIS : TIOCMBIC, TIOCM_DTR))
			{
				throw ComException(errno);
			}
#endif // WIN32
		}
		
		void
		Serial::doModemStatus(bool& ctsOn, bool& dsrOn, bool& riOn, bool& dcdOn)
		{
#ifdef WIN32
			::DWORD status;
			
			if (0 == ::GetCommModemStatus(this->impl->fd, &status))
			{
				throw ComException(::GetLastError());
			}
			
			ctsOn = (status & MS_CTS_ON) ? true : false;
			dsrOn = (status & MS_DSR_ON) ? true : false;
			riOn = (status & MS_RING_ON) ? true : false;
			dcdOn = (status & MS_RLSD_ON) ? true : false;
#else // WIN32
			int status = 0;
			
			if (-1 == ::ioctl(this->impl->fd, TIOCMGET, status))
			{
				throw ComException(errno);
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
			if (0 == ::EscapeCommFunction(this->impl->fd, doOn ? SETRTS : CLRRTS))
			{
				throw ComException(::GetLastError());
			}
#else // WIN32
			if (-1 == ::ioctl(this->impl->fd, doOn ? TIOCMBIS : TIOCMBIC, TIOCM_RTS))
			{
				throw ComException(errno);
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
				if (0 == ::PurgeComm(this->impl->fd, PURGE_RXCLEAR | PURGE_TXCLEAR))
				{
					throw ComException(::GetLastError());
				}
			}
			else if (read)
			{
				if (0 == ::PurgeComm(this->impl->fd, PURGE_RXCLEAR))
				{
					throw ComException(::GetLastError());
				}
			}
			else if (write)
			{
				if (0 == ::PurgeComm(this->impl->fd, PURGE_TXCLEAR))
				{
					throw ComException(::GetLastError());
				}
			}
#else // WIN32
			if (read && write)
			{
				if (-1 == ::tcflush(this->impl->fd, TCIOFLUSH))
				{
					throw ComException(errno);
				}
			}
			else if (read)
			{
				if (-1 == ::tcflush(this->impl->fd, TCIFLUSH))
				{
					throw ComException(errno);
				}
			}
			else if (write)
			{
				if (-1 == ::tcflush(this->impl->fd, TCOFLUSH))
				{
					throw ComException(errno);
				}
			}
#endif // WIN32
		}
		
		const Serial::BaudRate&
		Serial::getBaudRate() const
		{
			return this->baudRate;
		}
		
		const Serial::DataBits&
		Serial::getDataBits() const
		{
			return this->dataBits;
		}
		
		const ::std::string&
		Serial::getFilename() const
		{
			return this->filename;
		}
		
		const Serial::FlowControl&
		Serial::getFlowControl() const
		{
			return this->flowControl;
		}
		
		const Serial::Parity&
		Serial::getParity() const
		{
			return this->parity;
		}
		
		const Serial::StopBits&
		Serial::getStopBits() const
		{
			return this->stopBits;
		}
		
		void
		Serial::open()
		{
			assert(!this->isConnected());
			
#ifdef WIN32
			this->impl->fd = ::CreateFile(this->filename.c_str(), this->flags, 0, nullptr, OPEN_EXISTING, 0, nullptr);
			
			if (INVALID_HANDLE_VALUE == this->impl->fd)
			{
				this->impl->fd = 0;
				throw ComException(::GetLastError());
			}
			
			this->setConnected(true);
			
			if (0 == GetCommState(this->impl->fd, &this->impl->restore))
			{
				throw ComException(::GetLastError());
			}
			
			this->flush(true, true);
			
			if (0 == ::SetCommState(this->impl->fd, &this->impl->current))
			{
				throw ComException(::GetLastError());
			}
#else // WIN32
			this->impl->fd = ::open(this->filename.c_str(), this->flags);
			
			if (-1 == this->impl->fd)
			{
				throw ComException(errno);
			}
			
			this->setConnected(true);
			
			if (-1 == ::tcgetattr(this->impl->fd, &this->impl->restore))
			{
				throw ComException(errno);
			}
			
			this->flush(true, true);
			
			this->impl->current.c_cflag |= CREAD | CLOCAL;
			
			if (-1 == ::tcsetattr(this->impl->fd, TCSANOW, &this->impl->current))
			{
				throw ComException(errno);
			}
#endif // WIN32
		}
		
		::std::size_t
		Serial::read(void* buf, const ::std::size_t& count)
		{
			assert(this->isConnected());
			
#ifdef WIN32
			::DWORD numbytes;
			
			if (0 == ::ReadFile(this->impl->fd, buf, count, &numbytes, nullptr))
			{
				throw ComException(::GetLastError());
			}
#else // WIN32
			::ssize_t numbytes = ::read(this->impl->fd, buf, count);
			
			if (-1 == numbytes)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			return numbytes;
		}
		
		::std::size_t
		Serial::select(const bool& read, const bool& write, const ::std::chrono::nanoseconds& timeout)
		{
#ifdef WIN32
			assert(false);
			return 0;
#else
			::timeval tv;
			tv.tv_sec = ::std::chrono::duration_cast<::std::chrono::seconds>(timeout).count();
			tv.tv_usec = ::std::chrono::duration_cast<::std::chrono::microseconds>(timeout - ::std::chrono::duration_cast<::std::chrono::seconds>(timeout)).count();
			
			::fd_set readfds;
			FD_ZERO(&readfds);
			FD_SET(this->impl->fd, &readfds);
			
			::fd_set writefds;
			FD_ZERO(&writefds);
			FD_SET(this->impl->fd, &writefds);
			
			::ssize_t numdescriptors = ::select(this->impl->fd + 1, read ? &readfds : nullptr, write ? &writefds : nullptr, nullptr, &tv);
			
			if (0 == numdescriptors)
			{
				throw TimeoutException();
			}
			else if (-1 == numdescriptors)
			{
				throw ComException(errno);
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
			case BaudRate::b110:
				this->impl->current.BaudRate = CBR_110;
				break;
			case BaudRate::b300:
				this->impl->current.BaudRate = CBR_300;
				break;
			case BaudRate::b600:
				this->impl->current.BaudRate = CBR_600;
				break;
			case BaudRate::b1200:
				this->impl->current.BaudRate = CBR_1200;
				break;
			case BaudRate::b2400:
				this->impl->current.BaudRate = CBR_2400;
				break;
			case BaudRate::b4800:
				this->impl->current.BaudRate = CBR_4800;
				break;
			case BaudRate::b9600:
				this->impl->current.BaudRate = CBR_9600;
				break;
			case BaudRate::b14400:
				this->impl->current.BaudRate = CBR_14400;
				break;
			case BaudRate::b19200:
				this->impl->current.BaudRate = CBR_19200;
				break;
			case BaudRate::b38400:
				this->impl->current.BaudRate = CBR_38400;
				break;
			case BaudRate::b57600:
				this->impl->current.BaudRate = CBR_57600;
				break;
			case BaudRate::b115200:
				this->impl->current.BaudRate = CBR_115200;
				break;
			case BaudRate::b128000:
				this->impl->current.BaudRate = CBR_128000;
				break;
			case BaudRate::b256000:
				this->impl->current.BaudRate = CBR_256000;
				break;
			default:
				break;
			}
#else // WIN32
			::speed_t speed = B110;
			
			switch (baudRate)
			{
			case BaudRate::b110:
				speed = B110;
				break;
			case BaudRate::b300:
				speed = B300;
				break;
			case BaudRate::b600:
				speed = B600;
				break;
			case BaudRate::b1200:
				speed = B1200;
				break;
			case BaudRate::b2400:
				speed = B2400;
				break;
			case BaudRate::b4800:
				speed = B4800;
				break;
			case BaudRate::b9600:
				speed = B9600;
				break;
			case BaudRate::b19200:
				speed = B19200;
				break;
			case BaudRate::b38400:
				speed = B38400;
				break;
			case BaudRate::b57600:
				speed = B57600;
				break;
			case BaudRate::b115200:
				speed = B115200;
				break;
#ifndef __QNX__
			case BaudRate::b230400:
				speed = B230400;
				break;
#ifndef __APPLE__
			case BaudRate::b460800:
				speed = B460800;
				break;
			case BaudRate::b500000:
				speed = B500000;
				break;
			case BaudRate::b576000:
				speed = B576000;
				break;
			case BaudRate::b921600:
				speed = B921600;
				break;
			case BaudRate::b1000000:
				speed = B1000000;
				break;
			case BaudRate::b1152000:
				speed = B1152000;
				break;
			case BaudRate::b1500000:
				speed = B1500000;
				break;
			case BaudRate::b2000000:
				speed = B2000000;
				break;
			case BaudRate::b2500000:
				speed = B2500000;
				break;
			case BaudRate::b3000000:
				speed = B3000000;
				break;
#ifndef __CYGWIN__
			case BaudRate::b3500000:
				speed = B3500000;
				break;
			case BaudRate::b4000000:
				speed = B4000000;
				break;
#endif // __CYGWIN__
#endif // __APPLE__
#endif // __QNX__
			default:
				break;
			}
			
			if (-1 == ::cfsetispeed(&this->impl->current, speed))
			{
				throw ComException(errno);
			}
			
			if (-1 == ::cfsetospeed(&this->impl->current, speed))
			{
				throw ComException(errno);
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
			case DataBits::d5:
				this->impl->current.ByteSize = 5;
				break;
			case DataBits::d6:
				this->impl->current.ByteSize = 6;
				break;
			case DataBits::d7:
				this->impl->current.ByteSize = 7;
				break;
			case DataBits::d8:
				this->impl->current.ByteSize = 8;
				break;
			default:
				break;
			}
#else // WIN32
			switch (dataBits)
			{
			case DataBits::d5:
				this->impl->current.c_cflag &= ~(CS6 | CS7 | CS8);
				this->impl->current.c_cflag |= CS5;
				this->impl->current.c_iflag |= ISTRIP;
				break;
			case DataBits::d6:
				this->impl->current.c_cflag &= ~(CS5 | CS7 | CS8);
				this->impl->current.c_cflag |= CS6;
				this->impl->current.c_iflag |= ISTRIP;
				break;
			case DataBits::d7:
				this->impl->current.c_cflag &= ~(CS5 | CS6 | CS8);
				this->impl->current.c_cflag |= CS7;
				this->impl->current.c_iflag |= ISTRIP;
				break;
			case DataBits::d8:
				this->impl->current.c_cflag &= ~(CS5 | CS6 | CS7);
				this->impl->current.c_cflag |= CS8;
				this->impl->current.c_iflag &= ~ISTRIP;
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
			case FlowControl::off:
				this->impl->current.fOutxCtsFlow = false;
				this->impl->current.fOutxDsrFlow = false;
				this->impl->current.fDtrControl = DTR_CONTROL_DISABLE;
				this->impl->current.fOutX = false;
				this->impl->current.fInX = false;
				this->impl->current.fRtsControl = RTS_CONTROL_DISABLE;
				this->impl->current.XoffChar = 0x00;
				this->impl->current.XonChar = 0x00;
				break;
			case FlowControl::rtscts:
				this->impl->current.fOutxCtsFlow = true;
				this->impl->current.fOutxDsrFlow = true;
				this->impl->current.fDtrControl = DTR_CONTROL_HANDSHAKE;
				this->impl->current.fOutX = false;
				this->impl->current.fInX = false;
				this->impl->current.fRtsControl = RTS_CONTROL_HANDSHAKE;
				this->impl->current.XoffChar = 0x00;
				this->impl->current.XonChar = 0x00;
				break;
			case FlowControl::xonxoff:
				this->impl->current.fOutxCtsFlow = false;
				this->impl->current.fOutxDsrFlow = false;
				this->impl->current.fDtrControl = DTR_CONTROL_DISABLE;
				this->impl->current.fOutX = true;
				this->impl->current.fInX = true;
				this->impl->current.fRtsControl = RTS_CONTROL_DISABLE;
				this->impl->current.XoffChar = 0x13;
				this->impl->current.XonChar = 0x11;
				break;
			default:
				break;
			}
#else // WIN32
			switch (flowControl)
			{
			case FlowControl::off:
				this->impl->current.c_cflag &= ~CRTSCTS;
				this->impl->current.c_iflag &= ~(IXANY | IXOFF | IXON);
				this->impl->current.c_cc[VSTART] = 0x00;
				this->impl->current.c_cc[VSTOP] = 0x00;
				break;
			case FlowControl::rtscts:
				this->impl->current.c_cflag |= CRTSCTS;
				this->impl->current.c_iflag &= ~(IXANY | IXOFF | IXON);
				this->impl->current.c_cc[VSTART] = 0x00;
				this->impl->current.c_cc[VSTOP] = 0x00;
				break;
			case FlowControl::xonxoff:
				this->impl->current.c_cflag &= ~CRTSCTS;
				this->impl->current.c_iflag |= IXANY | IXOFF | IXON;
				this->impl->current.c_cc[VSTART] = 0x11;
				this->impl->current.c_cc[VSTOP] = 0x13;
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
			case Parity::even:
				this->impl->current.Parity = EVENPARITY;
				break;
			case Parity::none:
				this->impl->current.Parity = NOPARITY;
				break;
			case Parity::odd:
				this->impl->current.Parity = ODDPARITY;
				break;
			default:
				break;
			}
#else // WIN32
			switch (parity)
			{
			case Parity::even:
				this->impl->current.c_cflag &= ~PARODD;
				this->impl->current.c_cflag |= PARENB;
				this->impl->current.c_iflag &= ~IGNPAR;
				this->impl->current.c_iflag |= INPCK;
				break;
			case Parity::none:
				this->impl->current.c_cflag &= ~(PARENB | PARODD);
				this->impl->current.c_iflag &= ~INPCK;
				this->impl->current.c_iflag |= IGNPAR;
				break;
			case Parity::odd:
				this->impl->current.c_cflag |= PARENB | PARODD;
				this->impl->current.c_iflag &= ~IGNPAR;
				this->impl->current.c_iflag |= INPCK;
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
			case StopBits::s1:
				this->impl->current.StopBits = ONESTOPBIT;
				break;
			case StopBits::s2:
				this->impl->current.StopBits = TWOSTOPBITS;
				break;
			default:
				break;
			}
#else // WIN32
			switch (stopBits)
			{
			case StopBits::s1:
				this->impl->current.c_cflag &= ~CSTOPB;
				break;
			case StopBits::s2:
				this->impl->current.c_cflag |= CSTOPB;
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
			::DWORD numbytes;
			
			if (0 == ::WriteFile(this->impl->fd, buf, count, &numbytes, nullptr))
			{
				throw ComException(::GetLastError());
			}
#else // WIN32
			::ssize_t numbytes = ::write(this->impl->fd, buf, count);
			
			if (-1 == numbytes)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			return numbytes;
		}
	}
}
