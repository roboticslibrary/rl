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

#ifndef RL_HAL_SERIAL_H
#define RL_HAL_SERIAL_H

#ifdef WIN32
#include <windows.h>
#else // WIN32
#include <fcntl.h>
#include <termios.h>
#endif // WIN32

#include <chrono>
#include <string>
#include <rl/math/Real.h>

#include "Com.h"

namespace rl
{
	namespace hal
	{
		class Serial : public Com
		{
		public:
			enum BaudRate
			{
				/** 110 bps. */
				BAUDRATE_110BPS,
				/** 300 bps. */
				BAUDRATE_300BPS,
				/** 600 bps. */
				BAUDRATE_600BPS,
				/** 1,200 bps. */
				BAUDRATE_1200BPS,
				/** 2,400 bps. */
				BAUDRATE_2400BPS,
				/** 4,800 bps. */
				BAUDRATE_4800BPS,
				/** 9,600 bps. */
				BAUDRATE_9600BPS,
#ifdef WIN32
				/** 14,400 bps. */
				BAUDRATE_14400BPS,
#endif // WIN32
				/** 19,200 bps. */
				BAUDRATE_19200BPS,
				/** 38,400 bps. */
				BAUDRATE_38400BPS,
				/** 57,600 bps. */
				BAUDRATE_57600BPS,
				/** 115,200 bps. */
#ifdef __QNX__
				BAUDRATE_115200BPS
#else // __QNX__
				BAUDRATE_115200BPS,
#endif // __QNX__
#ifdef WIN32
				/** 128,000 bps. */
				BAUDRATE_128000BPS,
				/** 256,000 bps. */
				BAUDRATE_256000BPS
#else // WIN32
#ifndef __QNX__
				/** 230,400 bps. */
				BAUDRATE_230400BPS,
				/** 460,800 bps. */
				BAUDRATE_460800BPS,
				/** 500,000 bps. */
				BAUDRATE_500000BPS,
				/** 576,000 bps. */
				BAUDRATE_576000BPS,
				/** 921,600 bps. */
				BAUDRATE_921600BPS,
				/** 1,000,000 bps. */
				BAUDRATE_1000000BPS,
				/** 1,152000 bps. */
				BAUDRATE_1152000BPS,
				/** 1,500,000 bps. */
				BAUDRATE_1500000BPS,
				/** 2,000,000 bps. */
				BAUDRATE_2000000BPS,
				/** 2,500,000 bps. */
				BAUDRATE_2500000BPS,
				/** 3,000,000 bps. */
				BAUDRATE_3000000BPS,
				/** 3,500,000 bps. */
				BAUDRATE_3500000BPS,
				/** 4,000,000 bps. */
				BAUDRATE_4000000BPS
#endif // __QNX__
#endif // WIN32
			};
			
			enum DataBits
			{
				/** 5 data bits. */
				DATABITS_5BITS,
				/** 6 data bits. */
				DATABITS_6BITS,
				/** 7 data bits. */
				DATABITS_7BITS,
				/** 8 data bits. */
				DATABITS_8BITS
			};
			
			enum FlowControl
			{
				/** No flow control. */
				FLOWCONTROL_OFF,
				/** Hardware flow control (RTS/CTS). */
				FLOWCONTROL_RTSCTS,
				/** Software flow control (XON/XOFF). */
				FLOWCONTROL_XONXOFF
			};
			
			enum Parity
			{
				/** Even parity. */
				PARITY_EVENPARITY,
				/** No parity. */
				PARITY_NOPARITY,
				/** Odd parity. */
				PARITY_ODDPARITY
			};
			
			enum StopBits
			{
				/** 1 stop bit. */
				STOPBITS_1BIT,
				/** 2 stop bits. */
				STOPBITS_2BITS
			};
			
			Serial(
				const ::std::string& filename,
				const BaudRate& baudRate = BAUDRATE_9600BPS,
				const DataBits& dataBits = DATABITS_8BITS,
				const FlowControl& flowControl = FLOWCONTROL_OFF,
				const Parity& parity = PARITY_NOPARITY,
				const StopBits& stopBits = STOPBITS_1BIT,
#ifdef WIN32
				const int& flags = GENERIC_READ | GENERIC_WRITE
#else // WIN32
				const int& flags = O_RDWR | O_NONBLOCK | O_NOCTTY
#endif // WIN32
			);
			
			virtual ~Serial();
			
			void changeParameters();
			
			void close();
			
			void doBreak(const bool& doOn);
			
			void doDtr(const bool& doOn);
			
			void doModemStatus(bool& ctsOn, bool& dsrOn, bool& riOn, bool& dcdOn);
			
			void doRts(const bool& doOn);
			
			void flush(const bool& read, const bool& write);
			
			const BaudRate& getBaudRate() const;
			
			const DataBits& getDataBits() const;
			
			const ::std::string& getFilename() const;
			
			const FlowControl& getFlowControl() const;
			
			const Parity& getParity() const;
			
			const StopBits& getStopBits() const;
			
			void open();
			
			::std::size_t read(void* buf, const ::std::size_t& count);
			
			::std::size_t select(const bool& read, const bool& write, const ::std::chrono::nanoseconds& timeout);
			
			void setBaudRate(const BaudRate& baudRate);
			
			void setDataBits(const DataBits& dataBits);
			
			void setFilename(const ::std::string& filename);
			
			void setFlowControl(const FlowControl& flowControl);
			
			void setParity(const Parity& parity);
			
			void setStopBits(const StopBits& stopBits);
			
			::std::size_t write(const void* buf, const ::std::size_t& count);
			
		protected:
			
		private:
			BaudRate baudRate;
			
#ifdef WIN32
			DCB current;
#else // WIN32
			struct termios current;
#endif // WIN32
			
			DataBits dataBits;
			
#ifdef WIN32
			HANDLE fd;
#else // WIN32
			int fd;
#endif // WIN32
			
			::std::string filename;
			
			int flags;
			
			FlowControl flowControl;
			
			Parity parity;
			
#ifdef WIN32
			DCB restore;
#else // WIN32
			struct termios restore;
#endif // WIN32
			
			StopBits stopBits;
		};
	}
}

#endif // RL_HAL_SERIAL_H
