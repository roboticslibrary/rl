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

#include <chrono>
#include <memory>
#include <string>
#include <rl/math/Real.h>

#include "Com.h"

namespace rl
{
	namespace hal
	{
		class RL_HAL_EXPORT Serial : public Com
		{
		public:
			enum class BaudRate
			{
				/** 110 bps. */
				b110,
				/** 300 bps. */
				b300,
				/** 600 bps. */
				b600,
				/** 1,200 bps. */
				b1200,
				/** 2,400 bps. */
				b2400,
				/** 4,800 bps. */
				b4800,
				/** 9,600 bps. */
				b9600,
#ifdef WIN32
				/** 14,400 bps. */
				b14400,
#endif // WIN32
				/** 19,200 bps. */
				b19200,
				/** 38,400 bps. */
				b38400,
				/** 57,600 bps. */
				b57600,
				/** 115,200 bps. */
#ifdef __QNX__
				b115200
#else // __QNX__
				b115200,
#endif // __QNX__
#ifdef WIN32
				/** 128,000 bps. */
				b128000,
				/** 256,000 bps. */
				b256000
#else // WIN32
#ifndef __QNX__
				/** 230,400 bps. */
				b230400,
				/** 460,800 bps. */
				b460800,
				/** 500,000 bps. */
				b500000,
				/** 576,000 bps. */
				b576000,
				/** 921,600 bps. */
				b921600,
				/** 1,000,000 bps. */
				b1000000,
				/** 1,152000 bps. */
				b1152000,
				/** 1,500,000 bps. */
				b1500000,
				/** 2,000,000 bps. */
				b2000000,
				/** 2,500,000 bps. */
				b2500000,
				/** 3,000,000 bps. */
#ifdef __CYGWIN__
				b3000000
#else // __CYGWIN__
				b3000000,
				/** 3,500,000 bps. */
				b3500000,
				/** 4,000,000 bps. */
				b4000000
#endif // __CYGWIN__
#endif // __QNX__
#endif // WIN32
			};
			
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_110BPS = BaudRate::b110;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_300BPS = BaudRate::b300;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_600BPS = BaudRate::b600;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_1200BPS = BaudRate::b1200;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_2400BPS = BaudRate::b2400;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_4800BPS = BaudRate::b4800;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_9600BPS = BaudRate::b9600;
#ifdef WIN32
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_14400BPS = BaudRate::b14400;
#endif // WIN32
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_19200BPS = BaudRate::b19200;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_38400BPS = BaudRate::b38400;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_57600BPS = BaudRate::b57600;
#ifdef __QNX__
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_115200BPS = BaudRate::b115200;
#else // __QNX__
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_115200BPS = BaudRate::b115200;
#endif // __QNX__
#ifdef WIN32
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_128000BPS = BaudRate::b128000;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_256000BPS = BaudRate::b256000;
#else // WIN32
#ifndef __QNX__
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_230400BPS = BaudRate::b230400;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_460800BPS = BaudRate::b460800;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_500000BPS = BaudRate::b500000;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_576000BPS = BaudRate::b576000;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_921600BPS = BaudRate::b921600;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_1000000BPS = BaudRate::b1000000;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_1152000BPS = BaudRate::b1152000;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_1500000BPS = BaudRate::b1500000;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_2000000BPS = BaudRate::b2000000;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_2500000BPS = BaudRate::b2500000;
#ifdef __CYGWIN__
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_3000000BPS = BaudRate::b3000000;
#else // __CYGWIN__
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_3000000BPS = BaudRate::b3000000;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_3500000BPS = BaudRate::b3500000;
			RL_HAL_DEPRECATED static constexpr BaudRate BAUDRATE_4000000BPS = BaudRate::b4000000;
#endif // __CYGWIN__
#endif // __QNX__
#endif // WIN32
			
			enum class DataBits
			{
				/** 5 data bits. */
				d5,
				/** 6 data bits. */
				d6,
				/** 7 data bits. */
				d7,
				/** 8 data bits. */
				d8
			};
			
			RL_HAL_DEPRECATED static constexpr DataBits DATABITS_5BITS = DataBits::d5;
			RL_HAL_DEPRECATED static constexpr DataBits DATABITS_6BITS = DataBits::d6;
			RL_HAL_DEPRECATED static constexpr DataBits DATABITS_7BITS = DataBits::d7;
			RL_HAL_DEPRECATED static constexpr DataBits DATABITS_8BITS = DataBits::d8;
			
			enum class FlowControl
			{
				/** No flow control. */
				off,
				/** Hardware flow control (RTS/CTS). */
				rtscts,
				/** Software flow control (XON/XOFF). */
				xonxoff
			};
			
			RL_HAL_DEPRECATED static constexpr FlowControl FLOWCONTROL_OFF = FlowControl::off;
			RL_HAL_DEPRECATED static constexpr FlowControl FLOWCONTROL_RTSCTS = FlowControl::rtscts;
			RL_HAL_DEPRECATED static constexpr FlowControl FLOWCONTROL_XONXOFF = FlowControl::xonxoff;
			
			enum class Parity
			{
				/** Even parity. */
				even,
				/** No parity. */
				none,
				/** Odd parity. */
				odd
			};
			
			RL_HAL_DEPRECATED static constexpr Parity PARITY_EVENPARITY = Parity::even;
			RL_HAL_DEPRECATED static constexpr Parity PARITY_NOPARITY = Parity::none;
			RL_HAL_DEPRECATED static constexpr Parity PARITY_ODDPARITY = Parity::odd;
			
			enum class StopBits
			{
				/** 1 stop bit. */
				s1,
				/** 2 stop bits. */
				s2
			};
			
			RL_HAL_DEPRECATED static constexpr StopBits STOPBITS_1BIT = StopBits::s1;
			RL_HAL_DEPRECATED static constexpr StopBits STOPBITS_2BITS = StopBits::s2;
			
			Serial(
				const ::std::string& filename,
				const BaudRate& baudRate = BaudRate::b9600,
				const DataBits& dataBits = DataBits::d8,
				const FlowControl& flowControl = FlowControl::off,
				const Parity& parity = Parity::none,
				const StopBits& stopBits = StopBits::s1
			);
			
			Serial(
				const ::std::string& filename,
				const BaudRate& baudRate,
				const DataBits& dataBits,
				const FlowControl& flowControl,
				const Parity& parity,
				const StopBits& stopBits,
				const int& flags
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
			struct Impl;
			
			BaudRate baudRate;
			
			DataBits dataBits;
			
			::std::string filename;
			
			int flags;
			
			FlowControl flowControl;
			
			::std::unique_ptr<Impl> impl;
			
			Parity parity;
			
			StopBits stopBits;
		};
	}
}

#endif // RL_HAL_SERIAL_H
