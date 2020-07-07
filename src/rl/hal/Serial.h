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
			
			enum class FlowControl
			{
				/** No flow control. */
				off,
				/** Hardware flow control (RTS/CTS). */
				rtscts,
				/** Software flow control (XON/XOFF). */
				xonxoff
			};
			
			enum class Parity
			{
				/** Even parity. */
				even,
				/** No parity. */
				none,
				/** Odd parity. */
				odd
			};
			
			enum class StopBits
			{
				/** 1 stop bit. */
				s1,
				/** 2 stop bits. */
				s2
			};
			
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
