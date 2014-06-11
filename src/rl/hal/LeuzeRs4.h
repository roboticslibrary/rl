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

#ifndef _RL_HAL_LEUZERS4_H_
#define _RL_HAL_LEUZERS4_H_

#include <rl/util/Timer.h>

#include "Device.h"
#include "Lidar.h"
#include "types.h"

namespace rl
{
	namespace hal
	{
		class Serial;
		
		class LeuzeRs4 : public Lidar
		{
		public:
			enum BaudRate
			{
				/** 4,800 bps. */
				BAUDRATE_4800BPS,
				/** 9,600 bps. */
				BAUDRATE_9600BPS,
				/** 19,200 bps. */
				BAUDRATE_19200BPS,
				/** 38,400 bps. */
				BAUDRATE_38400BPS,
				/** 57,600 bps. */
				BAUDRATE_57600BPS,
				/** 115,200 bps. */
				BAUDRATE_115200BPS,
				/** 345,600 bps. */
				BAUDRATE_345600BPS,
				/** 625,000 bps. */
				BAUDRATE_625000BPS
			};
			
			/**
			 * @param password String with 8 characters.
			 */
			LeuzeRs4(
				const ::std::string& device = "/dev/ttyS0",
				const BaudRate& baudRate = BAUDRATE_57600BPS,
				const ::std::string& password = "ROD4LE"
			);
			
			virtual ~LeuzeRs4();
			
			void close();
			
			BaudRate getBaudRate() const;
			
			void getDistances(::rl::math::Vector& distances) const;
			
			::std::size_t getDistancesCount() const;
			
			::rl::math::Real getDistancesMaximum(const ::std::size_t& i) const;
			
			::rl::math::Real getDistancesMinimum(const ::std::size_t& i) const;
			
			::rl::math::Real getResolution() const;
			
			::rl::math::Real getStartAngle() const;
			
			::rl::math::Real getStopAngle() const;
			
			void open();
			
			void reset();
			
#if 0
			void setBaudRate(const BaudRate& baudRate);
			
			void setOutputParameters(const uint16_t& startIndex, const uint16_t& stopIndex, const uint16_t& step, const uint16_t& type);
#endif
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			uint8_t crc(const uint8_t* buf, const ::std::size_t& len) const;
			
			::std::size_t recv(uint8_t* buf, const ::std::size_t& len);
			
			void send(uint8_t* buf, const ::std::size_t& len);
			
			BaudRate baudRate;
			
			uint8_t data[1099];
			
			BaudRate desired;
			
			bool far1;
			
			bool far2;
			
			bool fn1Fn2;
			
			bool near1;
			
			bool near2;
			
			::std::string password;
			
			Serial* serial;
			
			uint16_t startIndex;
			
			uint16_t stepSize;
			
			uint16_t stopIndex;
			
			::rl::util::Timer timer;
			
			uint16_t type;
		};
	}
}

#endif // _RL_HAL_LEUZERS4_H_
