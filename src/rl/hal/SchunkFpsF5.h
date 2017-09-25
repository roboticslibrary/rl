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

#ifndef RL_HAL_SCHUNKFPSF5_H
#define RL_HAL_SCHUNKFPSF5_H

#include <cstdint>
#include <set>

#include "CyclicDevice.h"
#include "RangeSensor.h"
#include "Serial.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Schunk FPS-F5 flexible position sensor.
		 */
		class SchunkFpsF5 : public CyclicDevice, public RangeSensor
		{
		public:
			SchunkFpsF5(const ::std::string& device = "/dev/ttyS0");
			
			virtual ~SchunkFpsF5();
			
			void close();
			
			::rl::math::Vector getDistances() const;
			
			::std::size_t getDistancesCount() const;
			
			::rl::math::Real getDistancesMaximum(const ::std::size_t& i) const;
			
			::rl::math::Real getDistancesMinimum(const ::std::size_t& i) const;
			
			::rl::math::Real getTemperature() const;
			
			::rl::math::Real getVoltage() const;
			
			bool isA() const;
			
			bool isArea() const;
			
			bool isB() const;
			
			bool isC() const;
			
			bool isClosed() const;
			
			bool isOpened() const;
			
			bool isReCalc() const;
			
			bool isRecord() const;
			
			bool isUpdate() const;
			
			void open();
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			::std::uint16_t crc(const ::std::uint8_t* buf, const ::std::size_t& len) const;
			
			::std::size_t recv(::std::uint8_t* buf, const ::std::size_t& len, const ::std::uint8_t& command);
			
			void send(::std::uint8_t* buf, const ::std::size_t& len);
			
			bool a;
			
			bool area;
			
			bool b;
			
			bool c;
			
			bool closed;
			
			::std::set< ::std::pair< ::rl::math::Real, ::rl::math::Real>> fulcrums;
			
			::rl::math::Real interpolated;
			
			bool opened;
			
			bool reCalc;
			
			bool record;
			
			Serial serial;
			
			::rl::math::Real temperature;
			
			bool update;
			
			::rl::math::Real value;
			
			::rl::math::Real voltage;
		};
	}
}

#endif // RL_HAL_SCHUNKFPSF5_H
