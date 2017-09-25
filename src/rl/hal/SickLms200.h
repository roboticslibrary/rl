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

#ifndef RL_HAL_SICKLMS200_H
#define RL_HAL_SICKLMS200_H

#include <array>
#include <cstdint>

#include "CyclicDevice.h"
#include "Device.h"
#include "Lidar.h"
#include "Serial.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Sick LMS 200 laser measurement system.
		 */
		class SickLms200 : public CyclicDevice, public Lidar
		{
		public:
			enum BaudRate
			{
				/** 9,600 bps. */
				BAUDRATE_9600BPS,
				/** 19,200 bps. */
				BAUDRATE_19200BPS,
				/** 38,400 bps. */
#if defined(WIN32) || defined(__QNX__)
				BAUDRATE_38400BPS
#else // defined(WIN32) || defined(__QNX__)
				BAUDRATE_38400BPS,
				/** 500,000 bps. */
				BAUDRATE_500000BPS
#endif // defined(WIN32) || defined(__QNX__)
			};
			
			enum Measuring
			{
				MEASURING_8M,
				MEASURING_16M,
				MEASURING_32M,
				MEASURING_80M,
				MEASURING_160M,
				MEASURING_320M
			};
			
			enum Monitoring
			{
				MONITORING_CONTINUOUS,
				MONITORING_SINGLE
			};
			
			enum Variant
			{
				/** Angle = 100 degrees, resolution = 0.25 degrees. */
				VARIANT_100_25,
				/** Angle = 100 degrees, resolution = 0.5 degrees. */
				VARIANT_100_50,
				/** Angle = 100 degrees, resolution = 1 degree. */
				VARIANT_100_100,
				/** Angle = 180 degrees, resolution = 0.5 degrees. */
				VARIANT_180_50,
				/** Angle = 180 degrees, resolution = 1 degree. */
				VARIANT_180_100
			};
			
			/**
			 * @param[in] password String with 8 characters comprising "0...9", "a...z", "A...Z", and "_".
			 */
			SickLms200(
				const ::std::string& device = "/dev/ttyS0",
				const BaudRate& baudRate = BAUDRATE_9600BPS,
				const Monitoring& monitoring = MONITORING_SINGLE,
				const Variant& variant = VARIANT_180_50,
				const Measuring& measuring = MEASURING_8M,
				const ::std::string& password = "SICK_LMS"
			);
			
			virtual ~SickLms200();
			
			void close();
			
			void dumpConfiguration();
			
			void dumpStatus();
			
			BaudRate getBaudRate() const;
			
			::rl::math::Vector getDistances() const;
			
			::std::size_t getDistancesCount() const;
			
			::rl::math::Real getDistancesMaximum(const ::std::size_t& i) const;
			
			::rl::math::Real getDistancesMinimum(const ::std::size_t& i) const;
			
			Measuring getMeasuring() const;
			
			Monitoring getMonitoring() const;
			
			::rl::math::Real getResolution() const;
			
			::rl::math::Real getStartAngle() const;
			
			::rl::math::Real getStopAngle() const;
			
			::std::string getType();
			
			Variant getVariant() const;
			
			void open();
			
			void reset();
			
			void setBaudRate(const BaudRate& baudRate);
			
			void setMeasuring(const Measuring& measuring);
			
			void setMonitoring(const Monitoring& monitoring);
			
			void setVariant(const Variant& variant);
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			::std::uint16_t crc(const ::std::uint8_t* buf, const ::std::size_t& len) const;
			
			::std::size_t recv(::std::uint8_t* buf, const ::std::size_t& len, const ::std::uint8_t& command);
			
			void send(::std::uint8_t* buf, const ::std::size_t& len);
			
			bool waitAck();
			
			BaudRate baudRate;
			
			::std::uint8_t configuration;
			
			::std::array< ::std::uint8_t, 812> data;
			
			BaudRate desired;
			
			Measuring measuring;
			
			Monitoring monitoring;
			
			::std::string password;
			
			Serial serial;
			
			Variant variant;
		};
	}
}

#endif // RL_HAL_SICKLMS200_H
