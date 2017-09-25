//
// Copyright (c) 2016, Markus Rickert
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

#ifndef RL_HAL_WEISSKMS40_H
#define RL_HAL_WEISSKMS40_H

#include <cstdint>
#include <string>
#include <boost/dynamic_bitset.hpp>
#include <rl/math/Matrix.h>

#include "CyclicDevice.h"
#include "Device.h"
#include "SixAxisForceTorqueSensor.h"
#include "Socket.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Weiss Robotics Force-Torque Sensor KMS 40.
		 * 
		 * See "KMS Command Set Reference Manual":
		 * <www.weiss-robotics.de/en/download.html?cid=70&fid=37&id=164>
		 */
		class WeissKms40 : public CyclicDevice, public SixAxisForceTorqueSensor
		{
		public:
			enum SystemState
			{
				SYSTEM_STATE_SCRIPT_FAILURE = 1073741824,
				SYSTEM_STATE_CMD_FAILURE = 536870912,
				SYSTEM_STATE_POWER_FAULT = 268435456,
				SYSTEM_STATE_TEMP_FAULT = 134217728,
				SYSTEM_STATE_CALIBRATION_FAULT = 67108864,
				SYSTEM_STATE_OVERRUN_MZ = 33554432,
				SYSTEM_STATE_OVERRUN_MY = 16777216,
				SYSTEM_STATE_OVERRUN_MX = 8388608,
				SYSTEM_STATE_OVERRUN_FZ = 4194304,
				SYSTEM_STATE_OVERRUN_FY = 2097152,
				SYSTEM_STATE_OVERRUN_FX = 1048576,
				SYSTEM_STATE_TEMP_WARNING = 2048,
				SYSTEM_STATE_CALIBRATION_EXPIRED = 1024,
				SYSTEM_STATE_SCRIPT_RUNNING = 32,
				SYSTEM_STATE_DAQ_RUNNING = 16,
				SYSTEM_STATE_FILTER_ENABLED = 8,
				SYSTEM_STATE_TARA = 4,
				SYSTEM_STATE_STABLE = 2,
				SYSTEM_STATE_CALIBRATION_VALID = 1
			};
			
			/**
			 * @param[in] address TCP hostname
			 * @param[in] port TCP port
			 * @param[in] filter Value representing the ID of the filter that should be used
			 * @param[in] divider Value representing the frame rate divider
			 */
			WeissKms40(
				const ::std::string& address = "192.168.1.30",
				const unsigned short int& port = 1000,
				const ::std::size_t& filter = 0,
				const ::std::size_t& divider = 1
			);
			
			virtual ~WeissKms40();
			
			void close();
			
			::rl::math::Vector doAcquireSingleFrame();
			
			::std::pair< ::std::chrono::system_clock::time_point, ::std::chrono::system_clock::duration> doGetCalibrationDateLifetime();
			
			::rl::math::Matrix doGetCalibrationMatrix();
			
			::boost::dynamic_bitset<> doGetDataAcquisitionMask();
			
			::std::string doGetDescriptorString();
			
			::std::size_t doGetFilter();
			
			::std::string doGetFirmwareVersion();
			
			::std::size_t doGetFrameSendDivider();
			
			::std::size_t doGetSerialNumber();
			
			SystemState doGetSystemFlags();
			
			::std::string doGetSystemType();
			
			bool doGetTare();
			
			/**
			 * @return [@htmlonly &#176C @endhtmlonly]
			 */
			float doGetTemperature();
			
			::std::size_t doGetVerboseLevel();
			
			::std::vector< ::std::string> doPrintVariable(const ::std::vector< ::std::string>& variables);
			
			::boost::dynamic_bitset<> doSetDataAcquisitionMask(const ::boost::dynamic_bitset<>& mask);
			
			::std::string doSetDescriptorString(const ::std::string& value);
			
			::std::size_t doSetFilter(const ::std::size_t& value);
			
			::std::size_t doSetFrameSendDivider(const ::std::size_t& value);
			
			bool doSetTare(const bool& doOn);
			
			::std::size_t doSetVerboseLevel(const ::std::size_t& level);
			
			void doStartContinuousDataAcquisition();
			
			void doStopContinuousDataAcquisition();
			
			::rl::math::Vector getForces() const;
			
			::rl::math::Vector getForcesTorques() const;
			
			::rl::math::Real getForcesTorquesMaximum(const ::std::size_t& i) const;
			
			::rl::math::Real getForcesTorquesMinimum(const ::std::size_t& i) const;
			
			::rl::math::Vector getTorques() const;
			
			void open();
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			::std::string recv(const ::std::string& command);
			
			::std::size_t divider;
			
			::std::size_t filter;
			
			::rl::math::Vector frame;
			
			Socket socket;
		};
	}
}

#endif // RL_HAL_WEISSKMS40_H
