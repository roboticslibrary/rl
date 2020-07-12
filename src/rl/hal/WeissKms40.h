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
		class RL_HAL_EXPORT WeissKms40 : public CyclicDevice, public SixAxisForceTorqueSensor
		{
		public:
			enum class SystemState
			{
				calibrationValid = 1,
				stable = 2,
				tara = 4,
				filterEnabled = 8,
				daqRunning = 16,
				scriptRunning = 32,
				calibrationExpired = 1024,
				tempWarning = 2048,
				overrunFx = 1048576,
				overrunFy = 2097152,
				overrunFz = 4194304,
				overrunMx = 8388608,
				overrunMy = 16777216,
				overrunMz = 33554432,
				calibrationFault = 67108864,
				tempFault = 134217728,
				powerFault = 268435456,
				cmdFailure = 536870912,
				scriptFailure = 1073741824
			};
			
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_SCRIPT_FAILURE = SystemState::scriptFailure;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_CMD_FAILURE = SystemState::cmdFailure;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_POWER_FAULT = SystemState::powerFault;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_TEMP_FAULT = SystemState::tempFault;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_CALIBRATION_FAULT = SystemState::calibrationFault;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_OVERRUN_MZ = SystemState::overrunMx;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_OVERRUN_MY = SystemState::overrunMy;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_OVERRUN_MX = SystemState::overrunMz;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_OVERRUN_FZ = SystemState::overrunFx;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_OVERRUN_FY = SystemState::overrunFy;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_OVERRUN_FX = SystemState::overrunFz;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_TEMP_WARNING = SystemState::tempWarning;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_CALIBRATION_EXPIRED = SystemState::calibrationExpired;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_SCRIPT_RUNNING = SystemState::scriptRunning;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_DAQ_RUNNING = SystemState::daqRunning;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_FILTER_ENABLED = SystemState::filterEnabled;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_TARA = SystemState::tara;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_STABLE = SystemState::stable;
			RL_HAL_DEPRECATED static constexpr SystemState SYSTEM_STATE_CALIBRATION_VALID = SystemState::calibrationValid;
			
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
			
			::std::pair<::std::chrono::system_clock::time_point, ::std::chrono::system_clock::duration> doGetCalibrationDateLifetime();
			
			::rl::math::Matrix doGetCalibrationMatrix();
			
			::boost::dynamic_bitset<> doGetDataAcquisitionMask();
			
			::std::string doGetDescriptorString();
			
			::std::size_t doGetFilter();
			
			::std::string doGetFirmwareVersion();
			
			::std::size_t doGetFrameSendDivider();
			
			::std::size_t doGetSerialNumber();
			
			int doGetSystemFlags();
			
			::std::string doGetSystemType();
			
			bool doGetTare();
			
			/**
			 * @return [@htmlonly &#176C @endhtmlonly]
			 */
			float doGetTemperature();
			
			::std::size_t doGetVerboseLevel();
			
			::std::vector<::std::string> doPrintVariable(const ::std::vector<::std::string>& variables);
			
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
