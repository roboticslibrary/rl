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

#ifndef RL_HAL_ROBOTIQMODELC_H
#define RL_HAL_ROBOTIQMODELC_H

#include <array>
#include <cstdint>

#include "CyclicDevice.h"
#include "DeviceException.h"
#include "Gripper.h"
#include "Serial.h"

namespace rl
{
	namespace hal
	{
		class RL_HAL_EXPORT RobotiqModelC : public CyclicDevice, public Gripper
		{
		public:
			enum class ActionStatus
			{
				stopped = 0x00,
				going = 0x01
			};
			
			enum class ActivationStatus
			{
				reset = 0x00,
				ready = 0x01
			};
			
			enum class FaultStatus
			{
				none = 0x00,
				unknown1 = 0x01,
				unknown2 = 0x02,
				unknown3 = 0x03,
				unknown4 = 0x04,
				noticeActivationDelayed = 0x05,
				noticeModeDelayed = 0x06,
				noticeActivationNeeded = 0x07,
				warningTemperature = 0x08,
				warningCommNotReady = 0x09,
				warningVoltage = 0x0a,
				warningAutomaticRelease = 0x0b,
				errorInternal = 0x0c,
				errorActivationFault = 0x0d,
				errorModeFault = 0x0e,
				errorAutomaticReleaseComplete = 0x0f
			};
			
			enum class GripperStatus
			{
				reset = 0x00,
				activating = 0x01,
				unused = 0x02,
				ready = 0x03
			};
			
			enum class ObjectStatus
			{
				motion = 0x00,
				contactOpen = 0x01,
				contactClose = 0x02,
				motionComplete = 0x03
			};
			
			class Exception : public DeviceException
			{
			public:
				Exception(const FaultStatus& faultStatus);
				
				virtual ~Exception() throw();
				
				FaultStatus getFaultStatus() const;
				
				virtual const char* what() const throw();
				
			protected:
				
			private:
				FaultStatus faultStatus;
			};
			
			RobotiqModelC(const ::std::string& filename);
			
			virtual ~RobotiqModelC();
			
			void close();
			
			ActionStatus getActionStatus() const;
			
			ActivationStatus getActivationStatus() const;
			
			::rl::math::Real getCurrent() const;
			
			FaultStatus getFaultStatus() const;
			
			GripperStatus getGripperStatus() const;
			
			ObjectStatus getObjectStatus() const;
			
			::rl::math::Real getPositionPercentage() const;
			
			::rl::math::Real getPositionRequestEchoPercentage() const;
			
			void halt();
			
			void open();
			
			void release();
			
			void setForcePercentage(const ::rl::math::Real& forcePercentage);
			
			void setPositionPercentage(const ::rl::math::Real& positionPercentage);
			
			void setSpeedPercentage(const ::rl::math::Real& speedPercentage);
			
			void shut();
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			::std::uint16_t crc(const ::std::uint8_t* buf, const ::std::size_t& len) const;
			
			::std::size_t recv(::std::uint8_t* buf, const ::std::size_t& len);
			
			void send(::std::uint8_t* buf, const ::std::size_t& len);
			
			::std::array<::std::uint8_t, 32> in;
			
			::std::array<::std::uint8_t, 32> out;
			
			Serial serial;
		};
	}
}

#endif // RL_HAL_ROBOTIQMODELC_H
