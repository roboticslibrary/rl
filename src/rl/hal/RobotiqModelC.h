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
			enum ActionStatus
			{
				ACTION_STATUS_STOPPED = 0x00,
				ACTION_STATUS_GOING = 0x01
			};
			
			enum ActivationStatus
			{
				ACTIVATION_STATUS_RESET = 0x00,
				ACTIVATION_STATUS_READY = 0x01
			};
			
			enum FaultStatus
			{
				FAULT_STATUS_NONE = 0x00,
				FAULT_STATUS_UNKNOWN_1 = 0x01,
				FAULT_STATUS_UNKNOWN_2 = 0x02,
				FAULT_STATUS_UNKNOWN_3 = 0x03,
				FAULT_STATUS_UNKNOWN_4 = 0x04,
				FAULT_STATUS_NOTICE_ACTIVATION_DELAYED = 0x05,
				FAULT_STATUS_NOTICE_MODE_DELAYED = 0x06,
				FAULT_STATUS_NOTICE_ACTIVATION_NEEDED = 0x07,
				FAULT_STATUS_WARNING_TEMPERATURE = 0x08,
				FAULT_STATUS_WARNING_COMM_NOT_READY = 0x09,
				FAULT_STATUS_WARNING_VOLTAGE = 0x0A,
				FAULT_STATUS_WARNING_AUTOMATIC_RELEASE = 0x0B,
				FAULT_STATUS_ERROR_INTERNAL = 0x0C,
				FAULT_STATUS_ERROR_ACTIVATION_FAULT = 0x0D,
				FAULT_STATUS_ERROR_MODE_FAULT = 0x0E,
				FAULT_STATUS_ERROR_AUTOMATIC_RELEASE_COMPLETE = 0x0F
			};
			
			enum GripperStatus
			{
				GRIPPER_STATUS_RESET = 0x00,
				GRIPPER_STATUS_ACTIVATING = 0x01,
				GRIPPER_STATUS_UNUSED = 0x02,
				GRIPPER_STATUS_READY = 0x03
			};
			
			enum ObjectStatus
			{
				OBJECT_STATUS_MOTION = 0x00,
				OBJECT_STATUS_CONTACT_OPEN = 0x01,
				OBJECT_STATUS_CONTACT_CLOSE = 0x02,
				OBJECT_STATUS_MOTION_COMPLETE = 0x03
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
			
			::std::array< ::std::uint8_t, 32> in;
			
			::std::array< ::std::uint8_t, 32> out;
			
			Serial serial;
		};
	}
}

#endif // RL_HAL_ROBOTIQMODELC_H
