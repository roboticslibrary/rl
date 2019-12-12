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

#ifndef RL_HAL_UNIVERSALROBOTSREALTIME_H
#define RL_HAL_UNIVERSALROBOTSREALTIME_H

#include <cstdint>

#include "CartesianForceSensor.h"
#include "CartesianPositionSensor.h"
#include "CartesianVelocitySensor.h"
#include "CyclicDevice.h"
#include "DigitalInputReader.h"
#include "DigitalOutputReader.h"
#include "Endian.h"
#include "JointCurrentSensor.h"
#include "JointPositionSensor.h"
#include "JointVelocitySensor.h"
#include "Socket.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Universal Robots realtime interface.
		 * 
		 * Supports versions 1.5, 1.6, 1.7, 1.8, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6,
		 * 3.7, 3.8, 3.9, 3.10, 3.11, 3.12, 5.0, 5.1, 5.2, 5.3, 5.4, 5.5, 5.6.
		 */
		class RL_HAL_EXPORT UniversalRobotsRealtime :
			public CartesianForceSensor,
			public CartesianPositionSensor,
			public CartesianVelocitySensor,
			public CyclicDevice,
			public DigitalInputReader,
			public DigitalOutputReader,
			public JointCurrentSensor,
			public JointPositionSensor,
			public JointVelocitySensor
		{
		public:
			enum JointMode
			{
				JOINT_MODE_RESET = 235,
				JOINT_MODE_SHUTTING_DOWN = 236,
				JOINT_MODE_PART_D_CALIBRATION = 237,
				JOINT_MODE_BACKDRIVE = 238,
				JOINT_MODE_POWER_OFF = 239,
				JOINT_MODE_NOT_RESPONDING = 245,
				JOINT_MODE_MOTOR_INITIALISATION = 246,
				JOINT_MODE_BOOTING = 247,
				JOINT_MODE_PART_D_CALIBRATION_ERROR = 248,
				JOINT_MODE_BOOTLOADER = 249,
				JOINT_MODE_CALIBRATION = 250,
				JOINT_MODE_VIOLATION = 251,
				JOINT_MODE_FAULT = 252,
				JOINT_MODE_RUNNING = 253,
				JOINT_MODE_IDLE = 255
			};
			
			enum ProgramState
			{
				PROGRAM_STATE_STOPPING = 0,
				PROGRAM_STATE_STOPPED = 1,
				PROGRAM_STATE_PLAYING = 2,
				PROGRAM_STATE_PAUSING = 3,
				PROGRAM_STATE_PAUSED = 4,
				PROGRAM_STATE_RESUMING = 5
			};
			
			enum RobotMode
			{
				ROBOT_MODE_NO_CONTROLLER = -1,
				ROBOT_MODE_DISCONNECTED = 0,
				ROBOT_MODE_CONFIRM_SAFETY = 1,
				ROBOT_MODE_BOOTING = 2,
				ROBOT_MODE_POWER_OFF = 3,
				ROBOT_MODE_POWER_ON = 4,
				ROBOT_MODE_IDLE = 5,
				ROBOT_MODE_BACKDRIVE = 6,
				ROBOT_MODE_RUNNING = 7,
				ROBOT_MODE_UPDATING_FIRMWARE = 8
			};
			
			enum SafetyMode
			{
				SAFETY_MODE_NORMAL = 1,
				SAFETY_MODE_REDUCED = 2,
				SAFETY_MODE_PROTECTIVE_STOP = 3,
				SAFETY_MODE_RECOVERY = 4,
				SAFETY_MODE_SAFEGUARD_STOP = 5,
				SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 6,
				SAFETY_MODE_ROBOT_EMERGENCY_STOP = 7,
				SAFETY_MODE_VIOLATION = 8,
				SAFETY_MODE_FAULT = 9,
				SAFETY_MODE_VALIDATE_JOINT_ID = 10,
				SAFETY_MODE_UNDEFINED_SAFETY_MODE = 11
			};
			
			enum SafetyStatus
			{
				SAFETY_STATUS_NORMAL = 1,
				SAFETY_STATUS_REDUCED = 2,
				SAFETY_STATUS_PROTECTIVE_STOP = 3,
				SAFETY_STATUS_RECOVERY = 4,
				SAFETY_STATUS_SAFEGUARD_STOP = 5,
				SAFETY_STATUS_SYSTEM_EMERGENCY_STOP = 6,
				SAFETY_STATUS_ROBOT_EMERGENCY_STOP = 7,
				SAFETY_STATUS_VIOLATION = 8,
				SAFETY_STATUS_FAULT = 9,
				SAFETY_STATUS_VALIDATE_JOINT_ID = 10,
				SAFETY_STATUS_UNDEFINED_SAFETY_MODE = 11,
				SAFETY_STATUS_AUTOMATIC_MODE_SAFEGUARD_STOP = 12,
				SAFETY_STATUS_SYSTEM_THREE_POSITION_ENABLING_STOP = 13
			};
			
			UniversalRobotsRealtime(const ::std::string& address);
			
			virtual ~UniversalRobotsRealtime();
			
			void close();
			
			void doScript(const ::std::string& script);
			
			::rl::math::ForceVector getCartesianForce() const;
			
			::rl::math::Transform getCartesianPosition() const;
			
			::rl::math::MotionVector getCartesianVelocity() const;
			
			::boost::dynamic_bitset<> getDigitalInput() const;
			
			bool getDigitalInput(const ::std::size_t& i) const;
			
			::std::size_t getDigitalInputCount() const;
			
			::boost::dynamic_bitset<> getDigitalOutput() const;
			
			bool getDigitalOutput(const ::std::size_t& i) const;
			
			::std::size_t getDigitalOutputCount() const;
			
			::rl::math::Vector getJointCurrent() const;
			
			JointMode getJointMode(const ::std::size_t& i) const;
			
			::rl::math::Vector getJointPosition() const;
			
			::rl::math::Vector getJointVelocity() const;
			
			ProgramState getProgramState() const;
			
			RobotMode getRobotMode() const;
			
			SafetyMode getSafetyMode() const;
			
			SafetyStatus getSafetyStatus() const;
			
			void open();
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			struct Message
			{
				void unserialize(::std::uint8_t* ptr);
				
				template<typename T>
				void unserialize(::std::uint8_t*& ptr, T& t)
				{
					::std::memcpy(&t, ptr, sizeof(t));
					Endian::bigToHost(t);
					ptr += sizeof(t);
				}
				
				template<typename T, ::std::size_t N>
				void unserialize(::std::uint8_t*& ptr, T (&t)[N])
				{
					for (::std::size_t i = 0; i < N; ++i)
					{
						this->unserialize(ptr, t[i]);
					}
				}
				
				::std::uint32_t messageSize;
				
				double time;
				
				double qTarget[6];
				
				double qdTarget[6];
				
				double qddTarget[6];
				
				double iTarget[6];
				
				double mTarget[6];
				
				double qActual[6];
				
				double qdActual[6];
				
				double iActual[6];
				
				double iControl[6];
				
				double toolVectorActual[6];
				
				double tcpSpeedActual[6];
				
				double tcpForce[6];
				
				double toolVectorTarget[6];
				
				double tcpSpeedTarget[6];
				
				::std::int64_t digitalInputBits;
				
				double motorTemperatures[6];
				
				double controllerTimer;
				
				double testValue;
				
				double robotMode;
				
				double jointModes[6];
				
				double safetyMode;
				
				double toolAccelerometerValues[3];
				
				double speedScaling;
				
				double linearMomentumNorm;
				
				double vMain;
				
				double vRobot;
				
				double iRobot;
				
				double vActual[6];
				
				::std::int64_t digitalOutputs;
				
				double programState;
				
				double elbowPosition[3];
				
				double elbowVelocity[3];
				
				double safetyStatus;
			};
			
			Message in;
			
			Socket socket;
		};
		
		template<>
		void UniversalRobotsRealtime::Message::unserialize(::std::uint8_t*& ptr, ::std::int64_t& t);
	}
}

#endif // RL_HAL_UNIVERSALROBOTSREALTIME_H
