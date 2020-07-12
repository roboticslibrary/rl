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
			enum class JointMode
			{
				reset = 235,
				shuttingDown = 236,
				partDCalibration = 237,
				backdrive = 238,
				powerOff = 239,
				notResponding = 245,
				motorInitialisation = 246,
				booting = 247,
				partDCalibrationError = 248,
				bootloader = 249,
				calibration = 250,
				violation = 251,
				fault = 252,
				running = 253,
				idle = 255
			};
			
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_RESET = JointMode::reset;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_SHUTTING_DOWN = JointMode::shuttingDown;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_PART_D_CALIBRATION = JointMode::partDCalibration;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_BACKDRIVE = JointMode::backdrive;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_POWER_OFF = JointMode::powerOff;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_NOT_RESPONDING = JointMode::notResponding;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_MOTOR_INITIALISATION = JointMode::motorInitialisation;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_BOOTING = JointMode::booting;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_PART_D_CALIBRATION_ERROR = JointMode::partDCalibrationError;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_BOOTLOADER = JointMode::bootloader;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_CALIBRATION = JointMode::calibration;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_VIOLATION = JointMode::violation;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_FAULT = JointMode::fault;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_RUNNING = JointMode::running;
			RL_HAL_DEPRECATED static constexpr JointMode JOINT_MODE_IDLE = JointMode::idle;
			
			enum class ProgramState
			{
				stopping = 0,
				stopped = 1,
				playing = 2,
				pausing = 3,
				paused = 4,
				resuming = 5
			};
			
			RL_HAL_DEPRECATED static constexpr ProgramState PROGRAM_STATE_STOPPING = ProgramState::stopping;
			RL_HAL_DEPRECATED static constexpr ProgramState PROGRAM_STATE_STOPPED = ProgramState::stopped;
			RL_HAL_DEPRECATED static constexpr ProgramState PROGRAM_STATE_PLAYING = ProgramState::playing;
			RL_HAL_DEPRECATED static constexpr ProgramState PROGRAM_STATE_PAUSING = ProgramState::pausing;
			RL_HAL_DEPRECATED static constexpr ProgramState PROGRAM_STATE_PAUSED = ProgramState::paused;
			RL_HAL_DEPRECATED static constexpr ProgramState PROGRAM_STATE_RESUMING = ProgramState::resuming;
			
			enum class RobotMode
			{
				noController = -1,
				disconnected = 0,
				confirmSafety = 1,
				booting = 2,
				powerOff = 3,
				powerOn = 4,
				idle = 5,
				backdrive = 6,
				running = 7,
				updatingFirmware = 8
			};
			
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_NO_CONTROLLER = RobotMode::noController;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_DISCONNECTED = RobotMode::disconnected;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_CONFIRM_SAFETY = RobotMode::confirmSafety;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_BOOTING = RobotMode::booting;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_POWER_OFF = RobotMode::powerOff;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_POWER_ON = RobotMode::powerOn;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_IDLE = RobotMode::idle;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_BACKDRIVE = RobotMode::backdrive;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_RUNNING = RobotMode::running;
			RL_HAL_DEPRECATED static constexpr RobotMode ROBOT_MODE_UPDATING_FIRMWARE = RobotMode::updatingFirmware;
			
			enum class SafetyMode
			{
				normal = 1,
				reduced = 2,
				protectiveStop = 3,
				recovery = 4,
				safeguardStop = 5,
				systemEmergencyStop = 6,
				robotEmergencyStop = 7,
				violation = 8,
				fault = 9,
				validateJointId = 10,
				undefinedSafetyMode = 11
			};
			
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_NORMAL = SafetyMode::normal;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_REDUCED = SafetyMode::reduced;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_PROTECTIVE_STOP = SafetyMode::protectiveStop;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_RECOVERY = SafetyMode::recovery;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_SAFEGUARD_STOP = SafetyMode::safeguardStop;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_SYSTEM_EMERGENCY_STOP = SafetyMode::systemEmergencyStop;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_ROBOT_EMERGENCY_STOP = SafetyMode::robotEmergencyStop;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_VIOLATION = SafetyMode::violation;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_FAULT = SafetyMode::fault;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_VALIDATE_JOINT_ID = SafetyMode::validateJointId;
			RL_HAL_DEPRECATED static constexpr SafetyMode SAFETY_MODE_UNDEFINED_SAFETY_MODE = SafetyMode::undefinedSafetyMode;
			
			enum class SafetyStatus
			{
				normal = 1,
				reduced = 2,
				protectiveStop = 3,
				recovery = 4,
				safeguardStop = 5,
				systemEmergencyStop = 6,
				robotEmergencyStop = 7,
				violation = 8,
				fault = 9,
				validateJointId = 10,
				undefinedSafetyMode = 11,
				automaticModeSafeguardStop = 12,
				systemThreePositionEnablingStop = 13
			};
			
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_NORMAL = SafetyStatus::normal;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_REDUCED = SafetyStatus::reduced;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_PROTECTIVE_STOP = SafetyStatus::protectiveStop;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_RECOVERY = SafetyStatus::recovery;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_SAFEGUARD_STOP = SafetyStatus::safeguardStop;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_SYSTEM_EMERGENCY_STOP = SafetyStatus::systemEmergencyStop;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_ROBOT_EMERGENCY_STOP = SafetyStatus::robotEmergencyStop;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_VIOLATION = SafetyStatus::violation;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_FAULT = SafetyStatus::fault;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_VALIDATE_JOINT_ID = SafetyStatus::validateJointId;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_UNDEFINED_SAFETY_MODE = SafetyStatus::undefinedSafetyMode;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_AUTOMATIC_MODE_SAFEGUARD_STOP = SafetyStatus::automaticModeSafeguardStop;
			RL_HAL_DEPRECATED static constexpr SafetyStatus SAFETY_STATUS_SYSTEM_THREE_POSITION_ENABLING_STOP = SafetyStatus::systemThreePositionEnablingStop;
			
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
