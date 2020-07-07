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
			
			enum class ProgramState
			{
				stopping = 0,
				stopped = 1,
				playing = 2,
				pausing = 3,
				paused = 4,
				resuming = 5
			};
			
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
