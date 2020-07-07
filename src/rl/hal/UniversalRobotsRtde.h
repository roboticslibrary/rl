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

#ifndef RL_HAL_UNIVERSALROBOTSRTDE_H
#define RL_HAL_UNIVERSALROBOTSRTDE_H

#include <cstdint>
#include <boost/optional.hpp>

#include "AnalogInputReader.h"
#include "AnalogOutputReader.h"
#include "AnalogOutputWriter.h"
#include "CartesianForceSensor.h"
#include "CartesianPositionSensor.h"
#include "CartesianVelocitySensor.h"
#include "CyclicDevice.h"
#include "DigitalInputReader.h"
#include "DigitalOutputReader.h"
#include "DigitalOutputWriter.h"
#include "Endian.h"
#include "JointAccelerationActuator.h"
#include "JointCurrentSensor.h"
#include "JointPositionActuator.h"
#include "JointPositionSensor.h"
#include "JointVelocityActuator.h"
#include "JointVelocitySensor.h"
#include "Socket.h"

namespace rl
{
	namespace hal
	{
		/**
		 * Universal Robots RTDE interface (3.3).
		 */
		class RL_HAL_EXPORT UniversalRobotsRtde :
			public CyclicDevice,
			public AnalogInputReader,
			public AnalogOutputReader,
			public AnalogOutputWriter,
			public CartesianForceSensor,
			public CartesianPositionSensor,
			public CartesianVelocitySensor,
			public DigitalInputReader,
			public DigitalOutputReader,
			public DigitalOutputWriter,
			public JointAccelerationActuator,
			public JointCurrentSensor,
			public JointPositionActuator,
			public JointPositionSensor,
			public JointVelocityActuator,
			public JointVelocitySensor
		{
		public:
			enum class JointMode
			{
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
				fault = 252,
				running = 253,
				idleMode = 255
			};
			
			enum class RobotMode
			{
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
			
			enum class RobotStatus
			{
				powerOn = 1,
				programRunning = 2,
				teachButtonPressed = 4,
				powerButtonPressed = 8
			};
			
			enum class RuntimeState
			{
				stopping = 0,
				stopped = 1,
				playing = 2,
				pausing = 3,
				paused = 4,
				resuming = 5
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
				fault = 9
			};
			
			enum class SafetyStatus
			{
				normalMode = 1,
				reducedMode = 2,
				protectiveStopped = 4,
				recoveryMode = 8,
				safeguardStopped = 16,
				systemEmergencyStopped = 32,
				robotEmergencyStopped = 64,
				emergencyStopped = 128,
				violation = 256,
				fault = 512,
				stoppedDueToSafety = 1024
			};
			
			UniversalRobotsRtde(const ::std::string& address, const ::std::chrono::nanoseconds& updateRate = ::std::chrono::milliseconds(8));
			
			virtual ~UniversalRobotsRtde();
			
			void close();
			
			void doScript(const ::std::string& script);
			
			using AnalogInputReader::getAnalogInput;
			
			::rl::math::Real getAnalogInput(const ::std::size_t& i) const;
			
			::std::size_t getAnalogInputCount() const;
			
			::rl::math::Real getAnalogInputMaximum(const ::std::size_t& i) const;
			
			::rl::math::Real getAnalogInputMinimum(const ::std::size_t& i) const;
			
			::std::vector<::rl::math::Units> getAnalogInputUnit() const;
			
			::rl::math::Units getAnalogInputUnit(const ::std::size_t& i) const;
			
			using AnalogOutputReader::getAnalogOutput;
			
			::rl::math::Real getAnalogOutput(const ::std::size_t& i) const;
			
			::std::size_t getAnalogOutputCount() const;
			
			::rl::math::Real getAnalogOutputMaximum(const ::std::size_t& i) const;
			
			::rl::math::Real getAnalogOutputMinimum(const ::std::size_t& i) const;
			
			::std::vector<::rl::math::Units> getAnalogOutputUnit() const;
			
			::rl::math::Units getAnalogOutputUnit(const ::std::size_t& i) const;
			
			::rl::math::ForceVector getCartesianForce() const;
			
			::rl::math::Transform getCartesianPosition() const;
			
			::rl::math::Transform getCartesianPositionTarget() const;
			
			::rl::math::MotionVector getCartesianVelocity() const;
			
			::rl::math::MotionVector getCartesianVelocityTarget() const;
			
			::boost::dynamic_bitset<> getDigitalInput() const;
			
			bool getDigitalInput(const ::std::size_t& i) const;
			
			::std::size_t getDigitalInputCount() const;
			
			::boost::dynamic_bitset<> getDigitalOutput() const;
			
			bool getDigitalOutput(const ::std::size_t& i) const;
			
			::std::size_t getDigitalOutputCount() const;
			
			::rl::math::Vector getJointCurrent() const;
			
			JointMode getJointMode(const ::std::size_t& i) const;
			
			::rl::math::Vector getJointPosition() const;
			
			::rl::math::Vector getJointTemperature() const;
			
			::rl::math::Vector getJointVelocity() const;
			
			RobotMode getRobotMode() const;
			
			::std::uint32_t getRobotStatusBits() const;
			
			RuntimeState getRuntimeState() const;
			
			SafetyMode getSafetyMode() const;
			
			::std::uint32_t getSafetyStatusBits() const;
			
			void open();
			
			using AnalogOutputWriter::setAnalogOutput;
			
			void setAnalogOutput(const ::std::size_t& i, const ::rl::math::Real& value);
			
			void setAnalogOutputUnit(const ::std::vector<::rl::math::Units>& values);
			
			void setAnalogOutputUnit(const ::std::size_t& i, const ::rl::math::Units& value);
			
			using DigitalOutputWriter::setDigitalOutput;
			
			void setDigitalOutput(const ::std::size_t& i, const bool& value);
			
			void setJointAcceleration(const ::rl::math::Vector& qdd);
			
			void setJointPosition(const ::rl::math::Vector& q);
			
			void setJointVelocity(const ::rl::math::Vector& qd);
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			enum class Command : ::std::uint8_t
			{
				controlPackagePause = 80,
				controlPackageSetupInputs = 73,
				controlPackageSetupOutputs = 79,
				controlPackageStart = 83,
				dataPackage = 85,
				getUrcontrolVersion = 118,
				requestProtocolVersion = 86,
				textMessage = 77
			};
			
			struct Input
			{
				::boost::optional<::std::uint8_t> configurableDigitalOutput;
				
				::boost::optional<::std::uint8_t> configurableDigitalOutputMask;
				
				::boost::optional<::std::uint32_t> inputBitRegisters0;
				
				::boost::optional<::std::uint32_t> inputBitRegisters1;
				
				::std::vector<double> inputDoubleRegister;
				
				::std::vector<::std::int32_t> inputIntRegister;
				
				::boost::optional<double> standardAnalogOutput0;
				
				::boost::optional<double> standardAnalogOutput1;
				
				::boost::optional<::std::uint8_t> standardAnalogOutputMask;
				
				::std::uint8_t standardAnalogOutputType;
				
				::boost::optional<::std::uint8_t> standardDigitalOutput;
				
				::boost::optional<::std::uint8_t> standardDigitalOutputMask;
			};
			
			struct Output
			{
				double actualCurrent[6];
				
				::std::uint64_t actualDigitalInputBits;
				
				::std::uint64_t actualDigitalOutputBits;
				
				double actualQ[6];
				
				double actualQd[6];
				
				double actualTcpForce[6];
				
				double actualTcpPose[6];
				
				double actualTcpSpeed[6];
				
				::std::uint32_t analogIoTypes;
				
				::std::int32_t jointMode[6];
				
				double jointTemperatures[6];
				
				::std::uint32_t outputBitRegisters0;
				
				::std::uint32_t outputBitRegisters1;
				
				double outputDoubleRegister[24];
				
				::std::int32_t outputIntRegister[24];
				
				::std::int32_t robotMode;
				
				::std::uint32_t robotStatusBits;
				
				::std::uint32_t runtimeState;
				
				::std::int32_t safetyMode;
				
				::std::uint32_t safetyStatusBits;
				
				double speedScaling;
				
				double standardAnalogInput0;
				
				double standardAnalogInput1;
				
				double standardAnalogOutput0;
				
				double standardAnalogOutput1;
				
				double targetCurrent[6];
				
				double targetMoment[6];
				
				double targetQ[6];
				
				double targetQd[6];
				
				double targetQdd[6];
				
				double targetTcpPose[6];
				
				double targetTcpSpeed[6];
				
				double timestamp;
				
				double toolAnalogInput0;
				
				double toolAnalogInput1;
				
				::std::uint32_t toolAnalogInputTypes;
				
				double toolOutputCurrent;
				
				::std::int32_t toolOutputVoltage;
			};
			
			struct Version
			{
				::std::uint32_t bugfix;
				
				::std::uint32_t build;
				
				::std::uint32_t major;
				
				::std::uint32_t minor;
			};
			
			void recv();
			
			void send(::std::uint8_t* buffer, const ::std::size_t& size);
			
			void send(const Command& command);
			
			void send(const Command& command, const ::std::vector<::std::string>& strings);
			
			void send(const Command& command, const ::std::uint16_t& word);
			
			void sendAnalogOutputs();
			
			void sendBitRegisters();
			
			void sendDigitalOutputs();
			
			void sendDoubleRegister();
			
			void sendIntegerRegister();
			
			template<typename T>
			void serialize(T& t, ::std::uint8_t*& ptr)
			{
				Endian::hostToBig(t);
				::std::memcpy(ptr, &t, sizeof(t));
				ptr += sizeof(t);
			}
			
			template<typename T, ::std::size_t N>
			void serialize(T (&t)[N], ::std::uint8_t*& ptr)
			{
				for (::std::size_t i = 0; i < N; ++i)
				{
					this->serialize(t[i], ptr);
				}
			}
			
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
			
			Input input;
			
			Output output;
			
			Socket socket2;
			
			Socket socket4;
			
			Version version;
		};
	}
}

#endif // RL_HAL_UNIVERSALROBOTSRTDE_H
