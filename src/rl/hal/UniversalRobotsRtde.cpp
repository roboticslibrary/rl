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

#include <array>
#include <iostream>
#include <rl/math/Constants.h>
#include <rl/math/Units.h>

#include "DeviceException.h"
#include "UniversalRobotsRtde.h"

namespace rl
{
	namespace hal
	{
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_SHUTTING_DOWN;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_PART_D_CALIBRATION;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_BACKDRIVE;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_POWER_OFF;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_NOT_RESPONDING;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_MOTOR_INITIALISATION;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_BOOTING;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_PART_D_CALIBRATION_ERROR;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_BOOTLOADER;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_CALIBRATION;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_FAULT;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_RUNNING;
		constexpr UniversalRobotsRtde::JointMode UniversalRobotsRtde::JOINT_MODE_IDLE_MODE;
		
		constexpr UniversalRobotsRtde::RobotMode UniversalRobotsRtde::ROBOT_MODE_DISCONNECTED;
		constexpr UniversalRobotsRtde::RobotMode UniversalRobotsRtde::ROBOT_MODE_CONFIRM_SAFETY;
		constexpr UniversalRobotsRtde::RobotMode UniversalRobotsRtde::ROBOT_MODE_BOOTING;
		constexpr UniversalRobotsRtde::RobotMode UniversalRobotsRtde::ROBOT_MODE_POWER_OFF;
		constexpr UniversalRobotsRtde::RobotMode UniversalRobotsRtde::ROBOT_MODE_POWER_ON;
		constexpr UniversalRobotsRtde::RobotMode UniversalRobotsRtde::ROBOT_MODE_IDLE;
		constexpr UniversalRobotsRtde::RobotMode UniversalRobotsRtde::ROBOT_MODE_BACKDRIVE;
		constexpr UniversalRobotsRtde::RobotMode UniversalRobotsRtde::ROBOT_MODE_RUNNING;
		constexpr UniversalRobotsRtde::RobotMode UniversalRobotsRtde::ROBOT_MODE_UPDATING_FIRMWARE;
		
		constexpr UniversalRobotsRtde::RobotStatus UniversalRobotsRtde::ROBOT_STATUS_POWER_ON;
		constexpr UniversalRobotsRtde::RobotStatus UniversalRobotsRtde::ROBOT_STATUS_PROGRAM_RUNNING;
		constexpr UniversalRobotsRtde::RobotStatus UniversalRobotsRtde::ROBOT_STATUS_TEACH_BUTTON_PRESSED;
		constexpr UniversalRobotsRtde::RobotStatus UniversalRobotsRtde::ROBOT_STATUS_POWER_BUTTON_PRESSED;
		
		constexpr UniversalRobotsRtde::RuntimeState UniversalRobotsRtde::RUNTIME_STATE_STOPPING;
		constexpr UniversalRobotsRtde::RuntimeState UniversalRobotsRtde::RUNTIME_STATE_STOPPED;
		constexpr UniversalRobotsRtde::RuntimeState UniversalRobotsRtde::RUNTIME_STATE_PLAYING;
		constexpr UniversalRobotsRtde::RuntimeState UniversalRobotsRtde::RUNTIME_STATE_PAUSING;
		constexpr UniversalRobotsRtde::RuntimeState UniversalRobotsRtde::RUNTIME_STATE_PAUSED;
		constexpr UniversalRobotsRtde::RuntimeState UniversalRobotsRtde::RUNTIME_STATE_RESUMING;
		
		constexpr UniversalRobotsRtde::SafetyMode UniversalRobotsRtde::SAFETY_MODE_NORMAL;
		constexpr UniversalRobotsRtde::SafetyMode UniversalRobotsRtde::SAFETY_MODE_REDUCED;
		constexpr UniversalRobotsRtde::SafetyMode UniversalRobotsRtde::SAFETY_MODE_PROTECTIVE_STOP;
		constexpr UniversalRobotsRtde::SafetyMode UniversalRobotsRtde::SAFETY_MODE_RECOVERY;
		constexpr UniversalRobotsRtde::SafetyMode UniversalRobotsRtde::SAFETY_MODE_SAFEGUARD_STOP;
		constexpr UniversalRobotsRtde::SafetyMode UniversalRobotsRtde::SAFETY_MODE_SYSTEM_EMERGENCY_STOP;
		constexpr UniversalRobotsRtde::SafetyMode UniversalRobotsRtde::SAFETY_MODE_ROBOT_EMERGENCY_STOP;
		constexpr UniversalRobotsRtde::SafetyMode UniversalRobotsRtde::SAFETY_MODE_VIOLATION;
		constexpr UniversalRobotsRtde::SafetyMode UniversalRobotsRtde::SAFETY_MODE_FAULT;
		
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_NORMAL_MODE;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_REDUCED_MODE;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_PROTECTIVE_STOPPED;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_RECOVERY_MODE;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_SAFEGUARD_STOPPED;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_SYSTEM_EMERGENCY_STOPPED;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_ROBOT_EMERGENCY_STOPPED;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_EMERGENCY_STOPPED;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_VIOLATION;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_FAULT;
		constexpr UniversalRobotsRtde::SafetyStatus UniversalRobotsRtde::SAFETY_STATUS_STOPPED_DUE_TO_SAFETY;
		
		UniversalRobotsRtde::UniversalRobotsRtde(const ::std::string& address, const ::std::chrono::nanoseconds& updateRate) :
			AxisController(6),
			CyclicDevice(updateRate),
			AnalogInputReader(),
			AnalogOutputReader(),
			AnalogOutputWriter(),
			CartesianForceSensor(6),
			CartesianPositionSensor(6),
			CartesianVelocitySensor(6),
			DigitalInputReader(),
			DigitalOutputReader(),
			DigitalOutputWriter(),
			JointAccelerationActuator(6),
			JointCurrentSensor(6),
			JointPositionActuator(6),
			JointPositionSensor(6),
			JointVelocityActuator(6),
			JointVelocitySensor(6),
			input(),
			output(),
			socket2(Socket::Tcp(Socket::Address::Ipv4(address, 30002))),
			socket4(Socket::Tcp(Socket::Address::Ipv4(address, 30004))),
			version()
		{
		}
		
		UniversalRobotsRtde::~UniversalRobotsRtde()
		{
			if (this->isRunning())
			{
				this->stop();
			}
			
			if (this->isConnected())
			{
				this->close();
			}
		}
		
		void
		UniversalRobotsRtde::close()
		{
			this->socket2.close();
			this->socket4.close();
			this->setConnected(false);
		}
		
		::rl::math::Real
		UniversalRobotsRtde::getAnalogInput(const ::std::size_t& i) const
		{
			switch (i)
			{
			case 0:
				return this->output.standardAnalogInput0;
				break;
			case 1:
				return this->output.standardAnalogInput1;
				break;
			case 2:
				return this->output.toolAnalogInput0;
				break;
			case 3:
				return this->output.toolAnalogInput1;
				break;
			default:
				return ::std::numeric_limits<::rl::math::Real>::signaling_NaN();
			}
		}
		
		::std::size_t
		UniversalRobotsRtde::getAnalogInputCount() const
		{
			return 4;
		}
		
		::rl::math::Real
		UniversalRobotsRtde::getAnalogInputMaximum(const ::std::size_t& i) const
		{
			switch (i)
			{
			case 0:
				return 0 == (this->output.analogIoTypes & 1) ? 20 * ::rl::math::constants::milli2unit : 10;
				break;
			case 1:
				return 0 == (this->output.analogIoTypes & 2) ? 20 * ::rl::math::constants::milli2unit : 10;
				break;
			case 2:
				return 0 == (this->output.toolAnalogInputTypes & 1) ? 1 : 24;
				break;
			case 3:
				return 0 == (this->output.toolAnalogInputTypes & 2) ? 1 : 24;
				break;
			default:
				return ::std::numeric_limits<::rl::math::Real>::signaling_NaN();
				break;
			}
		}
		
		::rl::math::Real
		UniversalRobotsRtde::getAnalogInputMinimum(const ::std::size_t& i) const
		{
			return 0;
		}
		
		::std::vector<::rl::math::Units>
		UniversalRobotsRtde::getAnalogInputUnit() const
		{
			::std::vector<::rl::math::Units> values;
			
			for (::std::size_t i = 0; i < this->getAnalogInputCount(); ++i)
			{
				values.push_back(this->getAnalogInputUnit(i));
			}
			
			return values;
		}
		
		::rl::math::Units
		UniversalRobotsRtde::getAnalogInputUnit(const ::std::size_t& i) const
		{
			switch (i)
			{
			case 0:
				return 0 == (this->output.analogIoTypes & 1) ? ::rl::math::Units::ampere : ::rl::math::Units::volt;
				break;
			case 1:
				return 0 == (this->output.analogIoTypes & 2) ? ::rl::math::Units::ampere : ::rl::math::Units::volt;
				break;
			case 2:
				return 0 == (this->output.toolAnalogInputTypes & 1) ? ::rl::math::Units::ampere : ::rl::math::Units::volt;
				break;
			case 3:
				return 0 == (this->output.toolAnalogInputTypes & 2) ? ::rl::math::Units::ampere : ::rl::math::Units::volt;
				break;
			default:
				return ::rl::math::Units::none;
				break;
			}
		}
		
		::rl::math::Real
		UniversalRobotsRtde::getAnalogOutput(const ::std::size_t& i) const
		{
			switch (i)
			{
			case 0:
				return this->output.standardAnalogOutput0;
				break;
			case 1:
				return this->output.standardAnalogOutput1;
				break;
			case 2:
				return this->output.toolOutputVoltage;
				break;
			case 3:
				return this->output.toolOutputCurrent;
				break;
			default:
				return ::std::numeric_limits<::rl::math::Real>::signaling_NaN();
			}
		}
		
		::std::size_t
		UniversalRobotsRtde::getAnalogOutputCount() const
		{
			return 4;
		}
		
		::rl::math::Real
		UniversalRobotsRtde::getAnalogOutputMaximum(const ::std::size_t& i) const
		{
			switch (i)
			{
			case 0:
				return 0 == (this->output.analogIoTypes & 4) ? 20 * ::rl::math::constants::milli2unit : 10;
				break;
			case 1:
				return 0 == (this->output.analogIoTypes & 8) ? 20 * ::rl::math::constants::milli2unit : 10;
				break;
			case 2:
				return 24;
				break;
			case 3:
				return 1;
				break;
			default:
				return ::std::numeric_limits<::rl::math::Real>::signaling_NaN();
				break;
			}
		}
		
		::rl::math::Real
		UniversalRobotsRtde::getAnalogOutputMinimum(const ::std::size_t& i) const
		{
			return 0;
		}
		
		::std::vector<::rl::math::Units>
		UniversalRobotsRtde::getAnalogOutputUnit() const
		{
			::std::vector<::rl::math::Units> values;
			
			for (::std::size_t i = 0; i < this->getAnalogOutputCount(); ++i)
			{
				values.push_back(this->getAnalogOutputUnit(i));
			}
			
			return values;
		}
		
		::rl::math::Units
		UniversalRobotsRtde::getAnalogOutputUnit(const ::std::size_t& i) const
		{
			switch (i)
			{
			case 0:
				return 0 == (this->output.analogIoTypes & 4) ? ::rl::math::Units::ampere : ::rl::math::Units::volt;
				break;
			case 1:
				return 0 == (this->output.analogIoTypes & 8) ? ::rl::math::Units::ampere : ::rl::math::Units::volt;
				break;
			case 2:
				return ::rl::math::Units::volt;
				break;
			case 3:
				return ::rl::math::Units::ampere;
				break;
			default:
				return ::rl::math::Units::none;
				break;
			}
		}
		
		::rl::math::ForceVector
		UniversalRobotsRtde::getCartesianForce() const
		{
			::rl::math::ForceVector f;
			f.force().x() = this->output.actualTcpForce[0];
			f.force().y() = this->output.actualTcpForce[1];
			f.force().z() = this->output.actualTcpForce[2];
			f.moment().x() = this->output.actualTcpForce[3];
			f.moment().y() = this->output.actualTcpForce[4];
			f.moment().z() = this->output.actualTcpForce[5];
			return f;
		}
		
		::rl::math::Transform
		UniversalRobotsRtde::getCartesianPosition() const
		{
			::rl::math::Transform x = ::rl::math::Transform::Identity();
			
			::rl::math::Vector3 orientation(this->output.actualTcpPose[3], this->output.actualTcpPose[4], this->output.actualTcpPose[5]);
			::rl::math::Real norm = orientation.norm();
			
			if (::std::abs(norm) <= ::std::numeric_limits<::rl::math::Real>::epsilon())
			{
				x.linear().setIdentity();
			}
			else
			{
				x.linear() = ::rl::math::AngleAxis(norm, orientation.normalized()).matrix();
			}
			
			x.translation().x() = this->output.actualTcpPose[0];
			x.translation().y() = this->output.actualTcpPose[1];
			x.translation().z() = this->output.actualTcpPose[2];
			
			return x;
		}
		
		::rl::math::Transform
		UniversalRobotsRtde::getCartesianPositionTarget() const
		{
			::rl::math::Transform x = ::rl::math::Transform::Identity();
			
			::rl::math::Vector3 orientation(this->output.targetTcpPose[3], this->output.targetTcpPose[4], this->output.targetTcpPose[5]);
			::rl::math::Real norm = orientation.norm();
			
			if (::std::abs(norm) <= ::std::numeric_limits<::rl::math::Real>::epsilon())
			{
				x.linear().setIdentity();
			}
			else
			{
				x.linear() = ::rl::math::AngleAxis(norm, orientation.normalized()).matrix();
			}
			
			x.translation().x() = this->output.targetTcpPose[0];
			x.translation().y() = this->output.targetTcpPose[1];
			x.translation().z() = this->output.targetTcpPose[2];
			
			return x;
		}
		
		::rl::math::MotionVector
		UniversalRobotsRtde::getCartesianVelocity() const
		{
			::rl::math::MotionVector v;
			v.linear().x() = this->output.actualTcpSpeed[0];
			v.linear().y() = this->output.actualTcpSpeed[1];
			v.linear().z() = this->output.actualTcpSpeed[2];
			v.angular().x() = this->output.actualTcpSpeed[3];
			v.angular().y() = this->output.actualTcpSpeed[4];
			v.angular().z() = this->output.actualTcpSpeed[5];
			return v;
		}
		
		::rl::math::MotionVector
		UniversalRobotsRtde::getCartesianVelocityTarget() const
		{
			::rl::math::MotionVector v;
			v.linear().x() = this->output.targetTcpSpeed[0];
			v.linear().y() = this->output.targetTcpSpeed[1];
			v.linear().z() = this->output.targetTcpSpeed[2];
			v.angular().x() = this->output.targetTcpSpeed[3];
			v.angular().y() = this->output.targetTcpSpeed[4];
			v.angular().z() = this->output.targetTcpSpeed[5];
			return v;
		}
		
		::boost::dynamic_bitset<>
		UniversalRobotsRtde::getDigitalInput() const
		{
			return ::boost::dynamic_bitset<>(18, this->output.actualDigitalInputBits);
		}
		
		bool
		UniversalRobotsRtde::getDigitalInput(const ::std::size_t& i) const
		{
			return (this->output.actualDigitalInputBits & (1ULL << i)) ? true : false;
		}
		
		::std::size_t
		UniversalRobotsRtde::getDigitalInputCount() const
		{
			return 18;
		}
		
		::boost::dynamic_bitset<>
		UniversalRobotsRtde::getDigitalOutput() const
		{
			return ::boost::dynamic_bitset<>(18, this->output.actualDigitalOutputBits);
		}
		
		bool
		UniversalRobotsRtde::getDigitalOutput(const ::std::size_t& i) const
		{
			return (this->output.actualDigitalOutputBits & (1ULL << i)) ? true : false;
		}
		
		::std::size_t
		UniversalRobotsRtde::getDigitalOutputCount() const
		{
			return 18;
		}
		
		::rl::math::Vector
		UniversalRobotsRtde::getJointCurrent() const
		{
			::rl::math::Vector i(this->getDof());
			
			for (::std::ptrdiff_t j = 0; j < i.size(); ++j)
			{
				i(j) = this->output.targetCurrent[j];
			}
			
			return i;
		}
		
		UniversalRobotsRtde::JointMode
		UniversalRobotsRtde::getJointMode(const ::std::size_t& i) const
		{
			assert(i < 6);
			return static_cast<JointMode>(this->output.jointMode[i]);
		}
		
		::rl::math::Vector
		UniversalRobotsRtde::getJointPosition() const
		{
			::rl::math::Vector q(this->getDof());
			
			for (::std::ptrdiff_t i = 0; i < q.size(); ++i)
			{
				q(i) = this->output.targetQ[i];
			}
			
			return q;
		}
		
		::rl::math::Vector
		UniversalRobotsRtde::getJointTemperature() const
		{
			::rl::math::Vector temperature(this->getDof());
			
			for (::std::ptrdiff_t i = 0; i < temperature.size(); ++i)
			{
				temperature(i) = this->output.jointTemperatures[i];
			}
			
			return temperature;
		}
		
		::rl::math::Vector
		UniversalRobotsRtde::getJointVelocity() const
		{
			::rl::math::Vector qd(this->getDof());
			
			for (::std::ptrdiff_t i = 0; i < qd.size(); ++i)
			{
				qd(i) = this->output.targetQd[i];
			}
			
			return qd;
		}
		
		UniversalRobotsRtde::RobotMode
		UniversalRobotsRtde::getRobotMode() const
		{
			return static_cast<RobotMode>(this->output.robotMode);
		}
		
		::std::uint32_t
		UniversalRobotsRtde::getRobotStatusBits() const
		{
			return this->output.robotStatusBits;
		}
		
		UniversalRobotsRtde::RuntimeState
		UniversalRobotsRtde::getRuntimeState() const
		{
			return static_cast<RuntimeState>(this->output.runtimeState);
		}
		
		UniversalRobotsRtde::SafetyMode
		UniversalRobotsRtde::getSafetyMode() const
		{
			return static_cast<SafetyMode>(this->output.safetyMode);
		}
		
		::std::uint32_t
		UniversalRobotsRtde::getSafetyStatusBits() const
		{
			return this->output.safetyStatusBits;
		}
		
		void
		UniversalRobotsRtde::open()
		{
			this->socket2.open();
			this->socket2.connect();
			this->socket2.setOption(Socket::Option::nodelay, 1);
			this->socket4.open();
			this->socket4.connect();
			this->socket4.setOption(Socket::Option::nodelay, 1);
			this->setConnected(true);
			
			this->send(Command::requestProtocolVersion, 1);
			this->recv();
			
			this->send(Command::getUrcontrolVersion);
			this->recv();
			
			static constexpr const char* outputsArray[] = {
				"timestamp",
				"target_q",
				"target_qd",
				"target_qdd",
				"target_current",
				"target_moment",
				"actual_q",
				"actual_qd",
				"actual_current",
				"actual_TCP_pose",
				"actual_TCP_speed",
				"actual_TCP_force",
				"target_TCP_pose",
				"target_TCP_speed",
				"actual_digital_input_bits",
				"joint_temperatures",
				"robot_mode",
				"joint_mode",
				"safety_mode",
				"speed_scaling",
				"actual_digital_output_bits",
				"runtime_state",
				"robot_status_bits",
				"safety_status_bits",
				"analog_io_types",
				"standard_analog_input0",
				"standard_analog_input1",
				"standard_analog_output0",
				"standard_analog_output1",
				"tool_analog_input_types",
				"tool_analog_input0",
				"tool_analog_input1",
				"tool_output_voltage",
				"tool_output_current",
				"output_bit_registers0_to_31",
				"output_bit_registers32_to_63",
				"output_int_register_0",
				"output_int_register_1",
				"output_int_register_2",
				"output_int_register_3",
				"output_int_register_4",
				"output_int_register_5",
				"output_int_register_6",
				"output_int_register_7",
				"output_int_register_8",
				"output_int_register_9",
				"output_int_register_10",
				"output_int_register_11",
				"output_int_register_12",
				"output_int_register_13",
				"output_int_register_14",
				"output_int_register_15",
				"output_int_register_16",
				"output_int_register_17",
				"output_int_register_18",
				"output_int_register_19",
				"output_int_register_20",
				"output_int_register_21",
				"output_int_register_22",
				"output_int_register_23",
				"output_double_register_0",
				"output_double_register_1",
				"output_double_register_2",
				"output_double_register_3",
				"output_double_register_4",
				"output_double_register_5",
				"output_double_register_6",
				"output_double_register_7",
				"output_double_register_8",
				"output_double_register_9",
				"output_double_register_10",
				"output_double_register_11",
				"output_double_register_12",
				"output_double_register_13",
				"output_double_register_14",
				"output_double_register_15",
				"output_double_register_16",
				"output_double_register_17",
				"output_double_register_18",
				"output_double_register_19",
				"output_double_register_20",
				"output_double_register_21",
				"output_double_register_22",
				"output_double_register_23"
			};
			static const ::std::vector<::std::string> outputs(::std::begin(outputsArray), ::std::end(outputsArray));
			this->send(Command::controlPackageSetupOutputs, outputs);
			this->recv();
			
			static constexpr const char* inputs1Array[] = {
				"input_int_register_0"
			};
			static const ::std::vector<::std::string> inputs1(::std::begin(inputs1Array), ::std::end(inputs1Array));
			this->send(Command::controlPackageSetupInputs, inputs1);
			this->recv();
			
			static constexpr const char* inputs2Array[] = {
				"input_double_register_0",
				"input_double_register_1",
				"input_double_register_2",
				"input_double_register_3",
				"input_double_register_4",
				"input_double_register_5",
				"input_double_register_6",
				"input_double_register_7",
				"input_double_register_8",
				"input_double_register_9",
				"input_double_register_10",
				"input_double_register_11",
				"input_double_register_12"
			};
			static const ::std::vector<::std::string> inputs2(::std::begin(inputs2Array), ::std::end(inputs2Array));
			this->send(Command::controlPackageSetupInputs, inputs2);
			this->recv();
			
			static constexpr const char* inputs3Array[] = {
				"standard_digital_output_mask",
				"configurable_digital_output_mask",
				"standard_digital_output",
				"configurable_digital_output"
			};
			static const ::std::vector<::std::string> inputs3(::std::begin(inputs3Array), ::std::end(inputs3Array));
			this->send(Command::controlPackageSetupInputs, inputs3);
			this->recv();
			
			static constexpr const char* inputs4Array[] = {
				"standard_analog_output_mask",
				"standard_analog_output_type",
				"standard_analog_output_0",
				"standard_analog_output_1"
			};
			static const ::std::vector<::std::string> inputs4(::std::begin(inputs4Array), ::std::end(inputs4Array));
			this->send(Command::controlPackageSetupInputs, inputs4);
			this->recv();
			
			static constexpr const char* inputs5Array[] = {
				"input_bit_registers0_to_31",
				"input_bit_registers32_to_63"
			};
			static const ::std::vector<::std::string> inputs5(::std::begin(inputs5Array), ::std::end(inputs5Array));
			this->send(Command::controlPackageSetupInputs, inputs5);
			this->recv();
		}
		
		void
		UniversalRobotsRtde::recv()
		{
			::std::array<::std::uint8_t, 4096> buffer;
			::std::size_t size = this->socket4.recv(buffer.data(), buffer.size());
#if !defined(__APPLE__) && !defined(__QNX__) && !defined(WIN32) && !defined(__CYGWIN__)
			this->socket4.setOption(Socket::Option::quickack, 1);
#endif // !__APPLE__ && !__QNX__ && !WIN32 && !__CYGWIN__
//			::std::cout << "size: " << size << ::std::endl;
			
			::std::uint8_t* ptr = buffer.data();
			
			while (size >= 3)
			{
				::std::uint16_t packageSize;
				this->unserialize(ptr, packageSize);
//				::std::cout << "packageSize: " << packageSize << ::std::endl;
				
				::std::uint8_t packageType;
				this->unserialize(ptr, packageType);
//				::std::cout << "packageType: " << packageType << ::std::endl;
				
				switch (static_cast<Command>(packageType))
				{
				case Command::controlPackagePause:
					::std::uint8_t pauseAccepted;
					this->unserialize(ptr, pauseAccepted);
//					::std::cout << "accepted: " << static_cast<bool>(pauseAccepted & 0x01) << ::std::endl;
					
					if (!pauseAccepted)
					{
						throw DeviceException("Package pause command not accepted by controller");
					}
					
					break;
				case Command::controlPackageSetupInputs:
				{
					::std::uint8_t recipeId;
					this->unserialize(ptr, recipeId);
//					::std::cout << "recipeId: " << static_cast<int>(recipeId) << ::std::endl;
					
					if (0 == recipeId)
					{
						throw DeviceException("Input recipe invalid");
					}
					
					::std::string variableTypes(reinterpret_cast<char*>(ptr), packageSize - 4);
//					::std::cout << "variableTypes: " << variableTypes << ::std::endl;
					ptr += packageSize - 4;
				}
					break;
				case Command::controlPackageSetupOutputs:
				{
					::std::string variableTypes(reinterpret_cast<char*>(ptr), packageSize - 3);
//					::std::cout << "variableTypes: " << variableTypes << ::std::endl;
					ptr += packageSize - 3;
					
					if (::std::string::npos != variableTypes.find("NOT_FOUND"))
					{
						throw DeviceException("Output recipe invalid");
					}
				}
					break;
				case Command::controlPackageStart:
					::std::uint8_t startAccepted;
					this->unserialize(ptr, startAccepted);
//					::std::cout << "accepted: " << static_cast<bool>(startAccepted & 0x01) << ::std::endl;
					
					if (!startAccepted)
					{
						throw DeviceException("Package start command not accepted by controller");
					}
					
					break;
				case Command::dataPackage:
					this->unserialize(ptr, this->output.timestamp);
					this->unserialize(ptr, this->output.targetQ);
					this->unserialize(ptr, this->output.targetQd);
					this->unserialize(ptr, this->output.targetQdd);
					this->unserialize(ptr, this->output.targetCurrent);
					this->unserialize(ptr, this->output.targetMoment);
					this->unserialize(ptr, this->output.actualQ);
					this->unserialize(ptr, this->output.actualQd);
					this->unserialize(ptr, this->output.actualCurrent);
					this->unserialize(ptr, this->output.actualTcpPose);
					this->unserialize(ptr, this->output.actualTcpSpeed);
					this->unserialize(ptr, this->output.actualTcpForce);
					this->unserialize(ptr, this->output.targetTcpPose);
					this->unserialize(ptr, this->output.targetTcpSpeed);
					this->unserialize(ptr, this->output.actualDigitalInputBits);
					this->unserialize(ptr, this->output.jointTemperatures);
					this->unserialize(ptr, this->output.robotMode);
					this->unserialize(ptr, this->output.jointMode);
					this->unserialize(ptr, this->output.safetyMode);
					this->unserialize(ptr, this->output.speedScaling);
					this->unserialize(ptr, this->output.actualDigitalOutputBits);
					this->unserialize(ptr, this->output.runtimeState);
					this->unserialize(ptr, this->output.robotStatusBits);
					this->unserialize(ptr, this->output.safetyStatusBits);
					this->unserialize(ptr, this->output.analogIoTypes);
					this->unserialize(ptr, this->output.standardAnalogInput0);
					this->unserialize(ptr, this->output.standardAnalogInput1);
					this->unserialize(ptr, this->output.standardAnalogOutput0);
					this->unserialize(ptr, this->output.standardAnalogOutput1);
					this->unserialize(ptr, this->output.toolAnalogInputTypes);
					this->unserialize(ptr, this->output.toolAnalogInput0);
					this->unserialize(ptr, this->output.toolAnalogInput1);
					this->unserialize(ptr, this->output.toolOutputVoltage);
					this->unserialize(ptr, this->output.toolOutputCurrent);
					this->unserialize(ptr, this->output.outputBitRegisters0);
					this->unserialize(ptr, this->output.outputBitRegisters1);
					this->unserialize(ptr, this->output.outputIntRegister);
					this->unserialize(ptr, this->output.outputDoubleRegister);
					
					this->input.inputDoubleRegister.resize(this->getDof() + this->getDof() + 1);
					
					for (::std::size_t i = 0; i < this->getDof(); ++i)
					{
						this->input.inputDoubleRegister[i] = this->output.targetQ[i];
						this->input.inputDoubleRegister[this->getDof() + i] = this->output.targetQd[i];
					}
					
					this->input.inputDoubleRegister[this->getDof() + this->getDof()] = 0;
					
					switch (this->getSafetyMode())
					{
					case SafetyMode::normal:
						break;
					case SafetyMode::reduced:
						break;
					case SafetyMode::protectiveStop:
						throw DeviceException("Protective stop");
						break;
					case SafetyMode::recovery:
						break;
					case SafetyMode::safeguardStop:
						throw DeviceException("Safeguard stop");
						break;
					case SafetyMode::systemEmergencyStop:
						throw DeviceException("System emergency stop");
						break;
					case SafetyMode::robotEmergencyStop:
						throw DeviceException("Robot emergency stop");
						break;
					case SafetyMode::violation:
						throw DeviceException("Mode violation");
						break;
					case SafetyMode::fault:
						throw DeviceException("Fault");
						break;
					default:
						break;
					}
					break;
				case Command::getUrcontrolVersion:
					this->unserialize(ptr, this->version.major);
//					::std::cout << "major: " << this->version.major << ::std::endl;
					this->unserialize(ptr, this->version.minor);
//					::std::cout << "minor: " << this->version.minor << ::std::endl;
					this->unserialize(ptr, this->version.bugfix);
//					::std::cout << "bugfix: " << this->version.bugfix << ::std::endl;
					this->unserialize(ptr, this->version.build);
//					::std::cout << "build: " << this->version.build << ::std::endl;
					break;
				case Command::requestProtocolVersion:
					::std::uint8_t protocolAccepted;
					this->unserialize(ptr, protocolAccepted);
//					::std::cout << "accepted: " << static_cast<bool>(protocolAccepted & 0x01) << ::std::endl;
					
					if (!protocolAccepted)
					{
						throw DeviceException("Requested protocol version not accepted by controller");
					}
					
					break;
				case Command::textMessage:
				{
					::std::uint8_t level;
					this->unserialize(ptr, level);
//					::std::cout << "level: " << static_cast<int>(level) << ::std::endl;
					
					::std::string message(reinterpret_cast<char*>(ptr), packageSize - 4);
//					::std::cout << "message: " << message << ::std::endl;
					ptr += packageSize - 4;
					
					if (level < 2)
					{
						throw DeviceException(message);
					}
				}
					break;
				default:
					break;
				}
				
				size -= packageSize;
			}
		}
		
		void
		UniversalRobotsRtde::send(::std::uint8_t* buffer, const ::std::size_t& size)
		{
			if (size > ::std::numeric_limits<::std::uint16_t>::max())
			{
				throw DeviceException("Package size too large");
			}
			
			::std::uint16_t packageSize = static_cast<::std::uint16_t>(size);
			
			buffer[0] = Endian::hostHighByte(packageSize);
			buffer[1] = Endian::hostLowByte(packageSize);
			
			this->socket4.send(buffer, size);
		}
		
		void
		UniversalRobotsRtde::send(const Command& command)
		{
			::std::vector<::std::uint8_t> buffer;
			
			buffer.reserve(3);
			
			buffer.push_back(0);
			buffer.push_back(0);
			buffer.push_back(static_cast<::std::uint8_t>(command));
			
			this->send(buffer.data(), buffer.size());
		}
		
		void
		UniversalRobotsRtde::send(const Command& command, const ::std::vector<::std::string>& strings)
		{
			::std::vector<::std::uint8_t> buffer;
			
			buffer.reserve(3);
			
			buffer.push_back(0);
			buffer.push_back(0);
			buffer.push_back(static_cast<::std::uint8_t>(command));
			
			for (::std::size_t i = 0; i < strings.size(); ++i)
			{
				if (i > 0)
				{
					buffer.reserve(buffer.size() + 1 + strings[i].size());
					buffer.push_back(',');
				}
				else
				{
					buffer.reserve(buffer.size() + strings[i].size());
				}
				
				for (::std::size_t j = 0; j < strings[i].size(); ++j)
				{
					buffer.push_back(strings[i][j]);
				}
			}
			
			this->send(buffer.data(), buffer.size());
		}
		
		void
		UniversalRobotsRtde::send(const Command& command, const ::std::uint16_t& word)
		{
			::std::vector<::std::uint8_t> buffer;
			
			buffer.reserve(5);
			
			buffer.push_back(0);
			buffer.push_back(0);
			buffer.push_back(static_cast<::std::uint8_t>(command));
			buffer.push_back(Endian::hostHighByte(word));
			buffer.push_back(Endian::hostLowByte(word));
			
			this->send(buffer.data(), buffer.size());
		}
		
		void
		UniversalRobotsRtde::sendAnalogOutputs()
		{
			::std::vector<::std::uint8_t> buffer;
			
			buffer.reserve(4);
			
			buffer.push_back(0);
			buffer.push_back(0);
			buffer.push_back(static_cast<::std::uint8_t>(Command::dataPackage));
			buffer.push_back(4);
			
			buffer.resize(buffer.size() + 2 + 2 * sizeof(double));
			
			::std::uint8_t* ptr = &buffer[4];
			
			this->serialize(this->input.standardAnalogOutputMask.get(), ptr);
			this->serialize(this->input.standardAnalogOutputType, ptr);
			this->serialize(this->input.standardAnalogOutput0.get(), ptr);
			this->serialize(this->input.standardAnalogOutput1.get(), ptr);
			
			this->send(buffer.data(), buffer.size());
		}
		
		void
		UniversalRobotsRtde::sendBitRegisters()
		{
			::std::vector<::std::uint8_t> buffer;
			
			buffer.reserve(4);
			
			buffer.push_back(0);
			buffer.push_back(0);
			buffer.push_back(static_cast<::std::uint8_t>(Command::dataPackage));
			buffer.push_back(5);
			
			::std::uint8_t* ptr = &buffer[4];
			
			if (this->input.inputBitRegisters0)
			{
				buffer.resize(buffer.size() + sizeof(::std::uint32_t));
				this->serialize(this->input.inputBitRegisters0.get(), ptr);
			}
			
			if (this->input.inputBitRegisters1)
			{
				buffer.resize(buffer.size() + sizeof(::std::uint32_t));
				this->serialize(this->input.inputBitRegisters1.get(), ptr);
			}
			
			this->send(buffer.data(), buffer.size());
		}
		
		void
		UniversalRobotsRtde::sendDigitalOutputs()
		{
			::std::vector<::std::uint8_t> buffer;
			
			buffer.reserve(4);
			
			buffer.push_back(0);
			buffer.push_back(0);
			buffer.push_back(static_cast<::std::uint8_t>(Command::dataPackage));
			buffer.push_back(3);
			
			buffer.resize(buffer.size() + 4);
			
			::std::uint8_t* ptr = &buffer[4];
			
			this->serialize(this->input.standardDigitalOutputMask.get(), ptr);
			this->serialize(this->input.configurableDigitalOutputMask.get(), ptr);
			this->serialize(this->input.standardDigitalOutput.get(), ptr);
			this->serialize(this->input.configurableDigitalOutput.get(), ptr);
			
			this->send(buffer.data(), buffer.size());
		}
		
		void
		UniversalRobotsRtde::sendDoubleRegister()
		{
			::std::vector<::std::uint8_t> buffer;
			
			buffer.reserve(4);
			
			buffer.push_back(0);
			buffer.push_back(0);
			buffer.push_back(static_cast<::std::uint8_t>(Command::dataPackage));
			buffer.push_back(2);
			
			buffer.resize(buffer.size() + this->input.inputDoubleRegister.size() * sizeof(double));
			
			::std::uint8_t* ptr = &buffer[4];
			
			for (::std::size_t i = 0; i < this->input.inputDoubleRegister.size(); ++i)
			{
				this->serialize(this->input.inputDoubleRegister[i], ptr);
			}
			
			this->send(buffer.data(), buffer.size());
		}
		
		void
		UniversalRobotsRtde::sendIntegerRegister()
		{
			::std::vector<::std::uint8_t> buffer;
			
			buffer.reserve(4);
			
			buffer.push_back(0);
			buffer.push_back(0);
			buffer.push_back(static_cast<::std::uint8_t>(Command::dataPackage));
			buffer.push_back(1);
			
			buffer.resize(buffer.size() + this->input.inputIntRegister.size() * sizeof(::std::int32_t));
			
			::std::uint8_t* ptr = &buffer[4];
			
			for (::std::size_t i = 0; i < this->input.inputIntRegister.size(); ++i)
			{
				this->serialize(this->input.inputIntRegister[i], ptr);
			}
			
			this->send(buffer.data(), buffer.size());
		}
		
		void
		UniversalRobotsRtde::setAnalogOutput(const ::std::size_t& i, const ::rl::math::Real& value)
		{
			if (!this->input.standardAnalogOutput0)
			{
				this->input.standardAnalogOutput0 = 0;
			}
			
			if (!this->input.standardAnalogOutput1)
			{
				this->input.standardAnalogOutput1 = 0;
			}
			
			if (!this->input.standardAnalogOutputMask)
			{
				this->input.standardAnalogOutputMask = 0;
			}
			
			switch (i)
			{
			case 0:
				this->input.standardAnalogOutput0 = value; // TODO convert from ratio to absolute
				break;
			case 1:
				this->input.standardAnalogOutput1 = value; // TODO convert from ratio to absolute
				break;
			default:
				break;
			}
			
			this->input.standardAnalogOutputMask.get() |= 1 << i;
		}
		
		void
		UniversalRobotsRtde::setAnalogOutputUnit(const ::std::vector<::rl::math::Units>& values)
		{
			for (::std::size_t i = 0; i < this->getAnalogOutputCount(); ++i)
			{
				this->setAnalogOutputUnit(i, values[i]);
			}
		}
		
		void
		UniversalRobotsRtde::setAnalogOutputUnit(const ::std::size_t& i, const ::rl::math::Units& value)
		{
			switch (value)
			{
			case ::rl::math::Units::ampere:
				this->input.standardAnalogOutputType &= ~(1 << i);
				break;
			case ::rl::math::Units::volt:
				this->input.standardAnalogOutputType |= 1 << i;
				break;
			default:
				break;
			}
		}
	
		void
		UniversalRobotsRtde::setDigitalOutput(const ::std::size_t& i, const bool& value)
		{
			if (i < 8)
			{
				if (!this->input.standardDigitalOutput)
				{
					this->input.standardDigitalOutput = 0;
				}
				
				if (value)
				{
					this->input.standardDigitalOutput.get() |= 1 << i;
				}
				else
				{
					this->input.standardDigitalOutput.get() &= ~(1 << i);
				}
				
				if (!this->input.standardDigitalOutputMask)
				{
					this->input.standardDigitalOutputMask = 0;
				}
				
				this->input.standardDigitalOutputMask.get() |= 1 << i;
			}
			else if (i < 16)
			{
				if (!this->input.configurableDigitalOutput)
				{
					this->input.configurableDigitalOutput = 0;
				}
				
				if (value)
				{
					this->input.configurableDigitalOutput.get() |= 1 << (i - 8);
				}
				else
				{
					this->input.configurableDigitalOutput.get() &= ~(1 << (i - 8));
				}
				
				if (!this->input.configurableDigitalOutputMask)
				{
					this->input.configurableDigitalOutputMask = 0;
				}
				
				this->input.configurableDigitalOutputMask.get() |= 1 << (i - 8);
			}
			else if (i < 18)
			{
				if (!this->input.inputBitRegisters0)
				{
					this->input.inputBitRegisters0 = 0;
				}
				
				if (value)
				{
					this->input.inputBitRegisters0.get() |= 1 << (i - 16);
				}
				else
				{
					this->input.inputBitRegisters0.get() &= ~(1 << (i - 16));
				}
				
				if (!this->input.inputBitRegisters1)
				{
					this->input.inputBitRegisters1 = 0;
				}
				
				this->input.inputBitRegisters1.get() |= 1 << (i - 16);
			}
		}
		
		void
		UniversalRobotsRtde::setJointAcceleration(const ::rl::math::Vector& qdd)
		{
			this->input.inputDoubleRegister.resize(this->getDof() + this->getDof() + 1);
			
			this->input.inputDoubleRegister[12] = qdd.maxCoeff();
		}
		
		void
		UniversalRobotsRtde::setJointPosition(const ::rl::math::Vector& q)
		{
			this->input.inputDoubleRegister.resize(this->getDof() + this->getDof() + 1);
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->input.inputDoubleRegister[i] = q(i);
			}
		}
		
		void
		UniversalRobotsRtde::setJointVelocity(const ::rl::math::Vector& qd)
		{
			this->input.inputDoubleRegister.resize(this->getDof() + this->getDof() + 1);
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->input.inputDoubleRegister[this->getDof() + i] = qd(i);
			}
		}
		
		void
		UniversalRobotsRtde::start()
		{
			this->send(Command::controlPackageStart);
			this->recv();
			
			this->input.inputIntRegister.push_back(1);
			this->sendIntegerRegister();
			this->input.inputIntRegister.clear();
			
			::std::stringstream program;
			program << "def robotics_library():" << '\n';
			program << '\t' << "q = get_actual_joint_positions()" << '\n';
			program << '\t' << "qd = get_actual_joint_speeds()" << '\n';
			program << '\t' << "rtde_set_watchdog(\"input_double_register_0\", 1, \"pause\")" << '\n';
			program << '\t' << "while (1 == read_input_integer_register(0)):" << '\n';
#if 1
			program << '\t' << '\t' << "q[0] = read_input_float_register(0)" << '\n';
			program << '\t' << '\t' << "q[1] = read_input_float_register(1)" << '\n';
			program << '\t' << '\t' << "q[2] = read_input_float_register(2)" << '\n';
			program << '\t' << '\t' << "q[3] = read_input_float_register(3)" << '\n';
			program << '\t' << '\t' << "q[4] = read_input_float_register(4)" << '\n';
			program << '\t' << '\t' << "q[5] = read_input_float_register(5)" << '\n';
			program << '\t' << '\t' << "servoj(q, 0, 0, " << ::std::chrono::duration_cast<::std::chrono::duration<::rl::math::Real>>(this->getUpdateRate()).count() << ", 0.03, 2000)" << '\n';
#else
			program << '\t' << '\t' << "qd[0] = read_input_float_register(6)" << '\n';
			program << '\t' << '\t' << "qd[1] = read_input_float_register(7)" << '\n';
			program << '\t' << '\t' << "qd[2] = read_input_float_register(8)" << '\n';
			program << '\t' << '\t' << "qd[3] = read_input_float_register(9)" << '\n';
			program << '\t' << '\t' << "qd[4] = read_input_float_register(10)" << '\n';
			program << '\t' << '\t' << "qd[5] = read_input_float_register(11)" << '\n';
			program << '\t' << '\t' << "qdd = read_input_float_register(12)" << '\n';
			program << '\t' << '\t' << "speedj(qd, qdd, " << ::std::chrono::duration_cast<::std::chrono::duration<::rl::math::Real>>(this->getUpdateRate()).count() << ")" << '\n';
#endif
			program << '\t' << "end" << '\n';
			program << "end" << '\n';
			this->socket2.send(program.str().c_str(), program.str().size());
			
			do
			{
				this->recv();
			}
			while (RobotMode::running != this->getRobotMode() || RuntimeState::playing != this->getRuntimeState());
			
			this->setRunning(true);
		}
		
		void
		UniversalRobotsRtde::step()
		{
			if (!this->input.inputDoubleRegister.empty())
			{
				this->sendDoubleRegister();
			}
			
			if (this->input.standardDigitalOutputMask)
			{
				this->sendDigitalOutputs();
			}
			
			if (this->input.standardAnalogOutputMask)
			{
				this->sendAnalogOutputs();
			}
			
			if (this->input.inputBitRegisters0 && this->input.inputBitRegisters1)
			{
				this->sendBitRegisters();
			}
			
			this->input.configurableDigitalOutput.reset();
			this->input.configurableDigitalOutputMask.reset();
			this->input.inputBitRegisters0.reset();
			this->input.inputBitRegisters1.reset();
			this->input.inputDoubleRegister.clear();
			this->input.inputIntRegister.clear();
			this->input.standardAnalogOutput0.reset();
			this->input.standardAnalogOutput1.reset();
			this->input.standardAnalogOutputMask.reset();
			this->input.standardDigitalOutput.reset();
			this->input.standardDigitalOutputMask.reset();
			
			this->recv();
		}
		
		void
		UniversalRobotsRtde::stop()
		{
			this->input.inputIntRegister.push_back(0);
			this->sendIntegerRegister();
			this->input.inputIntRegister.clear();
			
			do
			{
				this->recv();
			}
			while (RuntimeState::stopped != this->getRuntimeState());
			
			this->send(Command::controlPackagePause);
			this->recv();
			
			this->setRunning(false);
		}
	}
}
