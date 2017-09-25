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
#include <rl/math/Unit.h>

#include "UniversalRobotsRealtime.h"

namespace rl
{
	namespace hal
	{
		UniversalRobotsRealtime::UniversalRobotsRealtime(const ::std::string& address) :
			AxisController(6),
			DigitalInput(),
			DigitalOutput(),
			CartesianForceSensor(6),
			CartesianPositionSensor(6),
			CartesianVelocitySensor(6),
			CyclicDevice(::std::chrono::milliseconds(8)),
			DigitalInputReader(),
			DigitalOutputReader(),
			JointCurrentSensor(6),
			JointPositionSensor(6),
			JointVelocitySensor(6),
			in(),
			socket(Socket::Tcp(Socket::Address::Ipv4(address, 30003)))
		{
		}
		
		UniversalRobotsRealtime::~UniversalRobotsRealtime()
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
		UniversalRobotsRealtime::close()
		{
			this->socket.close();
			this->setConnected(false);
		}
		
		void
		UniversalRobotsRealtime::doScript(const ::std::string& script)
		{
			this->socket.send(script.c_str(), script.size());
		}
		
		::rl::math::ForceVector
		UniversalRobotsRealtime::getCartesianForce() const
		{
			::rl::math::ForceVector f;
			f.force().x() = this->in.tcpForce[0];
			f.force().y() = this->in.tcpForce[1];
			f.force().z() = this->in.tcpForce[2];
			f.moment().x() = this->in.tcpForce[3];
			f.moment().y() = this->in.tcpForce[4];
			f.moment().z() = this->in.tcpForce[5];
			return f;
		}
		
		::rl::math::Transform
		UniversalRobotsRealtime::getCartesianPosition() const
		{
			::rl::math::Transform x;
			x.setIdentity();
			
			::rl::math::Vector3 orientation(this->in.toolVectorActual[3], this->in.toolVectorActual[4], this->in.toolVectorActual[5]);
			::rl::math::Real norm = orientation.norm();
			
			if (::std::abs(norm) <= ::std::numeric_limits< ::rl::math::Real>::epsilon())
			{
				x.linear().setIdentity();
			}
			else
			{
				x.linear() = ::rl::math::AngleAxis(norm, orientation.normalized()).matrix();
			}
			
			x.translation().x() = this->in.toolVectorActual[0];
			x.translation().y() = this->in.toolVectorActual[1];
			x.translation().z() = this->in.toolVectorActual[2];
			
			return x;
		}
		
		::rl::math::MotionVector
		UniversalRobotsRealtime::getCartesianVelocity() const
		{
			::rl::math::MotionVector v;
			v.linear().x() = this->in.tcpSpeedActual[0];
			v.linear().y() = this->in.tcpSpeedActual[1];
			v.linear().z() = this->in.tcpSpeedActual[2];
			v.angular().x() = this->in.tcpSpeedActual[3];
			v.angular().y() = this->in.tcpSpeedActual[4];
			v.angular().z() = this->in.tcpSpeedActual[5];
			return v;
		}
		
		::boost::dynamic_bitset<>
		UniversalRobotsRealtime::getDigitalInput() const
		{
			return ::boost::dynamic_bitset<>(64, this->in.digitalInputBits);
		}
		
		bool
		UniversalRobotsRealtime::getDigitalInput(const ::std::size_t& i) const
		{
			return this->in.digitalInputBits & (1ULL << i) ? true : false;
		}
		
		::std::size_t
		UniversalRobotsRealtime::getDigitalInputCount() const
		{
			return 64;
		}
		
		::boost::dynamic_bitset<>
		UniversalRobotsRealtime::getDigitalOutput() const
		{
			return ::boost::dynamic_bitset<>(64, this->in.digitalOutputs);
		}
		
		bool
		UniversalRobotsRealtime::getDigitalOutput(const ::std::size_t& i) const
		{
			return this->in.digitalOutputs & (1ULL << i) ? true : false;
		}
		
		::std::size_t
		UniversalRobotsRealtime::getDigitalOutputCount() const
		{
			return 64;
		}
		
		::rl::math::Vector
		UniversalRobotsRealtime::getJointCurrent() const
		{
			::rl::math::Vector i(this->getDof());
			
			for (::std::size_t j = 0; j < 6; ++j)
			{
				i(j) = this->in.iActual[j];
			}
			
			return i;
		}
		
		::rl::math::Vector
		UniversalRobotsRealtime::getJointPosition() const
		{
			::rl::math::Vector q(this->getDof());
			
			for (::std::size_t i = 0; i < 6; ++i)
			{
				q(i) = this->in.qActual[i];
			}
			
			return q;
		}
		
		::rl::math::Vector
		UniversalRobotsRealtime::getJointVelocity() const
		{
			::rl::math::Vector qd(this->getDof());
			
			for (::std::size_t i = 0; i < 6; ++i)
			{
				qd(i) = this->in.qdActual[i];
			}
			
			return qd;
		}
		
		void
		UniversalRobotsRealtime::open()
		{
			this->socket.open();
			this->socket.connect();
			this->socket.setOption(::rl::hal::Socket::OPTION_NODELAY, 1);
			this->setConnected(true);
		}
		
		void
		UniversalRobotsRealtime::start()
		{
			this->setRunning(true);
		}
		
		void
		UniversalRobotsRealtime::step()
		{
			::std::array< ::std::uint8_t, 1060> buffer;
			this->socket.recv(buffer.data(), buffer.size());
#if !defined(__APPLE__) && !defined(__QNX__) && !defined(WIN32)
			this->socket.setOption(::rl::hal::Socket::OPTION_QUICKACK, 1);
#endif // __APPLE__ || __QNX__ || WIN32
			this->in.unserialize(buffer.data());
			
#if 0
			std::cout << "messageSize: " << this->in.messageSize << std::endl;
			std::cout << "time: " << this->in.time << std::endl;
			std::cout << "qTarget[0]: " << this->in.qTarget[0] * rl::math::RAD2DEG << std::endl;
			std::cout << "qTarget[1]: " << this->in.qTarget[1] * rl::math::RAD2DEG << std::endl;
			std::cout << "qTarget[2]: " << this->in.qTarget[2] * rl::math::RAD2DEG << std::endl;
			std::cout << "qTarget[3]: " << this->in.qTarget[3] * rl::math::RAD2DEG << std::endl;
			std::cout << "qTarget[4]: " << this->in.qTarget[4] * rl::math::RAD2DEG << std::endl;
			std::cout << "qTarget[5]: " << this->in.qTarget[5] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdTarget[0]: " << this->in.qdTarget[0] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdTarget[1]: " << this->in.qdTarget[1] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdTarget[2]: " << this->in.qdTarget[2] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdTarget[3]: " << this->in.qdTarget[3] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdTarget[4]: " << this->in.qdTarget[4] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdTarget[5]: " << this->in.qdTarget[5] * rl::math::RAD2DEG << std::endl;
			std::cout << "qddTarget[0]: " << this->in.qddTarget[0] * rl::math::RAD2DEG << std::endl;
			std::cout << "qddTarget[1]: " << this->in.qddTarget[1] * rl::math::RAD2DEG << std::endl;
			std::cout << "qddTarget[2]: " << this->in.qddTarget[2] * rl::math::RAD2DEG << std::endl;
			std::cout << "qddTarget[3]: " << this->in.qddTarget[3] * rl::math::RAD2DEG << std::endl;
			std::cout << "qddTarget[4]: " << this->in.qddTarget[4] * rl::math::RAD2DEG << std::endl;
			std::cout << "qddTarget[5]: " << this->in.qddTarget[5] * rl::math::RAD2DEG << std::endl;
			std::cout << "iTarget[0]: " << this->in.iTarget[0] << std::endl;
			std::cout << "iTarget[1]: " << this->in.iTarget[1] << std::endl;
			std::cout << "iTarget[2]: " << this->in.iTarget[2] << std::endl;
			std::cout << "iTarget[3]: " << this->in.iTarget[3] << std::endl;
			std::cout << "iTarget[4]: " << this->in.iTarget[4] << std::endl;
			std::cout << "iTarget[5]: " << this->in.iTarget[5] << std::endl;
			std::cout << "mTarget[0]: " << this->in.mTarget[0] << std::endl;
			std::cout << "mTarget[1]: " << this->in.mTarget[1] << std::endl;
			std::cout << "mTarget[2]: " << this->in.mTarget[2] << std::endl;
			std::cout << "mTarget[3]: " << this->in.mTarget[3] << std::endl;
			std::cout << "mTarget[4]: " << this->in.mTarget[4] << std::endl;
			std::cout << "mTarget[5]: " << this->in.mTarget[5] << std::endl;
			std::cout << "qActual[0]: " << this->in.qActual[0] * rl::math::RAD2DEG << std::endl;
			std::cout << "qActual[1]: " << this->in.qActual[1] * rl::math::RAD2DEG << std::endl;
			std::cout << "qActual[2]: " << this->in.qActual[2] * rl::math::RAD2DEG << std::endl;
			std::cout << "qActual[3]: " << this->in.qActual[3] * rl::math::RAD2DEG << std::endl;
			std::cout << "qActual[4]: " << this->in.qActual[4] * rl::math::RAD2DEG << std::endl;
			std::cout << "qActual[5]: " << this->in.qActual[5] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdActual[0]: " << this->in.qdActual[0] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdActual[1]: " << this->in.qdActual[1] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdActual[2]: " << this->in.qdActual[2] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdActual[3]: " << this->in.qdActual[3] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdActual[4]: " << this->in.qdActual[4] * rl::math::RAD2DEG << std::endl;
			std::cout << "qdActual[5]: " << this->in.qdActual[5] * rl::math::RAD2DEG << std::endl;
#endif
		}
		
		void
		UniversalRobotsRealtime::stop()
		{
			this->setRunning(false);
		}
		
		void
		UniversalRobotsRealtime::Message::unserialize(::std::uint8_t* ptr)
		{
			this->unserialize(ptr, this->messageSize);
			
			if (756 != this->messageSize && 764 != this->messageSize && 812 != this->messageSize && 1044 != this->messageSize && 1060 != this->messageSize)
			{
				return;
			}
			
			this->unserialize(ptr, this->time);
			this->unserialize(ptr, this->qTarget);
			this->unserialize(ptr, this->qdTarget);
			this->unserialize(ptr, this->qddTarget);
			this->unserialize(ptr, this->iTarget);
			this->unserialize(ptr, this->mTarget);
			this->unserialize(ptr, this->qActual);
			this->unserialize(ptr, this->qdActual);
			this->unserialize(ptr, this->iActual);
			
			if (756 == this->messageSize)
			{
				ptr += 3 * sizeof(double);
				ptr += 15 * sizeof(double);
				this->unserialize(ptr, this->tcpForce);
				this->unserialize(ptr, this->toolVectorActual);
				this->unserialize(ptr, this->tcpSpeedActual);
			}
			else if (764 == this->messageSize || 812 == this->messageSize)
			{
				this->unserialize(ptr, this->toolAccelerometerValues);
				ptr += 15 * sizeof(double);
				this->unserialize(ptr, this->tcpForce);
				this->unserialize(ptr, this->toolVectorActual);
				this->unserialize(ptr, this->tcpSpeedActual);
			}
			else if (1044 == this->messageSize || 1060 == this->messageSize)
			{
				this->unserialize(ptr, this->iControl);
				this->unserialize(ptr, this->toolVectorActual);
				this->unserialize(ptr, this->tcpSpeedActual);
				this->unserialize(ptr, this->tcpForce);
				this->unserialize(ptr, this->toolVectorTarget);
				this->unserialize(ptr, this->tcpSpeedTarget);
			}
			
			this->unserialize(ptr, this->digitalInputBits);
			this->unserialize(ptr, this->motorTemperatures);
			this->unserialize(ptr, this->controllerTimer);
			this->unserialize(ptr, this->testValue);
			
			if (764 == this->messageSize || 812 == this->messageSize || 1044 == this->messageSize || 1060 == this->messageSize)
			{
				this->unserialize(ptr, this->robotMode);
			}
			
			if (812 == this->messageSize || 1044 == this->messageSize || 1060 == this->messageSize)
			{
				this->unserialize(ptr, this->jointModes);
			}
			
			if (1044 == this->messageSize || 1060 == this->messageSize)
			{
				this->unserialize(ptr, this->safetyMode);
				ptr += 6 * sizeof(double);
				this->unserialize(ptr, this->toolAccelerometerValues);
				ptr += 6 * sizeof(double);
				this->unserialize(ptr, this->speedScaling);
				this->unserialize(ptr, this->linearMomentumNorm);
				ptr += 1 * sizeof(double);
				ptr += 1 * sizeof(double);
				this->unserialize(ptr, this->vMain);
				this->unserialize(ptr, this->vRobot);
				this->unserialize(ptr, this->iRobot);
				this->unserialize(ptr, this->vActual);
			}
			
			if (1060 == this->messageSize)
			{
				this->unserialize(ptr, this->digitalOutputs);
				this->unserialize(ptr, this->programState);
			}
		}
		
		template<>
		void
		UniversalRobotsRealtime::Message::unserialize(::std::uint8_t*& ptr, ::std::int64_t& t)
		{
			::std::memcpy(&t, ptr, sizeof(t));
			ptr += sizeof(t);
		}
	}
}
