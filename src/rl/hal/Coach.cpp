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

#include <cassert>
#include <string>
#include <thread>
#include <boost/iostreams/stream.hpp>

#include "Coach.h"

namespace rl
{
	namespace hal
	{
		Coach::Coach(
			const ::std::size_t& dof,
			const ::std::chrono::nanoseconds& updateRate,
			const ::std::size_t& i,
			const ::std::string& address,
			const unsigned short int& port
		) :
			AxisController(dof),
			CyclicDevice(updateRate),
			JointPositionActuator(dof),
			JointPositionSensor(dof),
			JointTorqueActuator(dof),
			JointVelocityActuator(dof),
			i(i),
			in(),
			out(),
			socket(Socket::Tcp(Socket::Address::Ipv4(address, port)))
		{
		}
		
		Coach::~Coach()
		{
		}
		
		void
		Coach::close()
		{
			this->socket.close();
			this->setConnected(false);
		}
		
		::rl::math::Vector
		Coach::getJointPosition() const
		{
			::rl::math::Vector q(this->getDof());
			
			::boost::iostreams::stream< ::boost::iostreams::basic_array_source<char>> stream(this->in.data(), this->in.size());
			
			::std::size_t cmd;
			stream >> cmd;
			::std::size_t id;
			stream >> id;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				stream >> q(i);
			}
			
			return q;
		}
		
		void
		Coach::open()
		{
			this->socket.open();
			this->socket.connect();
			this->setConnected(true);
		}
		
		void
		Coach::setJointPosition(const ::rl::math::Vector& q)
		{
			assert(this->getDof() >= q.size());
			
			this->out << 2 << " " << this->i;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->out << " " << q(i);
			}
			
			this->out << ::std::endl;
		}
		
		void
		Coach::setJointTorque(const ::rl::math::Vector& tau)
		{
			assert(this->getDof() >= tau.size());
			
			this->out << 5 << " " << this->i;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->out << " " << tau(i);
			}
			
			this->out << ::std::endl;
		}
		
		void
		Coach::setJointVelocity(const ::rl::math::Vector& qd)
		{
			assert(this->getDof() >= qd.size());
			
			this->out << 3 << " " << this->i;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->out << " " << qd(i);
			}
			
			this->out << ::std::endl;
		}
		
		void
		Coach::start()
		{
			this->setRunning(true);
		}
		
		void
		Coach::step()
		{
			::std::chrono::steady_clock::time_point start = ::std::chrono::steady_clock::now();
			
			this->out << 6 << " " << this->i << ::std::endl;
			
			this->socket.send(this->out.str().c_str(), this->out.str().length());
			
			this->out.clear();
			this->out.str("");
			
			this->in.fill(0);
			this->socket.recv(this->in.data(), this->in.size());
			
			::std::this_thread::sleep_until(start + this->getUpdateRate());
		}
		
		void
		Coach::stop()
		{
			this->setRunning(false);
		}
	}
}
