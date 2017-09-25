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
#include <sstream>
#include <rl/math/Rotation.h>

#include "MitsubishiH7.h"
#include "TimeoutException.h"

namespace rl
{
	namespace hal
	{
		MitsubishiH7::MitsubishiH7(
			const ::std::size_t& dof,
			const ::std::string& addressServer,
			const ::std::string& addressClient,
			const unsigned short int& portTcp,
			const unsigned short int& portUdp,
			const Mode& mode,
			const ::std::uint16_t& haltIoData,
			const ::std::uint16_t& releaseIoData,
			const ::std::uint16_t& shutIoData
		) :
			AxisController(dof),
			CartesianPositionActuator(dof),
			CartesianPositionSensor(dof),
			CyclicDevice(::std::chrono::microseconds(7110)),
			Gripper(),
			JointPositionActuator(dof),
			JointPositionSensor(dof),
			addressClient(addressClient),
			addressServer(addressServer),
			filter(0),
			haltIoData(haltIoData),
			in(),
			mode(mode),
			out(),
			r3(addressServer, portTcp),
			releaseIoData(releaseIoData),
			shutIoData(shutIoData),
			socket(Socket::Udp(Socket::Address::Ipv4(addressServer, portUdp)))
		{
			assert(dof < 9);
			
			this->out.cCount = 0;
		}
		
		MitsubishiH7::~MitsubishiH7()
		{
		}
		
		void
		MitsubishiH7::close()
		{
			this->r3.close();
			this->socket.close();
			this->setConnected(false);
		}
		
		::rl::math::Transform
		MitsubishiH7::getCartesianPosition() const
		{
			::rl::math::Transform x;
			x.setIdentity();
			
			x.linear() = (
				::rl::math::AngleAxis(this->in.dat1.pos.w.c, ::rl::math::Vector3::UnitZ()) *
				::rl::math::AngleAxis(this->in.dat1.pos.w.b, ::rl::math::Vector3::UnitY()) *
				::rl::math::AngleAxis(this->in.dat1.pos.w.a, ::rl::math::Vector3::UnitX())
			).matrix();
			
			x.translation().x() = this->in.dat1.pos.w.x / 1000.0f;
			x.translation().y() = this->in.dat1.pos.w.y / 1000.0f;
			x.translation().z() = this->in.dat1.pos.w.z / 1000.0f;
			
			return x;
		}
		
		::Eigen::Matrix< ::std::int32_t, ::Eigen::Dynamic, 1>
		MitsubishiH7::getCurrentFeedback() const
		{
			::Eigen::Matrix< ::std::int32_t, ::Eigen::Dynamic, 1> c(this->getDof());
			
			switch (this->getDof())
			{
			case 8:
				c(7) = this->in.dat3.pls.p8;
			case 7:
				c(6) = this->in.dat3.pls.p7;
			case 6:
				c(5) = this->in.dat3.pls.p6;
			case 5:
				c(4) = this->in.dat3.pls.p5;
			case 4:
				c(3) = this->in.dat3.pls.p4;
			case 3:
				c(2) = this->in.dat3.pls.p3;
			case 2:
				c(1) = this->in.dat3.pls.p2;
			case 1:
				c(0) = this->in.dat3.pls.p1;
			default:
				break;
			}
			
			return c;
		}
		
		::std::size_t
		MitsubishiH7::getFilter() const
		{
			return this->filter;
		}
		
		::std::uint16_t
		MitsubishiH7::getIoData() const
		{
			return this->in.ioData;
		}
		
		::rl::math::Vector
		MitsubishiH7::getJointPosition() const
		{
			::rl::math::Vector q(this->getDof());
			
			switch (this->getDof())
			{
			case 8:
				q(7) = this->in.dat.jnt.j8;
			case 7:
				q(6) = this->in.dat.jnt.j7;
			case 6:
				q(5) = this->in.dat.jnt.j6;
			case 5:
				q(4) = this->in.dat.jnt.j5;
			case 4:
				q(3) = this->in.dat.jnt.j4;
			case 3:
				q(2) = this->in.dat.jnt.j3;
			case 2:
				q(1) = this->in.dat.jnt.j2;
			case 1:
				q(0) = this->in.dat.jnt.j1;
			default:
				break;
			}
			
			return q;
		}
		
		MitsubishiH7::Mode
		MitsubishiH7::getMode() const
		{
			return this->mode;
		}
		
		::Eigen::Matrix< ::std::int32_t, ::Eigen::Dynamic, 1>
		MitsubishiH7::getMotorPulse() const
		{
			::Eigen::Matrix< ::std::int32_t, ::Eigen::Dynamic, 1> p(this->getDof());
			
			switch (this->getDof())
			{
			case 8:
				p(7) = this->in.dat2.pls.p8;
			case 7:
				p(6) = this->in.dat2.pls.p7;
			case 6:
				p(5) = this->in.dat2.pls.p6;
			case 5:
				p(4) = this->in.dat2.pls.p5;
			case 4:
				p(3) = this->in.dat2.pls.p4;
			case 3:
				p(2) = this->in.dat2.pls.p3;
			case 2:
				p(1) = this->in.dat2.pls.p2;
			case 1:
				p(0) = this->in.dat2.pls.p1;
			default:
				break;
			}
			
			return p;
		}
		
		void
		MitsubishiH7::halt()
		{
			this->setOutput(900, 0x00FF, this->haltIoData);
		}
		
		void
		MitsubishiH7::loadProgram(const ::std::string& name, const ::std::string& program)
		{
			this->r3.doNew();
			this->r3.doLoad(name);
			this->r3.doEclr();
			this->r3.doEmdat(program);
			this->r3.doSave();
		}
		
		void
		MitsubishiH7::open()
		{
			this->r3.open();
			this->socket.open();
			this->socket.connect();
			this->setConnected(true);
		}
		
		void
		MitsubishiH7::release()
		{
			this->setOutput(900, 0x00FF, this->releaseIoData);
		}
		
		void
		MitsubishiH7::setCartesianPosition(const ::rl::math::Transform& x)
		{
			::rl::math::Vector3 abc = x.rotation().eulerAngles(2, 1, 0).reverse();
			
			this->out.dat.pos.w.a = static_cast<float>(abc(0));
			this->out.dat.pos.w.b = static_cast<float>(abc(1));
			this->out.dat.pos.w.c = static_cast<float>(abc(2));
			
			this->out.dat.pos.w.x = static_cast<float>(x.translation().x() * 1000.0f);
			this->out.dat.pos.w.y = static_cast<float>(x.translation().y() * 1000.0f);
			this->out.dat.pos.w.z = static_cast<float>(x.translation().z() * 1000.0f);
			
			this->out.command = MXT_COMMAND_MOVE;
			this->out.sendType = MXT_TYPE_POSE;
		}
		
		void
		MitsubishiH7::setFilter(const ::std::size_t& filter)
		{
			this->filter = filter;
		}
		
		void
		MitsubishiH7::setInput(const ::std::uint16_t& bitTop)
		{
			this->out.bitTop = bitTop;
			this->out.recvIoType = MXT_IO_IN;
			this->out.sendIoType = MXT_IO_NULL;
		}
		
		void
		MitsubishiH7::setJointPosition(const ::rl::math::Vector& q)
		{
			assert(q.size() >= this->getDof());
			
			this->out.dat.jnt.j1 = 0.0f;
			this->out.dat.jnt.j2 = 0.0f;
			this->out.dat.jnt.j3 = 0.0f;
			this->out.dat.jnt.j4 = 0.0f;
			this->out.dat.jnt.j5 = 0.0f;
			this->out.dat.jnt.j6 = 0.0f;
			this->out.dat.jnt.j7 = 0.0f;
			this->out.dat.jnt.j8 = 0.0f;
			
			switch (this->getDof())
			{
			case 8:
				this->out.dat.jnt.j8 = static_cast<float>(q(7));
			case 7:
				this->out.dat.jnt.j7 = static_cast<float>(q(6));
			case 6:
				this->out.dat.jnt.j6 = static_cast<float>(q(5));
			case 5:
				this->out.dat.jnt.j5 = static_cast<float>(q(4));
			case 4:
				this->out.dat.jnt.j4 = static_cast<float>(q(3));
			case 3:
				this->out.dat.jnt.j3 = static_cast<float>(q(2));
			case 2:
				this->out.dat.jnt.j2 = static_cast<float>(q(1));
			case 1:
				this->out.dat.jnt.j1 = static_cast<float>(q(0));
			default:
				break;
			}
			
			this->out.command = MXT_COMMAND_MOVE;
			this->out.sendType = MXT_TYPE_JOINT;
		}
		
		void
		MitsubishiH7::setMode(const Mode& mode)
		{
			this->mode = mode;
		}
		
		void
		MitsubishiH7::setMotorPulse(const ::Eigen::Matrix< ::std::int32_t, ::Eigen::Dynamic, 1>& p)
		{
			assert(p.size() >= this->getDof());
			
			this->out.dat2.pls.p1 = 0;
			this->out.dat2.pls.p2 = 0;
			this->out.dat2.pls.p3 = 0;
			this->out.dat2.pls.p4 = 0;
			this->out.dat2.pls.p5 = 0;
			this->out.dat2.pls.p6 = 0;
			this->out.dat2.pls.p7 = 0;
			this->out.dat2.pls.p8 = 0;
			
			switch (this->getDof())
			{
			case 8:
				this->out.dat2.pls.p8 = p(7);
			case 7:
				this->out.dat2.pls.p7 = p(6);
			case 6:
				this->out.dat2.pls.p6 = p(5);
			case 5:
				this->out.dat2.pls.p5 = p(4);
			case 4:
				this->out.dat2.pls.p4 = p(3);
			case 3:
				this->out.dat2.pls.p3 = p(2);
			case 2:
				this->out.dat2.pls.p2 = p(1);
			case 1:
				this->out.dat2.pls.p1 = p(0);
			default:
				break;
			}
			
			this->out.command = MXT_COMMAND_MOVE;
			this->out.sendType = MXT_TYPE_PULSE;
		}
		
		void
		MitsubishiH7::setOutput(const ::std::uint16_t& bitTop, const ::std::uint16_t& bitMask, const ::std::uint16_t& ioData)
		{
			this->out.bitMask = bitMask;
			this->out.bitTop = bitTop;
			this->out.ioData = ioData;
			this->out.recvIoType = MXT_IO_NULL;
			this->out.sendIoType = MXT_IO_OUT;
		}
		
		void
		MitsubishiH7::shut()
		{
			this->setOutput(900, 0x00FF, this->shutIoData);
		}
		
		void
		MitsubishiH7::start()
		{
			this->stopProgram();
			
			Socket::Address address = Socket::Address::Ipv4(this->addressClient, "");
			
			::std::stringstream program;
			program << "10 OPEN \"ENET:" << address.getNameInfo(true) << "\" AS #1" << "\v";
			program << "20 MXT 1," << this->mode << "," << this->filter << "\v";
			program << "30 HLT" << "\v";
			program << "40 END";
			
			this->loadProgram("1432", program.str());
			
			this->r3.doCntl(true);
			this->r3.doSrv(true);
			this->r3.doCntl(false);
			
			MitsubishiR3::StopState state;
			state.runSts.isServoOn = false;
			
			while (!state.runSts.isServoOn)
			{
				state = this->r3.doDstate();
			}
			
			this->startProgram("1432");
			
			this->setRunning(true);
		}
		
		void
		MitsubishiH7::startProgram(const ::std::string& name)
		{
			this->r3.doCntl(true);
			this->r3.doSlotinit();
			this->r3.doRun(name, true);
			this->r3.doCntl(false);
		}
		
		void
		MitsubishiH7::step()
		{
			this->out.recvType = MXT_TYPE_JOINT;
			this->out.recvType1 = MXT_TYPE_POSE;
			this->out.recvType2 = MXT_TYPE_PULSE;
			this->out.recvType3 = MXT_TYPE_CURRENT_FEEDBACK;
			
			this->socket.send(&this->out, sizeof(Command));
			
			++this->out.cCount;
			
			try
			{
				this->socket.select(true, false, this->filter * this->getUpdateRate() + 6 * this->getUpdateRate());
			}
			catch (const TimeoutException& e)
			{
				throw e;
			}
			
			this->socket.recv(&this->in, sizeof(Command));
			
			this->out.bitMask = 0;
			this->out.bitTop = 0;
			this->out.command = MXT_COMMAND_NULL;
			this->out.ioData = 0;
			this->out.recvIoType = MXT_IO_NULL;
			this->out.recvType = MXT_TYPE_NULL;
			this->out.recvType1 = MXT_TYPE_NULL;
			this->out.recvType2 = MXT_TYPE_NULL;
			this->out.recvType3 = MXT_TYPE_NULL;
			this->out.sendIoType = MXT_IO_NULL;
			this->out.sendType = MXT_TYPE_NULL;
		}
		
		void
		MitsubishiH7::stop()
		{
			this->out.bitMask = 0;
			this->out.bitTop = 0;
			this->out.command = MXT_COMMAND_END;
			this->out.ioData = 0;
			this->out.recvType = MXT_TYPE_NULL;
			this->out.recvType1 = MXT_TYPE_NULL;
			this->out.recvType2 = MXT_TYPE_NULL;
			this->out.recvType3 = MXT_TYPE_NULL;
			this->out.recvIoType = MXT_IO_NULL;
			this->out.sendType = MXT_TYPE_NULL;
			this->out.sendIoType = MXT_IO_NULL;
			
			this->socket.send(&this->out, sizeof(Command));
			
			this->stopProgram();
			
			MitsubishiR3::StopState state = this->r3.doDstate();
			
			if (state.runSts.isServoOn)
			{
				this->r3.doSrv(false);
				
				while (state.runSts.isServoOn)
				{
					state = this->r3.doDstate();
				}
			}
			
			this->setRunning(false);
		}
		
		void
		MitsubishiH7::stopProgram()
		{
			this->r3.doStop();
		}
	}
}
