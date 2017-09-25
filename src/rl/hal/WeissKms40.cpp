//
// Copyright (c) 2016, Markus Rickert
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
#include <cassert>

#include "WeissException.h"
#include "WeissKms40.h"

namespace rl
{
	namespace hal
	{
		WeissKms40::WeissKms40(
			const ::std::string& address,
			const unsigned short int& port,
			const ::std::size_t& filter,
			const ::std::size_t& divider
		) :
			CyclicDevice(::std::chrono::milliseconds(2 * divider)),
			SixAxisForceTorqueSensor(),
			divider(divider),
			filter(filter),
			frame(6),
			socket(Socket::Tcp(Socket::Address::Ipv4(address, port)))
		{
		}
		
		WeissKms40::~WeissKms40()
		{
			if (this->isRunning())
			{
				this->stop(); // TODO
			}
		}
		
		void
		WeissKms40::close()
		{
			assert(this->isConnected());
			this->socket.close();
			this->setConnected(false);
		}
		
		::rl::math::Vector
		WeissKms40::doAcquireSingleFrame()
		{
			::std::string command = "F()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("F=");
			
			::rl::math::Vector frame(6);
			
			::std::size_t begin = reply.find_first_of('{') + 1;
			::std::size_t end = reply.find_last_of('}');
			
			for (::std::size_t i = 0, start = begin; i < 6; ++i)
			{
				::std::size_t stop = ::std::min(end, reply.find_first_of(',', start));
				frame(i) = ::std::stod(reply.substr(start, stop - start));
				start = ++stop;
			}
			
			return frame;
		}
		
		::std::pair< ::std::chrono::system_clock::time_point, ::std::chrono::system_clock::duration>
		WeissKms40::doGetCalibrationDateLifetime()
		{
			::std::string command = "CALDATE()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("CALDATE=");
			
			::std::size_t comma = reply.find_first_of(',', 8);
			
			::std::chrono::system_clock::time_point date(::std::chrono::seconds(::std::stoi(reply.substr(8, comma))));
			::std::chrono::system_clock::duration lifetime(::std::chrono::seconds(::std::stoi(reply.substr(comma + 1))));
			
			return ::std::make_pair(date, lifetime);
		}
		
		::rl::math::Matrix
		WeissKms40::doGetCalibrationMatrix()
		{
			return ::rl::math::Matrix();
		}
		
		::boost::dynamic_bitset<>
		WeissKms40::doGetDataAcquisitionMask()
		{
			::std::string command = "LMASK()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("LMASK=");
			
			::boost::dynamic_bitset<> mask(6);
			
			::std::size_t begin = reply.find_first_of('{') + 1;
			::std::size_t end = reply.find_last_of('}');
			
			for (::boost::dynamic_bitset<>::size_type i = 0, start = begin; i < 6; ++i)
			{
				::std::size_t stop = ::std::min(end, reply.find_first_of(',', start));
				mask[i] = 0 == ::std::stoi(reply.substr(start, stop - start)) ? false : true;
				start = ++stop;
			}
			
			return mask;
		}
		
		::std::string
		WeissKms40::doGetDescriptorString()
		{
			::std::string command = "D()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("D=");
			
			return reply.substr(2, reply.size() - 3);
		}
		
		::std::size_t
		WeissKms40::doGetFilter()
		{
			::std::string command = "FLT()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("FLT=");
			
			return ::std::stoi(reply.substr(4));
		}
		
		::std::string
		WeissKms40::doGetFirmwareVersion()
		{
			::std::string command = "V()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("V=");
			
			return reply.substr(3, reply.size() - 5);
		}
		
		::std::size_t
		WeissKms40::doGetFrameSendDivider()
		{
			::std::string command = "LDIV()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("LDIV=");
			
			return ::std::stoi(reply.substr(5));
		}
		
		::std::size_t
		WeissKms40::doGetSerialNumber()
		{
			::std::string command = "SN()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("SN=");
			
			return ::std::stoi(reply.substr(3));
		}
		
		WeissKms40::SystemState
		WeissKms40::doGetSystemFlags()
		{
			::std::string command = "FLAGS()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("FLAGS=");
			
			return static_cast<SystemState>(::std::stoi(reply.substr(6)));
		}
		
		::std::string
		WeissKms40::doGetSystemType()
		{
			::std::string command = "ID()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("ID=");
			
			return reply.substr(4, reply.size() - 6);
		}
		
		bool
		WeissKms40::doGetTare()
		{
			::std::string command = "TARE()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("TARE=");
			
			return 0 == ::std::stoi(reply.substr(5)) ? false : true;
		}
		
		float
		WeissKms40::doGetTemperature()
		{
			::std::string command = "T()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("T=");
			
			return ::std::stof(reply.substr(2));
		}
		
		::std::size_t
		WeissKms40::doGetVerboseLevel()
		{
			::std::string command = "VL()\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("VL=");
			
			return ::std::stoi(reply.substr(3));
		}
		
		::std::vector< ::std::string>
		WeissKms40::doPrintVariable(const ::std::vector< ::std::string>& variables)
		{
			::std::string command = "PRINT(";
			
			for (::std::size_t i = 0; i < variables.size(); ++i)
			{
				command += (i > 0 ? "," : "") + variables[i];
			}
			
			command += ")\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("PRINT=");
			
			::std::vector< ::std::string> values(variables.size());
			
			for (::std::size_t i = 0, start = 6; i < 6; ++i)
			{
				::std::size_t stop = reply.find_first_of(',', start);
				values[i] = reply.substr(start, stop - start);
				start = ++stop;
			}
			
			return variables;
		}
		
		::boost::dynamic_bitset<>
		WeissKms40::doSetDataAcquisitionMask(const ::boost::dynamic_bitset<>& mask)
		{
			::std::string command = "LMASK({";
			
			for (::boost::dynamic_bitset<>::size_type i = 0; i < mask.size(); ++i)
			{
				command += (i > 0 ? "," : "") + ::std::to_string(mask[i] ? 1 : 0);
			}
			
			command += "})\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("LMASK=");
			
			::boost::dynamic_bitset<> mask2(6);
			
			::std::size_t begin = reply.find_first_of('{') + 1;
			::std::size_t end = reply.find_last_of('}');
			
			for (::boost::dynamic_bitset<>::size_type i = 0, start = begin; i < 6; ++i)
			{
				::std::size_t stop = ::std::min(end, reply.find_first_of(',', start));
				mask2[i] = 0 == ::std::stoi(reply.substr(start, stop - start)) ? false : true;
				start = ++stop;
			}
			
			return mask2;
		}
		
		::std::string
		WeissKms40::doSetDescriptorString(const ::std::string& value)
		{
			::std::string command = "D(" + value + ")\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("D=");
			
			return reply.substr(2, reply.size() - 3);
		}
		
		::std::size_t
		WeissKms40::doSetFilter(const ::std::size_t& value)
		{
			::std::string command = "FLT(" + ::std::to_string(value) + ")\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("FLT=");
			
			return ::std::stoi(reply.substr(4));
		}
		
		::std::size_t
		WeissKms40::doSetFrameSendDivider(const ::std::size_t& value)
		{
			::std::string command = "LDIV(" + ::std::to_string(value) + ")\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("LDIV=");
			
			return ::std::stoi(reply.substr(5));
		}
		
		bool
		WeissKms40::doSetTare(const bool& doOn)
		{
			::std::string command = "TARE(" + ::std::to_string(doOn ? 1 : 0) + ")\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("TARE=");
			
			return 0 == ::std::stoi(reply.substr(5)) ? false : true;
		}
		
		::std::size_t
		WeissKms40::doSetVerboseLevel(const ::std::size_t& level)
		{
			::std::string command = "VL(" + ::std::to_string(level) + ")\n";
			this->socket.send(command.c_str(), command.size());
			
			::std::string reply = this->recv("VL=");
			
			return ::std::stoi(reply.substr(3));
		}
		
		void
		WeissKms40::doStartContinuousDataAcquisition()
		{
			::std::string command = "L1()\n";
			this->socket.send(command.c_str(), command.size());
			
			this->recv("L1");
		}
		
		void
		WeissKms40::doStopContinuousDataAcquisition()
		{
			::std::string command = "L0()\n";
			this->socket.send(command.c_str(), command.size());
			
			this->recv("L0");
		}
		
		::rl::math::Vector
		WeissKms40::getForces() const
		{
			return this->frame.head(3);
		}
		
		::rl::math::Vector
		WeissKms40::getForcesTorques() const
		{
			return this->frame;
		}
		
		::rl::math::Real
		WeissKms40::getForcesTorquesMaximum(const ::std::size_t& i) const
		{
			switch (i)
			{
			case 0:
			case 1:
			case 2:
				return 120;
				break;
			case 3:
			case 4:
			case 5:
				return 3;
				break;
			default:
				return ::std::numeric_limits< ::rl::math::Real>::signaling_NaN();
				break;
			}
		}
		
		::rl::math::Real
		WeissKms40::getForcesTorquesMinimum(const ::std::size_t& i) const
		{
			switch (i)
			{
			case 0:
			case 1:
			case 2:
				return -120;
				break;
			case 3:
			case 4:
			case 5:
				return -3;
				break;
			default:
				return ::std::numeric_limits< ::rl::math::Real>::signaling_NaN();
				break;
			}
		}
		
		::rl::math::Vector
		WeissKms40::getTorques() const
		{
			return this->frame.tail(3);;
		}
		
		void
		WeissKms40::open()
		{
			assert(!this->isConnected());
			this->socket.open();
			this->socket.connect();
			this->setConnected(true);
		}
		
		::std::string
		WeissKms40::recv(const ::std::string& command)
		{
			::std::string reply;
			
			::std::array<char, 256> buffer;
			char* ptr;
			::std::size_t sumbytes;
			::std::size_t numbytes;
			
			do
			{
				ptr = buffer.data();
				sumbytes = 0;
				
				do
				{
					numbytes = this->socket.recv(ptr, 1);
					ptr += numbytes;
					sumbytes += numbytes;
				}
				while ('\n' != buffer.data()[sumbytes - 1]);
				
				reply = ::std::string(buffer.data(), sumbytes);
				
				if ("ERROR" == reply.substr(0, 5))
				{
					throw WeissException(static_cast<WeissException::Code>(::std::stoi(reply.substr(6, reply.find_first_of(",)") - 6))));
				}
			}
			while (command != reply.substr(0, command.size()));
			
			return reply;
		}
		
		void
		WeissKms40::start()
		{
			assert(this->isConnected());
			
			this->doSetDataAcquisitionMask(::boost::dynamic_bitset<>(6, 63));
			this->doSetFilter(this->filter);
			this->doSetFrameSendDivider(this->divider);
			this->doSetVerboseLevel(0);
			
			this->doStartContinuousDataAcquisition();
			
			this->setRunning(true);
		}
		
		void
		WeissKms40::step()
		{
			assert(this->isConnected());
			assert(this->isRunning());
			
			::std::string reply = this->recv("F=");
			
			::std::size_t begin = reply.find_first_of('{') + 1;
			::std::size_t end = reply.find_last_of('}');
			
			for (::std::size_t i = 0, start = begin; i < 6; ++i)
			{
				::std::size_t stop = ::std::min(end, reply.find_first_of(',', start));
				this->frame(i) = ::std::stod(reply.substr(start, stop - start));
				start = ++stop;
			}
		}
		
		void
		WeissKms40::stop()
		{
			assert(this->isConnected());
			assert(this->isRunning());
			
			this->doStopContinuousDataAcquisition();
			
			this->setRunning(false);
		}
	}
}
