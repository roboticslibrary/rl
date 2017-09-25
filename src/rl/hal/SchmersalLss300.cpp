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
#include <iostream>
#include <rl/math/Unit.h>

#include "DeviceException.h"
#include "Endian.h"
#include "SchmersalLss300.h"
#include "TimeoutException.h"

namespace rl
{
	namespace hal
	{
		SchmersalLss300::SchmersalLss300(
			const ::std::string& filename,
			const BaudRate& baudRate,
			const Monitoring& monitoring,
			const ::std::string& password
		) :
			CyclicDevice(::std::chrono::nanoseconds::zero()),
			Lidar(),
			baudRate(BAUDRATE_9600BPS),
			data(),
			desired(baudRate),
			monitoring(monitoring),
			password(password),
			serial(
				filename,
				Serial::BAUDRATE_9600BPS,
				Serial::DATABITS_8BITS,
				Serial::FLOWCONTROL_OFF,
				Serial::PARITY_NOPARITY,
				Serial::STOPBITS_1BIT
			)
		{
			assert(8 == password.length());
		}
		
		SchmersalLss300::~SchmersalLss300()
		{
		}
		
		void
		SchmersalLss300::close()
		{
			assert(this->isConnected());
			
			if (MONITORING_SINGLE != this->monitoring)
			{
				this->setMonitoring(MONITORING_SINGLE);
			}
			
			if (BAUDRATE_9600BPS != this->baudRate)
			{
				this->setBaudRate(BAUDRATE_9600BPS);
			}
			
			this->serial.close();
			
			this->setConnected(false);
		}
		
		::std::uint16_t
		SchmersalLss300::crc(const ::std::uint8_t* buf, const ::std::size_t& len) const
		{	
			::std::uint16_t checksum = buf[0];
			
			for (::std::size_t i = 1; i < len; ++i)
			{
				if (checksum & 0x8000)
				{
					checksum <<= 1;
					checksum ^= 0x8005;
				}
				else
				{
					checksum <<= 1;
				}
				
				checksum ^= (buf[i - 1] << 8) | buf[i];
			}
			
			return checksum;
		}
		
		SchmersalLss300::BaudRate
		SchmersalLss300::getBaudRate() const
		{
			return this->baudRate;
		}
		
		::rl::math::Vector
		SchmersalLss300::getDistances() const
		{
			assert(this->isConnected());
			
			::rl::math::Vector distances(this->getDistancesCount());
			
			::rl::math::Real scale = 0.01f;
			::std::uint16_t count = Endian::hostWord(this->data[8], this->data[7]);
			::std::uint8_t mask = 0x1F;
			
			for (::std::size_t i = 0; i < count; ++i)
			{
				if (this->data[10 + i * 2] & 32)
				{
					distances(i) = ::std::numeric_limits< ::rl::math::Real>::quiet_NaN();
				}
				else
				{
					distances(i) = Endian::hostWord(this->data[10 + i * 2] & mask, this->data[9 + i * 2]);
					distances(i) *= scale;
				}
			}
			
			return distances;
		}
		
		::std::size_t
		SchmersalLss300::getDistancesCount() const
		{
			return 501;
		}
		
		::rl::math::Real
		SchmersalLss300::getDistancesMaximum(const ::std::size_t& i) const
		{
			assert(i < this->getDistancesCount());
			
			return 60.0f;
		}
		
		::rl::math::Real
		SchmersalLss300::getDistancesMinimum(const ::std::size_t& i) const
		{
			assert(i < this->getDistancesCount());
			
			return 0.0f;
		}
		
		SchmersalLss300::Monitoring
		SchmersalLss300::getMonitoring() const
		{
			assert(this->isConnected());
			
			return this->monitoring;
		}
		
		::rl::math::Real
		SchmersalLss300::getResolution() const
		{
			assert(this->isConnected());
			
			return 0.36f * ::rl::math::DEG2RAD;
		}
		
		::rl::math::Real
		SchmersalLss300::getStartAngle() const
		{
			assert(this->isConnected());
			
			return 0.0f;
		}
		
		::rl::math::Real
		SchmersalLss300::getStopAngle() const
		{
			assert(this->isConnected());
			
			return 180.0f * ::rl::math::DEG2RAD;
		}
		
		::std::string
		SchmersalLss300::getType()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 1013> buf;
			
			buf[4] = 0x00;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 8 + 2, 0x80);
			
			switch (buf[5])
			{
			case 0x01:
				return "LSS 1;V" + ::std::string(reinterpret_cast<char*>(buf.data() + 6), 8);
				break;
			case 0x02:
				return "LSS 2;V" + ::std::string(reinterpret_cast<char*>(buf.data() + 6), 8);
				break;
			default:
				break;
			}
			
			return "";
		}
		
		void
		SchmersalLss300::open()
		{
			this->serial.open();
			
			this->setConnected(true);
			
			::std::array< ::std::uint8_t, 1013> buf;
			
			// synchronize baud rates
			
			buf[4] = 0x66;
			buf[5] = 0x00;
			
			Serial::BaudRate baudRates[4] = {
				Serial::BAUDRATE_57600BPS,
				Serial::BAUDRATE_38400BPS,
				Serial::BAUDRATE_19200BPS,
				Serial::BAUDRATE_9600BPS
			};
			
			for (::std::size_t i = 0; i < 4; ++i)
			{
				this->serial.setBaudRate(baudRates[i]);
				this->serial.changeParameters();
				this->send(buf.data(), 1 + 1 + 2 + 1 + 1 + 2);
				
				if (this->waitAck())
				{
					break;
				}
				
				if (3 == i)
				{
					throw DeviceException("could not sync baud rate.");
				}
			}
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 2, 0xE6);
			
			if (0x81 == buf[5])
			{
				throw DeviceException("mode switchover not possible due to internal processing error");
			}
			
			// status
			
			buf[4] = 0x31;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 1 + 9 + 17 + 17 + 17 + 17 + 1 + 1 + 4 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 2 + 4 + 50 + 2, 0xB1);
			
			// baud rate
			
			if (this->desired != this->baudRate)
			{
				this->setBaudRate(this->desired);
			}
			
			// monitoring
			
			if (0x25 != buf[5])
			{
				this->setMonitoring(MONITORING_SINGLE);
			}
		}
		
		::std::size_t
		SchmersalLss300::recv(::std::uint8_t* buf, const ::std::size_t& len, const ::std::uint8_t& command)
		{
			assert(this->isConnected());
			assert(len > 6);
			
			::std::uint8_t* ptr;
			::std::size_t sumbytes;
			::std::size_t numbytes;
			
			do
			{
				do
				{
					ptr = buf;
					sumbytes = 0;
					
					do
					{
						numbytes = this->serial.read(ptr, 1);
					}
					while (0x02 != buf[0]);
					
					ptr += numbytes;
					sumbytes += numbytes;
					
					numbytes = this->serial.read(ptr, 1);
				}
				while (0x80 != buf[1]);
				
				ptr += numbytes;
				sumbytes += numbytes;
				
				for (::std::size_t i = 0; i < 4; ++i)
				{
					numbytes = this->serial.read(ptr, 1);
					
					ptr += numbytes;
					sumbytes += numbytes;
				}
			}
			while (command != buf[4]);
			
			::std::uint16_t length = Endian::hostWord(buf[3], buf[2]);
			
			if (len != static_cast< ::std::size_t>(length) + 6)
			{
				throw DeviceException("data length mismatch in command " + ::std::to_string(command));
			}
			
			while (sumbytes < len)
			{
				numbytes = this->serial.read(ptr, len - sumbytes);
				
				ptr += numbytes;
				sumbytes += numbytes;
			}
			
			if (this->crc(buf, sumbytes - 2) != Endian::hostWord(buf[sumbytes - 1], buf[sumbytes - 2]))
			{
				throw DeviceException("checksum error");
			}
			
			return sumbytes;
		}
		
		void
		SchmersalLss300::reset()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 1013> buf;
			
			buf[4] = 0x10;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 2, 0x90);
		}
		
		void
		SchmersalLss300::send(::std::uint8_t* buf, const ::std::size_t& len)
		{
			assert(this->isConnected());
			assert(len > 6);
			
			buf[0] = 0x02;
			buf[1] = 0x00;
			
			::std::uint16_t length = len - 6;
			
			buf[2] = Endian::hostLowByte(length);
			buf[3] = Endian::hostHighByte(length);
			
			::std::uint16_t checksum = this->crc(buf, len - 2);
			
			buf[len - 2] = Endian::hostLowByte(checksum);
			buf[len - 1] = Endian::hostHighByte(checksum);
			
			if (len != this->serial.write(buf, len))
			{
				throw DeviceException("could not send complete data");
			}
			
			this->serial.flush(true, false);
		}
		
		void
		SchmersalLss300::setBaudRate(const BaudRate& baudRate)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 1013> buf;
			
			buf[4] = 0x66;
			
			switch (baudRate)
			{
			case BAUDRATE_9600BPS:
				buf[5] = 0x00;
				break;
			case BAUDRATE_19200BPS:
				buf[5] = 0x01;
				break;
			case BAUDRATE_38400BPS:
				buf[5] = 0x02;
				break;
			case BAUDRATE_57600BPS:
				buf[5] = 0x03;
				break;
			default:
				break;
			}
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 2, 0xE6);
			
			if (0x81 == buf[5])
			{
				throw DeviceException("mode switchover not possible due to internal processing error");
			}
			
			switch (baudRate)
			{
			case BAUDRATE_9600BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_9600BPS);
				break;
			case BAUDRATE_19200BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_19200BPS);
				break;
			case BAUDRATE_38400BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_38400BPS);
				break;
			case BAUDRATE_57600BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_57600BPS);
				break;
			default:
				break;
			}
			
			this->serial.changeParameters();
			
			this->baudRate = baudRate;
		}
		
		void
		SchmersalLss300::setMonitoring(const Monitoring& monitoring)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 1013> buf;
			
			buf[4] = 0x20;
			
			switch (monitoring)
			{
			case MONITORING_CONTINUOUS:
				buf[5] = 0x24;
				break;
			case MONITORING_SINGLE:
				buf[5] = 0x25;
				break;
			default:
				break;
			}
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 2, 0xA0);
			
			switch (buf[5])
			{
			case 0x10:
				throw DeviceException("mode switchover not possible due to incorrect operating mode");
				break;
			case 0x12:
				throw DeviceException("mode switchover not possible due to incorrect password");
				break;
			case 0x13:
				throw DeviceException("mode switchover not possible due to not rectangular field");
				break;
			case 0x80:
				throw DeviceException("mode switchover not possible due to a sensor fault");
				break;
			default:
				break;
			}
			
			this->monitoring = monitoring;
		}
		
		void
		SchmersalLss300::start()
		{
			assert(this->isConnected());
			
			if (MONITORING_SINGLE != this->monitoring)
			{
				this->setMonitoring(this->monitoring);
			}
		}
		
		void
		SchmersalLss300::step()
		{
			assert(this->isConnected());
			
			if (MONITORING_SINGLE == this->monitoring)
			{
				this->data[4] = 0x30;
				this->data[5] = 0x01;
				
				do
				{
					this->send(this->data.data(), 1 + 1 + 2 + 1 + 1 + 2);
				}
				while (!this->waitAck());
			}
			
			this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 2 + 1002 + 2, 0xB0);
		}
		
		void
		SchmersalLss300::stop()
		{
			assert(this->isConnected());
			
			if (MONITORING_SINGLE != this->monitoring)
			{
				this->setMonitoring(MONITORING_SINGLE);
			}
		}
		
		bool
		SchmersalLss300::waitAck()
		{
			assert(this->isConnected());
			
			::std::uint8_t ack;
			
			try
			{
				::std::chrono::steady_clock::time_point start = ::std::chrono::steady_clock::now();
				
				do
				{
					this->serial.select(true, false, ::std::chrono::milliseconds(60));
					this->serial.read(&ack, 1);
					
					switch (ack)
					{
					case 0x06:
						return true;
						break;
					case 0x15:
						return false;
						break;
					default:
						break;
					}
				}
				while ((::std::chrono::steady_clock::now() - start) < ::std::chrono::milliseconds(60));
			}
			catch (const TimeoutException&)
			{
			}
			
			return false;
		}
	}
}
