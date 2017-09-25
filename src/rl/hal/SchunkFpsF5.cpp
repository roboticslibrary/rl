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

#include "DeviceException.h"
#include "Endian.h"
#include "SchunkFpsF5.h"

namespace rl
{
	namespace hal
	{
		SchunkFpsF5::SchunkFpsF5(const ::std::string& filename) :
			CyclicDevice(::std::chrono::nanoseconds::zero()),
			RangeSensor(),
			a(false),
			area(false),
			b(false),
			c(false),
			closed(false),
			fulcrums(),
			interpolated(0),
			opened(false),
			reCalc(false),
			record(false),
			serial(
				filename,
				Serial::BAUDRATE_9600BPS,
				Serial::DATABITS_8BITS,
				Serial::FLOWCONTROL_OFF,
				Serial::PARITY_NOPARITY,
				Serial::STOPBITS_1BIT
			),
			temperature(0),
			update(false),
			value(0),
			voltage(0)
		{
		}
		
		SchunkFpsF5::~SchunkFpsF5()
		{
		}
		
		void
		SchunkFpsF5::close()
		{
			assert(this->isConnected());
			this->serial.close();
			this->setConnected(false);
		}
		
		::std::uint16_t
		SchunkFpsF5::crc(const ::std::uint8_t* buf, const ::std::size_t& len) const
		{
			::std::uint16_t checksum = 0xFFFF;
			
			for (::std::size_t i = 0; i < len; ++i)
			{
				checksum ^= buf[i] << 8;
				
				for (::std::size_t j = 0; j < 8; ++j)
				{
					if (checksum & 0x8000)
					{
						checksum <<= 1;
						checksum ^= 0x1021;
					}
					else
					{
						checksum <<= 1;
					}
				}
			}
			
			return checksum;
		}
		
		::rl::math::Vector
		SchunkFpsF5::getDistances() const
		{
			assert(this->isConnected());
			
			::rl::math::Vector distances(this->getDistancesCount());
			distances(0) = this->interpolated;
			return distances;
		}
		
		::std::size_t
		SchunkFpsF5::getDistancesCount() const
		{
			return 1;
		}
		
		::rl::math::Real
		SchunkFpsF5::getDistancesMaximum(const ::std::size_t& i) const
		{
			assert(i < this->getDistancesCount());
			return 0.06f;
		}
		
		::rl::math::Real
		SchunkFpsF5::getDistancesMinimum(const ::std::size_t& i) const
		{
			assert(i < this->getDistancesCount());
			return 0;
		}
		
		::rl::math::Real
		SchunkFpsF5::getTemperature() const
		{
			return this->temperature;
		}
		
		::rl::math::Real
		SchunkFpsF5::getVoltage() const
		{
			return this->voltage;
		}
		
		bool
		SchunkFpsF5::isA() const
		{
			return this->a;
		}
		
		bool
		SchunkFpsF5::isArea() const
		{
			return this->area;
		}
		
		bool
		SchunkFpsF5::isB() const
		{
			return this->b;
		}
		
		bool
		SchunkFpsF5::isC() const
		{
			return this->c;
		}
		
		bool
		SchunkFpsF5::isClosed() const
		{
			return this->closed;
		}
		
		bool
		SchunkFpsF5::isOpened() const
		{
			return this->opened;
		}
		
		bool
		SchunkFpsF5::isReCalc() const
		{
			return this->reCalc;
		}
		
		bool
		SchunkFpsF5::isRecord() const
		{
			return this->record;
		}
		
		bool
		SchunkFpsF5::isUpdate() const
		{
			return this->update;
		}
		
		void
		SchunkFpsF5::open()
		{
			this->serial.open();
			this->setConnected(true);
		}
		
		::std::size_t
		SchunkFpsF5::recv(::std::uint8_t* buf, const ::std::size_t& len, const ::std::uint8_t& command)
		{
			assert(this->isConnected());
			assert(len > 7);
			assert(len < 264);
			
			::std::uint8_t* ptr;
			::std::size_t sumbytes;
			::std::size_t numbytes;
			
			do
			{
				ptr = buf;
				sumbytes = 0;
				
				do
				{
					this->serial.select(true, false, ::std::chrono::seconds(1));
					numbytes = this->serial.read(ptr, 1);
				}
				while (0x02 != buf[0]);
				
				ptr += numbytes;
				sumbytes += numbytes;
				
				this->serial.select(true, false, ::std::chrono::seconds(1));
				numbytes = this->serial.read(ptr, 1);
				
				ptr += numbytes;
				sumbytes += numbytes;
			}
			while (command != buf[1]);
			
			while (sumbytes < len)
			{
				this->serial.select(true, false, ::std::chrono::seconds(1));
				numbytes = this->serial.read(ptr, len - sumbytes);
				
				ptr += numbytes;
				sumbytes += numbytes;
			}
			
			if (this->crc(buf, sumbytes - 2) != Endian::hostWord(buf[sumbytes - 2], buf[sumbytes - 1]))
			{
				throw DeviceException("checksum error");
			}
			
			return sumbytes;
		}
		
		void
		SchunkFpsF5::send(::std::uint8_t* buf, const ::std::size_t& len)
		{
			assert(this->isConnected());
			assert(len > 7);
			assert(len < 264);
			
			buf[0] = 0x02;
			buf[len - 3] = 0x03;
			
			::std::uint16_t checksum = this->crc(buf, len - 2);
			
			buf[len - 2] = Endian::hostHighByte(checksum);
			buf[len - 1] = Endian::hostLowByte(checksum);
			
			if (len != this->serial.write(buf, len))
			{
				throw DeviceException("could not send complete data");
			}
		}
		
		void
		SchunkFpsF5::start()
		{
			::std::array< ::std::uint8_t, 263> buf;
			
			// fulcrums
			
			buf[1] = 0x01;
			buf[2] = 0x64;
			buf[3] = 0x00;
			buf[4] = 128;
			
			this->send(buf.data(), 1 + 1 + 2 + 1 + 1 + 2);
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 128 + 1 + 2, 0x81);
			
			if (0x01 == buf[5] && 0x01 == buf[6])
			{
				int size = buf[7];
				
				for (int i = 0; i < ::std::min(size, 31); ++i)
				{
					this->fulcrums.insert(::std::make_pair(
						5.0f * (Endian::hostWord(~buf[9 + i * 4], ~buf[9 + i * 4 + 1]) - 0x0000) / (0xFFF0 - 0x0000),
						Endian::hostWord(buf[11 + i * 4], buf[11 + i * 4 + 1]) / 1000.0f / 1000.0f
					));
				}
				
				if (size > 31)
				{
					buf[1] = 0x01;
					buf[2] = 0xE4;
					buf[3] = 0x00;
					buf[4] = 128;
					
					this->send(buf.data(), 1 + 1 + 2 + 1 + 1 + 2);
					this->recv(buf.data(), 1 + 1 + 2 + 1 + 128 + 1 + 2, 0x81);
					
					for (int i = 0; i < ::std::min(size - 31, 31); ++i)
					{
						this->fulcrums.insert(::std::make_pair(
							5.0f * (Endian::hostWord(~buf[5 + i * 4], ~buf[5 + i * 4 + 1]) - 0x0000) / (0xFFF0 - 0x0000),
							Endian::hostWord(buf[7 + i * 4], buf[7 + i * 4 + 1]) / 1000.0f / 1000.0f
						));
					}
				}
			}
		}
		
		void
		SchunkFpsF5::step()
		{
			::std::array< ::std::uint8_t, 263> buf;
			
			buf[1] = 0x03;
			buf[2] = 0x06;
			buf[3] = 0x00;
			buf[4] = 32;
			
			this->send(buf.data(), 1 + 1 + 2 + 1 + 1 + 2);
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 32 + 1 + 2, 0x83);
			
			// 00001000
			this->opened = (buf[5] & 8) ? true : false;
			// 00010000
			this->c = (buf[5] & 16) ? true : false;
			// 00100000
			this->b = (buf[5] & 32) ? true : false;
			// 01000000
			this->a = (buf[5] & 64) ? true : false;
			// 10000000
			this->closed = (buf[5] & 128) ? true : false;
			
			this->value = 5.0f * (Endian::hostWord(~buf[31], ~buf[32]) - 0x0000) / (0xFFF0 - 0x0000);
			
			::std::set< ::std::pair< ::rl::math::Real, ::rl::math::Real>>::iterator lower = this->fulcrums.upper_bound(::std::make_pair(this->value, ::std::numeric_limits< ::rl::math::Real>::max()));
			
			if (!this->fulcrums.empty())
			{
				if (this->value <= (*this->fulcrums.begin()).first)
				{
					this->interpolated = (*this->fulcrums.begin()).second;
				}
				else if (this->value >= (*this->fulcrums.rbegin()).first)
				{
					this->interpolated = (*this->fulcrums.rbegin()).second;
				}
				else
				{
					::std::set< ::std::pair< ::rl::math::Real, ::rl::math::Real>>::iterator upper = lower--;
					this->interpolated = (this->value - (*lower).first) / ((*upper).first - (*lower).first) * ((*upper).second - (*lower).second) + (*lower).second;
				}
			}
			else
			{
				this->interpolated = ::std::numeric_limits< ::rl::math::Real>::quiet_NaN();
			}
			
			this->voltage = 23.0f * (Endian::hostWord(buf[33], buf[34]) - 0x3D00) / (0xB700 - 0x3D00) + 12.0f;
			this->temperature = 90.0f * (Endian::hostWord(buf[35], 0x0) - 0x0A00) / (0xC300 - 0x0A00) - 20.0f;
			
			// 00000001
			this->area = (buf[36] & 1) ? true : false;
			// 00100000
			this->update = (buf[36] & 32) ? true : false;
			// 01000000
			this->record = (buf[36] & 64) ? true : false;
			// 10000000
			this->reCalc = (buf[36] & 128) ? true : false;
		}
		
		void
		SchunkFpsF5::stop()
		{
		}
	}
}
