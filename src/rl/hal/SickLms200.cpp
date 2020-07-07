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
#include <rl/math/Constants.h>
#include <rl/util/io/Hex.h>

#include "DeviceException.h"
#include "Endian.h"
#include "SickLms200.h"
#include "TimeoutException.h"

namespace rl
{
	namespace hal
	{
		SickLms200::SickLms200(
			const ::std::string& filename,
			const BaudRate& baudRate,
			const Monitoring& monitoring,
			const Variant& variant,
			const Measuring& measuring,
			const ::std::string& password
		) :
			CyclicDevice(::std::chrono::nanoseconds::zero()),
			Lidar(),
			baudRate(BaudRate::b9600),
			configuration(0x00),
			data(),
			desired(baudRate),
			measuring(measuring),
			monitoring(monitoring),
			password(password),
			serial(
				filename,
				Serial::BaudRate::b9600,
				Serial::DataBits::d8,
				Serial::FlowControl::off,
				Serial::Parity::none,
				Serial::StopBits::s1
			),
			variant(variant)
		{
			assert(8 == password.length());
		}
		
		SickLms200::~SickLms200()
		{
		}
		
		void
		SickLms200::close()
		{
			assert(this->isConnected());
			
			if (Measuring::m8 != this->measuring)
			{
				this->setMeasuring(Measuring::m8);
			}
			
			if (Variant::v180_50 != this->variant)
			{
				this->setVariant(Variant::v180_50);
			}
			
			if (Monitoring::single != this->monitoring)
			{
				this->setMonitoring(Monitoring::single);
			}
			
			if (BaudRate::b9600 != this->baudRate)
			{
				this->setBaudRate(BaudRate::b9600);
			}
			
			this->serial.close();
			
			this->setConnected(false);
		}
		
		::std::uint16_t
		SickLms200::crc(const ::std::uint8_t* buf, const ::std::size_t& len) const
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
		
		void
		SickLms200::dumpConfiguration()
		{
			using namespace ::rl::util::io;
			
			assert(this->isConnected());
			
			::std::array<::std::uint8_t, 812> buf;
			
			buf[4] = 0x74;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 32 + 1 + 2, 0xF4);
			
			::std::cout << "A:  " << Endian::hostWord(buf[6], buf[5]) << ::std::endl;
			::std::cout << "B:  H:" << hex(buf[8]) << " L:" << hex(buf[7]) << ::std::endl;
			::std::cout << "C:  " << hex(buf[9]) << ::std::endl;
			
			if (buf[9] & 1)
			{
				::std::cout << "Availability level 3" << ::std::endl;
			}
			
			if (buf[9] & 2)
			{
				::std::cout << "Send real-time indices" << ::std::endl;
			}
			
			if (buf[9] & 4)
			{
				::std::cout << "Availability level 2" << ::std::endl;
			}
			
			::std::cout << "D:  " << hex(buf[10]) << ::std::endl;
			::std::cout << "E:  " << hex(buf[11]) << ::std::endl;
			::std::cout << "F:  " << hex(buf[12]) << ::std::endl;
			::std::cout << "G:  " << hex(buf[13]) << ::std::endl;
			::std::cout << "H:  " << hex(buf[14]) << ::std::endl;
			::std::cout << "I:  " << hex(buf[15]) << ::std::endl;
			::std::cout << "J:  " << hex(buf[16]) << ::std::endl;
			::std::cout << "K:  " << hex(buf[17]) << ::std::endl;
			::std::cout << "L:  " << hex(buf[18]) << ::std::endl;
			::std::cout << "M:  " << hex(buf[19]) << ::std::endl;
			::std::cout << "N:  " << hex(buf[20]) << ::std::endl;
			::std::cout << "O:  " << hex(buf[21]) << ::std::endl;
			::std::cout << "P:  " << hex(buf[22]) << ::std::endl;
			::std::cout << "Q:  " << hex(buf[23]) << ::std::endl;
			::std::cout << "R:  " << hex(buf[24]) << ::std::endl;
			::std::cout << "S:  " << hex(buf[25]) << ::std::endl;
			::std::cout << "T:  " << hex(buf[26]) << ::std::endl;
			::std::cout << "U:  " << hex(buf[27]) << ::std::endl;
			::std::cout << "V:  " << hex(buf[28]) << ::std::endl;
			::std::cout << "W:  " << hex(buf[29]) << ::std::endl;
			::std::cout << "X:  " << hex(buf[30]) << ::std::endl;
			::std::cout << "Y:  " << hex(buf[31]) << ::std::endl;
			::std::cout << "Z:  " << hex(buf[32]) << ::std::endl;
			::std::cout << "A1: " << hex(buf[33]) << ::std::endl;
			::std::cout << "A2: " << hex(buf[34]) << ::std::endl;
			::std::cout << "A3: H:" << hex(buf[36]) << " L:" << hex(buf[35]) << ::std::endl;
			::std::cout << "A4: " << Endian::hostWord(buf[38], buf[37]) << ::std::endl;
		}
		
		void
		SickLms200::dumpStatus()
		{
			using namespace ::rl::util::io;
			
			assert(this->isConnected());
			
			::std::array<::std::uint8_t, 812> buf;
			
			buf[4] = 0x31;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 152 + 1 + 2, 0xB1);
			
			::std::cout << "A:  " << buf[5] << " " << buf[6] << " " << buf[7] << " " << buf[8] << " " << buf[9] << " " << buf[10] << " " << buf[11] << ::std::endl;
			::std::cout << "B:  " << hex(buf[12]) << ::std::endl;
			::std::cout << "C:  " << hex(buf[13]) << ::std::endl;
			::std::cout << "D:  reserved" << ::std::endl;
			::std::cout << "?:  " << buf[16] << " " << buf[17] << " " << buf[18] << " " << buf[19] << " " << buf[20] << " " << buf[21] << ::std::endl;
			::std::cout << "E:  " << hex(buf[22]) << ::std::endl;
			::std::cout << "F:  " <<
				Endian::hostWord(buf[24], buf[23]) << " " <<
				Endian::hostWord(buf[26], buf[25]) << " " <<
				Endian::hostWord(buf[28], buf[27]) << " " <<
				Endian::hostWord(buf[30], buf[29]) << " " <<
				Endian::hostWord(buf[32], buf[31]) << " " <<
				Endian::hostWord(buf[34], buf[33]) << " " <<
				Endian::hostWord(buf[36], buf[35]) << " " <<
				Endian::hostWord(buf[38], buf[37]) << ::std::endl;
			::std::cout << "G:  " <<
				Endian::hostWord(buf[40], buf[39]) << " " <<
				Endian::hostWord(buf[42], buf[41]) << " " <<
				Endian::hostWord(buf[44], buf[43]) << " " <<
				Endian::hostWord(buf[46], buf[45]) << ::std::endl;
			::std::cout << "H:  " <<
				Endian::hostWord(buf[48], buf[47]) << " " <<
				Endian::hostWord(buf[50], buf[49]) << " " <<
				Endian::hostWord(buf[52], buf[51]) << " " <<
				Endian::hostWord(buf[54], buf[53]) << " " <<
				Endian::hostWord(buf[56], buf[55]) << " " <<
				Endian::hostWord(buf[58], buf[57]) << " " <<
				Endian::hostWord(buf[60], buf[59]) << " " <<
				Endian::hostWord(buf[62], buf[61]) << ::std::endl;
			::std::cout << "I:  " <<
				Endian::hostWord(buf[64], buf[63]) << " " <<
				Endian::hostWord(buf[66], buf[65]) << " " <<
				Endian::hostWord(buf[68], buf[67]) << " " <<
				Endian::hostWord(buf[70], buf[69]) << ::std::endl;
			::std::cout << "J:  " << Endian::hostWord(buf[72], buf[71]) << ::std::endl;
			::std::cout << "K:  reserved" << ::std::endl;
			::std::cout << "L:  " << Endian::hostWord(buf[76], buf[75]) << ::std::endl;
			::std::cout << "M:  reserved" << ::std::endl;
			::std::cout << "N:  " << Endian::hostWord(buf[80], buf[79]) << ::std::endl;
			::std::cout << "O:  " << Endian::hostWord(buf[82], buf[81]) << ::std::endl;
			::std::cout << "P:  reserved" << ::std::endl;
			::std::cout << "Q:  " << Endian::hostWord(buf[86], buf[85]) << ::std::endl;
			::std::cout << "R:  " << Endian::hostWord(buf[88], buf[87]) << ::std::endl;
			::std::cout << "S:  " << Endian::hostWord(buf[90], buf[89]) << ::std::endl;
			::std::cout << "T:  " << Endian::hostWord(buf[92], buf[91]) << ::std::endl;
			::std::cout << "U:  " << Endian::hostWord(buf[94], buf[93]) << ::std::endl;
			::std::cout << "V:  " << Endian::hostWord(buf[96], buf[95]) << ::std::endl;
			::std::cout << "W:  " << Endian::hostWord(buf[98], buf[97]) << ::std::endl;
			::std::cout << "X:  " << Endian::hostWord(buf[100], buf[99]) << ::std::endl;
			::std::cout << "Y:  " << Endian::hostWord(buf[102], buf[101]) << ::std::endl;
			::std::cout << "Z:  " << Endian::hostWord(buf[104], buf[103]) << ::std::endl;
			::std::cout << "A1: reserved" << ::std::endl;
			::std::cout << "A2: " << hex(buf[106]) << ::std::endl;
			::std::cout << "A3: H:" << hex(buf[108]) << " L:" << hex(buf[107]) << ::std::endl;
			::std::cout << "A4: H:" << hex(buf[110]) << " L:" << hex(buf[109]) << ::std::endl;
			::std::cout << "A5: " << Endian::hostWord(buf[112], buf[111]) << ::std::endl;
			::std::cout << "A6: " << Endian::hostWord(buf[114], buf[113]) << ::std::endl;
			::std::cout << "A7: " << hex(buf[115]) << ::std::endl;
			::std::cout << "A8: " << Endian::hostWord(buf[117], buf[116]) << ::std::endl;
			::std::cout << "A9: " << buf[118] << ::std::endl;
			::std::cout << "B1: reserved" << ::std::endl;
			::std::cout << "B2: " << hex(Endian::hostWord(buf[121], buf[120])) << ::std::endl;
			::std::cout << "B3: " << hex(buf[122]) << ::std::endl;
			::std::cout << "B4: " << hex(buf[123]) << ::std::endl;
			::std::cout << "B5: " << hex(buf[124]) << ::std::endl;
			::std::cout << "B6: " << hex(buf[125]) << ::std::endl;
			::std::cout << "B7: " << hex(buf[126]) << ::std::endl;
			::std::cout << "B8: " << hex(buf[127]) << ::std::endl;
			::std::cout << "B9: " << " " << buf[128] << " " << buf[129] << " " << buf[130] << " " << buf[131] << " " << buf[132] << " " << buf[133] << " " << buf[134] << ::std::endl;
			::std::cout << "C1: " << Endian::hostDoubleWord(Endian::hostWord(buf[138], buf[137]), Endian::hostWord(buf[136], buf[135])) << ::std::endl;
			::std::cout << "C2: " << Endian::hostDoubleWord(Endian::hostWord(buf[142], buf[141]), Endian::hostWord(buf[140], buf[139])) << ::std::endl;
			::std::cout << "C3: " << Endian::hostDoubleWord(Endian::hostWord(buf[146], buf[145]), Endian::hostWord(buf[144], buf[143])) << ::std::endl;
			::std::cout << "C4: " << Endian::hostDoubleWord(Endian::hostWord(buf[150], buf[149]), Endian::hostWord(buf[148], buf[147])) << ::std::endl;
			::std::cout << "C5: " << Endian::hostWord(buf[152], buf[151]) << ::std::endl;
			::std::cout << "C6: " << Endian::hostWord(buf[154], buf[153]) << ::std::endl;
			::std::cout << "C7: " << Endian::hostWord(buf[156], buf[155]) << ::std::endl;
		}
		
		SickLms200::BaudRate
		SickLms200::getBaudRate() const
		{
			return this->baudRate;
		}
		
		::rl::math::Vector
		SickLms200::getDistances() const
		{
			assert(this->isConnected());
			
			::rl::math::Vector distances(this->getDistancesCount());
			
			if (this->data[6] & 32)
			{
				throw DeviceException("partial scan");
			}
			
			::rl::math::Real scale;
			
			switch (this->data[6] & 192)
			{
			case 0:
				scale = static_cast<::rl::math::Real>(0.01);
				break;
			case 64:
				scale = static_cast<::rl::math::Real>(0.001);
				break;
			default:
				throw DeviceException("unknown scale");
				break;
			}
			
			::std::uint16_t count = Endian::hostWord(this->data[6] & 11, this->data[5]);
			
			::std::uint8_t mask;
			
			switch (this->configuration)
			{
			case 0x00:
			case 0x01:
			case 0x02:
				mask = 0x1F;
				break;
			case 0x03:
			case 0x04:
			case 0x05:
				mask = 0x3F;
				break;
			case 0x06:
				mask = 0x7F;
				break;
			default:
				mask = 0x00;
				break;
			}
			
			::std::uint16_t value;
			
			for (::std::size_t i = 0; i < count; ++i)
			{
				value = Endian::hostWord(this->data[8 + i * 2] & mask, this->data[7 + i * 2]);
				
				switch (this->configuration)
				{
				case 0x00:
				case 0x01:
				case 0x02:
					switch (value)
					{
					case 0x1FFF:
::std::cerr << "Measured value not valid" << ::std::endl;
					case 0x1FFE:
::std::cerr << "Dazzling" << ::std::endl;
					case 0x1FFD:
::std::cerr << "Operation overflow" << ::std::endl;
					case 0x1FFB:
::std::cerr << "Signal-to-noise ratio too small" << ::std::endl;
					case 0x1FFA:
::std::cerr << "Error when reading channel 1" << ::std::endl;
						distances(i) = ::std::numeric_limits<::rl::math::Real>::quiet_NaN();
						break;
					case 0x1FF7:
::std::cerr << "Measured value > Maximum value" << ::std::endl;
						distances(i) = ::std::numeric_limits<::rl::math::Real>::infinity();
						break;
					default:
						distances(i) = value;
						distances(i) *= scale;
						break;
					}
					break;
				case 0x03:
				case 0x04:
				case 0x05:
					switch (value)
					{
					case 0x3FFF:
::std::cerr << "Measured value not valid" << ::std::endl;
					case 0x3FFE:
::std::cerr << "Dazzling" << ::std::endl;
					case 0x3FFD:
::std::cerr << "Operation overflow" << ::std::endl;
					case 0x3FFB:
::std::cerr << "Signal-to-noise ratio too small" << ::std::endl;
					case 0x3FFA:
::std::cerr << "Error when reading channel 1" << ::std::endl;
						distances(i) = ::std::numeric_limits<::rl::math::Real>::quiet_NaN();
						break;
					case 0x3FF7:
::std::cerr << "Measured value > Maximum value" << ::std::endl;
						distances(i) = ::std::numeric_limits<::rl::math::Real>::infinity();
						break;
					default:
						distances(i) = value;
						distances(i) *= scale;
						break;
					}
					break;
				case 0x06:
					switch (value)
					{
					case 0x7FFF:
::std::cerr << "Measured value not valid" << ::std::endl;
					case 0x7FFE:
::std::cerr << "Dazzling" << ::std::endl;
					case 0x7FFD:
::std::cerr << "Operation overflow" << ::std::endl;
					case 0x7FFB:
::std::cerr << "Signal-to-noise ratio too small" << ::std::endl;
					case 0x7FFA:
::std::cerr << "Error when reading channel 1" << ::std::endl;
						distances(i) = ::std::numeric_limits<::rl::math::Real>::quiet_NaN();
						break;
					case 0x7FF7:
::std::cerr << "Measured value > Maximum value" << ::std::endl;
						distances(i) = ::std::numeric_limits<::rl::math::Real>::infinity();
						break;
					default:
						distances(i) = value;
						distances(i) *= scale;
						break;
					}
				default:
					distances(i) = value;
					distances(i) *= scale;
					break;
				}
			}
			
			return distances;
		}
		
		::std::size_t
		SickLms200::getDistancesCount() const
		{
			assert(this->isConnected());
			
			switch (this->variant)
			{
			case Variant::v100_25:
				return 401;
				break;
			case Variant::v100_50:
				return 201;
				break;
			case Variant::v100_100:
				return 101;
				break;
			case Variant::v180_50:
				return 361;
				break;
			case Variant::v180_100:
				return 181;
				break;
			default:
				break;
			}
			
			return 0;
		}
		
		::rl::math::Real
		SickLms200::getDistancesMaximum(const ::std::size_t& i) const
		{
			assert(this->isConnected());
			assert(i < this->getDistancesCount());
			
			switch (this->measuring)
			{
			case Measuring::m8:
				return static_cast<::rl::math::Real>(8.182);
				break;
			case Measuring::m16:
				return static_cast<::rl::math::Real>(16.374);
				break;
			case Measuring::m32:
				return static_cast<::rl::math::Real>(32.758);
				break;
			case Measuring::m80:
				return static_cast<::rl::math::Real>(81.82);
				break;
			case Measuring::m160:
				return static_cast<::rl::math::Real>(163.74);
				break;
			case Measuring::m320:
				return static_cast<::rl::math::Real>(327.58);
				break;
			default:
				break;
			}
			
			return 0;
		}
		
		::rl::math::Real
		SickLms200::getDistancesMinimum(const ::std::size_t& i) const
		{
			assert(this->isConnected());
			assert(i < this->getDistancesCount());
			
			return 0;
		}
		
		SickLms200::Measuring
		SickLms200::getMeasuring() const
		{
			assert(this->isConnected());
			
			return this->measuring;
		}
		
		SickLms200::Monitoring
		SickLms200::getMonitoring() const
		{
			assert(this->isConnected());
			
			return this->monitoring;
		}
		
		::rl::math::Real
		SickLms200::getResolution() const
		{
			assert(this->isConnected());
			
			switch (this->variant)
			{
			case Variant::v100_25:
				return static_cast<::rl::math::Real>(0.25) * ::rl::math::constants::deg2rad;
				break;
			case Variant::v100_50:
			case Variant::v180_50:
				return static_cast<::rl::math::Real>(0.5) * ::rl::math::constants::deg2rad;
				break;
			case Variant::v100_100:
			case Variant::v180_100:
				return 1 * ::rl::math::constants::deg2rad;
				break;
			default:
				break;
			}
			
			return 0;
		}
		
		::rl::math::Real
		SickLms200::getStartAngle() const
		{
			assert(this->isConnected());
			
			switch (this->variant)
			{
			case Variant::v100_25:
			case Variant::v100_50:
			case Variant::v100_100:
				return 40 * ::rl::math::constants::deg2rad;
				break;
			case Variant::v180_50:
			case Variant::v180_100:
				return 0;
				break;
			default:
				break;
			}
			
			return 0;
		}
		
		::rl::math::Real
		SickLms200::getStopAngle() const
		{
			assert(this->isConnected());
			
			switch (this->variant)
			{
			case Variant::v100_25:
			case Variant::v100_50:
			case Variant::v100_100:
				return 140 * ::rl::math::constants::deg2rad;
				break;
			case Variant::v180_50:
			case Variant::v180_100:
				return 180 * ::rl::math::constants::deg2rad;
				break;
			default:
				break;
			}
			
			return 0;
		}
		
		::std::string
		SickLms200::getType()
		{
			assert(this->isConnected());
			
			::std::array<::std::uint8_t, 812> buf;
			
			buf[4] = 0x3A;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 21 + 1 + 2, 0xBA);
			
			return ::std::string(reinterpret_cast<char*>(buf.data() + 5), 20);
		}
		
		SickLms200::Variant
		SickLms200::getVariant() const
		{
			assert(this->isConnected());
			
			return this->variant;
		}
		
		void
		SickLms200::open()
		{
			this->serial.open();
			
			this->setConnected(true);
			
			::std::array<::std::uint8_t, 812> buf;
			
			// synchronize baud rates
			
			buf[4] = 0x20;
			buf[5] = 0x42;
			
#if defined(WIN32) || defined(__QNX__)
			Serial::BaudRate baudRates[3] = {
#else // defined(WIN32) || defined(__QNX__)
			Serial::BaudRate baudRates[4] = {
				Serial::BaudRate::b500000,
#endif // defined(WIN32) || defined(__QNX__)
				Serial::BaudRate::b38400,
				Serial::BaudRate::b19200,
				Serial::BaudRate::b9600
			};
			
#if defined(WIN32) || defined(__QNX__)
			for (::std::size_t i = 0; i < 3; ++i)
#else // defined(WIN32) || defined(__QNX__)
			for (::std::size_t i = 0; i < 4; ++i)
#endif // defined(WIN32) || defined(__QNX__)
			{
				this->serial.setBaudRate(baudRates[i]);
				this->serial.changeParameters();
				this->send(buf.data(), 1 + 1 + 2 + 1 + 1 + 2);
				
				if (this->waitAck())
				{
					break;
				}
				
#if defined(WIN32) || defined(__QNX__)
				if (2 == i)
#else // defined(WIN32) || defined(__QNX__)
				if (3 == i)
#endif // defined(WIN32) || defined(__QNX__)
				{
					throw DeviceException("could not sync baud rate.");
				}
			}
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 1 + 2, 0xA0);
			
			switch (buf[5])
			{
			case 0x01:
				throw DeviceException("mode switchover not possible due to incorrect password");
				break;
			case 0x02:
				throw DeviceException("mode switchover not possible due to a fault in the LMS2xx");
				break;
			default:
				break;
			}
			
			// status
			
			buf[4] = 0x31;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 152 + 1 + 2, 0xB1);
			
			// baud rate
			
			switch (buf[120])
			{
#if !(defined(WIN32) || defined(__QNX__))
			case 0x01:
				this->baudRate = BaudRate::b500000;
				break;
#endif // !(defined(WIN32) || defined(__QNX__))
			case 0x19:
				this->baudRate = BaudRate::b38400;
				break;
			case 0x33:
				this->baudRate = BaudRate::b19200;
				break;
			case 0x67:
				this->baudRate = BaudRate::b9600;
				break;
			default:
				break;
			}
			
			if (this->desired != this->baudRate)
			{
				this->setBaudRate(this->desired);
			}
			
			// monitoring
			
			if (0x25 != buf[12])
			{
				this->setMonitoring(Monitoring::single);
			}
			
			// measuring
			
			this->setMeasuring(this->measuring);
			
			// variant
			
			::std::uint16_t angle = Endian::hostWord(buf[112], buf[111]);
			::std::uint16_t resolution = Endian::hostWord(buf[114], buf[113]);
			
			Variant variant;
			
			switch (resolution)
			{
			case 25:
				variant = Variant::v100_25;
				break;
			case 50:
				switch (angle)
				{
				case 100:
					variant = Variant::v100_50;
					break;
				case 180:
					variant = Variant::v180_50;
					break;
				default:
					throw DeviceException("unknown variant");
					break;
				}
				break;
			case 100:
				switch (angle)
				{
				case 100:
					variant = Variant::v100_100;
					break;
				case 180:
					variant = Variant::v180_100;
					break;
				default:
					throw DeviceException("unknown variant");
					break;
				}
				break;
			default:
				throw DeviceException("unknown variant");
				break;
			}
			
			if (variant != this->variant)
			{
				this->setVariant(this->variant);
			}
		}
		
		::std::size_t
		SickLms200::recv(::std::uint8_t* buf, const ::std::size_t& len, const ::std::uint8_t& command)
		{
			assert(this->isConnected());
			assert(len > 7);
			
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
				
				if ((0x02 == buf[2]) && (0x00 == buf[3]) && (0x92 == buf[4]))
				{
					throw DeviceException("not acknowledge, incorrect command");
				}
			}
			while (command != buf[4]);
			
			::std::uint16_t length = Endian::hostWord(buf[3], buf[2]);
			
			if (len != static_cast<::std::size_t>(length) + 6)
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
			
			switch (buf[sumbytes - 3] & 7)
			{
			case 1:
				throw DeviceException("info");
				break;
			case 2:
				throw DeviceException("warning");
				break;
			case 3:
				throw DeviceException("error");
				break;
			case 4:
				throw DeviceException("fatal error");
				break;
			default:
				break;
			}
			
			if (buf[sumbytes - 3] & 64)
			{
				throw DeviceException("implausible measured values");
			}
			
			if (buf[sumbytes - 3] & 128)
			{
				throw DeviceException("pollution");
			}
			
			return sumbytes;
		}
		
		void
		SickLms200::reset()
		{
			assert(this->isConnected());
			
			::std::array<::std::uint8_t, 812> buf;
			
			buf[4] = 0x10;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 2, 0x91);
			
			// message during power-on
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 21 + 1 + 2, 0x90);
		}
		
		void
		SickLms200::send(::std::uint8_t* buf, const ::std::size_t& len)
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
		SickLms200::setBaudRate(const BaudRate& baudRate)
		{
			assert(this->isConnected());
			
			::std::array<::std::uint8_t, 812> buf;
			
			buf[4] = 0x20;
			
			switch (baudRate)
			{
			case BaudRate::b9600:
				buf[5] = 0x42;
				break;
			case BaudRate::b19200:
				buf[5] = 0x41;
				break;
			case BaudRate::b38400:
				buf[5] = 0x40;
				break;
#if !(defined(WIN32) || defined(__QNX__))
			case BaudRate::b500000:
				buf[5] = 0x48;
				break;
#endif // !(defined(WIN32) || defined(__QNX__))
			default:
				break;
			}
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 1 + 2, 0xA0);
			
			switch (buf[5])
			{
			case 0x01:
				throw DeviceException("mode switchover not possible due to incorrect password");
				break;
			case 0x02:
				throw DeviceException("mode switchover not possible due to a fault in the LMS2xx");
				break;
			default:
				break;
			}
			
			switch (baudRate)
			{
			case BaudRate::b9600:
				this->serial.setBaudRate(Serial::BaudRate::b9600);
				break;
			case BaudRate::b19200:
				this->serial.setBaudRate(Serial::BaudRate::b19200);
				break;
			case BaudRate::b38400:
				this->serial.setBaudRate(Serial::BaudRate::b38400);
				break;
#if !(defined(WIN32) || defined(__QNX__))
			case BaudRate::b500000:
				this->serial.setBaudRate(Serial::BaudRate::b500000);
				break;
#endif // !(defined(WIN32) || defined(__QNX__))
			default:
				break;
			}
			
			this->serial.changeParameters();
			
			this->baudRate = baudRate;
		}
		
		void
		SickLms200::setMeasuring(const Measuring& measuring)
		{
			assert(this->isConnected());
			
			::std::array<::std::uint8_t, 812> buf;
			
			// transmit password
			
			buf[4] = 0x20;
			buf[5] = 0x00;
			
			for (::std::size_t i = 0; i < 8; ++i)
			{
				buf[6 + i] = this->password[i];
			}
			
			do
			{
				this->send(buf.data(), 16);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 1 + 2, 0xA0);
			
			if (0x00 != buf[5])
			{
				throw DeviceException("could not switch mode");
			}
			
			// get current configuration
			
			buf[4] = 0x74;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 32 + 1 + 2, 0xF4);
			
			this->configuration = buf[10];
			
			// set new configuration
			
			buf[4] = 0x77;
			
			switch (measuring)
			{
			case Measuring::m8:
				buf[10] = 0x00;
				buf[11] = 0x01;
				break;
			case Measuring::m16:
				buf[10] = 0x04;
				buf[11] = 0x01;
				break;
			case Measuring::m32:
				buf[10] = 0x06;
				buf[11] = 0x01;
				break;
			case Measuring::m80:
				buf[10] = 0x00;
				buf[11] = 0x00;
				break;
			case Measuring::m160:
				buf[10] = 0x04;
				buf[11] = 0x00;
				break;
			case Measuring::m320:
				buf[10] = 0x06;
				buf[11] = 0x00;
				break;
			default:
				break;
			}
			
			::std::uint8_t configuration = buf[10];
			
			do
			{
				this->send(buf.data(), 40);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 33 + 1 + 2, 0xF7);
			
			if (0x00 == buf[5])
			{
				throw DeviceException("LMS configuration not accepted");
			}
			
			this->configuration = configuration;
			
			this->measuring = measuring;
		}
		
		void
		SickLms200::setMonitoring(const Monitoring& monitoring)
		{
			assert(this->isConnected());
			
			::std::array<::std::uint8_t, 812> buf;
			
			buf[4] = 0x20;
			
			switch (monitoring)
			{
			case Monitoring::continuous:
				buf[5] = 0x24;
				break;
			case Monitoring::single:
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
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 1 + 1 + 2, 0xA0);
			
			switch (buf[5])
			{
			case 0x01:
				throw DeviceException("mode switchover not possible due to incorrect password");
				break;
			case 0x02:
				throw DeviceException("mode switchover not possible due to a fault in the LMS2xx");
				break;
			default:
				break;
			}
			
			this->monitoring = monitoring;
		}
		
		void
		SickLms200::setVariant(const Variant& variant)
		{
			assert(this->isConnected());
			
			::std::array<::std::uint8_t, 812> buf;
			
			buf[4] = 0x3B;
			
			switch (variant)
			{
			case Variant::v100_25:
				buf[5] = 0x64;
				buf[6] = 0x0;
				buf[7] = 0x19;
				buf[8] = 0x0;
				break;
			case Variant::v100_50:
				buf[5] = 0x64;
				buf[6] = 0x0;
				buf[7] = 0x32;
				buf[8] = 0x0;
				break;
			case Variant::v100_100:
				buf[5] = 0x64;
				buf[6] = 0x0;
				buf[7] = 0x64;
				buf[8] = 0x0;
				break;
			case Variant::v180_50:
				buf[5] = 0xB4;
				buf[6] = 0x0;
				buf[7] = 0x32;
				buf[8] = 0x0;
				break;
			case Variant::v180_100:
				buf[5] = 0xB4;
				buf[6] = 0x0;
				buf[7] = 0x64;
				buf[8] = 0x0;
				break;
			default:
				break;
			}
			
			do
			{
				this->send(buf.data(), 11);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 5 + 1 + 2, 0xBB);
			
			if (0x00 == buf[5])
			{
				throw DeviceException("switchover aborted, previous variant still active");
			}
			
			this->variant = variant;
		}
		
		void
		SickLms200::start()
		{
			assert(this->isConnected());
			
			if (Monitoring::single != this->monitoring)
			{
				this->setMonitoring(this->monitoring);
			}
		}
		
		void
		SickLms200::step()
		{
			assert(this->isConnected());
			
			if (Monitoring::single == this->monitoring)
			{
				this->data[4] = 0x30;
				this->data[5] = 0x01;
				
				do
				{
					this->send(this->data.data(), 1 + 1 + 2 + 1 + 1 + 2);
				}
				while (!this->waitAck());
			}
			
			switch (this->variant)
			{
			case Variant::v100_25:
				this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 802 + 1 + 2, 0xB0);
				break;
			case Variant::v100_50:
				this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 402 + 1 + 2, 0xB0);
				break;
			case Variant::v100_100:
				this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 202 + 1 + 2, 0xB0);
				break;
			case Variant::v180_50:
				this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 722 + 1 + 2, 0xB0);
				break;
			case Variant::v180_100:
				this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 362 + 1 + 2, 0xB0);
				break;
			default:
				break;
			}
		}
		
		void
		SickLms200::stop()
		{
			assert(this->isConnected());
			
			if (Monitoring::single != this->monitoring)
			{
				this->setMonitoring(Monitoring::single);
			}
		}
		
		bool
		SickLms200::waitAck()
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
