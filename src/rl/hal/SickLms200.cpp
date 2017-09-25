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
#include <cstdio>
#include <iostream>
#include <rl/math/Unit.h>

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
			baudRate(BAUDRATE_9600BPS),
			configuration(0x00),
			data(),
			desired(baudRate),
			measuring(measuring),
			monitoring(monitoring),
			password(password),
			serial(
				filename,
				Serial::BAUDRATE_9600BPS,
				Serial::DATABITS_8BITS,
				Serial::FLOWCONTROL_OFF,
				Serial::PARITY_NOPARITY,
				Serial::STOPBITS_1BIT
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
			
			if (MEASURING_8M != this->measuring)
			{
				this->setMeasuring(MEASURING_8M);
			}
			
			if (VARIANT_180_50 != this->variant)
			{
				this->setVariant(VARIANT_180_50);
			}
			
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
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 812> buf;
			
			buf[4] = 0x74;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 32 + 1 + 2, 0xF4);
			
			printf("A:  %i\n", Endian::hostWord(buf[6], buf[5]));
			printf("B:  %02x %02x\n", buf[8], buf[7]);
			printf("C:  %02x\n", buf[9]);
			
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
			
			printf("D:  %02x\n", buf[10]);
			printf("E:  %02x\n", buf[11]);
			printf("F:  %02x\n", buf[12]);
			printf("G:  %02x\n", buf[13]);
			printf("H:  %02x\n", buf[14]);
			printf("I:  %02x\n", buf[15]);
			printf("J:  %02x\n", buf[16]);
			printf("K:  %02x\n", buf[17]);
			printf("L:  %02x\n", buf[18]);
			printf("M:  %02x\n", buf[19]);
			printf("N:  %02x\n", buf[20]);
			printf("O:  %02x\n", buf[21]);
			printf("P:  %02x\n", buf[22]);
			printf("Q:  %02x\n", buf[23]);
			printf("R:  %02x\n", buf[24]);
			printf("S:  %02x\n", buf[25]);
			printf("T:  %02x\n", buf[26]);
			printf("U:  %02x\n", buf[27]);
			printf("V:  %02x\n", buf[28]);
			printf("W:  %02x\n", buf[29]);
			printf("X:  %02x\n", buf[30]);
			printf("Y:  %02x\n", buf[31]);
			printf("Z:  %02x\n", buf[32]);
			printf("A1: %02x\n", buf[33]);
			printf("A2: %02x\n", buf[34]);
			printf("A3: %02x %02x\n", buf[36], buf[35]);
			printf("A4: %02x %02x\n", buf[38], buf[37]);
		}
		
		void
		SickLms200::dumpStatus()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 812> buf;
			
			buf[4] = 0x31;
			
			do
			{
				this->send(buf.data(), 1 + 1 + 2 + 1 + 2);
			}
			while (!this->waitAck());
			
			this->recv(buf.data(), 1 + 1 + 2 + 1 + 152 + 1 + 2, 0xB1);
			
			printf("A:  %c %c %c %c %c %c %c\n", buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]);
			printf("B:  %02x\n", buf[12]);
			printf("C:  %02x\n", buf[13]);
			printf("D:  reserved\n");
			printf("?:  %c %c %c %c %c %c\n", buf[16], buf[17], buf[18], buf[19], buf[20], buf[21]);
			printf("E:  %02x\n", buf[22]);
			printf("F:  %i %i %i %i %i %i %i %i\n",
				Endian::hostWord(buf[24], buf[23]),
				Endian::hostWord(buf[26], buf[25]),
				Endian::hostWord(buf[28], buf[27]),
				Endian::hostWord(buf[30], buf[29]),
				Endian::hostWord(buf[32], buf[31]),
				Endian::hostWord(buf[34], buf[33]),
				Endian::hostWord(buf[36], buf[35]),
				Endian::hostWord(buf[38], buf[37])
			);
			printf("G:  %i %i %i %i\n",
				Endian::hostWord(buf[40], buf[39]),
				Endian::hostWord(buf[42], buf[41]),
				Endian::hostWord(buf[44], buf[43]),
				Endian::hostWord(buf[46], buf[45])
			);
			printf("H:  %i %i %i %i %i %i %i %i\n",
				Endian::hostWord(buf[48], buf[47]),
				Endian::hostWord(buf[50], buf[49]),
				Endian::hostWord(buf[52], buf[51]),
				Endian::hostWord(buf[54], buf[53]),
				Endian::hostWord(buf[56], buf[55]),
				Endian::hostWord(buf[58], buf[57]),
				Endian::hostWord(buf[60], buf[59]),
				Endian::hostWord(buf[62], buf[61])
			);
			printf("I:  %i %i %i %i\n",
				Endian::hostWord(buf[64], buf[63]),
				Endian::hostWord(buf[66], buf[65]),
				Endian::hostWord(buf[68], buf[67]),
				Endian::hostWord(buf[70], buf[69])
			);
			printf("J:  %i\n", Endian::hostWord(buf[72], buf[71]));
			printf("K:  reserved\n");
			printf("L:  %i\n", Endian::hostWord(buf[76], buf[75]));
			printf("M:  reserved\n");
			printf("N:  %i\n", Endian::hostWord(buf[80], buf[79]));
			printf("O:  %i\n", Endian::hostWord(buf[82], buf[81]));
			printf("P:  reserved\n");
			printf("Q:  %i\n", Endian::hostWord(buf[86], buf[85]));
			printf("R:  %i\n", Endian::hostWord(buf[88], buf[87]));
			printf("S:  %i\n", Endian::hostWord(buf[90], buf[89]));
			printf("T:  %i\n", Endian::hostWord(buf[92], buf[91]));
			printf("U:  %i\n", Endian::hostWord(buf[94], buf[93]));
			printf("V:  %i\n", Endian::hostWord(buf[96], buf[95]));
			printf("W:  %i\n", Endian::hostWord(buf[98], buf[97]));
			printf("X:  %i\n", Endian::hostWord(buf[100], buf[99]));
			printf("Y:  %i\n", Endian::hostWord(buf[102], buf[101]));
			printf("Z:  %i\n", Endian::hostWord(buf[104], buf[103]));
			printf("A1: reserved\n");
			printf("A2: %02x\n", buf[106]);
			printf("A3: H:%02x L:%02x\n", buf[108], buf[107]);
			printf("A4: H:%02x L:%02x\n", buf[110], buf[109]);
			printf("A5: %i\n", Endian::hostWord(buf[112], buf[111]));
			printf("A6: %i\n", Endian::hostWord(buf[114], buf[113]));
			printf("A7: %02x\n", buf[115]);
			printf("A8: %i\n", Endian::hostWord(buf[117], buf[116]));
			printf("A9: %c\n", buf[118]);
			printf("B1: reserved\n");
			printf("B2: %04x\n", Endian::hostWord(buf[121], buf[120]));
			printf("B3: %02x\n", buf[122]);
			printf("B4: %02x\n", buf[123]);
			printf("B5: %02x\n", buf[124]);
			printf("B6: %02x\n", buf[125]);
			printf("B7: %02x\n", buf[126]);
			printf("B8: %02x\n", buf[127]);
			printf("B9: %c %c %c %c %c %c %c\n", buf[128], buf[129], buf[130], buf[131], buf[132], buf[133], buf[134]);
			printf("C1: %i\n", Endian::hostDoubleWord(Endian::hostWord(buf[138], buf[137]), Endian::hostWord(buf[136], buf[135])));
			printf("C2: %i\n", Endian::hostDoubleWord(Endian::hostWord(buf[142], buf[141]), Endian::hostWord(buf[140], buf[139])));
			printf("C3: %i\n", Endian::hostDoubleWord(Endian::hostWord(buf[146], buf[145]), Endian::hostWord(buf[144], buf[143])));
			printf("C4: %i\n", Endian::hostDoubleWord(Endian::hostWord(buf[150], buf[149]), Endian::hostWord(buf[148], buf[147])));
			printf("C5: %i\n", Endian::hostWord(buf[152], buf[151]));
			printf("C6: %i\n", Endian::hostWord(buf[154], buf[153]));
			printf("C7: %i\n", Endian::hostWord(buf[156], buf[155]));
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
				scale = 0.01f;
				break;
			case 64:
				scale = 0.001f;
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
						distances(i) = ::std::numeric_limits< ::rl::math::Real>::quiet_NaN();
						break;
					case 0x1FF7:
::std::cerr << "Measured value > Maximum value" << ::std::endl;
						distances(i) = ::std::numeric_limits< ::rl::math::Real>::infinity();
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
						distances(i) = ::std::numeric_limits< ::rl::math::Real>::quiet_NaN();
						break;
					case 0x3FF7:
::std::cerr << "Measured value > Maximum value" << ::std::endl;
						distances(i) = ::std::numeric_limits< ::rl::math::Real>::infinity();
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
						distances(i) = ::std::numeric_limits< ::rl::math::Real>::quiet_NaN();
						break;
					case 0x7FF7:
::std::cerr << "Measured value > Maximum value" << ::std::endl;
						distances(i) = ::std::numeric_limits< ::rl::math::Real>::infinity();
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
			case VARIANT_100_25:
				return 401;
				break;
			case VARIANT_100_50:
				return 201;
				break;
			case VARIANT_100_100:
				return 101;
				break;
			case VARIANT_180_50:
				return 361;
				break;
			case VARIANT_180_100:
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
			case MEASURING_8M:
				return 8.182f;
				break;
			case MEASURING_16M:
				return 16.374f;
				break;
			case MEASURING_32M:
				return 32.758f;
				break;
			case MEASURING_80M:
				return 81.82f;
				break;
			case MEASURING_160M:
				return 163.74f;
				break;
			case MEASURING_320M:
				return 327.58f;
				break;
			default:
				break;
			}
			
			return 0.0f;
		}
		
		::rl::math::Real
		SickLms200::getDistancesMinimum(const ::std::size_t& i) const
		{
			assert(this->isConnected());
			assert(i < this->getDistancesCount());
			
			return 0.0f;
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
			case VARIANT_100_25:
				return 0.25f * ::rl::math::DEG2RAD;
				break;
			case VARIANT_100_50:
			case VARIANT_180_50:
				return 0.5f * ::rl::math::DEG2RAD;
				break;
			case VARIANT_100_100:
			case VARIANT_180_100:
				return 1.0f * ::rl::math::DEG2RAD;
				break;
			default:
				break;
			}
			
			return 0.0f;
		}
		
		::rl::math::Real
		SickLms200::getStartAngle() const
		{
			assert(this->isConnected());
			
			switch (this->variant)
			{
			case VARIANT_100_25:
			case VARIANT_100_50:
			case VARIANT_100_100:
				return 40.0f * ::rl::math::DEG2RAD;
				break;
			case VARIANT_180_50:
			case VARIANT_180_100:
				return 0.0f;
				break;
			default:
				break;
			}
			
			return 0.0f;
		}
		
		::rl::math::Real
		SickLms200::getStopAngle() const
		{
			assert(this->isConnected());
			
			switch (this->variant)
			{
			case VARIANT_100_25:
			case VARIANT_100_50:
			case VARIANT_100_100:
				return 140.0f * ::rl::math::DEG2RAD;
				break;
			case VARIANT_180_50:
			case VARIANT_180_100:
				return 180.0f * ::rl::math::DEG2RAD;
				break;
			default:
				break;
			}
			
			return 0.0f;
		}
		
		::std::string
		SickLms200::getType()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 812> buf;
			
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
			
			::std::array< ::std::uint8_t, 812> buf;
			
			// synchronize baud rates
			
			buf[4] = 0x20;
			buf[5] = 0x42;
			
#if defined(WIN32) || defined(__QNX__)
			Serial::BaudRate baudRates[3] = {
#else // defined(WIN32) || defined(__QNX__)
			Serial::BaudRate baudRates[4] = {
				Serial::BAUDRATE_500000BPS,
#endif // defined(WIN32) || defined(__QNX__)
				Serial::BAUDRATE_38400BPS,
				Serial::BAUDRATE_19200BPS,
				Serial::BAUDRATE_9600BPS
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
				if (3 == i)
#else // defined(WIN32) || defined(__QNX__)
				if (4 == i)
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
				this->baudRate = BAUDRATE_500000BPS;
				break;
#endif // !(defined(WIN32) || defined(__QNX__))
			case 0x19:
				this->baudRate = BAUDRATE_38400BPS;
				break;
			case 0x33:
				this->baudRate = BAUDRATE_19200BPS;
				break;
			case 0x67:
				this->baudRate = BAUDRATE_9600BPS;
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
				this->setMonitoring(MONITORING_SINGLE);
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
				variant = VARIANT_100_25;
				break;
			case 50:
				switch (angle)
				{
				case 100:
					variant = VARIANT_100_50;
					break;
				case 180:
					variant = VARIANT_180_50;
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
					variant = VARIANT_100_100;
					break;
				case 180:
					variant = VARIANT_180_100;
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
			
			::std::array< ::std::uint8_t, 812> buf;
			
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
			
			::std::array< ::std::uint8_t, 812> buf;
			
			buf[4] = 0x20;
			
			switch (baudRate)
			{
			case BAUDRATE_9600BPS:
				buf[5] = 0x42;
				break;
			case BAUDRATE_19200BPS:
				buf[5] = 0x41;
				break;
			case BAUDRATE_38400BPS:
				buf[5] = 0x40;
				break;
#if !(defined(WIN32) || defined(__QNX__))
			case BAUDRATE_500000BPS:
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
			case BAUDRATE_9600BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_9600BPS);
				break;
			case BAUDRATE_19200BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_19200BPS);
				break;
			case BAUDRATE_38400BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_38400BPS);
				break;
#if !(defined(WIN32) || defined(__QNX__))
			case BAUDRATE_500000BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_500000BPS);
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
			
			::std::array< ::std::uint8_t, 812> buf;
			
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
			case MEASURING_8M:
				buf[10] = 0x00;
				buf[11] = 0x01;
				break;
			case MEASURING_16M:
				buf[10] = 0x04;
				buf[11] = 0x01;
				break;
			case MEASURING_32M:
				buf[10] = 0x06;
				buf[11] = 0x01;
				break;
			case MEASURING_80M:
				buf[10] = 0x00;
				buf[11] = 0x00;
				break;
			case MEASURING_160M:
				buf[10] = 0x04;
				buf[11] = 0x00;
				break;
			case MEASURING_320M:
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
			
			::std::array< ::std::uint8_t, 812> buf;
			
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
			
			::std::array< ::std::uint8_t, 812> buf;
			
			buf[4] = 0x3B;
			
			switch (variant)
			{
			case VARIANT_100_25:
				buf[5] = 0x64;
				buf[6] = 0x0;
				buf[7] = 0x19;
				buf[8] = 0x0;
				break;
			case VARIANT_100_50:
				buf[5] = 0x64;
				buf[6] = 0x0;
				buf[7] = 0x32;
				buf[8] = 0x0;
				break;
			case VARIANT_100_100:
				buf[5] = 0x64;
				buf[6] = 0x0;
				buf[7] = 0x64;
				buf[8] = 0x0;
				break;
			case VARIANT_180_50:
				buf[5] = 0xB4;
				buf[6] = 0x0;
				buf[7] = 0x32;
				buf[8] = 0x0;
				break;
			case VARIANT_180_100:
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
			
			if (MONITORING_SINGLE != this->monitoring)
			{
				this->setMonitoring(this->monitoring);
			}
		}
		
		void
		SickLms200::step()
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
			
			switch (this->variant)
			{
			case VARIANT_100_25:
				this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 802 + 1 + 2, 0xB0);
				break;
			case VARIANT_100_50:
				this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 402 + 1 + 2, 0xB0);
				break;
			case VARIANT_100_100:
				this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 202 + 1 + 2, 0xB0);
				break;
			case VARIANT_180_50:
				this->recv(this->data.data(), 1 + 1 + 2 + 1 + 1 + 1 + 722 + 1 + 2, 0xB0);
				break;
			case VARIANT_180_100:
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
			
			if (MONITORING_SINGLE != this->monitoring)
			{
				this->setMonitoring(MONITORING_SINGLE);
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
