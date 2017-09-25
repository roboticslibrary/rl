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
#include "SickS300.h"

static const ::std::uint16_t crcTable[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

namespace rl
{
	namespace hal
	{
		SickS300::SickS300(
			const ::std::string& filename,
			const Serial::BaudRate& baudRate
		) :
			CyclicDevice(::std::chrono::nanoseconds::zero()),
			Lidar(),
			data(),
			serial(
				filename,
				baudRate,
				Serial::DATABITS_8BITS,
				Serial::FLOWCONTROL_OFF,
				Serial::PARITY_NOPARITY,
				Serial::STOPBITS_1BIT
			)
		{
		}
		
		SickS300::~SickS300()
		{
		}
		
		void
		SickS300::close()
		{
			assert(this->isConnected());
			
			this->serial.close();
			
			this->setConnected(false);
		}
		
		::std::uint16_t
		SickS300::crc(const ::std::uint8_t* buf, const ::std::size_t& len) const
		{
			::std::uint16_t checksum = 0xFFFF;
			
			for (::std::size_t i = 0; i < len; ++i)
			{
				checksum = (checksum << 8) ^ (crcTable[(checksum >> 8) ^ (buf[i])]);
			}
			
			return checksum;
		}
		
		::rl::math::Vector
		SickS300::getDistances() const
		{
			assert(this->isConnected());
			
			::rl::math::Vector distances(this->getDistancesCount());
			
			::rl::math::Real scale = 0.01f;
			
			for (::std::size_t i = 0; i < this->getDistancesCount(); ++i)
			{
				::std::uint16_t value = Endian::hostWord(this->data[24 + 1 + i * 2], this->data[24 + 0 + i * 2]) & 0x1FFF;
				
				switch (this->data[24 + 1 + i * 2] & 224)
				{
				case 128:
::std::cerr << "Measured value detected within warning field" << ::std::endl;
				case 64:
::std::cerr << "Measured value detected within protective field" << ::std::endl;
				case 32:
::std::cerr << "Glare (dazzling) detected" << ::std::endl;
					distances(i) = ::std::numeric_limits< ::rl::math::Real>::quiet_NaN();
					break;
				default:
					distances(i) = value * scale;
					break;
				}
			}
			
			return distances;
		}
		
		::std::size_t
		SickS300::getDistancesCount() const
		{
			assert(this->isConnected());
			
			return 541;
		}
		
		::rl::math::Real
		SickS300::getDistancesMaximum(const ::std::size_t& i) const
		{
			assert(this->isConnected());
			assert(i < this->getDistancesCount());
			
			return 10.0f;
		}
		
		::rl::math::Real
		SickS300::getDistancesMinimum(const ::std::size_t& i) const
		{
			assert(this->isConnected());
			assert(i < this->getDistancesCount());
			
			return 0.0f;
		}
		
		::rl::math::Real
		SickS300::getResolution() const
		{
			assert(this->isConnected());
			
			return std::abs(this->getStopAngle() - this->getStartAngle()) / (this->getDistancesCount() - 1.0f);
		}
		
		::std::size_t
		SickS300::getScanNumber() const
		{
			return Endian::hostDoubleWord(
				Endian::hostWord(this->data[17], this->data[16]),
				Endian::hostWord(this->data[15], this->data[14])
			);
		}
		
		::rl::math::Real
		SickS300::getStartAngle() const
		{
			assert(this->isConnected());
			
			return -45.0f * ::rl::math::DEG2RAD;
		}
		
		::rl::math::Real
		SickS300::getStopAngle() const
		{
			assert(this->isConnected());
			
			return 225.0f * ::rl::math::DEG2RAD;
		}
		
		::std::size_t
		SickS300::getTelegramNumber() const
		{
			return Endian::hostWord(this->data[19], this->data[18]);
		}
		
		void
		SickS300::open()
		{
			this->serial.open();
			
			this->setConnected(true);
		}
		
		::std::size_t
		SickS300::recv(::std::uint8_t* buf)
		{
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
						numbytes = serial.read(ptr, 1);
printf("buf[0] %02x\n", buf[0]);
					}
					while (0x00 != buf[0]); // 0x00
					
					ptr += numbytes;
					sumbytes += numbytes;
					
					numbytes = serial.read(ptr, 1);
printf("buf[1] %02x\n", buf[1]);
				}
				while (0x00 != buf[1]); // 0x00 0x00
				
				ptr += numbytes;
				sumbytes += numbytes;
				
				numbytes = serial.read(ptr, 1);
printf("buf[2] %02x\n", buf[2]);
			}
			while (0x00 != buf[2]); // 0x00 0x00 0x00
			
			ptr += numbytes;
			sumbytes += numbytes;
			
			numbytes = serial.read(ptr, 1);
printf("buf[3] %02x\n", buf[3]);
			ptr += numbytes;
			sumbytes += numbytes;
			
			if (0x00 != buf[3])
			{
				switch (buf[3])
				{
					case 0x01:
						throw DeviceException("The current device status does not permit access to the data block");
						break;
					case 0x02:
						throw DeviceException("Access to the data block by the current user group is not permitted");
						break;
					case 0x03:
						throw DeviceException("Incorrect password");
						break;
					case 0x04:
						throw DeviceException("System token is occupied");
						break;
					case 0x05:
						throw DeviceException("Incorrect parameter");
						break;
					case 0x0A:
						throw DeviceException("One of the communication monitoring processes failed (e.g. timeout of EFI-RK512 packages/EFI-RK512 acknowledgement or failed transmission of EFI-RK512 packages/EFI-RK512 acknowledgement)");
						break;
					case 0x0C:
						throw DeviceException("The data word number of the destination address or source address in command telegram (byte 6) is impermissible (not defined in interface register) | The co-ordination flag (byte number) in command telegram (byte 9) does not equal 0xFF | The device code in the command telegram (byte 10, bits 0 to 3) is invalid (i.e. equals 0) | The CPU number in the command telegram (byte 10, bits 5 to 7) is impermissible");
						break;
					case 0x10:
						throw DeviceException("The telegram identifier in the command telegram (byte 1) is not equal to 0x00 or 0xFF or is not followed by a further 0x00 byte (byte 2) | The command data type in the command telegram (byte 4) is impermissible");
						break;
					case 0x14:
						throw DeviceException("The data block number of the destination address or source address in the command telegram (byte 5) is impermissible (not defined in the interface register)");
						break;
					case 0x16:
						throw DeviceException("The command telegram type in the command telegram (byte 3) is impermissible");
						break;
					case 0x34:
						throw DeviceException("Telegram format error");
						break;
					case 0x36:
						throw DeviceException("A command telegram has been received though no reply telegram has been received yet");
						break;
					default:
						throw DeviceException("Unknown error");
						break;
				}
			}
			
			for (::std::size_t i = 0; i < 6; ++i)
			{
				numbytes = serial.read(ptr, 1);
printf("buf[%zi] %02x\n", i + 4, buf[i + 4]);
				ptr += numbytes;
				sumbytes += numbytes;
			}
			
			::std::uint16_t length = Endian::hostWord(buf[6], buf[7]);
std::cout << "length " << length << std::endl;
			
			if (length != 552)
			{
				throw DeviceException("Data length mismatch");
			}
printf("coordinationFlag %02x\n", buf[8]);
printf("deviceAddress %02x\n", buf[9]);
			
			for (::std::size_t i = 0; i < 1094; ++i)
			{
				numbytes = serial.read(ptr, 1);
				ptr += numbytes;
				sumbytes += numbytes;
			}
printf("version %02x%02x\n", buf[11], buf[10]);
			
			if (0x01 != buf[11] || 0x02 != buf[10])
			{
				throw DeviceException("Protocol version mismatch");
			}
			
			if (0x00 != buf[13] || 0x00 != buf[12])
			{
				throw DeviceException("Incorrect status");
			}
			
			::std::uint32_t scanNumber = Endian::hostDoubleWord(
				Endian::hostWord(buf[17], buf[16]),
				Endian::hostWord(buf[15], buf[14])
			);
std::cout << "scanNumber " << scanNumber << std::endl;
			
			::std::uint16_t telegramNumber = Endian::hostWord(buf[19], buf[18]);
std::cout << "telegramNumber " << telegramNumber << std::endl;
			
			for (::std::size_t i = 0; i < 2; ++i)
			{
				numbytes = serial.read(ptr, 1);
				ptr += numbytes;
				sumbytes += numbytes;
			}
			
			for (::std::size_t i = 0; i < 2; ++i)
			{
				numbytes = serial.read(ptr, 1);
				ptr += numbytes;
				sumbytes += numbytes;
			}
			
			if (this->crc(buf + 4, sumbytes - 2 - 4) != Endian::hostWord(buf[sumbytes - 1], buf[sumbytes - 2]))
			{
				throw DeviceException("Checksum error");
			}
for (::std::size_t i = 0; i < sumbytes; ++i) { printf("%02x ", buf[i]); } printf("\n");

			return sumbytes;
		}
		
		void
		SickS300::start()
		{
			assert(this->isConnected());
		}
		
		void
		SickS300::step()
		{
			assert(this->isConnected());
			
			this->recv(this->data.data());
		}
		
		void
		SickS300::stop()
		{
			assert(this->isConnected());
		}
	}
}
