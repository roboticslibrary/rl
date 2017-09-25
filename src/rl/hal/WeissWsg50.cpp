//
// Copyright (c) 2013, Andre Gaschler, Markus Rickert
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
#include <cstring>
#include <iostream>
#include <rl/math/Unit.h>

//#define DEBUG_TCP_DATA

#ifdef DEBUG_TCP_DATA
#include <cstdio>
#endif

#include "Endian.h"
#include "WeissException.h"
#include "WeissWsg50.h"

static const ::std::uint16_t CRC_TABLE_CCITT16[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

namespace rl
{
	namespace hal
	{
		WeissWsg50::WeissWsg50(
			const ::std::string& address,
			const unsigned short int& port,
			const float& acceleration,
			const float& forceLimit,
			const unsigned int& period
		) :
			CyclicDevice(::std::chrono::milliseconds(period)),
			Gripper(),
			acceleration(acceleration),
			accelerationMaximum(0),
			accelerationMinimum(0),
			force(0),
			forceLimit(forceLimit),
			forceMinimum(0),
			forceNominal(0),
			forceOverdrive(0),
			graspingState(GRASPING_STATE_IDLE),
			limitMinus(0),
			limitPlus(0),
			openingWidth(0),
			period(period),
			socket(Socket::Tcp(Socket::Address::Ipv4(address, port))),
			speed(0),
			speedMaximum(0),
			speedMinimum(0),
			stroke(0),
			systemState(SYSTEM_STATE_REFERENCED)
		{
			
		}
		
		WeissWsg50::~WeissWsg50()
		{
			if (this->isRunning())
			{
				this->stop(); // TODO
			}
		}
		
		void
		WeissWsg50::close()
		{
			assert(this->isConnected());
			this->doDisconnectAnnouncement();
			this->socket.close();
			this->setConnected(false);
		}
		
		::std::uint16_t
		WeissWsg50::crc(const ::std::uint8_t* buf, const ::std::size_t& len) const
		{
			::std::uint16_t checksum = 0xFFFF;
			
			for (::std::size_t i = 0; i < len; ++i)
			{
				::std::uint8_t index = checksum ^ buf[i];
				checksum = CRC_TABLE_CCITT16[index] ^ (checksum >> 8);
			}
			
			return checksum;
		}
		
		void
		WeissWsg50::doAcknowledgeFaults()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x24;
			buf[this->HEADER_SIZE] = 0x61;
			buf[this->HEADER_SIZE + 1] = 0x63;
			buf[this->HEADER_SIZE + 2] = 0x6B;
			
			this->send(buf.data(), this->HEADER_SIZE + 3 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x24);
		}
		
		void
		WeissWsg50::doClearSoftLimits()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x36;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x36);
			
			this->limitMinus = 0;
			this->limitPlus = 0;
		}
		
		void
		WeissWsg50::doDisconnectAnnouncement()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x07;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x07);
		}
		
		void
		WeissWsg50::doFastStop()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x23;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x23);
		}
		
		void
		WeissWsg50::doGetAcceleration()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x31;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 4 + 2, 0x31);
			
			::std::memcpy(&this->acceleration, &buf[this->HEADER_SIZE + 2], sizeof(this->acceleration));
			
			this->acceleration *= static_cast<float>(::rl::math::MILLI2UNIT);
		}
		
		void
		WeissWsg50::doGetForce(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x45;
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doAutomaticUpdate << 1);
			buf[this->HEADER_SIZE + 1] = Endian::hostLowByte(period);
			buf[this->HEADER_SIZE + 2] = Endian::hostHighByte(period);
			
			this->send(buf.data(), this->HEADER_SIZE + 3 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 4 + 2, 0x45);
		}
		
		void
		WeissWsg50::doGetForceLimit()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x33;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 4 + 2, 0x33);
			
			::std::memcpy(&this->forceLimit, &buf[this->HEADER_SIZE + 2], sizeof(this->forceLimit));
		}
		
		void
		WeissWsg50::doGetGraspingState(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x41;
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doUpdateOnChange << 1);
			buf[this->HEADER_SIZE + 1] = Endian::hostLowByte(period);
			buf[this->HEADER_SIZE + 2] = Endian::hostHighByte(period);
			
			this->send(buf.data(), this->HEADER_SIZE + 3 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 1 + 2, 0x41);
		}
		
		void
		WeissWsg50::doGetGraspingStatistics(const bool& doReset, int& numberGraspsTotal, int& numberGraspsNoPart, int& numberGraspsLostPart)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x42;
			buf[this->HEADER_SIZE] = 0x00 | doReset;
			
			this->send(buf.data(), this->HEADER_SIZE + 1 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 8 + 2, 0x42);
			
			numberGraspsTotal = Endian::hostDoubleWord(Endian::hostWord(buf[11], buf[10]), Endian::hostWord(buf[9], buf[8]));
			numberGraspsNoPart = Endian::hostWord(buf[13], buf[12]);
			numberGraspsLostPart = Endian::hostWord(buf[15], buf[14]);
		}
		
		void
		WeissWsg50::doGetOpeningWidth(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x43;
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doUpdateOnChange << 1);
			buf[this->HEADER_SIZE + 1] = Endian::hostLowByte(period);
			buf[this->HEADER_SIZE + 2] = Endian::hostHighByte(period);
			
			this->send(buf.data(), this->HEADER_SIZE + 3 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 4 + 2, 0x43);
		}
		
		void
		WeissWsg50::doGetSoftLimits()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x35;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 8 + 2, 0x35);
			
			::std::memcpy(&this->limitMinus, &buf[this->HEADER_SIZE + 2], sizeof(this->limitMinus));
			::std::memcpy(&this->limitPlus, &buf[this->HEADER_SIZE + 2 + 4], sizeof(this->limitPlus));
			
			this->limitMinus *= static_cast<float>(::rl::math::MILLI2UNIT);
			this->limitPlus *= static_cast<float>(::rl::math::MILLI2UNIT);
		}
		
		void
		WeissWsg50::doGetSpeed(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x44;
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doUpdateOnChange << 1);
			buf[this->HEADER_SIZE + 1] = Endian::hostLowByte(period);
			buf[this->HEADER_SIZE + 2] = Endian::hostHighByte(period);
			
			this->send(buf.data(), this->HEADER_SIZE + 3 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 4 + 2, 0x44);
		}
		
		void
		WeissWsg50::doGetSystemInformation(bool& isWsg50, int& hardwareRevision, int& firmwareRevision, int& serialNumber)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x50;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 8 + 2, 0x50);
			
			isWsg50 = (1 == buf[8]) ? true : false;
			hardwareRevision = buf[9];
			firmwareRevision = Endian::hostWord(buf[11], buf[10]);
			serialNumber = Endian::hostDoubleWord(Endian::hostWord(buf[15], buf[14]), Endian::hostWord(buf[13], buf[12]));
		}
		
		void
		WeissWsg50::doGetSystemLimits()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x53;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 32 + 2, 0x53);
			
			::std::memcpy(&this->stroke, &buf[8], sizeof(this->stroke));
			::std::memcpy(&this->speedMinimum, &buf[12], sizeof(this->speedMinimum));
			::std::memcpy(&this->speedMaximum, &buf[16], sizeof(this->speedMaximum));
			::std::memcpy(&this->accelerationMinimum, &buf[20], sizeof(this->accelerationMinimum));
			::std::memcpy(&this->accelerationMaximum, &buf[24], sizeof(this->accelerationMaximum));
			::std::memcpy(&this->forceMinimum, &buf[28], sizeof(this->forceMinimum));
			::std::memcpy(&this->forceNominal, &buf[32], sizeof(this->forceNominal));
			::std::memcpy(&this->forceOverdrive, &buf[36], sizeof(this->forceOverdrive));
			
			this->stroke *= static_cast<float>(::rl::math::MILLI2UNIT);
			this->speedMinimum *= static_cast<float>(::rl::math::MILLI2UNIT);
			this->speedMaximum *= static_cast<float>(::rl::math::MILLI2UNIT);
			this->accelerationMinimum *= static_cast<float>(::rl::math::MILLI2UNIT);
			this->accelerationMaximum *= static_cast<float>(::rl::math::MILLI2UNIT);
		}
		
		void
		WeissWsg50::doGetSystemState(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x40;
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doUpdateOnChange << 1);
			buf[this->HEADER_SIZE + 1] = Endian::hostLowByte(period);
			buf[this->HEADER_SIZE + 2] = Endian::hostHighByte(period);
			
			this->send(buf.data(), this->HEADER_SIZE + 3 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 4 + 2, 0x40);
			
			this->systemState = static_cast<SystemState>(
				Endian::hostDoubleWord(
					Endian::hostWord(buf[this->HEADER_SIZE + 5], buf[this->HEADER_SIZE + 4]),
					Endian::hostWord(buf[this->HEADER_SIZE + 3], buf[this->HEADER_SIZE + 2])
				)
			);
		}
		
		float
		WeissWsg50::doGetTemperature()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x46;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 2 + 2, 0x46);
			
			::std::int16_t temperature = Endian::hostWord(buf[9], buf[8]);
			
			return temperature / 10.0f;
		}
		
		void
		WeissWsg50::doGraspPart(const float& width, const float& speed)
		{
			assert(this->isConnected());
			assert(width >= 0.0f && width <= 0.11f);
			assert(speed >= 0.0f && speed <= 0.4f);
			
			float widthMilli = width * static_cast<float>(::rl::math::UNIT2MILLI);
			float speedMilli = speed * static_cast<float>(::rl::math::UNIT2MILLI);
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x25;
			::std::memcpy(&buf[this->HEADER_SIZE], &widthMilli, sizeof(widthMilli));
			::std::memcpy(&buf[this->HEADER_SIZE + 4], &speedMilli, sizeof(speedMilli));
			
			this->send(buf.data(), this->HEADER_SIZE + 8 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x25);
		}
		
		// Perform necessary homing motion for calibration.
		// This function call is blocking until the calibration is complete.
		void
		WeissWsg50::doHomingMotion(const unsigned int& direction)
		{
			assert(this->isConnected());
			assert(direction < 3);
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x20;
			buf[this->HEADER_SIZE] = direction;
			
			this->send(buf.data(), this->HEADER_SIZE + 1 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x20);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x20); // wait for completion message
		}
		
		void
		WeissWsg50::doOverdriveMode(const bool& doOverdriveMode)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x37;
			buf[4] = 0x00;
			
			if (doOverdriveMode)
			{
				buf[4] |= 1;
			}
			
			this->send(buf.data(), this->HEADER_SIZE + 1 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x37);
		}
		
		void
		WeissWsg50::doPrePositionFingers(const float& width, const float& speed, const bool& doRelativeMovement, const bool& doStopOnBlock)
		{
			assert(this->isConnected());
			assert(width >= 0.0f && width <= 0.11f);
			assert(speed >= 0.0f && speed <= 0.4f);
			
			float widthMilli = width * static_cast<float>(::rl::math::UNIT2MILLI);
			float speedMilli = speed * static_cast<float>(::rl::math::UNIT2MILLI);
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x21;
			buf[this->HEADER_SIZE] = 0x00;
			
			if (doRelativeMovement)
			{
				buf[this->HEADER_SIZE] |= 1;
			}
			
			if (doStopOnBlock)
			{
				buf[this->HEADER_SIZE] |= 2;
			}
			
			::std::memcpy(&buf[this->HEADER_SIZE + 1], &widthMilli, sizeof(widthMilli));
			::std::memcpy(&buf[this->HEADER_SIZE + 4 + 1], &speedMilli, sizeof(speedMilli));
			
			this->send(buf.data(), this->HEADER_SIZE + 9 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x21);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x21); // wait for completion
		}
		
		void
		WeissWsg50::doReleasePart(const float& width, const float& speed)
		{
			assert(this->isConnected());
			assert(width >= 0.0f && width <= 0.11f);
			assert(speed >= 0.005f && speed <= 0.4f);
			
			float widthMilli = width * static_cast<float>(::rl::math::UNIT2MILLI);
			float speedMilli = speed * static_cast<float>(::rl::math::UNIT2MILLI);
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x26;
			::std::memcpy(&buf[this->HEADER_SIZE], &widthMilli, sizeof(widthMilli));
			::std::memcpy(&buf[this->HEADER_SIZE + 4], &speedMilli, sizeof(speedMilli));
			
			this->send(buf.data(), this->HEADER_SIZE + 8 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x26);
		}
		
		void
		WeissWsg50::doSetAcceleration(const float& acceleration)
		{
			assert(this->isConnected());
			assert(acceleration >= 0.1f && acceleration <= 5.0f);
			
			float accelerationMilli = acceleration * static_cast<float>(::rl::math::UNIT2MILLI);
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x30;
			::std::memcpy(&buf[this->HEADER_SIZE], &accelerationMilli, sizeof(accelerationMilli));
			
			this->send(buf.data(), this->HEADER_SIZE + 4 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x30);
			
			this->acceleration = acceleration;
		}
		
		void
		WeissWsg50::doSetForceLimit(const float& force)
		{
			assert(this->isConnected());
			assert(force >= 5.0f && force <= 80.0f);
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x32;
			::std::memcpy(&buf[this->HEADER_SIZE], &force, sizeof(force));
			
			this->send(buf.data(), this->HEADER_SIZE + 4 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x32);
			
			this->forceLimit = force;
		}
		
		void
		WeissWsg50::doSetSoftLimits(const float& limitMinus, const float& limitPlus)
		{
			assert(this->isConnected());
			
			float limitMinusMilli = limitMinus * static_cast<float>(::rl::math::UNIT2MILLI);
			float limitPlusMilli = limitPlus * static_cast<float>(::rl::math::UNIT2MILLI);
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x34;
			::std::memcpy(&buf[this->HEADER_SIZE], &limitMinusMilli, sizeof(limitMinusMilli));
			::std::memcpy(&buf[this->HEADER_SIZE] + 4, &limitPlusMilli, sizeof(limitPlusMilli));
			
			this->send(buf.data(), this->HEADER_SIZE + 8 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x34);
			
			this->limitMinus = limitMinus;
			this->limitPlus = limitPlus;
		}
		
		void
		WeissWsg50::doStop()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x22;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x22);
		}
		
		void
		WeissWsg50::doTareForceSensor()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			buf[3] = 0x38;
			
			this->send(buf.data(), this->HEADER_SIZE + 0 + 2);
			this->recv(buf.data(), this->HEADER_SIZE + 2 + 0 + 2, 0x38);
		}
		
		float
		WeissWsg50::getAcceleration() const
		{
			return this->acceleration;
		}
		
		float
		WeissWsg50::getForce() const
		{
			return this->force;
		}
		
		float
		WeissWsg50::getForceLimit() const
		{
			return this->forceLimit;
		}
		
		WeissWsg50::GraspingState
		WeissWsg50::getGraspingState() const
		{
			return this->graspingState;
		}
		
		float
		WeissWsg50::getOpeningWidth() const
		{
			return this->openingWidth;
		}
		
		float
		WeissWsg50::getSpeed() const
		{
			return this->speed;
		}
		
		WeissWsg50::SystemState
		WeissWsg50::getSystemState() const
		{
			return this->systemState;
		}
		
		void
		WeissWsg50::halt()
		{
			assert(this->isConnected());
			
			this->doStop();
		}
		
		void
		WeissWsg50::open()
		{
			assert(!this->isConnected());
			this->socket.open();
			this->socket.connect();
			this->setConnected(true);
			this->doAcknowledgeFaults();
		}
		
		::std::size_t
		WeissWsg50::recv(::std::uint8_t* buf)
		{
			assert(this->isConnected());
			
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
						numbytes = this->socket.recv(ptr, 1);
					}
					while (0xAA != buf[0]);
					
					ptr += numbytes;
					sumbytes += numbytes;
					
					numbytes = this->socket.recv(ptr, 1);
				}
				while (0xAA != buf[1]);
				
				ptr += numbytes;
				sumbytes += numbytes;
				
				numbytes = this->socket.recv(ptr, 1);
			}
			while (0xAA != buf[2]);
			
			ptr += numbytes;
			sumbytes += numbytes;
			
			numbytes = this->socket.recv(ptr, 1);
			ptr += numbytes;
			sumbytes += numbytes;
			
			for (::std::size_t i = 0; i < 2 + 2; ++i)
			{
				numbytes = this->socket.recv(ptr, 1);
				ptr += numbytes;
				sumbytes += numbytes;
			}
			
#ifdef DEBUG_TCP_DATA
			printf("Debug: recv header buf:");
			for (::std::size_t i = 0; i < sumbytes; ++i)
			{
				printf(" %02X", buf[i]);
			}
			fflush(stdout);
#endif
			
			::std::uint16_t length = Endian::hostWord(buf[5], buf[4]);
			
			WeissException::Code code = static_cast<WeissException::Code>(Endian::hostWord(buf[7], buf[6]));
			
			switch (code)
			{
			case WeissException::CODE_SUCCESS:
				break;
			case WeissException::CODE_COMMAND_PENDING:
				break;
			case WeissException::CODE_AXIS_BLOCKED:
				break;
			default:
//				::std::cerr << "Debug: error: " << error << ::std::endl;
				throw WeissException(code);
				break;
			}
			
			::std::size_t len = length + 8;
			
			while (sumbytes < len)
			{
				numbytes = this->socket.recv(ptr, len - sumbytes);
				
				ptr += numbytes;
				sumbytes += numbytes;
			}
			
#ifdef DEBUG_TCP_DATA
			for (::std::size_t i = 8; i < sumbytes; ++i)
			{
				printf(" %02X", buf[i]);
			}
			printf("\n");
			fflush(stdout);
#endif
			
			if (this->crc(buf, sumbytes - 2) != Endian::hostWord(buf[sumbytes - 1], buf[sumbytes - 2]))
			{
				throw DeviceException("Checksum error.");
			}
			
			switch (buf[3])
			{
			case 0x40:
				this->systemState = static_cast<SystemState>(
					Endian::hostDoubleWord(
						Endian::hostWord(buf[11], buf[10]),
						Endian::hostWord(buf[9], buf[8])
					)
				);
				break;
			case 0x41:
				this->graspingState = static_cast<GraspingState>(buf[8]);
				break;
			case 0x43:
				::std::memcpy(&this->openingWidth, &buf[8], sizeof(this->openingWidth));
				this->openingWidth *= static_cast<float>(::rl::math::MILLI2UNIT);
				break;
			case 0x44:
				::std::memcpy(&this->speed, &buf[8], sizeof(this->speed));
				this->speed *= static_cast<float>(::rl::math::MILLI2UNIT);
				break;
			case 0x45:
				this->force = *reinterpret_cast<float*>(&buf[8]);
				::std::memcpy(&this->force, &buf[8], sizeof(this->force));
				break;
			default:
				break;
			}
			
			return sumbytes;
		}
		
		::std::size_t
		WeissWsg50::recv(::std::uint8_t* buf, const ::std::size_t& len, const ::std::uint8_t& command)
		{
			assert(this->isConnected());
			assert(len > 9);
			
			::std::size_t sumbytes;
			
			do
			{
				sumbytes = this->recv(buf);
			}
			while (command != buf[3]);
			
			return sumbytes;
		}
		
		void
		WeissWsg50::release()
		{
			assert(this->isConnected());
			
			this->doReleasePart(0.1097f, 0.2f);
		}
		
		void
		WeissWsg50::send(::std::uint8_t* buf, const ::std::size_t& len)
		{
			assert(this->isConnected());
			assert(len > 6);
			
			buf[0] = 0xAA;
			buf[1] = 0xAA;
			buf[2] = 0xAA;
			
			::std::uint16_t length = len - this->HEADER_SIZE - 2;
			
			buf[4] = Endian::hostLowByte(length);
			buf[5] = Endian::hostHighByte(length);
			
			::std::uint16_t checksum = this->crc(buf, len - 2);
			
			buf[len - 2] = Endian::hostLowByte(checksum);
			buf[len - 1] = Endian::hostHighByte(checksum);
			
			if (len != this->socket.send(buf, len))
			{
				throw DeviceException("Could not send complete data.");
			}
			
#ifdef DEBUG_TCP_DATA
			printf("Debug: send buf:");
			for (::std::size_t i = 0; i < len; ++i)
			{
				printf(" %02X", buf[i]);
			}
			printf("\n");
			fflush(stdout);
#endif
		}
		
		void
		WeissWsg50::shut()
		{
			assert(this->isConnected());
			
			this->doGraspPart(0, 0.2f);
		}
		
		void
		WeissWsg50::start()
		{
			assert(this->isConnected());
			
			this->doHomingMotion();
			
			this->doGetAcceleration();
			this->doGetForceLimit();
			// TODO this->doGetSoftLimits();
			this->doGetSystemLimits();
			
			this->doSetAcceleration(this->acceleration);
			this->doSetForceLimit(this->forceLimit);
			
			this->doGetForce(true, false, this->period);
			this->doGetGraspingState(true, false, this->period);
			this->doGetOpeningWidth(true, false, this->period);
			this->doGetSpeed(true, false, this->period);
			this->doGetSystemState(true, false, this->period);
			
			this->setRunning(true);
		}
		
		void
		WeissWsg50::step()
		{
			assert(this->isConnected());
			assert(this->isRunning());
			
			::std::array< ::std::uint8_t, 64> buf;
			
			this->recv(buf.data()); // doGetForce
			this->recv(buf.data()); // doGetGraspingState
			this->recv(buf.data()); // doGetOpeningWidth
			this->recv(buf.data()); // doGetSpeed
			this->recv(buf.data()); // doGetSystemState
		}
		
		void
		WeissWsg50::stop()
		{
			assert(this->isConnected());
			assert(this->isRunning());
			
			this->doStop();
			
			this->doGetForce(false, false);
			this->doGetGraspingState(false, false);
			this->doGetOpeningWidth(false, false);
			this->doGetSpeed(false, false);
			this->doGetSystemState(false, false);
			
			this->setRunning(false);
		}
	}
}
