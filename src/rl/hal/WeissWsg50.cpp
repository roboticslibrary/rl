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

#include <cassert>
#include <cstring>
#include <iostream>

//#define DEBUG_TCP_DATA

#ifdef DEBUG_TCP_DATA
#include <cstdio>
#endif

#include "ComException.h"
#include "DeviceException.h"
#include "endian.h"
#include "WeissWsg50.h"

static const uint16_t CRC_TABLE_CCITT16[256] = {
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
			const ::std::string& hostname,
			const unsigned short int& port,
			const float& acceleration,
			const float& forceLimit,
			const unsigned int& period
		) :
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
			speed(0),
			speedMaximum(0),
			speedMinimum(0),
			stroke(0),
			systemState(),
			tcp(hostname, port)
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
			this->tcp.close();
			this->setConnected(false);
		}
		
		uint16_t
		WeissWsg50::crc(const uint8_t* buf, const ::std::size_t& len) const
		{
			uint16_t checksum = 0xFFFF;
			
			for (::std::size_t i = 0; i < len; ++i)
			{
				uint8_t index = checksum ^ buf[i];
				checksum = CRC_TABLE_CCITT16[index] ^ (checksum >> 8);
			}
			
			return checksum;
		}
		
		void
		WeissWsg50::doAcknowledgeFaults()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x24;
			buf[this->HEADER_SIZE] = 0x61;
			buf[this->HEADER_SIZE + 1] = 0x63;
			buf[this->HEADER_SIZE + 2] = 0x6B;
			
			this->send(buf, this->HEADER_SIZE + 3 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x24);
		}
		
		void
		WeissWsg50::doClearSoftLimits()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x36;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x36);
			
			this->limitMinus = 0;
			this->limitPlus = 0;
		}
		
		void
		WeissWsg50::doDisconnectAnnouncement()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x07;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x07);
		}
		
		void
		WeissWsg50::doFastStop()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x23;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x23);
		}
		
		void
		WeissWsg50::doGetAcceleration()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x31;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 4 + 2, 0x31);
			
			this->acceleration = *reinterpret_cast< float* >(&buf[this->HEADER_SIZE + 2]) / 1000.0f;
		}
		
		void
		WeissWsg50::doGetForce(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x45;
			
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doAutomaticUpdate << 1);
			buf[this->HEADER_SIZE + 1] = lowByteFromHostEndian(period);
			buf[this->HEADER_SIZE + 2] = highByteFromHostEndian(period);
			
			this->send(buf, this->HEADER_SIZE + 3 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 4 + 2, 0x45);
		}
		
		void
		WeissWsg50::doGetForceLimit()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x33;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 4 + 2, 0x33);
			
			this->forceLimit = *reinterpret_cast< float* >(&buf[this->HEADER_SIZE + 2]);
		}
		
		void
		WeissWsg50::doGetGraspingState(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x41;
			
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doUpdateOnChange << 1);
			buf[this->HEADER_SIZE + 1] = lowByteFromHostEndian(period);
			buf[this->HEADER_SIZE + 2] = highByteFromHostEndian(period);
			
			this->send(buf, this->HEADER_SIZE + 3 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 1 + 2, 0x41);
		}
		
		void
		WeissWsg50::doGetGraspingStatistics(const bool& doReset, int& numberGraspsTotal, int& numberGraspsNoPart, int& numberGraspsLostPart)
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x42;
			buf[this->HEADER_SIZE] = 0x00 | doReset;
			
			this->send(buf, this->HEADER_SIZE + 1 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 8 + 2, 0x42);
			
			numberGraspsTotal = hostEndianDoubleWord(hostEndianWord(buf[11], buf[10]), hostEndianWord(buf[9], buf[8]));
			numberGraspsNoPart = hostEndianWord(buf[13], buf[12]);
			numberGraspsLostPart = hostEndianWord(buf[15], buf[14]);
		}
		
		void
		WeissWsg50::doGetOpeningWidth(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x43;
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doUpdateOnChange << 1);
			buf[this->HEADER_SIZE + 1] = lowByteFromHostEndian(period);
			buf[this->HEADER_SIZE + 2] = highByteFromHostEndian(period);
			
			this->send(buf, this->HEADER_SIZE + 3 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 4 + 2, 0x43);
		}
		
		void
		WeissWsg50::doGetSoftLimits()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x35;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 8 + 2, 0x35);
			
			this->limitMinus = *reinterpret_cast< float* >(&buf[this->HEADER_SIZE + 2]);
			this->limitPlus = *reinterpret_cast< float* >(&buf[this->HEADER_SIZE + 2 + 4]);
		}
		
		void
		WeissWsg50::doGetSpeed(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x44;
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doUpdateOnChange << 1);
			buf[this->HEADER_SIZE + 1] = lowByteFromHostEndian(period);
			buf[this->HEADER_SIZE + 2] = highByteFromHostEndian(period);
			
			this->send(buf, this->HEADER_SIZE + 3 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 4 + 2, 0x44);
		}
		
		void
		WeissWsg50::doGetSystemInformation(bool& isWsg50, int& hardwareRevision, int& firmwareRevision, int& serialNumber)
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x50;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 8 + 2, 0x50);
			
			isWsg50 = (1 == buf[8]) ? true : false;
			hardwareRevision = buf[9];
			firmwareRevision = hostEndianWord(buf[11], buf[10]);
			serialNumber = hostEndianDoubleWord(hostEndianWord(buf[15], buf[14]), hostEndianWord(buf[13], buf[12]));
		}
		
		void
		WeissWsg50::doGetSystemLimits()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x53;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 32 + 2, 0x53);
			
			this->stroke = *reinterpret_cast< float* >(&buf[8]) / 1000.0f;
			this->speedMinimum = *reinterpret_cast< float* >(&buf[12]) / 1000.0f;
			this->speedMaximum = *reinterpret_cast< float* >(&buf[16]) / 1000.0f;
			this->accelerationMinimum = *reinterpret_cast< float* >(&buf[20]) / 1000.0f;
			this->accelerationMaximum = *reinterpret_cast< float* >(&buf[24]) / 1000.0f;
			this->forceMinimum = *reinterpret_cast< float* >(&buf[28]);
			this->forceNominal = *reinterpret_cast< float* >(&buf[32]);
			this->forceOverdrive = *reinterpret_cast< float* >(&buf[36]);
		}
		
		void
		WeissWsg50::doGetSystemState(const bool& doAutomaticUpdate, const bool& doUpdateOnChange, const unsigned int& period)
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x40;
			buf[this->HEADER_SIZE] = 0x00 | (doAutomaticUpdate << 0) | (doUpdateOnChange << 1);
			buf[this->HEADER_SIZE + 1] = lowByteFromHostEndian(period);
			buf[this->HEADER_SIZE + 2] = highByteFromHostEndian(period);
			
			this->send(buf, this->HEADER_SIZE + 3 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 4 + 2, 0x40);
			
			this->systemState = static_cast< SystemState >(buf[this->HEADER_SIZE + 2]);
		}
		
		float
		WeissWsg50::doGetTemperature()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x46;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 2 + 2, 0x46);
			
			int16_t temperature = hostEndianWord(buf[9], buf[8]);
			
			return temperature / 10.0f;
		}
		
		void
		WeissWsg50::doGraspPart(const float& width, const float& speed)
		{
			assert(this->isConnected());
			assert(width >= 0.0f && width <= 0.11f);
			assert(speed >= 0.0f && speed <= 0.4f);
			
			uint8_t buf[64];
			
			buf[3] = 0x25;
			
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE]) = width * 1000.0f;
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE + 4]) = speed * 1000.0f;
			
			this->send(buf, this->HEADER_SIZE + 8 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x25);
		}
		
		// Perform necessary homing motion for calibration.
		// This function call is blocking until the calibration is complete.
		void
		WeissWsg50::doHomingMotion(const unsigned int& direction)
		{
			assert(this->isConnected());
			assert(direction < 3);
			
			uint8_t buf[64];
			
			buf[3] = 0x20;
			buf[this->HEADER_SIZE] = direction;
			
			this->send(buf, this->HEADER_SIZE + 1 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x20);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x20); // wait for completion message
		}
		
		void
		WeissWsg50::doOverdriveMode(const bool& doOverdriveMode)
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x37;
			buf[4] = 0x00;
			
			if (doOverdriveMode)
			{
				buf[4] |= 1;
			}
			
			this->send(buf, this->HEADER_SIZE + 1 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x37);
		}
		
		void
		WeissWsg50::doPrePositionFingers(const float& width, const float& speed, const bool& doRelativeMovement, const bool& doStopOnBlock)
		{
			assert(this->isConnected());
			assert(width >= 0.0f && width <= 0.11f);
			assert(speed >= 0.0f && speed <= 0.4f);
			
			uint8_t buf[64];
			
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
			
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE + 1]) = width * 1000.0f;
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE + 4 + 1]) = speed * 1000.0f;
			
			this->send(buf, this->HEADER_SIZE + 9 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x21);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x21); // wait for completion
		}
		
		void
		WeissWsg50::doReleasePart(const float& width, const float& speed)
		{
			assert(this->isConnected());
			assert(width >= 0.0f && width <= 0.11f);
			assert(speed >= 0.005f && speed <= 0.4f);
			
			uint8_t buf[64];
			
			buf[3] = 0x26;
			
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE]) = width * 1000.0f;
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE + 4]) = speed * 1000.0f;
			
			this->send(buf, this->HEADER_SIZE + 8 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x26);
		}
		
		void
		WeissWsg50::doSetAcceleration(const float& acceleration)
		{
			assert(this->isConnected());
			assert(acceleration >= 0.1f && acceleration <= 5.0f);
			
			uint8_t buf[64];
			
			buf[3] = 0x30;
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE]) = acceleration * 1000.0f;
			
			this->send(buf, this->HEADER_SIZE + 4 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x30);
			
			this->acceleration = acceleration;
		}
		
		void
		WeissWsg50::doSetForceLimit(const float& force)
		{
			assert(this->isConnected());
			assert(force >= 5.0f && force <= 80.0f);
			
			uint8_t buf[64];
			
			buf[3] = 0x32;
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE]) = force;
			
			this->send(buf, this->HEADER_SIZE + 4 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x32);
			
			this->forceLimit = force;
		}
		
		void
		WeissWsg50::doSetSoftLimits(const float& limitMinus, const float& limitPlus)
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x34;
			
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE]) = limitMinus / 1000.0f;
			*reinterpret_cast< float * >(&buf[this->HEADER_SIZE + 4]) = limitPlus / 1000.0f;
			
			this->send(buf, this->HEADER_SIZE + 8 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x34);
			
			this->limitMinus = limitMinus;
			this->limitPlus = limitPlus;
		}
		
		void
		WeissWsg50::doStop()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x22;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x22);
		}
		
		void
		WeissWsg50::doTareForceSensor()
		{
			assert(this->isConnected());
			
			uint8_t buf[64];
			
			buf[3] = 0x38;
			
			this->send(buf, this->HEADER_SIZE + 0 + 2);
			this->recv(buf, this->HEADER_SIZE + 2 + 0 + 2, 0x38);
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
			// TODO
		}
		
		void
		WeissWsg50::open()
		{
			assert(!this->isConnected());
			this->tcp.open();
			this->setConnected(true);
			this->doAcknowledgeFaults();
		}
		
		::std::size_t
		WeissWsg50::recv(uint8_t* buf)
		{
			assert(this->isConnected());
			
			uint8_t* ptr;
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
						numbytes = this->tcp.read(ptr, 1);
					}
					while (0xAA != buf[0]);
					
					ptr += numbytes;
					sumbytes += numbytes;
					
					numbytes = this->tcp.read(ptr, 1);
				}
				while (0xAA != buf[1]);
				
				ptr += numbytes;
				sumbytes += numbytes;
				
				numbytes = this->tcp.read(ptr, 1);
			}
			while (0xAA != buf[2]);
			
			ptr += numbytes;
			sumbytes += numbytes;
			
			numbytes = this->tcp.read(ptr, 1);
			ptr += numbytes;
			sumbytes += numbytes;
			
			for (::std::size_t i = 0; i < 2 + 2; ++i)
			{
				numbytes = this->tcp.read(ptr, 1);
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
			
			uint16_t length = hostEndianWord(buf[5], buf[4]);
			
			Exception::Error errorCode = static_cast< Exception::Error >(hostEndianWord(buf[7], buf[6]));
			
			switch (errorCode)
			{
			case Exception::ERROR_SUCCESS:
				break;
			case Exception::ERROR_CMD_PENDING:
				break;
			case Exception::ERROR_AXIS_BLOCKED:
				break;
			default:
//				::std::cerr << "Debug: errorCode: " << errorCode << ::std::endl;
				throw Exception(errorCode);
				break;
			}
			
			::std::size_t len = length + 8;
			
			while (sumbytes < len)
			{
				numbytes = this->tcp.read(ptr, len - sumbytes);
				
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
			
			if (this->crc(buf, sumbytes - 2) != hostEndianWord(buf[sumbytes - 1], buf[sumbytes - 2]))
			{
				throw DeviceException("Checksum error.");
			}
			
			switch (buf[3])
			{
			case 0x41:
				this->graspingState = static_cast< GraspingState >(buf[8]);
				break;
			case 0x43:
				this->openingWidth = *reinterpret_cast< float* >(&buf[8]) / 1000.0f;
				break;
			case 0x44:
				this->speed = *reinterpret_cast< float* >(&buf[8]) / 1000.0f;
				break;
			case 0x45:
				this->force = *reinterpret_cast< float* >(&buf[8]);
				break;
			default:
				break;
			}
			
			return sumbytes;
		}
		
		::std::size_t
		WeissWsg50::recv(uint8_t* buf, const ::std::size_t& len, const uint8_t& command)
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
			// TODO
		}
		
		void
		WeissWsg50::send(uint8_t* buf, const ::std::size_t& len)
		{
			assert(this->isConnected());
			assert(len > 6);
			
			buf[0] = 0xAA;
			buf[1] = 0xAA;
			buf[2] = 0xAA;
			
			uint16_t length = len - this->HEADER_SIZE - 2;
			
			buf[4] = lowByteFromHostEndian(length);
			buf[5] = highByteFromHostEndian(length);
			
			uint16_t checksum = this->crc(buf, len - 2);
			
			buf[len - 2] = lowByteFromHostEndian(checksum);
			buf[len - 1] = highByteFromHostEndian(checksum);
			
			if (len != this->tcp.write(buf, len))
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
			// TODO
		}
		
		void
		WeissWsg50::start()
		{
			assert(this->isConnected());
			
			this->doGetAcceleration();
			this->doGetForceLimit();
			//TODO this->doGetSoftLimits();
			this->doGetSystemLimits();
			
			this->doGetForce(true, true, this->period);
			this->doGetGraspingState(true, true, this->period);
			this->doGetOpeningWidth(true, true, this->period);
			this->doGetSpeed(true, true, this->period);
			this->doGetSystemState(true, true, this->period);
			
			this->doHomingMotion();
			this->doSetAcceleration(this->acceleration);
			this->doSetForceLimit(this->forceLimit);
			
			this->setRunning(true);
		}
		
		void
		WeissWsg50::step()
		{
			assert(this->isConnected());
			assert(this->isRunning());
			
			uint8_t buf[64];
			
			// TODO: implement
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
		
		WeissWsg50::Exception::Exception(const Error& error) :
			DeviceException(""),
			error(error)
		{
		}
		
		WeissWsg50::Exception::~Exception() throw()
		{
		}
		
		WeissWsg50::Exception::Error
		WeissWsg50::Exception::getError() const
		{
			return this->error;
		}
		
		const char*
		WeissWsg50::Exception::what() const throw()
		{
			switch (this->error)
			{
			case ERROR_SUCCESS:
				return "No error.";
				break;
			case ERROR_NOT_AVAILABLE:
				return "Device, service or data is not available.";
				break;
			case ERROR_NO_SENSOR:
				return "No sensor connected.";
				break;
			case ERROR_NOT_INITIALIZED:
				return "The device is not initialized.";
				break;
			case ERROR_ALREADY_RUNNING:
				return "Service is already running.";
				break;
			case ERROR_FEATURE_NOT_SUPPORTED:
				return "The asked feature is not supported.";
				break;
			case ERROR_INCONSISTENT_DATA:
				return "One or more dependent parameters mismatch.";
				break;
			case ERROR_TIMEOUT:
				return "Timeout error.";
				break;
			case ERROR_READ_ERROR:
				return "Error while reading from a device.";
				break;
			case ERROR_WRITE_ERROR:
				return "Error while writing to a device.";
				break;
			case ERROR_INSUFFICIENT_RESOURCES:
				return "No memory available.";
				break;
			case ERROR_CHECKSUM_ERROR:
				return "Checksum error.";
				break;
			case ERROR_NO_PARAM_EXPECTED:
				return "No parameters expected.";
				break;
			case ERROR_NOT_ENOUGH_PARAMS:
				return "Not enough parameters.";
				break;
			case ERROR_CMD_UNKNOWN:
				return "Unknown command.";
				break;
			case ERROR_CMD_FORMAT_ERROR:
				return "Command format error.";
				break;
			case ERROR_ACCESS_DENIED:
				return "Access denied.";
				break;
			case ERROR_ALREADY_OPEN:
				return "The interface is already open.";
				break;
			case ERROR_CMD_FAILED:
				return "Command failed.";
				break;
			case ERROR_CMD_ABORTED:
				return "Command aborted.";
				break;
			case ERROR_INVALID_HANDLE:
				return "invalid handle.";
				break;
			case ERROR_NOT_FOUND:
				return "device not found.";
				break;
			case ERROR_NOT_OPEN:
				return "device not open.";
				break;
			case ERROR_IO_ERROR:
				return "I/O error.";
				break;
			case ERROR_INVALID_PARAMETER:
				return "invalid parameter.";
				break;
			case ERROR_INDEX_OUT_OF_BOUNDS:
				return "index out of bounds.";
				break;
			case ERROR_CMD_PENDING:
				return "Command was received correctly, but the execution needs more time. If the command was completely processed, another status message is returned indicating the command's result.";
				break;
			case ERROR_OVERRUN:
				return "Data overrun.";
				break;
			case ERROR_RANGE_ERROR:
				return "Range error.";
				break;
			case ERROR_AXIS_BLOCKED:
				return "Axis is blocked.";
				break;
			case ERROR_FILE_EXISTS:
				return "File already exists.";
				break;
			default:
				return "Unknown error.";
				break;
			}
		}
	}
}
