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
#include <thread>
#include <rl/math/Unit.h>

#include "Endian.h"
#include "DeviceException.h"
#include "LeuzeRs4.h"

namespace rl
{
	namespace hal
	{
		LeuzeRs4::LeuzeRs4(
			const ::std::string& filename,
			const BaudRate& baudRate,
			const ::std::string& password
		) :
			CyclicDevice(::std::chrono::nanoseconds::zero()),
			Lidar(),
			baudRate(BAUDRATE_57600BPS),
			data(),
			desired(baudRate),
			far1(false),
			far2(false),
			fn1Fn2(false),
			near1(false),
			near2(false),
			password(password),
			serial(
				filename,
				Serial::BAUDRATE_57600BPS,
				Serial::DATABITS_8BITS,
				Serial::FLOWCONTROL_OFF,
				Serial::PARITY_NOPARITY,
				Serial::STOPBITS_1BIT
			),
			startIndex(0xFFFF),
			stepSize(0xFFFF),
			stopIndex(0xFFFF),
			type(0)
		{
			assert(password.length() < 9);
		}
		
		LeuzeRs4::~LeuzeRs4()
		{
		}
		
		void
		LeuzeRs4::close()
		{
			assert(this->isConnected());
			
//			if (MONITORING_SINGLE != this->monitoring)
//			{
//				this->setMonitoring(MONITORING_SINGLE);
//			}
//			
//			if (BAUDRATE_57600BPS != this->baudRate)
//			{
//				this->setBaudRate(BAUDRATE_57600BPS);
//			}
			
			this->serial.close();
			
			this->setConnected(false);
		}
		
		::std::uint8_t
		LeuzeRs4::crc(const ::std::uint8_t* buf, const ::std::size_t& len) const
		{	
			::std::uint8_t checksum = buf[0];
			
			for (::std::size_t i = 1; i < len; ++i)
			{
				checksum ^= buf[i];
			}
			
			if (0x00 == checksum)
			{
				checksum = 0xFF;
			}
			
			return checksum;
		}
		
		const ::std::uint8_t&
		LeuzeRs4::get(const ::std::uint8_t*& ptr) const
		{
			if (0x00 == ptr[-1] && 0x00 == ptr[0])
			{
				++ptr;
				++ptr;
				return ptr[-2];
			}
			else
			{
				++ptr;
				return ptr[-1];
			}
			
		}
		
		LeuzeRs4::BaudRate
		LeuzeRs4::getBaudRate() const
		{
			return this->baudRate;
		}
		
		void
		LeuzeRs4::getDistances(::rl::math::Vector& distances) const
		{
			assert(this->isConnected());
			assert(distances.size() >= this->getDistancesCount());
			
			const ::std::uint8_t* ptr;
			
			if (this->data[3] & 2)
			{
				ptr = this->data.data() + 5;
				
				if ((this->data[3] & 3) && (this->data[4] & 128))
				{
					ptr = this->data.data() + 6;
				}
			}
			else
			{
				ptr = this->data.data() + 4;
			}
			
			const ::std::uint8_t& number1 = this->get(ptr);
			this->get(ptr);//bool motion = this->get(ptr) & 64 ? true : false;
			const ::std::uint8_t& number2 = this->get(ptr);
			this->get(ptr);
			const ::std::uint8_t& number3 = this->get(ptr);
			this->get(ptr);
			const ::std::uint8_t& number4 = this->get(ptr);
			this->get(ptr);
			
			/*int number = */Endian::hostDoubleWord(Endian::hostWord(number1, number2), Endian::hostWord(number3, number4));
			
			int step = this->get(ptr);
			
			const ::std::uint8_t& startHigh = this->get(ptr);
			const ::std::uint8_t& startLow = this->get(ptr);
			int start = Endian::hostWord(startHigh, startLow);
			
			const ::std::uint8_t& stopHigh = this->get(ptr);
			const ::std::uint8_t& stopLow = this->get(ptr);
			int stop = Endian::hostWord(stopHigh, stopLow);
			
			for (int i = 0; i < (stop - start) / step + 1; ++i)
			{
				const ::std::uint8_t& dataHigh = this->get(ptr);
				const ::std::uint8_t& dataLow = this->get(ptr);
				distances(distances.size() - 1 - i) = Endian::hostWord(dataHigh, dataLow & 254) / 1000.0f;
			}
		}
		
		::std::size_t
		LeuzeRs4::getDistancesCount() const
		{
			assert(this->isConnected());
			
			return (this->stopIndex - this->startIndex) / this->stepSize + 1;
		}
		
		::rl::math::Real
		LeuzeRs4::getDistancesMaximum(const ::std::size_t& i) const
		{
			assert(i < this->getDistancesCount());
			
			return 4.0f;
		}
		
		::rl::math::Real
		LeuzeRs4::getDistancesMinimum(const ::std::size_t& i) const
		{
			assert(i < this->getDistancesCount());
			
			return 0.0f;
		}
		
		::rl::math::Real
		LeuzeRs4::getResolution() const
		{
			assert(this->isConnected());
			
			return this->stepSize * 0.36f * ::rl::math::DEG2RAD;
		}
		
		::rl::math::Real
		LeuzeRs4::getStartAngle() const
		{
			assert(this->isConnected());
			
			return (-5 + this->startIndex * 0.36f) * ::rl::math::DEG2RAD;
		}
		
		::rl::math::Real
		LeuzeRs4::getStopAngle() const
		{
			assert(this->isConnected());
			
			return (185 - (528 - this->stopIndex) * 0.36f) * ::rl::math::DEG2RAD;
		}
		
		void
		LeuzeRs4::open()
		{
			this->serial.open();
			
			this->setConnected(true);
			
#if 0
			::std::array< ::std::uint8_t, 1099> buf;
			
			// synchronize baud rates
			
			buf[4] = 0x20;
			buf[5] = 0x42;
			
			Serial::BaudRate baudRates[6] = {
//				Serial::BAUDRATE_625000BPS,
//				Serial::BAUDRATE_345600BPS,
				Serial::BAUDRATE_115200BPS,
				Serial::BAUDRATE_57600BPS,
				Serial::BAUDRATE_38400BPS,
				Serial::BAUDRATE_19200BPS,
				Serial::BAUDRATE_9600BPS,
				Serial::BAUDRATE_4800BPS
			};
			
			for (::std::size_t i = 0; i < 6; ++i)
			{
				this->serial.setBaudRate(baudRates[i]);
				this->serial.changeParameters();
				this->send(buf.data(), 1 + 1 + 2 + 1 + 1 + 2);
				
				if (this->waitAck())
				{
					break;
				}
				
				if (5 == i)
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
#ifndef WIN32
			case 0x01:
				this->baudRate = BAUDRATE_500000BPS;
				break;
#endif // WIN32
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
#endif
		}
		
		::std::size_t
		LeuzeRs4::recv(::std::uint8_t* buf, const ::std::size_t& len)
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
					while (buf[0] != 0x00);
					
					ptr += numbytes;
					sumbytes += numbytes;
					
					numbytes = this->serial.read(ptr, 1);
				}
				while (buf[1] != 0x00);
				
				ptr += numbytes;
				sumbytes += numbytes;
				
				numbytes = this->serial.read(ptr, 1);
			}
			while (buf[2] == 0x00 || buf[2] == 0xFF);
			
			ptr += numbytes;
			sumbytes += numbytes;
			
			do
			{
				numbytes = this->serial.read(ptr, 1);
				
				ptr += numbytes;
				sumbytes += numbytes;
				
				if (sumbytes > 1099 - 1)
				{
					throw DeviceException("invalid data length");
					return -1;
				}
			}
			while (*(ptr - 1) != 0x00 || *(ptr - 2) != 0x00 || *(ptr - 3) != 0x00);
			
			if (this->crc(buf, sumbytes - 4) != buf[sumbytes - 4])
			{
				throw DeviceException("checksum error");
			}
			
			switch (buf[2])
			{
			case 0x17:
				throw DeviceException("not acknowledged");
				break;
			case 0x53:
				throw DeviceException("error");
				break;
			case 0x54:
				throw DeviceException("warning");
				break;
			default:
				break;
			}
			
			// Option 1
			
			if (buf[3] & 16)
			{
				throw DeviceException("error / malfunction");
			}
			
			if (buf[3] & 2)
			{
				// Option 2
				
				this->near1 = (buf[4] & 1) ? true : false;
				
				this->far1 = (buf[4] & 2) ? true : false;
				
				if (buf[4] & 8)
				{
					throw DeviceException("malfunction");
				}
				
				if (buf[4] & 16)
				{
					throw DeviceException("restart inhibit");
				}
				
				this->near2 = (buf[4] & 32) ? true : false;
				
				this->far2 = (buf[4] & 64) ? true : false;
				
				if ((buf[3] & 3) && (buf[4] & 128))
				{
					// Option 3
					
					this->fn1Fn2 = (buf[5] & 64) ? true : false;
				}
			}
			
			return sumbytes;
		}
		
		void
		LeuzeRs4::reset()
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 1099> buf;
			
			buf[2] = 0x10;
			buf[3] = 0x21;
			
			for (::std::size_t i = 0; i < this->password.length(); ++i)
			{
				buf[4 + i] = this->password[i];
			}
			
			for (::std::size_t i = this->password.length(); i < 8; ++i)
			{
				buf[4 + i] = 0xFF;
			}
			
			::std::uint8_t buf2[1099];
			
			this->send(buf.data(), 2 + 1 + 1 + 8 + 1 + 3);
			
			do
			{
				try
				{
					this->recv(buf2, 1099);//2 + 1 + 1 + 1 + 3);
				}
				catch (const DeviceException&)
				{
				}
			}
			while (0x23 != buf2[2]);
		}
		
		void
		LeuzeRs4::send(::std::uint8_t* buf, const ::std::size_t& len)
		{
			assert(this->isConnected());
			assert(len > 7);
			
			buf[0] = 0x00;
			buf[1] = 0x00;
			
			buf[len - 4] = this->crc(buf, len - 4);
			
			buf[len - 3] = 0x00;
			buf[len - 2] = 0x00;
			buf[len - 1] = 0x00;
			
			::std::size_t numbytes;
			
			for (::std::size_t i = 0; i < len; ++i)
			{
				if (i > 0)
				{
					::std::this_thread::sleep_for(std::chrono::milliseconds(40));
				}
				
				do
				{
					numbytes = this->serial.write(&buf[i], 1);
				}
				while (numbytes < 1);
			}
			
			this->serial.flush(true, false);
		}
		
		void
		LeuzeRs4::set(const ::std::uint8_t& value, ::std::uint8_t*& ptr, ::std::size_t& len) const
		{
			ptr[0] = value;
			++ptr;
			++len;
			
			if (0x00 == ptr[-2] && 0x00 == ptr[-1])
			{
				ptr[0] = 0xFF;
				++ptr;
				++len;
			}
		}
		
#if 0
		void
		LeuzeRs4::setBaudRate(const BaudRate& baudRate)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 1099> buf;
			
			buf[2] = 0x19;
			buf[3] = 0x41;
			
			::std::uint8_t* ptr = buf + 4;
			::std::size_t len = 0;
			
			switch (baudRate)
			{
			case BAUDRATE_4800BPS:
::std::cout << "setting to 4,800 baud" << ::std::endl;
				this->set(0x00, ptr, len);
				this->set(0x00, ptr, len);
				break;
			case BAUDRATE_9600BPS:
::std::cout << "setting to 9,600 baud" << ::std::endl;
				this->set(0x00, ptr, len);
				this->set(0x01, ptr, len);
				break;
			case BAUDRATE_19200BPS:
::std::cout << "setting to 19,200 baud" << ::std::endl;
				this->set(0x00, ptr, len);
				this->set(0x02, ptr, len);
				break;
			case BAUDRATE_38400BPS:
::std::cout << "setting to 38,400 baud" << ::std::endl;
				this->set(0x00, ptr, len);
				this->set(0x03, ptr, len);
				break;
			case BAUDRATE_57600BPS:
::std::cout << "setting to 57,600 baud" << ::std::endl;
				this->set(0x00, ptr, len);
				this->set(0x04, ptr, len);
				break;
			case BAUDRATE_115200BPS:
::std::cout << "setting to 115,200 baud" << ::std::endl;
				this->set(0x00, ptr, len);
				this->set(0x05, ptr, len);
				break;
//			case BAUDRATE_345600BPS:
//::std::cout << "setting to 345,600 baud" << ::std::endl;
//				this->set(0x00, ptr, len);
//				this->set(0x06, ptr, len);
//				break;
//			case BAUDRATE_625000BPS:
//::std::cout << "setting to 625,000 baud" << ::std::endl;
//				this->set(0x00, ptr, len);
//				this->set(0xFF, ptr, len);
//				break;
			default:
				break;
			}
			
			::std::uint8_t buf2[1099];
			
			do
			{
				this->send(buf.data(), 2 + 1 + 1 + len + 1 + 3);
				this->recv(buf2, 1099);//2 + 1 + 1 + 1 + 3);
			}
			while (0x16 != buf2[2]);
			
			switch (baudRate)
			{
			case BAUDRATE_4800BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_4800BPS);
				break;
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
			case BAUDRATE_115200BPS:
				this->serial.setBaudRate(Serial::BAUDRATE_115200BPS);
				break;
//			case BAUDRATE_345600BPS:
//				this->serial.setBaudRate(Serial::BAUDRATE_345600BPS);
//				break;
//			case BAUDRATE_625000BPS:
//				this->serial.setBaudRate(Serial::BAUDRATE_625000BPS);
//				break;
			default:
				break;
			}
			
			this->serial.changeParameters();
			
			this->baudRate = baudRate;
		}
		
		void
		LeuzeRs4::setOutputParameters(const ::std::uint16_t& startIndex, const ::std::uint16_t& stopIndex, const ::std::uint16_t& stepSize, const ::std::uint16_t& type)
		{
			assert(this->isConnected());
			
			::std::array< ::std::uint8_t, 1099> buf;
			
			buf[2] = 0x1A;
			buf[3] = 0x41;
			
			::std::uint8_t* ptr = buf + 4;
			::std::size_t len = 0;
			
			this->set(Endian::hostHighByte(startIndex), ptr, len);
			this->set(Endian::hostLowByte(startIndex), ptr, len);
			
			this->set(Endian::hostHighByte(stopIndex), ptr, len);
			this->set(Endian::hostLowByte(stopIndex), ptr, len);
			
			this->set(Endian::hostHighByte(stepSize), ptr, len);
			this->set(Endian::hostLowByte(stepSize), ptr, len);
			
			this->set(Endian::hostHighByte(type), ptr, len);
			this->set(Endian::hostLowByte(type), ptr, len);
			
			::std::uint8_t buf2[1099];
			
			do
			{
				this->send(buf.data(), 2 + 1 + 1 + len + 1 + 3);
				this->recv(buf2, 1099);//2 + 1 + 1 + 1 + 3);
			}
			while (0x16 != buf2[2]);
			
			this->startIndex = startIndex;
			this->stepSize = stepSize;
			this->stopIndex = stopIndex;
			this->type = type;
		}
#endif
		
		void
		LeuzeRs4::start()
		{
			assert(this->isConnected());
		}
		
		void
		LeuzeRs4::step()
		{
			assert(this->isConnected());
			
			if (2 == this->type)
			{
				::std::array< ::std::uint8_t, 1099> buf;
				
				buf[2] = 0x24;
				buf[3] = 0x01;
				
				this->send(buf.data(), 2 + 1 + 1 + 1 + 3);
			}
			
			this->recv(this->data.data(), 1099);
			
			if (0x23 != this->data[2])
			{
				throw DeviceException("incorrect reply");
			}
		}
		
		void
		LeuzeRs4::stop()
		{
			assert(this->isConnected());
		}
	}
}
