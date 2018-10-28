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
#include <rl/math/Unit.h>

#include "Endian.h"
#include "RobotiqModelC.h"

namespace rl
{
	namespace hal
	{
		RobotiqModelC::RobotiqModelC(const ::std::string& filename) :
			CyclicDevice(::std::chrono::microseconds(5)),
			Gripper(),
			in(),
			out(),
			serial(
				filename,
				Serial::BAUDRATE_115200BPS,
				Serial::DATABITS_8BITS,
				Serial::FLOWCONTROL_OFF,
				Serial::PARITY_NOPARITY,
				Serial::STOPBITS_1BIT
			)
		{
			this->in.fill(0x00);
			this->out.fill(0x00);
		}
		
		RobotiqModelC::~RobotiqModelC()
		{
		}
		
		void
		RobotiqModelC::close()
		{
			assert(this->isConnected());
			
			this->serial.close();
			this->setConnected(false);
		}
		
		::std::uint16_t
		RobotiqModelC::crc(const ::std::uint8_t* buf, const ::std::size_t& len) const
		{	
			::std::uint16_t checksum = 0xFFFF;
			
			for (::std::size_t i = 0; i < len; ++i)
			{
				checksum ^= buf[i];
				
				for (::std::size_t i = 8; i != 0; --i)
				{
					if (checksum & 0x0001)
					{
						checksum >>= 1;
						checksum ^= 0xA001;
					}
					else
					{
						checksum >>= 1;
					}
				}
			}
			
			return checksum;
		}
		
		RobotiqModelC::ActionStatus
		RobotiqModelC::getActionStatus() const
		{
			return static_cast<ActionStatus>((this->in[3] >> 3) & 0x01);
		}
		
		RobotiqModelC::ActivationStatus
		RobotiqModelC::getActivationStatus() const
		{
			return static_cast<ActivationStatus>(this->in[3] & 0x01);
		}
		
		::rl::math::Real
		RobotiqModelC::getCurrent() const
		{
			return this->in[8] * ::rl::math::MILLI2UNIT;
		}
		
		RobotiqModelC::FaultStatus
		RobotiqModelC::getFaultStatus() const
		{
			return static_cast<FaultStatus>(this->in[5] & 0x0F);
		}
		
		RobotiqModelC::GripperStatus
		RobotiqModelC::getGripperStatus() const
		{
			return static_cast<GripperStatus>((this->in[3] >> 4) & 0x03);
		}
		
		RobotiqModelC::ObjectStatus
		RobotiqModelC::getObjectStatus() const
		{
			return static_cast<ObjectStatus>((this->in[3] >> 6) & 0x03);
		}
		
		::rl::math::Real
		RobotiqModelC::getPositionPercentage() const
		{
			return this->in[7] / static_cast< ::rl::math::Real>(0xFF);
		}
		
		::rl::math::Real
		RobotiqModelC::getPositionRequestEchoPercentage() const
		{
			return this->in[6] / static_cast< ::rl::math::Real>(0xFF);
		}
		
		void
		RobotiqModelC::halt()
		{
			this->out[10] = this->in[6];
		}
		
		void
		RobotiqModelC::open()
		{
			this->serial.open();
			this->setConnected(true);
		}
		
		::std::size_t
		RobotiqModelC::recv(::std::uint8_t* buf, const ::std::size_t& len)
		{
			assert(this->isConnected());
			
			::std::uint8_t* ptr = buf;
			::std::size_t sumbytes = 0;
			::std::size_t numbytes = 0;
			
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
		RobotiqModelC::release()
		{
			this->out[10] = 0x00;
		}
		
		void
		RobotiqModelC::send(::std::uint8_t* buf, const ::std::size_t& len)
		{
			assert(this->isConnected());
			
			buf[0] = 0x09;
			
			::std::uint16_t checksum = this->crc(buf, len - 2);
			
			buf[len - 2] = Endian::hostLowByte(checksum);
			buf[len - 1] = Endian::hostHighByte(checksum);
			
			if (len != this->serial.write(buf, len))
			{
				throw DeviceException("could not send complete data");
			}
		}
		
		void
		RobotiqModelC::setForcePercentage(const ::rl::math::Real& forcePercentage)
		{
			this->out[12] = static_cast< ::std::uint8_t>(0xFF * forcePercentage);
		}
		
		void
		RobotiqModelC::setPositionPercentage(const ::rl::math::Real& positionPercentage)
		{
			this->out[10] = static_cast< ::std::uint8_t>(0xFF * positionPercentage);
		}
		
		void
		RobotiqModelC::setSpeedPercentage(const ::rl::math::Real& speedPercentage)
		{
			this->out[11] = static_cast< ::std::uint8_t>(0xFF * speedPercentage);
		}
		
		void
		RobotiqModelC::shut()
		{
			this->out[10] = 0xFF;
		}
		
		void
		RobotiqModelC::start()
		{
			this->step();
			
			if (ACTIVATION_STATUS_READY != this->getActivationStatus() || GRIPPER_STATUS_READY != this->getGripperStatus())
			{
				this->out[1] = 0x10; // function code
				
				this->out[2] = 0x03; // address of register to write
				this->out[3] = 0xE8; // address of register to write
				
				this->out[4] = 0x00; // number of registers to write
				this->out[5] = 0x01; // number of registers to write
				
				this->out[6] = 0x02; // number of data bytes
				
				this->out[7] = 0x00; // value written to register 03E8 (action request)
				this->out[8] = 0x00; // value written to register 03E8 (gripper options)
				
				this->send(this->out.data(), 1 + 1 + 2 + 2 + 1 + 2 + 2);
				this->recv(this->in.data(), 1 + 1 + 2 + 2 + 2);
				
				this->out[1] = 0x10; // function code
				
				this->out[2] = 0x03; // address of register to write
				this->out[3] = 0xE8; // address of register to write
				
				this->out[4] = 0x00; // number of registers to write
				this->out[5] = 0x01; // number of registers to write
				
				this->out[6] = 0x02; // number of data bytes
				
				this->out[7] = 0x01; // value written to register 03E8 (action request)
				this->out[8] = 0x00; // value written to register 03E8 (gripper options)
				
				this->send(this->out.data(), 1 + 1 + 2 + 2 + 1 + 2 + 2);
				this->recv(this->in.data(), 1 + 1 + 2 + 2 + 2);
			}
		}
		
		void
		RobotiqModelC::step()
		{
			if (0x03 == this->in[1] && this->out[10] != this->in[6])
			{
				this->out[1] = 0x10; // function code
				
				this->out[2] = 0x03; // address of register to write
				this->out[3] = 0xE8; // address of register to write
				
				this->out[4] = 0x00; // number of registers to write
				this->out[5] = 0x03; // number of registers to write
				
				this->out[6] = 0x06; // number of data bytes
				
				this->out[7] = 0x09; // value written to register 03E8 (action request)
				this->out[8] = 0x00; // value written to register 03E8 (gripper options)
				
				this->out[9] = 0x00; // value written to register 03E9 (gripper options 2)
#if 0
				this->out[10] = 0x00; // value written to register 03E9 (position request)
				
				this->out[11] = 0x00; // value written to register 03EA (speed)
				this->out[12] = 0x00; // value written to register 03EA (force)
#endif
				
				this->send(this->out.data(), 1 + 1 + 2 + 2 + 1 + 3 * 2 + 2);
				this->recv(this->in.data(), 1 + 1 + 2 + 2 + 2);
			}
			
			this->out[1] = 0x03; // function code
			
			this->out[2] = 0x07; // address of register to read
			this->out[3] = 0xD0; // address of register to read
			
			this->out[4] = 0x00; // number of registers to read
			this->out[5] = 0x03; // number of registers to read
			
			this->send(this->out.data(), 1 + 1 + 2 + 2 + 2);
			this->recv(this->in.data(), 1 + 1 + 1 + 6 + 2);
			
			if (this->getFaultStatus() >= 0x0A)
			{
				throw Exception(this->getFaultStatus());
			}
			
			if (ACTIVATION_STATUS_READY == this->getActivationStatus() && GRIPPER_STATUS_READY == this->getGripperStatus())
			{
				this->setRunning(true);
			}
			else
			{
				this->setRunning(false);
			}
		}
		
		void
		RobotiqModelC::stop()
		{
			this->setRunning(false);
		}
		
		RobotiqModelC::Exception::Exception(const FaultStatus& faultStatus) :
			DeviceException(""),
			faultStatus(faultStatus)
		{
		}
		
		RobotiqModelC::Exception::~Exception() throw()
		{
		}
		
		RobotiqModelC::FaultStatus
		RobotiqModelC::Exception::getFaultStatus() const
		{
			return this->faultStatus;
		}
		
		const char*
		RobotiqModelC::Exception::what() const throw()
		{
			switch (this->faultStatus)
			{
			case FAULT_STATUS_NONE:
				return "No fault.";
				break;
			case FAULT_STATUS_NOTICE_ACTIVATION_NEEDED:
				return "The activation bit must be set prior to action";
				break;
			case FAULT_STATUS_WARNING_TEMPERATURE:
				return "Maximum operating temperature exceeded, wait for cool-down";
				break;
			case FAULT_STATUS_WARNING_COMM_NOT_READY:
				return "The communication chip is not ready (may be booting).";
				break;
			case FAULT_STATUS_WARNING_VOLTAGE:
				return "Under minimum operating voltage.";
				break;
			case FAULT_STATUS_WARNING_AUTOMATIC_RELEASE:
				return "Automatic release in progress.";
				break;
			case FAULT_STATUS_ERROR_INTERNAL:
				return "Internal fault.";
				break;
			case FAULT_STATUS_ERROR_ACTIVATION_FAULT:
				return "Activation fault, verify that no interference or other error occurred.";
				break;
			case FAULT_STATUS_ERROR_MODE_FAULT:
				return "Overcurrent triggered.";
				break;
			case FAULT_STATUS_ERROR_AUTOMATIC_RELEASE_COMPLETE:
				return "Automatic release completed.";
				break;
			default:
				return "Unknown fault.";
				break;
			}
		}
	}
}
