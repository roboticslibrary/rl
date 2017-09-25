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
#include <comedi.h>

#include "Ati.h"
#include "Comedi.h"
#include "DeviceException.h"

namespace rl
{
	namespace hal
	{
		Ati::Ati(
			const ::std::string& calFilePath,
			const ::std::chrono::nanoseconds& updateRate,
			const unsigned short int& index,
			const ::std::string& filename
		) :
			CyclicDevice(updateRate),
			SixAxisForceTorqueSensor(),
			cal(nullptr),
			calFilePath(calFilePath),
			comedi(filename),
			index(index),
			values(),
			voltages()
		{
			this->cal = createCalibration(const_cast<char*>(this->calFilePath.c_str()), this->index + 1);
			
			if (nullptr == this->cal)
			{
				throw DeviceException("Could not load the desired calibration");
			}
			
			for (::std::size_t i = 0; i < 6; ++i)
			{
				this->values[i] = ::std::numeric_limits< ::rl::math::Real>::quiet_NaN();
				this->voltages[i] = ::std::numeric_limits< ::rl::math::Real>::quiet_NaN();
			}
		}
		
		Ati::~Ati()
		{
			if (nullptr != this->cal)
			{
				destroyCalibration(this->cal);
			}
		}
		
		void
		Ati::bias()
		{
			Bias(this->cal, this->voltages);
		}
		
		void
		Ati::close()
		{
			this->comedi.close();
			this->setConnected(false);
		}
		
		::std::string
		Ati::getAxisName(const ::std::size_t& i) const
		{
			assert(i < 7);
			
			return this->cal->AxisNames[i];
		}
		
		::rl::math::Vector
		Ati::getForces() const
		{
			::rl::math::Vector forces(3);
			
			for (::std::size_t i = 0; i < 3; ++i)
			{
				forces(i) = this->values[i];
			}
			
			return forces;
		}
		
		::rl::math::Real
		Ati::getForcesMaximum(const ::std::size_t& i) const
		{
			assert(i < 4);
			
			return this->cal->MaxLoads[i];
		}
		
		::rl::math::Real
		Ati::getForcesMinimum(const ::std::size_t& i) const
		{
			assert(i < 4);
			
			return -this->cal->MaxLoads[i];
		}
		
		::rl::math::Vector
		Ati::getForcesTorques() const
		{
			::rl::math::Vector forcesTorques(6);
			
			for (::std::size_t i = 0; i < 6; ++i)
			{
				forcesTorques(i) = this->values[i];
			}
			
			return forcesTorques;
		}
		
		::rl::math::Real
		Ati::getForcesTorquesMaximum(const ::std::size_t& i) const
		{
			assert(i < 7);
			
			return this->cal->MaxLoads[i];
		}
		
		::rl::math::Real
		Ati::getForcesTorquesMinimum(const ::std::size_t& i) const
		{
			assert(i < 7);
			
			return -this->cal->MaxLoads[i];
		}
		
		::rl::math::Vector
		Ati::getTorques() const
		{
			::rl::math::Vector torques(3);
			
			for (::std::size_t i = 3; i < 6; ++i)
			{
				torques(i) = this->values[i];
			}
			
			return torques;
		}
		
		::rl::math::Real
		Ati::getTorquesMaximum(const ::std::size_t& i) const
		{
			assert(i < 4);
			
			return this->cal->MaxLoads[3 + i];
		}
		
		::rl::math::Real
		Ati::getTorquesMinimum(const ::std::size_t& i) const
		{
			assert(i < 4);
			
			return -this->cal->MaxLoads[3 + i];
		}
		
		void
		Ati::open()
		{
			this->comedi.open();
			this->setConnected(true);
		}
		
		void
		Ati::resetBias()
		{
			float voltages[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
			Bias(this->cal, voltages);
		}
		
		void
		Ati::start()
		{
		}
		
		void
		Ati::step()
		{
			for (::std::size_t i = 0; i < 6; ++i)
			{
				this->comedi.read(0, i, this->voltages[i]);
			}
			
			ConvertToFT(this->cal, this->voltages, this->values);
		}
		
		void
		Ati::stop()
		{
		}
	}
}
