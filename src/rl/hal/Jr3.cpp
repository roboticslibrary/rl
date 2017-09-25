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

#include "Comedi.h"
#include "Jr3.h"

namespace rl
{
	namespace hal
	{
		Jr3::Jr3(const ::std::string& filename, const ::std::chrono::nanoseconds& updateRate) :
			CyclicDevice(updateRate),
			SixAxisForceTorqueSensor(),
			comedi(filename),
			values(),
			zeroes()
		{
			for (::std::size_t i = 0; i < 6; ++i)
			{
				this->values[i] = ::std::numeric_limits< ::rl::math::Real>::quiet_NaN();
				this->zeroes[i] = 0;
			}
		}
		
		Jr3::~Jr3()
		{
		}
		
		void
		Jr3::bias()
		{
			for (::std::size_t i = 0; i < 6; ++i)
			{
				this->zeroes[i] = this->values[i];
			}
		}
		
		void
		Jr3::close()
		{
			this->comedi.close();
			this->setConnected(false);
		}
		
		::rl::math::Vector
		Jr3::getForces() const
		{
			::rl::math::Vector forces(3);
			
			for (::std::size_t i = 0; i < 3; ++i)
			{
				forces(i) = (this->values[i] - this->zeroes[i]) * 1000;
			}
			
			return forces;
		}
		
		::rl::math::Real
		Jr3::getForcesMaximum(const ::std::size_t& i) const
		{
			assert(i < 4);
			
			return -this->comedi.getMax(0, i);
		}
		
		::rl::math::Real
		Jr3::getForcesMinimum(const ::std::size_t& i) const
		{
			assert(i < 4);
			
			return -this->comedi.getMax(0, i);
		}
		
		::rl::math::Vector
		Jr3::getForcesTorques() const
		{
			::rl::math::Vector forcesTorques(6);
			
			for (::std::size_t i = 0; i < 6; ++i)
			{
				forcesTorques(i) = (this->values[i] - this->zeroes[i]) * 1000;
			}
			
			return forcesTorques;
		}
		
		::rl::math::Real
		Jr3::getForcesTorquesMaximum(const ::std::size_t& i) const
		{
			assert(i < 7);
			
			return this->comedi.getMax(0, i);
		}
		
		::rl::math::Real
		Jr3::getForcesTorquesMinimum(const ::std::size_t& i) const
		{
			assert(i < 7);
			
			return -this->comedi.getMax(0, i);
		}
		
		::rl::math::Vector
		Jr3::getTorques() const
		{
			::rl::math::Vector torques(3);
			
			for (::std::size_t i = 3; i < 6; ++i)
			{
				torques(i) = (this->values[i] - this->zeroes[i]) * 1000;
			}
			
			return torques;
		}
		
		::rl::math::Real
		Jr3::getTorquesMaximum(const ::std::size_t& i) const
		{
			assert(i < 4);
			
			return this->comedi.getMax(0, 3 + i);
		}
		
		::rl::math::Real
		Jr3::getTorquesMinimum(const ::std::size_t& i) const
		{
			assert(i < 4);
			
			return -this->comedi.getMax(0, 3 + i);
		}
		
		void
		Jr3::open()
		{
			this->comedi.open();
			this->setConnected(true);
		}
		
		void
		Jr3::resetBias()
		{
			for (::std::size_t i = 0; i < 6; ++i)
			{
				this->zeroes[i] = 0;
			}
		}
		
		void
		Jr3::start()
		{
		}
		
		void
		Jr3::step()
		{
			for (::std::size_t i = 0; i < 6; ++i)
			{
				this->comedi.read(0, i, this->values[i]);
			}
		}
		
		void
		Jr3::stop()
		{
		}
	}
}
