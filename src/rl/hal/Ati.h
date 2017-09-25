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

#ifndef RL_HAL_ATI_H
#define RL_HAL_ATI_H

#include <string>
#include <atidaq/ftconfig.h>

#include "Comedi.h"
#include "CyclicDevice.h"
#include "SixAxisForceTorqueSensor.h"

namespace rl
{
	namespace hal
	{
		/**
		 * ATI Industrial Automation force-torque sensor.
		 */
		class Ati : public CyclicDevice, public SixAxisForceTorqueSensor
		{
		public:
			Ati(
				const ::std::string& calFilePath,
				const ::std::chrono::nanoseconds& updateRate = ::std::chrono::milliseconds(1),
				const unsigned short int& index = 0,
				const ::std::string& filename = "/dev/comedi0"
			);
			
			virtual ~Ati();
			
			void bias();
			
			void close();
			
			::std::string getAxisName(const ::std::size_t& i) const;
			
			::rl::math::Vector getForces() const;
			
			::rl::math::Real getForcesMaximum(const ::std::size_t& i) const;
			
			::rl::math::Real getForcesMinimum(const ::std::size_t& i) const;
			
			::rl::math::Vector getForcesTorques() const;
			
			::rl::math::Real getForcesTorquesMaximum(const ::std::size_t& i) const;
			
			::rl::math::Real getForcesTorquesMinimum(const ::std::size_t& i) const;
			
			::rl::math::Vector getTorques() const;
			
			::rl::math::Real getTorquesMaximum(const ::std::size_t& i) const;
			
			::rl::math::Real getTorquesMinimum(const ::std::size_t& i) const;
			
			void open();
			
			void resetBias();
			
			void start();
			
			void step();
			
			void stop();
			
		protected:
			
		private:
			Calibration* cal;
			
			/** The name and path of the calibration file. */
			::std::string calFilePath;
			
			Comedi comedi;
			
			/** The number of the calibration within the file (usually 1). */
			unsigned short int index;
			
			float values[6];
			
			float voltages[6];
		};
	}
}

#endif // RL_HAL_ATI_H
