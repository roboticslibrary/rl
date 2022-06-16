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

#include "IterativeInverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		IterativeInverseKinematics::IterativeInverseKinematics(Kinematic* kinematic) :
			InverseKinematics(kinematic),
			duration(::std::chrono::milliseconds(1000)),
			epsilon(static_cast<::rl::math::Real>(1.0e-6)),
			iterations(10000),
			restarts(::std::numeric_limits<::std::size_t>::max())
		{
		}
		
		IterativeInverseKinematics::~IterativeInverseKinematics()
		{
		}
		
		const ::std::chrono::nanoseconds&
		IterativeInverseKinematics::getDuration() const
		{
			return this->duration;
		}
		
		const ::rl::math::Real&
		IterativeInverseKinematics::getEpsilon() const
		{
			return this->epsilon;
		}
		
		const ::std::size_t&
		IterativeInverseKinematics::getIterations() const
		{
			return this->iterations;
		}
		
		const ::std::size_t&
		IterativeInverseKinematics::getRandomRestarts() const
		{
			return this->restarts;
		}
		
		void
		IterativeInverseKinematics::setDuration(const ::std::chrono::nanoseconds& duration)
		{
			this->duration = duration;
		}
		
		void
		IterativeInverseKinematics::setEpsilon(const ::rl::math::Real& epsilon)
		{
			this->epsilon = epsilon;
		}
		
		void
		IterativeInverseKinematics::setIterations(const ::std::size_t& iterations)
		{
			this->iterations = iterations;
		}
		
		void
		IterativeInverseKinematics::setRandomRestarts(const ::std::size_t& restarts)
		{
			this->restarts = restarts;
		}
	}
}
