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

#ifndef RL_MDL_ITERATIVEINVERSEKINEMATICS_H
#define RL_MDL_ITERATIVEINVERSEKINEMATICS_H

#include <chrono>

#include "InverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		class RL_MDL_EXPORT IterativeInverseKinematics : public InverseKinematics
		{
		public:
			IterativeInverseKinematics(Kinematic* kinematic);
			
			virtual ~IterativeInverseKinematics();
			
			const ::std::chrono::nanoseconds& getDuration() const;
			
			const ::rl::math::Real& getEpsilon() const;
			
			const ::std::size_t& getIterations() const;
			
			const ::std::size_t& getRandomRestarts() const;
			
			void setDuration(const ::std::chrono::nanoseconds& duration);
			
			virtual void setEpsilon(const ::rl::math::Real& epsilon);
			
			void setIterations(const ::std::size_t& iterations);
			
			void setRandomRestarts(const ::std::size_t& restarts);
			
		protected:
			
		private:
			::std::chrono::nanoseconds duration;
			
			::rl::math::Real epsilon;
			
			::std::size_t iterations;
			
			::std::size_t restarts;
		};
	}
}

#endif // RL_MDL_ITERATIVEINVERSEKINEMATICS_H
