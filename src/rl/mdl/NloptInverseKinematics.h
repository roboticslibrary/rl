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

#ifndef RL_MDL_NLOPTINVERSEKINEMATICS_H
#define RL_MDL_NLOPTINVERSEKINEMATICS_H

#include <chrono>
#include <nlopt.h>
#include <random>
#include <utility>

#include "IterativeInverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		class RL_MDL_EXPORT NloptInverseKinematics : public IterativeInverseKinematics
		{
		public:
			NloptInverseKinematics(Kinematic* kinematic);
			
			virtual ~NloptInverseKinematics();
			
			const ::rl::math::Real& getDelta() const;
			
			const ::rl::math::Real& getEpsilonRotation() const;
			
			const ::rl::math::Real& getEpsilonTranslation() const;
			
			const double& getTolerance() const;
			
			void setDelta(const::rl::math::Real& delta);
			
			void setEpsilonRotation(const::rl::math::Real& epsilonRotation);
			
			void setEpsilonTranslation(const::rl::math::Real& epsilonTranslation);
			
			void setLowerBound(const ::rl::math::Vector& lb);
			
			void setTolerance(const double& tolerance);
			
			void setUpperBound(const ::rl::math::Vector& ub);
			
			bool solve();
			
		protected:
			
		private:
			::rl::math::Real error(const ::rl::math::Vector& q);
			
			static ::rl::math::Real f(unsigned n, const double* x, double* grad, void* data);
			
			::rl::math::Real delta;
			
			::rl::math::Real epsilonRotation;
			
			::rl::math::Real epsilonTranslation;
			
			::rl::math::Vector lb;
			
			::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
			
			::std::mt19937 randEngine;
			
			double tolerance;
			
			::rl::math::Vector ub;
		};
	}
}

#endif // RL_MDL_NLOPTINVERSEKINEMATICS_H
