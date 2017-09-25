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

#ifndef RL_MDL_METRIC_H
#define RL_MDL_METRIC_H

#include "Model.h"

namespace rl
{
	namespace mdl
	{
		class Metric : public Model
		{
		public:
			Metric();
			
			virtual ~Metric();
			
			void clip(::rl::math::Vector& q) const;
			
			Model* clone() const;
			
			::rl::math::Real distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;
			
			void interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const;
			
			::rl::math::Real inverseOfTransformedDistance(const ::rl::math::Real& d) const;
			
			bool isValid(const ::rl::math::Vector& q) const;
			
			::rl::math::Real maxDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const;
			
			::rl::math::Real minDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const;
			
			::rl::math::Real minDistanceToRectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cuttingDimension) const;
			
			::rl::math::Real newDistance(const ::rl::math::Real& dist, const ::rl::math::Real& oldOff, const ::rl::math::Real& newOff, const int& cuttingDimension) const;
			
			void normalize(::rl::math::Vector& q) const;
			
			void step(const ::rl::math::Vector& q1, const ::rl::math::Vector& qdot, ::rl::math::Vector& q2) const;
			
			::rl::math::Real transformedDistance(const ::rl::math::Real& d) const;
			
			::rl::math::Real transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;
			
			::rl::math::Real transformedDistance(const ::rl::math::Real& q1, const ::rl::math::Real& q2, const ::std::size_t& i) const;
			
		protected:
			
		private:
			
		};
	}
}

#endif // RL_MDL_METRIC_H
