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

#ifndef RL_MDL_SIXDOF_H
#define RL_MDL_SIXDOF_H

#include "Joint.h"

namespace rl
{
	namespace mdl
	{
		class RL_MDL_EXPORT SixDof : public Joint
		{
		public:
			SixDof();
			
			virtual ~SixDof();
			
			void clamp(::rl::math::VectorRef q) const;
			
			::rl::math::Real distance(const ::rl::math::ConstVectorRef& q1, const ::rl::math::ConstVectorRef& q2) const;
			
			void generatePositionGaussian(const ::rl::math::ConstVectorRef& rand, const ::rl::math::ConstVectorRef& mean, const ::rl::math::ConstVectorRef& sigma, ::rl::math::VectorRef q) const;
			
			void generatePositionUniform(const ::rl::math::ConstVectorRef& rand, ::rl::math::VectorRef q) const;
			
			void generatePositionUniform(const ::rl::math::ConstVectorRef& rand, const ::rl::math::ConstVectorRef& min, const ::rl::math::ConstVectorRef& max, ::rl::math::VectorRef q) const;
			
			void interpolate(const ::rl::math::ConstVectorRef& q1, const ::rl::math::ConstVectorRef& q2, const ::rl::math::Real& alpha, ::rl::math::VectorRef q) const;
			
			bool isValid(const ::rl::math::ConstVectorRef& q) const;
			
			void normalize(::rl::math::VectorRef q) const;
			
			void setPosition(const ::rl::math::ConstVectorRef& q);
			
			void step(const ::rl::math::ConstVectorRef& q1, const ::rl::math::ConstVectorRef& dq, ::rl::math::VectorRef q2) const;
			
			::rl::math::Real transformedDistance(const ::rl::math::ConstVectorRef& q1, const ::rl::math::ConstVectorRef& q2) const;
			
		protected:
			
		private:
			
		};
	}
}

#endif // RL_MDL_SIXDOF_H
