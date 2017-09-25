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

#ifndef RL_MDL_BODY_H
#define RL_MDL_BODY_H

#include <unordered_set>

#include "Frame.h"

namespace rl
{
	namespace mdl
	{
		class Body : public Frame
		{
		public:
			Body();
			
			virtual ~Body();
			
			void forwardAcceleration();
			
			void forwardDynamics1();
			
			void forwardDynamics2();
			
			void forwardDynamics3();
			
			void forwardPosition();
			
			void forwardVelocity();
			
			void inverseDynamics1();
			
			void inverseDynamics2();
			
			void setCenterOfMass(const ::rl::math::Real& x, const ::rl::math::Real& y, const ::rl::math::Real& z);
			
			void setMass(const ::rl::math::Real& m);
			
			void setInertia(const ::rl::math::Real& xx, const ::rl::math::Real& yy, const ::rl::math::Real& zz, const ::rl::math::Real& yz, const ::rl::math::Real& xz, const ::rl::math::Real& xy);
			
			::rl::math::Vector3 cm;
			
			bool collision;
			
			::rl::math::ForceVector fX;
			
			::rl::math::Matrix33 ic;
			
			::rl::math::Real m;
			
			::std::unordered_set<Body*> selfcollision;
			
		protected:
			
		private:
			
		};
	}
}

#endif // RL_MDL_BODY_H
