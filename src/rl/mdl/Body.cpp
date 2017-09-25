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

#include <rl/math/Rotation.h>

#include "Body.h"

namespace rl
{
	namespace mdl
	{
		Body::Body() :
			Frame(),
			cm(),
			collision(true),
			fX(),
			ic(),
			m(1),
			selfcollision()
		{
			this->cm.setZero(); // TODO
			this->fX.setZero(); // TODO
			this->ic.setIdentity(); // TODO
		}
		
		Body::~Body()
		{
		}
		
		void
		Body::forwardAcceleration()
		{
		}
		
		void
		Body::forwardDynamics1()
		{
			this->iA = this->i;
			
			// v x I * v - X_0 * f^x
			this->pA = this->v.cross(this->i * this->v) - this->x * this->fX;
		}
		
		void
		Body::forwardDynamics2()
		{
		}
		
		void
		Body::forwardDynamics3()
		{
		}
		
		void
		Body::forwardPosition()
		{
		}
		
		void
		Body::forwardVelocity()
		{
		}
		
		void
		Body::inverseDynamics1()
		{
			// I * a + v x I * v - X_0 * f^x
			this->f = this->i * this->a + this->v.cross(this->i * this->v) - this->x * this->fX;
		}
		
		void
		Body::inverseDynamics2()
		{
		}
		
		void
		Body::setCenterOfMass(const ::rl::math::Real& x, const ::rl::math::Real& y, const ::rl::math::Real& z)
		{
			this->cm.x() = x;
			this->cm.y() = y;
			this->cm.z() = z;
			
			// m * c
			this->i.cog() = this->m * this->cm;
			
			// Ic - m * cx * cx
			this->i.inertia() = this->ic - this->m * this->cm.cross33() * this->cm.cross33();
		}
		
		void
		Body::setMass(const ::rl::math::Real& m)
		{
			this->m = m;
			
			this->i.mass() = m;
			
			// m * c
			this->i.cog() = this->m * this->cm;
			
			// Ic - m * cx * cx
			this->i.inertia() = this->ic - this->m * this->cm.cross33() * this->cm.cross33(); // TODO
		}
		
		void
		Body::setInertia(const ::rl::math::Real& xx, const ::rl::math::Real& yy, const ::rl::math::Real& zz, const ::rl::math::Real& yz, const ::rl::math::Real& xz, const ::rl::math::Real& xy)
		{
			::rl::math::Vector6 ic;
			ic << xx, yy, zz, yz, xz, xy;
			this->ic = ic.voigt33();
			
			// Ic - m * cx * cx
			this->i.inertia() = this->ic - this->m * this->cm.cross33() * this->cm.cross33();
		}
	}
}
