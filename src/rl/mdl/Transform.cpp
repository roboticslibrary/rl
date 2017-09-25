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

#include "Frame.h"
#include "Transform.h"

namespace rl
{
	namespace mdl
	{
		Transform::Transform() :
			Element(),
			in(nullptr),
			out(nullptr),
			t(),
			x(),
			descriptor()
		{
			this->t.setIdentity(); // TODO
			this->x.setIdentity(); // TODO
		}
		
		Transform::~Transform()
		{
		}
		
		void
		Transform::forwardAcceleration()
		{
			this->out->a = this->x * this->in->a;
		}
		
		void
		Transform::forwardDynamics1()
		{
			this->forwardVelocity();
			this->out->c.setZero(); // TODO
		}
		
		void
		Transform::forwardDynamics2()
		{
			// I^A + X^* * I^a * X
			this->in->iA = this->in->iA + this->x / this->out->iA;
			
			// p^A + I^a * c
			::rl::math::ForceVector pa;
			pa = this->out->pA + this->out->iA * this->out->c; // TODO
			
			// p^A + X^* * p^a
			this->in->pA = this->in->pA + this->x / pa;
		}
		
		void
		Transform::forwardDynamics3()
		{
			// X * a + c
			this->out->a = this->x * this->in->a + this->out->c;
		}
		
		void
		Transform::forwardPosition()
		{
			this->out->t = this->in->t * this->t;
			this->out->x = this->x * this->in->x;
		}
		
		void
		Transform::forwardVelocity()
		{
			this->out->v = this->x * this->in->v;
		}
		
		const Transform::Edge&
		Transform::getEdgeDescriptor() const
		{
			return this->descriptor;
		}
		
		void
		Transform::inverseDynamics1()
		{
			this->forwardVelocity();
			this->forwardAcceleration();
		}
		
		void
		Transform::inverseDynamics2()
		{
			this->inverseForce();
		}
		
		void
		Transform::inverseForce()
		{
			// f + X * f
			this->in->f = this->in->f + this->x / this->out->f;
		}
		
		void
		Transform::setEdgeDescriptor(const Edge& descriptor)
		{
			this->descriptor = descriptor;
		}
	}
}
