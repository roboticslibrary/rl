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

namespace rl
{
	namespace mdl
	{
		Frame::Frame() :
			Element(),
			a(::rl::math::MotionVector::Zero()),
			c(::rl::math::MotionVector::Zero()),
			f(::rl::math::ForceVector::Zero()),
			i(::rl::math::RigidBodyInertia::Identity()),
			iA(::rl::math::ArticulatedBodyInertia::Identity()),
			pA(::rl::math::ForceVector::Zero()),
			v(::rl::math::MotionVector::Zero()),
			x(::rl::math::PlueckerTransform::Identity()),
			descriptor()
		{
		}
		
		Frame::~Frame()
		{
		}
		
		void
		Frame::forwardAcceleration()
		{
		}
		
		void
		Frame::forwardDynamics1()
		{
			this->iA.setZero();
			this->pA.setZero();
		}
		
		void
		Frame::forwardDynamics2()
		{
		}
		
		void
		Frame::forwardDynamics3()
		{
		}
		
		void
		Frame::forwardPosition()
		{
		}
		
		void
		Frame::forwardVelocity()
		{
		}
		
		const Frame::Vertex&
		Frame::getVertexDescriptor() const
		{
			return this->descriptor;
		}
		
		void
		Frame::inverseDynamics1()
		{
			this->f.setZero();
		}
		
		void
		Frame::inverseDynamics2()
		{
		}
		
		void
		Frame::inverseForce()
		{
		}
		
		void
		Frame::setVertexDescriptor(const Vertex& descriptor)
		{
			this->descriptor = descriptor;
		}
	}
}
