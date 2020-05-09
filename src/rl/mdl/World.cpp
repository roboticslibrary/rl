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

#include "World.h"

namespace rl
{
	namespace mdl
	{
		World::World() :
			Frame(),
			gravity(::rl::math::Vector3::Zero())
		{
		}
		
		World::~World()
		{
		}
		
		void
		World::forwardAcceleration()
		{
			this->a.angular().setZero();
			this->a.linear().setZero();
		}
		
		void
		World::forwardDynamics1()
		{
			this->a.angular().setZero();
			this->a.linear() = this->gravity;
			this->iA.setZero();
			this->pA.setZero();
		}
		
		const ::rl::math::Vector3&
		World::getGravity() const
		{
			return this->gravity;
		}
		
		void
		World::inverseDynamics1()
		{
			this->a.angular().setZero();
			this->a.linear() = this->gravity;
			this->f.setZero();
		}
		
		void
		World::setGravity(const ::rl::math::Vector3& gravity)
		{
			this->gravity = gravity;
		}
	}
}
