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

#include "Dynamic.h"
#include "RungeKuttaNystromIntegrator.h"

namespace rl
{
	namespace mdl
	{
		RungeKuttaNystromIntegrator::RungeKuttaNystromIntegrator(Dynamic* dynamic) :
			Integrator(dynamic)
		{
		}
		
		RungeKuttaNystromIntegrator::~RungeKuttaNystromIntegrator()
		{
		}
		
		void
		RungeKuttaNystromIntegrator::integrate(const ::rl::math::Real& dt)
		{
			::rl::math::Vector y0 = this->dynamic->getPosition();
			::rl::math::Vector dy0 = this->dynamic->getVelocity();
			
			this->dynamic->forwardDynamics();
			
			::rl::math::Vector f = this->dynamic->getAcceleration();
			
			// k1 = dt / 2 * f
			::rl::math::Vector k1 = dt / 2 * f;
			
			// y_0 + dt / 2 * dy_0 + dt / 4 * k_1
			::rl::math::Vector y = y0 + dt / 2 * dy0 + dt / 4 * k1;
			// dy_0 + k1
			::rl::math::Vector dy = dy0 + k1;
			
			this->dynamic->setPosition(y);
			this->dynamic->setVelocity(dy);
			this->dynamic->forwardDynamics();
			
			// k2 = dt / 2 * f
			::rl::math::Vector k2 = dt / 2 * this->dynamic->getAcceleration();
			
			// dy_0 + k_2
			dy = dy0 + k2;
			
			this->dynamic->setVelocity(dy);
			this->dynamic->forwardDynamics();
			
			// k3 = dt / 2 * f
			::rl::math::Vector k3 = dt / 2 * this->dynamic->getAcceleration();
			
			// y_0 + dt * dy_0 + dt * k_3
			y = y0 + dt * dy0 + dt * k3;
			// dy_0 + 2 * k_3
			dy = dy0 + 2 * k3;
			
			this->dynamic->setPosition(y);
			this->dynamic->setVelocity(dy);
			this->dynamic->forwardDynamics();
			
			// k4 = dt / 2 * f
			::rl::math::Vector k4 = dt / 2 * this->dynamic->getAcceleration();
			
			// y_0 + dy_0 * dt + dt / 3 * (k_1 + k_2 + k_3)
			y = y0 + dy0 * dt + dt / 3 * (k1 + k2 + k3);
			// dy_0 + 1 / 3 * (k_1 + 2 * k_2 + 2 * k_3 + k_4)
			dy = dy0 + static_cast<::rl::math::Real>(1) / static_cast<::rl::math::Real>(3) * (k1 + 2 * k2 + 2 * k3 + k4);
			
			this->dynamic->setPosition(y);
			this->dynamic->setVelocity(dy);
			this->dynamic->setAcceleration(f);
		}
	}
}
