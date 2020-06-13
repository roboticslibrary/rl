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

#ifndef RL_MDL_RUNGEKUTTANYSTROMINTEGRATOR_H
#define RL_MDL_RUNGEKUTTANYSTROMINTEGRATOR_H

#include "Integrator.h"

namespace rl
{
	namespace mdl
	{
		/**
		 * Integration via Runge-Kutta-Nystr&ouml;m.
		 *
		 * \f[ \vec{q}_{i + 1} = \vec{q}_{i} + \Delta t \, \dot{\vec{q}}_{i} + \frac{\Delta t}{3} (\vec{k}_{1} + \vec{k}_{2} + \vec{k}_{3}) \f]
		 * \f[ \dot{\vec{q}}_{i + 1} = \dot{\vec{q}}_{i} + \frac{1}{3} (\vec{k}_{1} + 2 \vec{k}_{2} + 2 \vec{k}_{3} + \vec{k}_{4}) \f]
		 * \f[ \ddot{\vec{q}} = f(t, \vec{q}, \dot{\vec{q}}) \f]
		 * \f[ t_{i + 1} = t_{i} + \Delta t \f]
		 * \f[ \vec{k}_{1} = \frac{\Delta t}{2} \, f(t_{i}, \vec{q}_{i}, \dot{\vec{q}}_{i}) \f]
		 * \f[ \vec{k}_{2} = \frac{\Delta t}{2} \, f(t_{i} + \frac{\Delta t}{2}, \vec{q}_{i} + \frac{\Delta t}{2} \, \dot{\vec{q}}_{i} + \frac{\Delta t}{4} \, \vec{k}_{1}, \dot{\vec{q}}_{i} + \vec{k}_{1}) \f]
		 * \f[ \vec{k}_{3} = \frac{\Delta t}{2} \, f(t_{i} + \frac{\Delta t}{2}, \vec{q}_{i} + \frac{\Delta t}{2} \, \dot{\vec{q}}_{i} + \frac{\Delta t}{4} \, \vec{k}_{1}, \dot{\vec{q}}_{i} + \vec{k}_{2}) \f]
		 * \f[ \vec{k}_{4} = \frac{\Delta t}{2} \, f(t_{i} + \Delta t, \vec{q}_{i} + \Delta t \, \dot{\vec{q}}_{i} + \Delta t \, \vec{k}_{3}, \dot{\vec{q}}_{i} + 2 \vec{k}_{3}) \f]
		 *
		 * @pre Dynamic::setPosition()
		 * @pre Dynamic::setVelocity()
		 * @pre Dynamic::setTorque()
		 * @post Dynamic::getPosition()
		 * @post Dynamic::getVelocity()
		 * @post Dynamic::getAcceleration()
		 *
		 * @see Dynamic::forwardDynamics()
		 */
		class RL_MDL_EXPORT RungeKuttaNystromIntegrator : public Integrator
		{
		public:
			RungeKuttaNystromIntegrator(Dynamic* dynamic);
			
			virtual ~RungeKuttaNystromIntegrator();
			
			void integrate(const ::rl::math::Real& dt);
			
		protected:
			
		private:
			
		};
	}
}

#endif // RL_MDL_RUNGEKUTTANYSTROMINTEGRATOR_H
