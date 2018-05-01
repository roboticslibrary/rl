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
		class RungeKuttaNystromIntegrator : public Integrator
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
