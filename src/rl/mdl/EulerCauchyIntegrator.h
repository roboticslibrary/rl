#ifndef RL_MDL_EULERCAUCHYINTEGRATOR_H
#define RL_MDL_EULERCAUCHYINTEGRATOR_H

#include "Integrator.h"

namespace rl
{
	namespace mdl
	{
		/**
		 * Integration via Euler-Cauchy.
		 * 
		 * \f[ \vec{q}_{i + 1} = \vec{q}_{i} + \Delta t \, \dot{\vec{q}}_{i} \f]
		 * \f[ \dot{\vec{q}}_{i + 1} = \dot{\vec{q}}_{i} +  \Delta t \, f(t_{i}, \vec{q}_{i}, \dot{\vec{q}}_{i}) \f]
		 * \f[ \ddot{\vec{q}} = f(t, \vec{q}, \dot{\vec{q}}) \f]
		 * \f[ t_{i + 1} = t_{i} + \Delta t \f]
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
		class EulerCauchyIntegrator : public Integrator
		{
		public:
			EulerCauchyIntegrator(Dynamic* dynamic);
			
			virtual ~EulerCauchyIntegrator();
			
			void integrate(const ::rl::math::Real& dt);
			
		protected:
			
		private:
			
		};
	}
}

#endif // RL_MDL_EULERCAUCHYINTEGRATOR_H
