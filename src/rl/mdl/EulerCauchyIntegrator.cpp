#include "Dynamic.h"
#include "EulerCauchyIntegrator.h"

namespace rl
{
	namespace mdl
	{
		EulerCauchyIntegrator::EulerCauchyIntegrator(Dynamic* dynamic) :
			Integrator(dynamic)
		{
		}
		
		EulerCauchyIntegrator::~EulerCauchyIntegrator()
		{
		}
		
		void
		EulerCauchyIntegrator::integrate(const ::rl::math::Real& dt)
		{
			::rl::math::Vector y = this->dynamic->getPosition();
			::rl::math::Vector dy = this->dynamic->getVelocity();
			
			this->dynamic->forwardDynamics();
			
			// f
			::rl::math::Vector f = this->dynamic->getAcceleration();
			
			// y_0 + dy_0 * dt
			y += dt * dy;
			// dy_0 + f * dt
			dy += dt * f;
			
			this->dynamic->setPosition(y);
			this->dynamic->setVelocity(dy);
			this->dynamic->setAcceleration(f);
		}
	}
}
