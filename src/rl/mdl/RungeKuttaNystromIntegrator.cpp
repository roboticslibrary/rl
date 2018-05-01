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
			dy = dy0 + 1.0f / 3.0f * (k1 + 2 * k2 + 2 * k3 + k4);
			
			this->dynamic->setPosition(y);
			this->dynamic->setVelocity(dy);
			this->dynamic->setAcceleration(f);
		}
	}
}
