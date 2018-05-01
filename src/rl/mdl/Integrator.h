#ifndef RL_MDL_INTEGRATOR_H
#define RL_MDL_INTEGRATOR_H

#include <rl/math/Real.h>

namespace rl
{
	namespace mdl
	{
		class Dynamic;
		
		class Integrator
		{
		public:
			Integrator(Dynamic* dynamic);
			
			virtual ~Integrator();
			
			/**
			 * @param[in] dt Integration time step \f$\Delta t\f$
			 */
			virtual void integrate(const ::rl::math::Real& dt) = 0;
			
		protected:
			Dynamic* dynamic;
			
		private:
			
		};
	}
}

#endif // RL_MDL_INTEGRATOR_H
