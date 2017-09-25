#ifndef RL_MDL_INVERSEKINEMATICS_H
#define RL_MDL_INVERSEKINEMATICS_H

#include <vector>
#include <rl/math/Transform.h>

namespace rl
{
	namespace mdl
	{
		class Kinematic;
		
		class InverseKinematics
		{
		public:
			InverseKinematics(Kinematic* kinematic);
			
			virtual ~InverseKinematics();
			
			virtual bool solve() = 0;
			
			::std::vector< ::std::pair< ::rl::math::Transform, ::std::size_t>> goals;
			
		protected:
			Kinematic* kinematic;
			
		private:
			
		};
	}
}

#endif // RL_MDL_INVERSEKINEMATICS_H
