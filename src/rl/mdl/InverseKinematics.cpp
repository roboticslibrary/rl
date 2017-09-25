#include "InverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		InverseKinematics::InverseKinematics(Kinematic* kinematic) :
			goals(),
			kinematic(kinematic)
		{
		}
		
		InverseKinematics::~InverseKinematics()
		{
		}
	}
}
