#ifndef RL_MDL_JACOBIANINVERSEKINEMATICS_H
#define RL_MDL_JACOBIANINVERSEKINEMATICS_H

#include <chrono>
#include <random>
#include <utility>

#include "InverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		class JacobianInverseKinematics : public InverseKinematics
		{
		public:
			JacobianInverseKinematics(Kinematic* kinematic);
			
			virtual ~JacobianInverseKinematics();
			
			bool solve();
			
			::rl::math::Real delta;
			
			::std::chrono::nanoseconds duration;
			
			::rl::math::Real epsilon;
			
			::std::size_t iterations;
			
			bool svd;
			
			bool transpose;
			
		protected:
			
		private:
			::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
			
			::std::mt19937 randEngine;
		};
	}
}

#endif // RL_MDL_JACOBIANINVERSEKINEMATICS_H
