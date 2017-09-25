#ifndef RL_MDL_NLOPTINVERSEKINEMATICS_H
#define RL_MDL_NLOPTINVERSEKINEMATICS_H

#include <chrono>
#include <nlopt.h>
#include <random>
#include <utility>

#include "InverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		class NloptInverseKinematics : public InverseKinematics
		{
		public:
			NloptInverseKinematics(Kinematic* kinematic);
			
			virtual ~NloptInverseKinematics();
			
			bool solve();
			
			::rl::math::Real delta;
			
			::std::chrono::nanoseconds duration;
			
			::rl::math::Real epsilonRotation;
			
			::rl::math::Real epsilonTranslation;
			
			double tolerance;
			
		protected:
			
		private:
			static void check(const nlopt_result& ret);
			
			::rl::math::Real error(const ::rl::math::Vector& q);
			
			static ::rl::math::Real f(unsigned n, const double* x, double* grad, void* data);
			
			::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
			
			::std::mt19937 randEngine;
		};
	}
}

#endif // RL_MDL_NLOPTINVERSEKINEMATICS_H
