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

#ifndef RL_MDL_NLOPTINVERSEKINEMATICS_H
#define RL_MDL_NLOPTINVERSEKINEMATICS_H

#include <chrono>
#include <memory>
#include <nlopt.h>
#include <random>
#include <utility>

#include "Exception.h"
#include "IterativeInverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		class RL_MDL_EXPORT NloptInverseKinematics : public IterativeInverseKinematics
		{
		public:
			class Exception : public ::rl::mdl::Exception
			{
			public:
				Exception(const ::nlopt_result& result);
				
				virtual ~Exception() throw();
				
				static void check(const ::nlopt_result& result);
				
				::nlopt_result getResult() const;
				
				virtual const char* what() const throw();
				
			protected:
				
			private:
				::nlopt_result result;
			};
			
			NloptInverseKinematics(Kinematic* kinematic);
			
			virtual ~NloptInverseKinematics();
			
			::rl::math::Real getFunctionToleranceAbsolute() const;
			
			::rl::math::Real getFunctionToleranceRelative() const;
			
			const ::rl::math::Vector& getLowerBound() const;
			
			::rl::math::Vector getOptimizationToleranceAbsolute() const;
			
			::rl::math::Real getOptimizationToleranceRelative() const;
			
			const ::rl::math::Vector& getUpperBound() const;
			
			void seed(const ::std::mt19937::result_type& value);
			
			void setEpsilon(const::rl::math::Real& epsilon);
			
			void setFunctionToleranceAbsolute(const ::rl::math::Real& functionToleranceAbsolute);
			
			void setFunctionToleranceRelative(const ::rl::math::Real& functionToleranceRelative);
			
			void setLowerBound(const ::rl::math::Vector& lb);
			
			void setOptimizationToleranceAbsolute(const ::rl::math::Real& optimizationToleranceAbsolute);
			
			void setOptimizationToleranceAbsolute(const ::rl::math::Vector& optimizationToleranceAbsolute);
			
			void setOptimizationToleranceRelative(const ::rl::math::Real& optimizationToleranceRelative);
			
			void setUpperBound(const ::rl::math::Vector& ub);
			
			bool solve();
			
		protected:
			
		private:
			static double f(unsigned int n, const double* x, double* grad, void* data);
			
			::std::size_t iteration;
			
			::rl::math::Vector lb;
			
			::std::unique_ptr< ::nlopt_opt_s, decltype(&::nlopt_destroy)> opt;
			
			::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
			
			::std::mt19937 randEngine;
			
			::rl::math::Vector ub;
		};
	}
}

#endif // RL_MDL_NLOPTINVERSEKINEMATICS_H
