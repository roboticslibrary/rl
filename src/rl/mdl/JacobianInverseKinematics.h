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

#ifndef RL_MDL_JACOBIANINVERSEKINEMATICS_H
#define RL_MDL_JACOBIANINVERSEKINEMATICS_H

#include <chrono>
#include <random>
#include <utility>

#include "IterativeInverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		/**
		 * Iterative inverse kinematics using Jacobian with random restarts.
		 *
		 * Samuel R. Buss, Introduction to Inverse Kinematics with Jacobian
		 * Transpose, Pseudoinverse and Damped Least Squares Methods, 2009.
		 *
		 * https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
		 */
		class RL_MDL_EXPORT JacobianInverseKinematics : public IterativeInverseKinematics
		{
		public:
			enum class Method
			{
				dls,
				svd,
				transpose
			};
			
			RL_MDL_DEPRECATED static constexpr Method METHOD_DLS = Method::dls;
			RL_MDL_DEPRECATED static constexpr Method METHOD_SVD = Method::svd;
			RL_MDL_DEPRECATED static constexpr Method METHOD_TRANSPOSE = Method::transpose;
			
			JacobianInverseKinematics(Kinematic* kinematic);
			
			virtual ~JacobianInverseKinematics();
			
			const ::rl::math::Real& getDelta() const;
			
			const Method& getMethod() const;
			
			const ::std::size_t& getSteps() const;
			
			void seed(const ::std::mt19937::result_type& value);
			
			void setDelta(const ::rl::math::Real& delta);
			
			void setMethod(const Method& method);
			
			void setSteps(const ::std::size_t& steps);
			
			bool solve();
			
		protected:
			
		private:
			::rl::math::Real delta;
			
			Method method;
			
			::std::uniform_real_distribution<::rl::math::Real> randDistribution;
			
			::std::mt19937 randEngine;
			
			::std::size_t steps;
		};
	}
}

#endif // RL_MDL_JACOBIANINVERSEKINEMATICS_H
