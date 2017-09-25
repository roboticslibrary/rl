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

#ifndef RL_PLAN_GAUSSIANSAMPLER_H
#define RL_PLAN_GAUSSIANSAMPLER_H

#include "UniformSampler.h"

namespace rl
{
	namespace plan
	{
		/**
		 * Gaussian sampling strategy.
		 * 
		 * Val&eacute;rie Boor, Mark H. Overmars, and A. Frank van der Stappen.
		 * The Gaussian sampling strategy for probabilistic roadmap planners.
		 * In Proceedings of the IEEE International Conference on Robotics and
		 * Automation, pages 1018-1023, Detroit, MI, USA, May 1999.
		 * 
		 * http://dx.doi.org/10.1109/ROBOT.1999.772447
		 */
		class GaussianSampler : public UniformSampler
		{
		public:
			GaussianSampler();
			
			virtual ~GaussianSampler();
			
			::rl::math::Vector generateCollisionFree();
			
			virtual void seed(const ::std::mt19937::result_type& value);
			
			::rl::math::Vector* sigma;
			
		protected:
			::std::normal_distribution< ::rl::math::Real>::result_type gauss();
			
			::std::normal_distribution< ::rl::math::Real> gaussDistribution;
			
			::std::mt19937 gaussEngine;
			
		private:
			
		};
	}
}

#endif // RL_PLAN_GAUSSIANSAMPLER_H
