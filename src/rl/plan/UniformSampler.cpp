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

#include <chrono>

#include "SimpleModel.h"
#include "UniformSampler.h"

namespace rl
{
	namespace plan
	{
		UniformSampler::UniformSampler() :
			Sampler(),
			randDistribution(0, 1),
			randEngine(::std::random_device()())
		{
		}
		
		UniformSampler::~UniformSampler()
		{
		}
		
		::rl::math::Vector
		UniformSampler::generate()
		{
			::rl::math::Vector rand(this->model->getDof());
			
			for (::std::size_t i = 0; i < this->model->getDof(); ++i)
			{
				rand(i) = this->rand();
			}
			
			return this->model->generatePositionUniform(rand);
		}
		
		::std::uniform_real_distribution< ::rl::math::Real>::result_type
		UniformSampler::rand()
		{
			return this->randDistribution(this->randEngine);
		}
		
		void
		UniformSampler::seed(const ::std::mt19937::result_type& value)
		{
			this->randEngine.seed(value);
		}
	}
}
