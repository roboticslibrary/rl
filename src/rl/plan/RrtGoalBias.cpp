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

#include <rl/util/Timer.h>

#include "RrtGoalBias.h"
#include "Sampler.h"

namespace rl
{
	namespace plan
	{
		RrtGoalBias::RrtGoalBias() :
			Rrt(),
			probability(0.05f),
			rand(
				::boost::mt19937(static_cast< ::boost::mt19937::result_type >(::rl::util::Timer::now() * 1000000.0f)),
				::boost::uniform_real< ::rl::math::Real >(0.0f, 1.0f)
			)
		{
		}
		
		RrtGoalBias::~RrtGoalBias()
		{
		}
		
		void
		RrtGoalBias::choose(::rl::math::Vector& chosen)
		{
			if (this->rand() > this->probability)
			{
				Rrt::choose(chosen);
			}
			else
			{
				chosen = *this->goal;
			}
		}
		
		::std::string
		RrtGoalBias::getName() const
		{
			return "RRT Goal Bias";
		}
		
		void
		RrtGoalBias::seed(const ::boost::mt19937::result_type& value)
		{
			this->rand.engine().seed(value);
		}
	}
}
