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

#include <queue>

#include "RecursiveVerifier.h"
#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		RecursiveVerifier::RecursiveVerifier() :
			Verifier()
		{
		}
		
		RecursiveVerifier::~RecursiveVerifier()
		{
		}
		
		bool
		RecursiveVerifier::isColliding(const ::rl::math::Vector& u, const ::rl::math::Vector& v, const ::rl::math::Real& d)
		{
			assert(u.size() == this->model->getDofPosition());
			assert(v.size() == this->model->getDofPosition());
			
			::std::size_t steps = this->getSteps(d);
			
			if (steps > 1)
			{
				::std::queue< ::std::pair< ::std::size_t, ::std::size_t>> queue;
				
#if (defined(_MSC_VER) && _MSC_VER < 1800) || (defined(__GNUC__) && __GNUC__ == 4 && __GNUC_MINOR__ < 8)
				queue.push(::std::make_pair(1, steps - 1));
#else
				queue.emplace(::std::piecewise_construct, ::std::forward_as_tuple(1), ::std::forward_as_tuple(steps - 1));
#endif
				
				::rl::math::Vector inter(u.size());
				
				while (!queue.empty())
				{
					::std::size_t midpoint = (queue.front().first + queue.front().second) / 2;
					
					this->model->interpolate(u, v, static_cast< ::rl::math::Real>(midpoint) / static_cast< ::rl::math::Real>(steps), inter);
					
					if (!this->model->isValid(inter))
					{
						return true;
					}
					
					if (this->model->isColliding(inter))
					{
						return true;
					}
					
					if (queue.front().first < midpoint)
					{
#if (defined(_MSC_VER) && _MSC_VER < 1800) || (defined(__GNUC__) && __GNUC__ == 4 && __GNUC_MINOR__ < 8)
						queue.push(::std::make_pair(queue.front().first, midpoint - 1));
#else
						queue.emplace(::std::piecewise_construct, ::std::forward_as_tuple(queue.front().first), ::std::forward_as_tuple(midpoint - 1));
#endif
					}
					
					if (queue.front().second > midpoint)
					{
#if (defined(_MSC_VER) && _MSC_VER < 1800) || (defined(__GNUC__) && __GNUC__ == 4 && __GNUC_MINOR__ < 8)
						queue.push(::std::make_pair(midpoint + 1, queue.front().second));
#else
						queue.emplace(::std::piecewise_construct, ::std::forward_as_tuple(midpoint + 1), ::std::forward_as_tuple(queue.front().second));
#endif
					}
					
					queue.pop();
				}
			}
			
			return false;
		}
	}
}
