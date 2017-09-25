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

#ifndef RL_PLAN_PLANNER_H
#define RL_PLAN_PLANNER_H

#include <chrono>
#include <string>
#include <rl/math/Vector.h>

#include "VectorList.h"

namespace rl
{
	/**
	 * Collision-free path planning. 
	 */
	namespace plan
	{
		class SimpleModel;
		class Viewer;
		
		class Planner
		{
		public:
			Planner();
			
			virtual ~Planner();
			
			virtual ::std::string getName() const = 0;
			
			/**
			 * Get solution path.
			 * 
			 * @pre solve()
			 */
			virtual VectorList getPath() = 0;
			
			/**
			 * Reset planner.
			 */
			virtual void reset() = 0;
			
			/**
			 * Find collision free path.
			 */
			virtual bool solve() = 0;
			
			/**
			 * Vertify that start and goal configuration are within joint limits and collision free.
			 */
			bool verify();
			
			/** Upper bound for search. */
			::std::chrono::steady_clock::duration duration;
			
			/** Goal configuration. */
			::rl::math::Vector* goal;
			
			SimpleModel* model;
			
			/** Start configuration. */
			::rl::math::Vector* start;
			
			Viewer* viewer;
			
		protected:
			::std::chrono::steady_clock::time_point time;
			
		private:
			
		};
	}
}

#endif // RL_PLAN_PLANNER_H
