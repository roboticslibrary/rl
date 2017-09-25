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

#ifndef RL_PLAN_NEARESTNEIGHBORS_H
#define RL_PLAN_NEARESTNEIGHBORS_H

#include <utility>
#include <vector>

#include "Metric.h"

namespace rl
{
	namespace plan
	{
		class Planner;
		
		class NearestNeighbors
		{
		public:
			typedef Metric::Distance Distance;
			
			typedef Metric::Value Value;
			
			typedef ::std::pair<Distance, Value> Neighbor;
			
			NearestNeighbors(const bool& transformed);
			
			virtual ~NearestNeighbors();
			
			virtual void clear() = 0;
			
			virtual bool empty() const = 0;
			
			bool isTransformedDistance() const;
			
			virtual ::std::vector<Neighbor> nearest(const Value& query, const ::std::size_t& k, const bool& sorted = true) const = 0;
			
			virtual void push(const Value& value) = 0;
			
			virtual ::std::size_t size() const = 0;
			
		protected:
			
		private:
			bool transformed;
		};
	}
}

#endif // RL_PLAN_NEARESTNEIGHBORS_H
