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

#ifndef RL_PLAN_GNATNEARESTNEIGHBORS_H
#define RL_PLAN_GNATNEARESTNEIGHBORS_H

#include <rl/math/GnatNearestNeighbors.h>

#include "NearestNeighbors.h"

namespace rl
{
	namespace plan
	{
		class Model;
		
		class GnatNearestNeighbors : public NearestNeighbors
		{
		public:
			GnatNearestNeighbors(Model* model);
			
			virtual ~GnatNearestNeighbors();
			
			void clear();
			
			bool empty() const;
			
			::boost::optional< ::std::size_t> getChecks() const;
			
			::std::size_t getNodeDataMax() const;
			
			::std::size_t getNodeDegree() const;
			
			::std::size_t getNodeDegreeMax() const;
			
			::std::size_t getNodeDegreeMin() const;
			
			::std::vector<NearestNeighbors::Neighbor> nearest(const NearestNeighbors::Value& query, const ::std::size_t& k, const bool& sorted = true) const;
			
			void push(const NearestNeighbors::Value& value);
			
			void seed(const ::std::mt19937::result_type& value);
			
			void setChecks(const ::boost::optional< ::std::size_t>& checks);
			
			void setNodeDataMax(const ::std::size_t& nodeDataMax);
			
			void setNodeDegree(const ::std::size_t& nodeDegree);
			
			void setNodeDegreeMax(const ::std::size_t& nodeDegreeMax);
			
			void setNodeDegreeMin(const ::std::size_t& nodeDegreeMin);
			
			::std::size_t size() const;
			
		protected:
			
		private:
			::rl::math::GnatNearestNeighbors<Metric> container;
		};
	}
}

#endif // RL_PLAN_GNATNEARESTNEIGHBORS_H
