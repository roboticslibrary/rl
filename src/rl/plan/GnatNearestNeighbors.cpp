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

#include "GnatNearestNeighbors.h"
#include "Model.h"

namespace rl
{
	namespace plan
	{
		GnatNearestNeighbors::GnatNearestNeighbors(Model* model) :
			NearestNeighbors(false),
			container(Metric(model, false))
		{
		}
		
		GnatNearestNeighbors::~GnatNearestNeighbors()
		{
		}
		
		void
		GnatNearestNeighbors::clear()
		{
			this->container.clear();
		}
		
		bool
		GnatNearestNeighbors::empty() const
		{
			return this->container.empty();
		}
		
		::boost::optional< ::std::size_t>
		GnatNearestNeighbors::getChecks() const
		{
			return this->container.getChecks();
		}
		
		::std::size_t
		GnatNearestNeighbors::getNodeDataMax() const
		{
			return this->container.getNodeDataMax();
		}
		
		::std::size_t
		GnatNearestNeighbors::getNodeDegree() const
		{
			return this->container.getNodeDegree();
		}
		
		::std::size_t
		GnatNearestNeighbors::getNodeDegreeMax() const
		{
			return this->container.getNodeDegreeMax();
		}
		
		::std::size_t
		GnatNearestNeighbors::getNodeDegreeMin() const
		{
			return this->container.getNodeDegreeMin();
		}
		
		::std::vector<NearestNeighbors::Neighbor>
		GnatNearestNeighbors::nearest(const NearestNeighbors::Value& query, const ::std::size_t& k, const bool& sorted) const
		{
			return this->container.nearest(query, k, sorted);
		}
		
		void
		GnatNearestNeighbors::push(const NearestNeighbors::Value& value)
		{
			this->container.push(value);
		}
		
		void
		GnatNearestNeighbors::seed(const ::std::mt19937::result_type& value)
		{
			this->container.seed(value);
		}
		
		void
		GnatNearestNeighbors::setChecks(const ::boost::optional< ::std::size_t>& checks)
		{
			this->container.setChecks(checks);
		}
		
		void
		GnatNearestNeighbors::setNodeDataMax(const ::std::size_t& nodeDataMax)
		{
			this->container.setNodeDataMax(nodeDataMax);
		}
		
		void
		GnatNearestNeighbors::setNodeDegree(const ::std::size_t& nodeDegree)
		{
			this->container.setNodeDegree(nodeDegree);
		}
		
		void
		GnatNearestNeighbors::setNodeDegreeMax(const ::std::size_t& nodeDegreeMax)
		{
			this->container.setNodeDegreeMax(nodeDegreeMax);
		}
		
		void
		GnatNearestNeighbors::setNodeDegreeMin(const ::std::size_t& nodeDegreeMin)
		{
			this->container.setNodeDegreeMin(nodeDegreeMin);
		}
		
		::std::size_t
		GnatNearestNeighbors::size() const
		{
			return this->container.size();
		}
	}
}
