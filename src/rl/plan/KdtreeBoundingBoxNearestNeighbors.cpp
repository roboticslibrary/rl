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

#include "KdtreeBoundingBoxNearestNeighbors.h"
#include "Model.h"

namespace rl
{
	namespace plan
	{
		KdtreeBoundingBoxNearestNeighbors::KdtreeBoundingBoxNearestNeighbors(Model* model) :
			NearestNeighbors(true),
			container(Metric(model, true))
		{
		}
		
		KdtreeBoundingBoxNearestNeighbors::~KdtreeBoundingBoxNearestNeighbors()
		{
		}
		
		void
		KdtreeBoundingBoxNearestNeighbors::clear()
		{
			this->container.clear();
		}
		
		bool
		KdtreeBoundingBoxNearestNeighbors::empty() const
		{
			return this->container.empty();
		}
		
		::boost::optional< ::std::size_t>
		KdtreeBoundingBoxNearestNeighbors::getChecks() const
		{
			return this->container.getChecks();
		}
		
		::std::size_t
		KdtreeBoundingBoxNearestNeighbors::getNodeDataMax() const
		{
			return this->container.getNodeDataMax();
		}
		
		::std::vector<NearestNeighbors::Neighbor>
		KdtreeBoundingBoxNearestNeighbors::nearest(const NearestNeighbors::Value& query, const ::std::size_t& k, const bool& sorted) const
		{
			return this->container.nearest(query, k, sorted);
		}
		
		void
		KdtreeBoundingBoxNearestNeighbors::push(const NearestNeighbors::Value& value)
		{
			this->container.push(value);
		}
		
		void
		KdtreeBoundingBoxNearestNeighbors::setChecks(const ::boost::optional< ::std::size_t>& checks)
		{
			this->container.setChecks(checks);
		}
		
		void
		KdtreeBoundingBoxNearestNeighbors::setNodeDataMax(const ::std::size_t& nodeDataMax)
		{
			this->container.setNodeDataMax(nodeDataMax);
		}
		
		::std::size_t
		KdtreeBoundingBoxNearestNeighbors::size() const
		{
			return this->container.size();
		}
	}
}
