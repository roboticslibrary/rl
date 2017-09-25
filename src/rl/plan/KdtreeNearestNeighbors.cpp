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

#include "KdtreeNearestNeighbors.h"
#include "Model.h"

namespace rl
{
	namespace plan
	{
		KdtreeNearestNeighbors::KdtreeNearestNeighbors(Model* model) :
			NearestNeighbors(true),
			container(Metric(model, true))
		{
		}
		
		KdtreeNearestNeighbors::~KdtreeNearestNeighbors()
		{
		}
		
		void
		KdtreeNearestNeighbors::clear()
		{
			this->container.clear();
		}
		
		bool
		KdtreeNearestNeighbors::empty() const
		{
			return this->container.empty();
		}
		
		::boost::optional< ::std::size_t>
		KdtreeNearestNeighbors::getChecks() const
		{
			return this->container.getChecks();
		}
		
		::std::size_t
		KdtreeNearestNeighbors::getSamples() const
		{
			return this->container.getSamples();
		}
		
		::std::vector<NearestNeighbors::Neighbor>
		KdtreeNearestNeighbors::nearest(const NearestNeighbors::Value& query, const ::std::size_t& k, const bool& sorted) const
		{
			return this->container.nearest(query, k, sorted);
		}
		
		void
		KdtreeNearestNeighbors::push(const NearestNeighbors::Value& value)
		{
			this->container.push(value);
		}
		
		void
		KdtreeNearestNeighbors::setChecks(const ::boost::optional< ::std::size_t>& checks)
		{
			this->container.setChecks(checks);
		}
		
		void
		KdtreeNearestNeighbors::setSamples(const ::std::size_t& samples)
		{
			this->container.setSamples(samples);
		}
		
		::std::size_t
		KdtreeNearestNeighbors::size() const
		{
			return this->container.size();
		}
	}
}
