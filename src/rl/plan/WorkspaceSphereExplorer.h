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

#ifndef _RL_PLAN_WORKSPACESPHEREEXPLORER_H_
#define _RL_PLAN_WORKSPACESPHEREEXPLORER_H_

#include <list>
#include <set>
#include <boost/graph/adjacency_list.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/random/variate_generator.hpp>
#include <rl/math/Vector.h>

#include "WorkspaceSphere.h"
#include "WorkspaceSphereList.h"

namespace rl
{
	namespace plan
	{
		class DistanceModel;
		class Viewer;
		
		class WorkspaceSphereExplorer
		{
		public:
			enum Greedy
			{
				GREEDY_DISTANCE,
				GREEDY_SOURCE_DISTANCE,
				GREEDY_SPACE
			};
			
			WorkspaceSphereExplorer();
			
			virtual ~WorkspaceSphereExplorer();
			
			bool explore();
			
			void getPath(WorkspaceSphereList& path) const;
			
			bool isCovered(const ::rl::math::Vector3& point) const;
			
			void reset();
			
			void seed(const ::boost::mt19937::result_type& value);
			
			::rl::math::Vector3* goal;
			
			Greedy greedy;
			
			DistanceModel* model;
			
			::rl::math::Real radius;
			
			::rl::math::Real range;
			
			::std::size_t samples;
			
			::rl::math::Vector3* start;
			
			Viewer* viewer;
			
		protected:
			struct VertexBundle
			{
				WorkspaceSphere sphere;
			};
			
			typedef ::boost::adjacency_list<
				::boost::listS,
				::boost::listS,
				::boost::bidirectionalS,
				VertexBundle
			> Graph;
			
			typedef ::boost::graph_traits< Graph >::edge_descriptor Edge;
			
			typedef ::boost::graph_traits< Graph >::edge_iterator EdgeIterator;
			
			typedef ::std::pair< EdgeIterator, EdgeIterator > EdgeIteratorPair;
			
			typedef ::boost::graph_traits< Graph >::vertex_descriptor Vertex;
			
			typedef ::boost::graph_traits< Graph >::vertex_iterator VertexIterator;
			
			typedef ::std::pair< VertexIterator, VertexIterator > VertexIteratorPair;
			
			Edge addEdge(const Vertex& u, const Vertex& v);
			
			Vertex addVertex(const WorkspaceSphere& sphere);
			
			bool isCovered(const Vertex& parent, const ::rl::math::Vector3& point) const;
			
			Vertex begin;
			
			Vertex end;
			
			Graph graph;
			
			::std::multiset< WorkspaceSphere > queue;
			
			::boost::variate_generator< ::boost::mt19937, ::boost::uniform_on_sphere< ::rl::math::Real > > rand;
			
		private:
			
		};
	}
}

#endif // _RL_PLAN_WORKSPACESPHEREEXPLORER_H_
