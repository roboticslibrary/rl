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

#ifndef RL_PLAN_RRT_H
#define RL_PLAN_RRT_H

#include <memory>
#include <boost/graph/adjacency_list.hpp>

#include "MatrixPtr.h"
#include "Metric.h"
#include "NearestNeighbors.h"
#include "Planner.h"
#include "TransformPtr.h"
#include "VectorPtr.h"

namespace rl
{
	namespace plan
	{
		class Model;
		class Sampler;
		class Verifier;
		
		/**
		 * Rapidly-Exploring Random Trees.
		 * 
		 * Steven M. LaValle. Rapidly-exploring random trees: A new tool for path
		 * planning. Technical Report TR 98-11, Iowa State University, Ames, IA,
		 * USA, October 1998.
		 * 
		 * http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf
		 */
		class Rrt : public Planner
		{
		public:
			Rrt(const ::std::size_t& trees = 1);
			
			virtual ~Rrt();
			
			virtual ::std::string getName() const;
			
			NearestNeighbors* getNearestNeighbors(const ::std::size_t& i) const;
			
			virtual ::std::size_t getNumEdges() const;
			
			virtual ::std::size_t getNumVertices() const;
			
			virtual VectorList getPath();
			
			virtual void reset();
			
			void setNearestNeighbors(NearestNeighbors* nearestNeighbors, const ::std::size_t& i);
			
			virtual bool solve();
			
			/** Configuration step size. */
			::rl::math::Real delta;
			
			/** Epsilon for configuration comparison. */
			::rl::math::Real epsilon;
			
			Sampler* sampler;
			
		protected:
			struct VertexBundle
			{
				::std::size_t index;
				
				VectorPtr q;
			};
			
			struct TreeBundle;
			
			typedef ::boost::adjacency_list<
				::boost::listS,
				::boost::listS,
				::boost::bidirectionalS,
				::std::shared_ptr<VertexBundle>,
				::boost::no_property,
				TreeBundle
			> Tree;
			
			typedef ::boost::adjacency_list_traits<
				::boost::listS,
				::boost::listS,
				::boost::bidirectionalS,
				::boost::listS
			>::vertex_descriptor Vertex; 
			
			struct TreeBundle
			{
				NearestNeighbors* nn;
			};
			
			typedef ::boost::graph_traits<Tree>::edge_descriptor Edge;
			
			typedef ::boost::graph_traits<Tree>::edge_iterator EdgeIterator;
			
			typedef ::std::pair<EdgeIterator, EdgeIterator> EdgeIteratorPair;
			
			typedef ::std::pair< ::rl::math::Real, Vertex> Neighbor;
			
			typedef ::boost::graph_traits<Tree>::vertex_iterator VertexIterator;
			
			typedef ::std::pair<VertexIterator, VertexIterator> VertexIteratorPair;
			
			virtual Edge addEdge(const Vertex& u, const Vertex& v, Tree& tree);
			
			virtual Vertex addVertex(Tree& tree, const VectorPtr& q);
			
			bool areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const;
			
			virtual ::rl::math::Vector choose();
			
			virtual Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);
			
			virtual Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);
			
			static VertexBundle* get(const Tree& tree, const Vertex& v);
			
			virtual Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);
			
			::std::vector<Vertex> begin;
			
			::std::vector<Vertex> end;
			
			::std::vector<Tree> tree;
			
		private:
			
		};
	}
}

#endif // RL_PLAN_RRT_H
