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

#ifndef _RL_PLAN_RRT_H_
#define _RL_PLAN_RRT_H_

#include <boost/graph/adjacency_list.hpp>
#include <CGAL/Search_traits.h>

#include "MatrixPtr.h"
#include "Orthogonal_k_neighbor_search.h"
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
		 */
		class Rrt : public Planner
		{
		public:
			Rrt(const ::std::size_t& trees = 1);
			
			virtual ~Rrt();
			
			virtual ::std::string getName() const;
			
			virtual ::std::size_t getNumEdges() const;
			
			virtual ::std::size_t getNumVertices() const;
			
			virtual void getPath(VectorList& path);
			
			virtual void reset();
			
			virtual bool solve();
			
			/** Configuration step size. */
			::rl::math::Real delta;
			
			/** Epsilon for configuration comparison. */
			::rl::math::Real epsilon;
			
			/** Use kd-tree for nearest neighbor search instead of brute-force. */
			bool kd;
			
			Sampler* sampler;
			
		protected:
			struct VertexBundle
			{
				::std::size_t index;
				
				VectorPtr q;
				
				::rl::math::Real radius;
				
				TransformPtr t;
			};
			
			struct TreeBundle;
			
			typedef ::boost::adjacency_list<
				::boost::listS,
				::boost::listS,
				::boost::bidirectionalS,
				VertexBundle,
				::boost::no_property,
				TreeBundle
			> Tree;
			
			typedef ::boost::adjacency_list_traits<
				::boost::listS,
				::boost::listS,
				::boost::bidirectionalS,
				::boost::listS
			>::vertex_descriptor Vertex; 
			
			typedef ::std::pair< const ::rl::math::Vector*, Vertex > QueryItem;
			
			struct CartesianIterator
			{
				typedef const ::rl::math::Real* result_type;
				
				const ::rl::math::Real* operator()(const QueryItem& p) const;
				
				const ::rl::math::Real* operator()(const QueryItem& p, const int&) const;
			};
			
			struct Distance
			{
				typedef QueryItem Query_item;
				
				Distance();
				
				Distance(Model* model);
				
				template< typename SearchTraits > ::rl::math::Real max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< SearchTraits >& r) const;
				
				template< typename SearchTraits > ::rl::math::Real min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< SearchTraits >& r) const;
				
				::rl::math::Real min_distance_to_rectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cutting_dimension) const;
				
				::rl::math::Real new_distance(const ::rl::math::Real& dist, const ::rl::math::Real& old_off, const ::rl::math::Real& new_off, const int& cutting_dimension) const;
				
				::rl::math::Real transformed_distance(const ::rl::math::Real& d) const;
				
				::rl::math::Real transformed_distance(const Query_item& q1, const Query_item& q2) const;
				
				Model* model;
			};
			
			typedef ::CGAL::Search_traits< ::rl::math::Real, QueryItem, const ::rl::math::Real*, CartesianIterator > SearchTraits;
			
			typedef Orthogonal_k_neighbor_search< SearchTraits, Distance > NeighborSearch;
			
			typedef NeighborSearch::Tree NeighborSearchTree;
			
			typedef ::boost::shared_ptr< NeighborSearchTree > NeighborSearchTreePtr;
			
			typedef ::std::vector< NeighborSearchTreePtr > NearestNeighbors;
			
			struct TreeBundle
			{
				NearestNeighbors nn;
			};
			
			typedef ::boost::graph_traits< Tree >::edge_descriptor Edge;
			
			typedef ::boost::graph_traits< Tree >::edge_iterator EdgeIterator;
			
			typedef ::std::pair< EdgeIterator, EdgeIterator > EdgeIteratorPair;
			
			typedef ::boost::graph_traits< Tree >::vertex_iterator VertexIterator;
			
			typedef ::std::pair< VertexIterator, VertexIterator > VertexIteratorPair;
			
			typedef ::std::pair< Vertex, ::rl::math::Real > Neighbor;
			
			virtual Edge addEdge(const Vertex& u, const Vertex& v, Tree& tree);
			
			void addPoint(NearestNeighbors& nn, const QueryItem& p);
			
			Vertex addVertex(Tree& tree, const VectorPtr& q);
			
			bool areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const;
			
			virtual void choose(::rl::math::Vector& chosen);
			
			virtual Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);
			
			virtual Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);
			
			virtual Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);
			
			::std::vector< Vertex > begin;
			
			::std::vector< Vertex > end;
			
			::std::vector< Tree > tree;
			
		private:
			
		};
	}
}

#endif // _RL_PLAN_RRT_H_
