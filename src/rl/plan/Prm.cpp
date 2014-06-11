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

#include <boost/make_shared.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "BridgeSampler.h"
#include "GaussianSampler.h"
#include "Prm.h"
#include "Sampler.h"
#include "SimpleModel.h"
#include "UniformSampler.h"
#include "Verifier.h"
#include "Viewer.h"

namespace rl
{
	namespace plan
	{
		Prm::Prm() :
			Planner(),
			degree(::std::numeric_limits< ::std::size_t >::max()),
			k(30),
			kd(true),
			radius(::std::numeric_limits< ::rl::math::Real >::max()),
			sampler(NULL),
			verifier(NULL),
			begin(NULL),
			ds(
				::boost::get(&VertexBundle::rank, graph),
				::boost::get(&VertexBundle::parent, graph)
			),
			end(NULL),
			graph()
		{
		}
		
		Prm::~Prm()
		{
		}
		
		Prm::Edge
		Prm::addEdge(const Vertex& u, const Vertex& v, const ::rl::math::Real& weight)
		{
			Edge e = ::boost::add_edge(u, v, this->graph).first;
			this->graph[e].weight = weight;
			
			this->ds.union_set(u, v);
			
			if (NULL != this->viewer)
			{
				this->viewer->drawConfigurationEdge(*this->graph[u].q, *this->graph[v].q);
			}
			
			return e;
		}
		
		void
		Prm::addPoint(NearestNeighbors& nn, const QueryItem& p)
		{
			NearestNeighbors::iterator i;
			
			for (i = nn.begin(); i != nn.end(); ++i)
			{
				if (NULL == *i)
				{
					break;
				}
			}
			
			NeighborSearchTreePtr tree(new NeighborSearchTree());
			
			if (nn.end() == i)
			{
				i = nn.insert(i, tree);
			}
			else
			{
				*i = tree;
			}
			
			for (NearestNeighbors::iterator j = nn.begin(); j != i; ++j)
			{
				if (NULL != *j)
				{
					(*i)->insert((*j)->begin(), (*j)->end());
					j->reset();
				}
			}
			
			tree->insert(p);
		}
		
		Prm::Vertex
		Prm::addVertex(const VectorPtr& q)
		{
			Vertex v = ::boost::add_vertex(this->graph);
			this->graph[v].index = ::boost::num_vertices(this->graph) - 1;
			this->graph[v].q = q;
			this->ds.make_set(v);
			
			if (NULL != this->viewer)
			{
				this->viewer->drawConfigurationVertex(*this->graph[v].q);
			}
			
			return v;
		}
		
		void
		Prm::construct(const ::std::size_t& steps)
		{
			for (::std::size_t i = 0; i < steps; ++i)
			{
				VectorPtr q = ::boost::make_shared< ::rl::math::Vector >(this->model->getDof());
				this->sampler->generateCollisionFree(*q);
				Vertex v = this->addVertex(q);
				this->insert(v);
			}
		}
		
		::std::string
		Prm::getName() const
		{
			if (NULL != this->sampler)
			{
				if (typeid(*this->sampler) == typeid(BridgeSampler))
				{
					return "Bridge PRM";
				}
				else if (typeid(*this->sampler) == typeid(GaussianSampler))
				{
					return "Gaussian PRM";
				}
			}
			
			return "PRM";
		}
		
		::std::size_t
		Prm::getNumEdges() const
		{
			return ::boost::num_edges(this->graph);
		}
		
		::std::size_t
		Prm::getNumVertices() const
		{
			return ::boost::num_vertices(this->graph);
		}
		
		void
		Prm::getPath(VectorList& path)
		{
			Vertex i = this->end;
			
			while (i != this->begin)
			{
				path.push_front(*this->graph[i].q);
				i = this->graph[i].predecessor;
			}
			
			path.push_front(*this->graph[i].q);
		}
		
		void
		Prm::insert(const Vertex& v)
		{
			if (this->kd)
			{
				QueryItem query(this->graph[v].q.get(), v);
				
				NeighborQueue queue;
				
				for (NearestNeighbors::const_iterator i = this->graph[::boost::graph_bundle].nn.begin(); i != this->graph[::boost::graph_bundle].nn.end(); ++i)
				{
					if (NULL != *i)
					{
						NeighborSearch search(
							*i->get(),
							query,
							static_cast< unsigned int >(this->k),
							0,
							true,
							Distance(this->model)
						);
						
						for (NeighborSearch::iterator j = search.begin(); j != search.end(); ++j)
						{
							queue.push(Neighbor(j->first.second, j->second));
						}
					}
				}
				
				for (::std::size_t i = 0; i < this->k && ::boost::degree(v, this->graph) < this->degree && !queue.empty(); ++i)
				{
					Vertex u = queue.top().first;
					
					if (::boost::degree(u, this->graph) < this->degree)
					{
						::rl::math::Real d = this->model->inverseOfTransformedDistance(queue.top().second);
						
						if (d < this->radius)
						{
							if (this->ds.find_set(u) != this->ds.find_set(v))
							{
								if (!this->verifier->isColliding(*this->graph[u].q, *this->graph[v].q, d))
								{
									this->addEdge(u, v, d);
								}
							}
						}
					}
					
					queue.pop();
				}
				
				this->addPoint(this->graph[::boost::graph_bundle].nn, QueryItem(this->graph[v].q.get(), v));
			}
			else
			{
				NeighborQueue queue;
				
				for (VertexIteratorPair i = ::boost::vertices(this->graph); i.first != i.second; ++i.first)
				{
					if (*i.first != v)
					{
						if (::boost::degree(*i.first, this->graph) < this->degree)
						{
							::rl::math::Real d = this->model->transformedDistance(*this->graph[v].q, *this->graph[*i.first].q);
							queue.push(Neighbor(*i.first, d));
						}
					}
				}
				
				for (::std::size_t i = 0; i < this->k && ::boost::degree(v, this->graph) < this->degree && !queue.empty(); ++i)
				{
					Vertex u = queue.top().first;
					
					::rl::math::Real d = this->model->inverseOfTransformedDistance(queue.top().second);
					
					if (d < this->radius)
					{
						if (this->ds.find_set(u) != this->ds.find_set(v))
						{
							if (!this->verifier->isColliding(*this->graph[u].q, *this->graph[v].q, d))
							{
								this->addEdge(u, v, d);
							}
						}
					}
					
					queue.pop();
				}
			}
		}
		
		void
		Prm::reset()
		{
			this->graph.clear();
			this->graph[::boost::graph_bundle].nn.clear();
			this->begin = NULL;
			this->end = NULL;
		}
		
		bool
		Prm::solve()
		{
			this->begin = this->addVertex(::boost::make_shared< ::rl::math::Vector >(*this->start));
			this->insert(this->begin);
			
			this->end = this->addVertex(::boost::make_shared< ::rl::math::Vector >(*this->goal));
			this->insert(this->end);
			
			timer.start();
			timer.stop();
			
			while (timer.elapsed() < this->duration && this->ds.find_set(this->begin) != this->ds.find_set(this->end))
			{
				this->construct(1);
				
				timer.stop();
			}
			
			if (this->ds.find_set(this->begin) != this->ds.find_set(this->end))
			{
				return false;
			}
			
			::boost::dijkstra_shortest_paths(
				this->graph,
				this->begin,
				::boost::get(&VertexBundle::predecessor, this->graph),
				::boost::get(&VertexBundle::distance, this->graph),
				::boost::get(&EdgeBundle::weight, this->graph),
				::boost::get(&VertexBundle::index, this->graph),
				::std::less< ::rl::math::Real >(),
				::boost::closed_plus< ::rl::math::Real >(),
				::std::numeric_limits< ::rl::math::Real >::max(),
				0,
				::boost::default_dijkstra_visitor()
			);
			
			return true;
		}
		
		const ::rl::math::Real*
		Prm::CartesianIterator::operator()(const QueryItem& p) const
		{
			return p.first->data();
		}
		
		const ::rl::math::Real*
		Prm::CartesianIterator::operator()(const QueryItem& p, const int&) const
		{
			return p.first->data() + p.first->size();
		}
		
		bool
		Prm::Compare::operator()(const Neighbor& x, const Neighbor& y) const
		{
			return x.second > y.second;
		}
		
		Prm::Distance::Distance() :
			model(NULL)
		{
		}
		
		Prm::Distance::Distance(Model* model) :
			model(model)
		{
		}
		
		template<>
		::rl::math::Real
#if (CGAL_VERSION_NR > 1030801000)
		Prm::Distance::max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< Prm::SearchTraits::FT >& r) const
#else
		Prm::Distance::max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< Prm::SearchTraits >& r) const
#endif
		{
			::rl::math::Vector min(r.dimension());
			::rl::math::Vector max(r.dimension());
			
			for (int i = 0; i < r.dimension(); ++i)
			{
				min(i) = r.min_coord(i);
				max(i) = r.max_coord(i);
			}
			
			return this->model->maxDistanceToRectangle(*q.first, min, max);
		}
		
		template<>
		::rl::math::Real
#if (CGAL_VERSION_NR > 1030801000)
		Prm::Distance::min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< Prm::SearchTraits::FT >& r) const
#else
		Prm::Distance::min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< Prm::SearchTraits >& r) const
#endif
		{
			::rl::math::Vector min(r.dimension());
			::rl::math::Vector max(r.dimension());
			
			for (int i = 0; i < r.dimension(); ++i)
			{
				min(i) = r.min_coord(i);
				max(i) = r.max_coord(i);
			}
			
			return this->model->minDistanceToRectangle(*q.first, min, max);
		}
		
		::rl::math::Real
		Prm::Distance::min_distance_to_rectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cutting_dimension) const
		{
			return this->model->minDistanceToRectangle(q, min, max, cutting_dimension);
		}
		
		::rl::math::Real
		Prm::Distance::new_distance(const ::rl::math::Real& dist, const ::rl::math::Real& old_off, const ::rl::math::Real& new_off, const int& cutting_dimension) const
		{
			return this->model->newDistance(dist, old_off, new_off, cutting_dimension);
		}
		
		::rl::math::Real
		Prm::Distance::transformed_distance(const ::rl::math::Real& d) const
		{
			return this->model->transformedDistance(d);
		}
		
		::rl::math::Real
		Prm::Distance::transformed_distance(const Query_item& q1, const Query_item& q2) const
		{
			return this->model->transformedDistance(*q1.first, *q2.first);
		}
	}
}
