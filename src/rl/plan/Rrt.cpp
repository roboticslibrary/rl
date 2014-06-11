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

#include "Rrt.h"
#include "Sampler.h"
#include "SimpleModel.h"
#include "Verifier.h"
#include "Viewer.h"

#if _MSC_VER < 1600
#define nullptr NULL // TODO
#endif

namespace rl
{
	namespace plan
	{
		Rrt::Rrt(const ::std::size_t& trees) :
			Planner(),
			delta(1.0f),
			epsilon(1.0e-3f),
			kd(true),
			sampler(NULL),
			begin(trees, NULL),
			end(trees, NULL),
			tree(trees)
		{
		}
		
		Rrt::~Rrt()
		{
		}
		
		Rrt::Edge
		Rrt::addEdge(const Vertex& u, const Vertex& v, Tree& tree)
		{
			Edge e = ::boost::add_edge(u, v, tree).first;
			
			if (NULL != this->viewer)
			{
				this->viewer->drawConfigurationEdge(*tree[u].q, *tree[v].q);
			}
			
			return e;
		}
		
		void
		Rrt::addPoint(NearestNeighbors& nn, const QueryItem& p)
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
		
		Rrt::Vertex
		Rrt::addVertex(Tree& tree, const VectorPtr& q)
		{
			Vertex v = ::boost::add_vertex(tree);
			tree[v].index = ::boost::num_vertices(tree) - 1;
			tree[v].q = q;
			tree[v].radius = ::std::numeric_limits< ::rl::math::Real >::max();
			
			if (this->kd)
			{
				this->addPoint(tree[::boost::graph_bundle].nn, QueryItem(q.get(), v));
			}
			
			if (NULL != this->viewer)
			{
				this->viewer->drawConfigurationVertex(*tree[v].q);
			}
			
			return v;
		}
		
		bool
		Rrt::areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const
		{
			if (this->model->distance(lhs, rhs) > this->epsilon)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		
		void
		Rrt::choose(::rl::math::Vector& chosen)
		{
			this->sampler->generate(chosen);
		}
		
		Rrt::Vertex
		Rrt::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
		{
			::rl::math::Real distance = nearest.second;
			::rl::math::Real step = distance;
			
			bool reached = false;
			
			if (step <= this->delta)
			{
				reached = true;
			}
			else
			{
				step = this->delta;
			}
			
			VectorPtr last = ::boost::make_shared< ::rl::math::Vector >(this->model->getDof());
			
			this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);
			
			if (NULL != this->viewer)
			{
//				this->viewer->drawConfiguration(*last);
			}
			
			this->model->setPosition(*last);
			this->model->updateFrames();
			
			if (this->model->isColliding())
			{
				return NULL;
			}
			
			::rl::math::Vector next(this->model->getDof());
			
			while (!reached)
			{
				distance = this->model->distance(*last, chosen);
				step = distance;
				
				if (step <= this->delta)
				{
					reached = true;
				}
				else
				{
					step = this->delta;
				}
				
				this->model->interpolate(*last, chosen, step / distance, next);
				
				if (NULL != this->viewer)
				{
//					this->viewer->drawConfiguration(next);
				}
				
				this->model->setPosition(next);
				this->model->updateFrames();
				
				if (this->model->isColliding())
				{
					break;
				}
				
				*last = next;
			}
			
			Vertex connected = this->addVertex(tree, last);
			this->addEdge(nearest.first, connected, tree);
			return connected;
		}
		
		Rrt::Vertex
		Rrt::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
		{
			::rl::math::Real distance = nearest.second;
			::rl::math::Real step = ::std::min(distance, this->delta);
			
			VectorPtr next = ::boost::make_shared< ::rl::math::Vector >(this->model->getDof());
			
			this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *next);
			
			this->model->setPosition(*next);
			this->model->updateFrames();
			
			if (!this->model->isColliding())
			{
				Vertex extended = this->addVertex(tree, next);
				this->addEdge(nearest.first, extended, tree);
				return extended;
			}
			
			return NULL;
		}
		
		::std::string
		Rrt::getName() const
		{
			return "RRT";
		}
		
		::std::size_t
		Rrt::getNumEdges() const
		{
			::std::size_t edges = 0;
			
			for (::std::size_t i = 0; i < this->tree.size(); ++i)
			{
				edges += ::boost::num_edges(this->tree[i]);
			}
			
			return edges;
		}
		
		::std::size_t
		Rrt::getNumVertices() const
		{
			::std::size_t vertices = 0;
			
			for (::std::size_t i = 0; i < this->tree.size(); ++i)
			{
				vertices += ::boost::num_vertices(this->tree[i]);
			}
			
			return vertices;
		}
		
		void
		Rrt::getPath(VectorList& path)
		{
			Vertex i = this->end[0];
			
			while (i != this->begin[0])
			{
				path.push_front(*this->tree[0][i].q);
				i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
			}
			
			path.push_front(*this->tree[0][i].q);
		}
		
		Rrt::Neighbor
		Rrt::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
		{
			Neighbor p(nullptr, ::std::numeric_limits< ::rl::math::Real >::max());
			
			if (this->kd)
			{
				QueryItem query(&chosen, nullptr);
				
				for (NearestNeighbors::const_iterator i = tree[::boost::graph_bundle].nn.begin(); i != tree[::boost::graph_bundle].nn.end(); ++i)
				{
					if (NULL != *i)
					{
						NeighborSearch search(
							*i->get(),
							query,
							1,
							0,
							true,
							Distance(this->model)
						);
						
						if (search.begin()->second < p.second)
						{
							p.first = search.begin()->first.second;
							p.second = search.begin()->second;
						}
					}
				}
			}
			else
			{
				for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
				{
					::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);
					
					if (d < p.second)
					{
						p.first = *i.first;
						p.second = d;
					}
				}
			}
			
			p.second = this->model->inverseOfTransformedDistance(p.second);
			
			return p;
		}
		
		void
		Rrt::reset()
		{
			for (::std::size_t i = 0; i < this->tree.size(); ++i)
			{
				this->tree[i].clear();
				this->tree[i][::boost::graph_bundle].nn.clear();
				this->begin[i] = NULL;
				this->end[i] = NULL;
			}
		}
		
		bool
		Rrt::solve()
		{
			this->begin[0] = this->addVertex(this->tree[0], ::boost::make_shared< ::rl::math::Vector >(*this->start));
			
			::rl::math::Vector chosen(this->model->getDof());
			
			timer.start();
			timer.stop();
			
			while (timer.elapsed() < this->duration)
			{
				this->choose(chosen);
				
				Neighbor nearest = this->nearest(this->tree[0], chosen);
				
				Vertex extended = this->extend(this->tree[0], nearest, chosen);
				
				if (NULL != extended)
				{
					if (this->areEqual(*this->tree[0][extended].q, *this->goal))
					{
						this->end[0] = extended;
						return true;
					}
				}
				
				timer.stop();
			}
			
			return false;
		}
		
		const ::rl::math::Real*
		Rrt::CartesianIterator::operator()(const QueryItem& p) const
		{
			return p.first->data(); // TODO
		}
		
		const ::rl::math::Real*
		Rrt::CartesianIterator::operator()(const QueryItem& p, const int&) const
		{
			return p.first->data() + p.first->size(); // TODO
		}
		
		Rrt::Distance::Distance() :
			model(NULL)
		{
		}
		
		Rrt::Distance::Distance(Model* model) :
			model(model)
		{
		}
		
		template<>
		::rl::math::Real
#if (CGAL_VERSION_NR > 1030801000)
		Rrt::Distance::max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< Rrt::SearchTraits::FT >& r) const
#else
		Rrt::Distance::max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< Rrt::SearchTraits >& r) const
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
		Rrt::Distance::min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< Rrt::SearchTraits::FT >& r) const
#else
		Rrt::Distance::min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< Rrt::SearchTraits >& r) const
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
		Rrt::Distance::min_distance_to_rectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cutting_dimension) const
		{
			return this->model->minDistanceToRectangle(q, min, max, cutting_dimension);
		}
		
		::rl::math::Real
		Rrt::Distance::new_distance(const ::rl::math::Real& dist, const ::rl::math::Real& old_off, const ::rl::math::Real& new_off, const int& cutting_dimension) const
		{
			return this->model->newDistance(dist, old_off, new_off, cutting_dimension);
		}
		
		::rl::math::Real
		Rrt::Distance::transformed_distance(const ::rl::math::Real& d) const
		{
			return this->model->transformedDistance(d);
		}
		
		::rl::math::Real
		Rrt::Distance::transformed_distance(const Query_item& q1, const Query_item& q2) const
		{
			return this->model->transformedDistance(*q1.first, *q2.first);
		}
	}
}
