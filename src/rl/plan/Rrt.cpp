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

#include "Rrt.h"
#include "Sampler.h"
#include "SimpleModel.h"
#include "Verifier.h"
#include "Viewer.h"

#include "NearestNeighbors.h"

namespace rl
{
	namespace plan
	{
		Rrt::Rrt(const ::std::size_t& trees) :
			Planner(),
			delta(1.0f),
			epsilon(1.0e-3f),
			sampler(nullptr),
			begin(trees, nullptr),
			end(trees, nullptr),
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
			
			if (nullptr != this->viewer)
			{
				this->viewer->drawConfigurationEdge(*get(tree, u)->q, *get(tree, v)->q);
			}
			
			return e;
		}
		
		Rrt::Vertex
		Rrt::addVertex(Tree& tree, const VectorPtr& q)
		{
			::std::shared_ptr<VertexBundle> bundle = ::std::make_shared<VertexBundle>();
			bundle->index = ::boost::num_vertices(tree) - 1;
			bundle->q = q;
			
			Vertex v = ::boost::add_vertex(tree);
			tree[v] = bundle;
			
			tree[::boost::graph_bundle].nn->push(Metric::Value(q.get(), v));
			
			if (nullptr != this->viewer)
			{
				this->viewer->drawConfigurationVertex(*get(tree, v)->q);
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
		
		::rl::math::Vector
		Rrt::choose()
		{
			return this->sampler->generate();
		}
		
		Rrt::Vertex
		Rrt::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
		{
			::rl::math::Real distance = nearest.first;
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
			
			VectorPtr last = ::std::make_shared< ::rl::math::Vector>(this->model->getDofPosition());
			
			this->model->interpolate(*get(tree, nearest.second)->q, chosen, step / distance, *last);
			
			if (nullptr != this->viewer)
			{
//				this->viewer->drawConfiguration(*last);
			}
			
			this->model->setPosition(*last);
			this->model->updateFrames();
			
			if (this->model->isColliding())
			{
				return nullptr;
			}
			
			::rl::math::Vector next(this->model->getDofPosition());
			
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
				
				if (nullptr != this->viewer)
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
			this->addEdge(nearest.second, connected, tree);
			return connected;
		}
		
		Rrt::Vertex
		Rrt::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
		{
			::rl::math::Real distance = nearest.first;
			::rl::math::Real step = ::std::min(distance, this->delta);
			
			VectorPtr next = ::std::make_shared< ::rl::math::Vector>(this->model->getDofPosition());
			
			this->model->interpolate(*get(tree, nearest.second)->q, chosen, step / distance, *next);
			
			this->model->setPosition(*next);
			this->model->updateFrames();
			
			if (!this->model->isColliding())
			{
				Vertex extended = this->addVertex(tree, next);
				this->addEdge(nearest.second, extended, tree);
				return extended;
			}
			
			return nullptr;
		}
		
		Rrt::VertexBundle*
		Rrt::get(const Tree& tree, const Vertex& v)
		{
			return tree[v].get();
		}
		
		::std::string
		Rrt::getName() const
		{
			return "RRT";
		}
		
		NearestNeighbors*
		Rrt::getNearestNeighbors(const ::std::size_t& i) const
		{
			return this->tree[i][::boost::graph_bundle].nn;
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
		
		VectorList
		Rrt::getPath()
		{
			VectorList path;
			
			Vertex i = this->end[0];
			
			while (i != this->begin[0])
			{
				path.push_front(*get(this->tree[0], i)->q);
				i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
			}
			
			path.push_front(*get(this->tree[0], i)->q);
			
			return path;
		}
		
		Rrt::Neighbor
		Rrt::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
		{
			::std::vector<NearestNeighbors::Neighbor> neighbors = tree[::boost::graph_bundle].nn->nearest(Metric::Value(&chosen, Vertex()), 1);
			return Neighbor(
				tree[::boost::graph_bundle].nn->isTransformedDistance() ? this->model->inverseOfTransformedDistance(neighbors.front().first) : neighbors.front().first,
				neighbors.front().second.second
			);
		}
		
		void
		Rrt::reset()
		{
			for (::std::size_t i = 0; i < this->tree.size(); ++i)
			{
				this->tree[i].clear();
				this->tree[i][::boost::graph_bundle].nn->clear();
				this->begin[i] = nullptr;
				this->end[i] = nullptr;
			}
		}
		
		void
		Rrt::setNearestNeighbors(NearestNeighbors* nearestNeighbors, const ::std::size_t& i)
		{
			this->tree[i][::boost::graph_bundle].nn = nearestNeighbors;
		}
		
		bool
		Rrt::solve()
		{
			this->time = ::std::chrono::steady_clock::now();
			
			this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector>(*this->start));
			
			while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
			{
				::rl::math::Vector chosen = this->choose();
				Neighbor nearest = this->nearest(this->tree[0], chosen);
				Vertex extended = this->extend(this->tree[0], nearest, chosen);
				
				if (nullptr != extended)
				{
					if (this->areEqual(*get(this->tree[0], extended)->q, *this->goal))
					{
						this->end[0] = extended;
						return true;
					}
				}
			}
			
			return false;
		}
	}
}
