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
			degree(::std::numeric_limits< ::std::size_t>::max()),
			k(30),
			radius(::std::numeric_limits< ::rl::math::Real>::max()),
			sampler(nullptr),
			verifier(nullptr),
			begin(nullptr),
			ds(
				::boost::get(&VertexBundle::rank, graph),
				::boost::get(&VertexBundle::parent, graph)
			),
			end(nullptr),
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
			
			if (nullptr != this->viewer)
			{
				this->viewer->drawConfigurationEdge(*this->graph[u].q, *this->graph[v].q);
			}
			
			return e;
		}
		
		Prm::Vertex
		Prm::addVertex(const VectorPtr& q)
		{
			Vertex v = ::boost::add_vertex(this->graph);
			this->graph[v].index = ::boost::num_vertices(this->graph) - 1;
			this->graph[v].q = q;
			this->ds.make_set(v);
			
			if (nullptr != this->viewer)
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
				VectorPtr q = ::std::make_shared< ::rl::math::Vector>(this->model->getDofPosition());
				*q = this->sampler->generateCollisionFree();
				Vertex v = this->addVertex(q);
				this->insert(v);
			}
		}
		
		::std::string
		Prm::getName() const
		{
			if (nullptr != this->sampler)
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
		
		NearestNeighbors*
		Prm::getNearestNeighbors() const
		{
			return this->graph[::boost::graph_bundle].nn;
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
		
		VectorList
		Prm::getPath()
		{
			VectorList path;
			
			Vertex i = this->end;
			
			while (i != this->begin)
			{
				path.push_front(*this->graph[i].q);
				i = this->graph[i].predecessor;
			}
			
			path.push_front(*this->graph[i].q);
			
			return path;
		}
		
		void
		Prm::insert(const Vertex& v)
		{
			::std::vector<Neighbor> neighbors = this->graph[::boost::graph_bundle].nn->nearest(Metric::Value(this->graph[v].q.get(), v), this->k);
			
			for (::std::size_t i = 0; i < neighbors.size() && ::boost::degree(v, this->graph) < this->degree; ++i)
			{
				Vertex u = neighbors[i].second.second;
				
				if (::boost::degree(u, this->graph) < this->degree)
				{
					::rl::math::Real d = this->graph[::boost::graph_bundle].nn->isTransformedDistance() ? this->model->inverseOfTransformedDistance(neighbors[i].first) : neighbors[i].first;
					
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
			}
			
			this->graph[::boost::graph_bundle].nn->push(Metric::Value(this->graph[v].q.get(), v));
		}
		
		void
		Prm::reset()
		{
			this->graph.clear();
			this->graph[::boost::graph_bundle].nn->clear();
			this->begin = nullptr;
			this->end = nullptr;
		}
		
		void
		Prm::setNearestNeighbors(NearestNeighbors* nearestNeighbors)
		{
			this->graph[::boost::graph_bundle].nn = nearestNeighbors;
		}
		
		bool
		Prm::solve()
		{
			this->time = ::std::chrono::steady_clock::now();
			
			this->begin = this->addVertex(::std::make_shared< ::rl::math::Vector>(*this->start));
			this->insert(this->begin);
			
			this->end = this->addVertex(::std::make_shared< ::rl::math::Vector>(*this->goal));
			this->insert(this->end);
			
			while ((::std::chrono::steady_clock::now() - this->time) < this->duration && this->ds.find_set(this->begin) != this->ds.find_set(this->end))
			{
				this->construct(1);
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
				::std::less< ::rl::math::Real>(),
				::boost::closed_plus< ::rl::math::Real>(),
				::std::numeric_limits< ::rl::math::Real>::max(),
				0,
				::boost::default_dijkstra_visitor()
			);
			
			return true;
		}
	}
}
