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

#include <chrono>

#include "DistanceModel.h"
#include "Viewer.h"
#include "WorkspaceSphereExplorer.h"

//#define PRINT_WORKSPACE_PATH

namespace rl
{
	namespace plan
	{
		WorkspaceSphereExplorer::WorkspaceSphereExplorer() :
			goal(),
			greedy(GREEDY_SPACE),
			model(nullptr),
			radius(0.0f),
			range(::std::numeric_limits< ::rl::math::Real>::max()),
			samples(10),
			start(),
			viewer(nullptr),
			begin(nullptr),
			end(nullptr),
			graph(),
			queue(),
			rand(
				::std::mt19937(::std::random_device()()),
				::boost::uniform_on_sphere< ::rl::math::Real>(3)
			)
		{
		}
		
		WorkspaceSphereExplorer::~WorkspaceSphereExplorer()
		{
		}
		
		WorkspaceSphereExplorer::Edge
		WorkspaceSphereExplorer::addEdge(const Vertex& u, const Vertex& v)
		{
			Edge edge = ::boost::add_edge(u, v, this->graph).first;
			
			if (nullptr != this->viewer)
			{
				this->viewer->drawWorkEdge(*this->graph[u].sphere.center, *this->graph[v].sphere.center);
			}
			
			return edge;
		}
		
		WorkspaceSphereExplorer::Vertex
		WorkspaceSphereExplorer::addVertex(const WorkspaceSphere& sphere)
		{
			Vertex vertex = ::boost::add_vertex(this->graph);
			this->graph[vertex].sphere = sphere;
			
#ifndef PRINT_WORKSPACE_PATH
			if (nullptr != this->viewer)
			{
				this->viewer->drawSphere(*this->graph[vertex].sphere.center, this->graph[vertex].sphere.radius);
				this->viewer->drawWorkVertex(*this->graph[vertex].sphere.center);
			}
#endif
			
			return vertex;
		}
		
		bool
		WorkspaceSphereExplorer::explore()
		{
			WorkspaceSphere start;
			start.center = ::std::make_shared< ::rl::math::Vector3>(*this->start);
			start.radius = this->model->distance(*start.center);
			start.radiusSum = start.radius;
			start.parent = nullptr;

			start.priority = (*this->goal - *start.center).norm() - start.radius;
			
			this->queue.insert(start);
			
			while (!this->queue.empty())
			{
				WorkspaceSphere top = *this->queue.begin();
				
				this->queue.erase(this->queue.begin());
				
				if (top.radius >= this->radius)
				{
					Vertex vertex = this->addVertex(top);
					
					if (nullptr != top.parent)
					{
						this->addEdge(top.parent, vertex);
					}
					else
					{
						this->begin = vertex;
					}
					
					if ((*this->goal - *top.center).norm() < top.radius)
					{
						WorkspaceSphere goal;
						goal.center = ::std::make_shared< ::rl::math::Vector3>(*this->goal);
						goal.radius = this->model->distance(*goal.center);
						goal.parent = vertex;
						goal.priority = (*this->goal - *goal.center).norm() - goal.radius;
						
						this->end = this->addVertex(goal);
						
						if (nullptr != top.parent)
						{
							this->addEdge(top.parent, this->end);
						}
						else
						{
							this->addEdge(this->begin, this->end);
						}
						
						return true;
					}
					
					::std::multiset<WorkspaceSphere>::iterator i = this->queue.begin();
					::std::multiset<WorkspaceSphere>::iterator j;
					
					while (i != this->queue.end())
					{
						if ((*i->center - *top.center).norm() < top.radius)
						{
							j = i;
							++i;
							this->queue.erase(j);
						}
						else
						{
							++i;
						}
					}
					
					for (::std::size_t i = 0; i < ::std::ceil(this->samples * top.radius); ++i)
//for (::std::size_t i = 0; i < this->samples; ++i) // TODO
					{
						WorkspaceSphere sphere;
						
						sphere.parent = vertex;
						
						::boost::uniform_on_sphere< ::rl::math::Real>::result_type sample = this->rand();
						
						sphere.center = ::std::make_shared< ::rl::math::Vector3>(
							top.radius * ::rl::math::Vector3(sample[0], sample[1], sample[2]) + *top.center // TODO
						);
						
						if ((*this->start - *sphere.center).norm() <= this->range)
						{
							if (!this->isCovered(top.parent, *sphere.center))
							{
								sphere.radius = this->model->distance(*sphere.center);
								sphere.radiusSum = sphere.radius + top.radiusSum;
								
								if (sphere.radius >= this->radius)
								{
									switch (this->greedy)
									{
									case GREEDY_DISTANCE:
										sphere.priority = (*this->goal - *sphere.center).norm() - sphere.radius;
										break;
									case GREEDY_SOURCE_DISTANCE:
										sphere.priority = (*this->goal - *sphere.center).norm() - sphere.radius + top.radiusSum;
										break;
									case GREEDY_SPACE:
										sphere.priority = 1.0f / sphere.radius;
										break;
									default:
										break;
									}
									
									this->queue.insert(sphere);
								}
							}
						}
					}
				}
			}
			
			return false;
		}
		
		WorkspaceSphereList
		WorkspaceSphereExplorer::getPath() const
		{
			WorkspaceSphereList path;
			
			Vertex i = this->end;
			
			while (i != this->begin)
			{
				path.push_front(this->graph[i].sphere);
				
#ifdef PRINT_WORKSPACE_PATH
				if (nullptr != this->viewer)
				{
					this->viewer->drawSphere(*this->graph[i].sphere.center, this->graph[i].sphere.radius);
					this->viewer->drawWorkVertex(*this->graph[i].sphere.center);
				}
#endif
				
				i = ::boost::source(*::boost::in_edges(i, this->graph).first, this->graph);
			}
			
			path.push_front(this->graph[i].sphere);
			
#ifdef PRINT_WORKSPACE_PATH
			if (nullptr != this->viewer)
			{
				this->viewer->drawSphere(*this->graph[i].sphere.center, this->graph[i].sphere.radius);
				this->viewer->drawWorkVertex(*this->graph[i].sphere.center);
			}
#endif
			
			return path;
		}
		
		bool
		WorkspaceSphereExplorer::isCovered(const ::rl::math::Vector3& point) const
		{
			for (VertexIteratorPair i = ::boost::vertices(this->graph); i.first != i.second; ++i.first)
			{
				if ((point - *this->graph[*i.first].sphere.center).norm() < this->graph[*i.first].sphere.radius)
				{
					return true;
				}
			}
			
			return false;
		}
		
		bool
		WorkspaceSphereExplorer::isCovered(const Vertex& parent, const ::rl::math::Vector3& point) const
		{
			for (VertexIteratorPair i = ::boost::vertices(this->graph); i.first != i.second; ++i.first)
			{
				if (parent != this->graph[*i.first].sphere.parent)
				{
					if ((point - *this->graph[*i.first].sphere.center).norm() < this->graph[*i.first].sphere.radius)
					{
						return true;
					}
				}
			}
			
			return false;
		}
		
		void
		WorkspaceSphereExplorer::reset()
		{
			this->graph.clear();
			this->queue.clear();
			this->begin = nullptr;
			this->end = nullptr;
		}
		
		void
		WorkspaceSphereExplorer::seed(const ::std::mt19937::result_type& value)
		{
			this->rand.engine().seed(value);
		}
	}
}
