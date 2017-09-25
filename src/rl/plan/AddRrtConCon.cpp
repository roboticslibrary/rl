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

#include "AddRrtConCon.h"
#include "SimpleModel.h"
#include "Viewer.h"

namespace rl
{
	namespace plan
	{
		AddRrtConCon::AddRrtConCon() :
			RrtConCon(),
			alpha(0.05f),
			lower(2.0f),
			radius(20.0f)
		{
		}
		
		AddRrtConCon::~AddRrtConCon()
		{
		}
		
		Rrt::Vertex
		AddRrtConCon::addVertex(Tree& tree, const VectorPtr& q)
		{
			::std::shared_ptr<VertexBundle> bundle = ::std::make_shared<VertexBundle>();
			bundle->index = ::boost::num_vertices(tree) - 1;
			bundle->q = q;
			bundle->radius = ::std::numeric_limits< ::rl::math::Real>::max();
			
			Vertex v = ::boost::add_vertex(tree);
			tree[v] = bundle;
			
			tree[::boost::graph_bundle].nn->push(Metric::Value(q.get(), v));
			
			if (nullptr != this->viewer)
			{
				this->viewer->drawConfigurationVertex(*get(tree, v)->q);
			}
			
			return v;
		}
		
		AddRrtConCon::VertexBundle*
		AddRrtConCon::get(const Tree& tree, const Vertex& v)
		{
			return static_cast<VertexBundle*>(tree[v].get());
		}
		
		::std::string
		AddRrtConCon::getName() const
		{
			return "Adaptive Dynamic Domain RRT Connect Connect";
		}
		
		bool
		AddRrtConCon::solve()
		{
			this->time = ::std::chrono::steady_clock::now();
			
			this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector>(*this->start));
			this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector>(*this->goal));
			
			Tree* a = &this->tree[0];
			Tree* b = &this->tree[1];
			
			::rl::math::Vector chosen(this->model->getDofPosition());
			
			while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
			{
				for (::std::size_t j = 0; j < 2; ++j)
				{
					Neighbor aNearest;
					
					do
					{
						chosen = this->choose();
						aNearest = this->nearest(*a, chosen);
					}
					while (aNearest.first > get(*a, aNearest.second)->radius);
					
					Vertex aConnected = this->connect(*a, aNearest, chosen);
					
					if (nullptr != aConnected)
					{
						if (get(*a, aNearest.second)->radius < ::std::numeric_limits< ::rl::math::Real>::max())
						{
							get(*a, aNearest.second)->radius *= (1.0f + this->alpha);
						}
						
						Neighbor bNearest = this->nearest(*b, chosen);
						Vertex bConnected = this->connect(*b, bNearest, *get(*a, aConnected)->q);
						
						if (nullptr != bConnected)
						{
							if (this->areEqual(*get(*a, aConnected)->q, *get(*b, bConnected)->q))
							{
								this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
								this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
								return true;
							}
						}
					}
					else
					{
						if (get(*a, aNearest.second)->radius < ::std::numeric_limits< ::rl::math::Real>::max())
						{
							get(*a, aNearest.second)->radius *= (1.0f - this->alpha);
							get(*a, aNearest.second)->radius = ::std::max(this->lower, get(*a, aNearest.second)->radius);
						}
						else
						{
							get(*a, aNearest.second)->radius = this->radius;
						}
					}
					
					using ::std::swap;
					swap(a, b);
				}
			}
			
			return false;
		}
	}
}
