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

#include "RrtDual.h"
#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		RrtDual::RrtDual() :
			Rrt(2)
		{
		}
		
		RrtDual::~RrtDual()
		{
		}
		
		::std::string
		RrtDual::getName() const
		{
			return "RRT Dual";
		}
		
		VectorList
		RrtDual::getPath()
		{
			VectorList path;
			
			Vertex i = this->end[0];
			
			while (i != this->begin[0])
			{
				path.push_front(*get(this->tree[0], i)->q);
				i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
			}
			
			path.push_front(*get(this->tree[0], i)->q);
			
			i = ::boost::source(*::boost::in_edges(this->end[1], this->tree[1]).first, this->tree[1]);
			
			while (i != this->begin[1])
			{
				path.push_back(*get(this->tree[1], i)->q);
				i = ::boost::source(*::boost::in_edges(i, this->tree[1]).first, this->tree[1]);
			}
			
			path.push_back(*this->tree[1][i]->q);
			
			return path;
		}
		
		bool
		RrtDual::solve()
		{
			this->time = ::std::chrono::steady_clock::now();
			
			this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector>(*this->start));
			this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector>(*this->goal));
			
			while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
			{
				::rl::math::Vector chosen = this->choose();
				Neighbor nearest = this->nearest(this->tree[0], chosen);
				Vertex extended = this->extend(this->tree[0], nearest, chosen);
				
				if (nullptr != extended)
				{
					Neighbor nearest2 = this->nearest(this->tree[1], chosen);
					Vertex extended2 = this->extend(this->tree[1], nearest2, chosen);
					
					if (nullptr != extended2)
					{
						if (this->areEqual(*get(this->tree[0], extended)->q, *get(this->tree[1], extended2)->q))
						{
							this->end[0] = extended;
							this->end[1] = extended2;
							return true;
						}
					}
				}
			}
			
			return false;
		}
	}
}
