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

#include "RrtExtCon.h"
#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		RrtExtCon::RrtExtCon() :
			RrtDual()
		{
		}
		
		RrtExtCon::~RrtExtCon()
		{
		}
		
		::std::string
		RrtExtCon::getName() const
		{
			return "RRT Extend Connect";
		}
		
		bool
		RrtExtCon::solve()
		{
			this->begin[0] = this->addVertex(this->tree[0], ::boost::make_shared< ::rl::math::Vector >(*this->start));
			this->begin[1] = this->addVertex(this->tree[1], ::boost::make_shared< ::rl::math::Vector >(*this->goal));
			
			Tree* a = &this->tree[0];
			Tree* b = &this->tree[1];
			
			::rl::math::Vector chosen(this->model->getDof());
			
			timer.start();
			timer.stop();
			
			while (timer.elapsed() < this->duration)
			{
				for (::std::size_t j = 0; j < 2; ++j)
				{
					this->choose(chosen);
					
					Neighbor aNearest = this->nearest(*a, chosen);
					
					Vertex aExtended = this->extend(*a, aNearest, chosen);
					
					if (NULL != aExtended)
					{
						Neighbor bNearest = this->nearest(*b, *(*a)[aExtended].q);
						
						Vertex bConnected = this->connect(*b, bNearest, *(*a)[aExtended].q);
						
						if (NULL != bConnected)
						{
							if (this->areEqual(*(*a)[aExtended].q, *(*b)[bConnected].q))
							{
								this->end[0] = &this->tree[0] == a ? aExtended : bConnected;
								this->end[1] = &this->tree[1] == b ? bConnected : aExtended;
								return true;
							}
						}
					}
					
					::std::swap(a, b);
				}
				
				timer.stop();
			}
			
			return false;
		}
	}
}
