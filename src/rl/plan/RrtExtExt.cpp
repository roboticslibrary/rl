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

#include "RrtExtExt.h"
#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		RrtExtExt::RrtExtExt() :
			RrtDual()
		{
		}
		
		RrtExtExt::~RrtExtExt()
		{
		}
		
		::std::string
		RrtExtExt::getName() const
		{
			return "RRT Extend Extend";
		}
		
		bool
		RrtExtExt::solve()
		{
			this->time = ::std::chrono::steady_clock::now();
			
			this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector>(*this->start));
			this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector>(*this->goal));
			
			Tree* a = &this->tree[0];
			Tree* b = &this->tree[1];
			
			while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
			{
				for (::std::size_t j = 0; j < 2; ++j)
				{
					::rl::math::Vector chosen = this->choose();
					Neighbor aNearest = this->nearest(*a, chosen);
					Vertex aExtended = this->extend(*a, aNearest, chosen);
					
					if (nullptr != aExtended)
					{
						Neighbor bNearest = this->nearest(*b, *get(*a, aExtended)->q);
						Vertex bExtended = this->extend(*b, bNearest, *get(*a, aExtended)->q);
						
						if (nullptr != bExtended)
						{
							if (this->areEqual(*get(*a, aExtended)->q, *get(*b, bExtended)->q))
							{
								this->end[0] = &this->tree[0] == a ? aExtended : bExtended;
								this->end[1] = &this->tree[1] == b ? bExtended : aExtended;
								return true;
							}
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
