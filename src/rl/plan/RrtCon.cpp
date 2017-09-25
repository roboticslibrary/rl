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

#include "RrtCon.h"
#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		RrtCon::RrtCon() :
			RrtGoalBias()
		{
		}
		
		RrtCon::~RrtCon()
		{
		}
		
		::std::string
		RrtCon::getName() const
		{
			return "RRT Connect";
		}
		
		bool
		RrtCon::solve()
		{
			this->time = ::std::chrono::steady_clock::now();
			
			this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector>(*this->start));
			
			while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
			{
				::rl::math::Vector chosen = this->choose();
				Neighbor nearest = this->nearest(this->tree[0], chosen);
				Vertex connected = this->connect(this->tree[0], nearest, chosen);
				
				if (nullptr != connected)
				{
					if (this->areEqual(*get(this->tree[0], connected)->q, *this->goal))
					{
						this->end[0] = connected;
						return true;
					}
				}
			}
			
			return false;
		}
	}
}
