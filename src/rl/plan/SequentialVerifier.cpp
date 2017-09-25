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

#include "SequentialVerifier.h"
#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		SequentialVerifier::SequentialVerifier() :
			Verifier()
		{
		}
		
		SequentialVerifier::~SequentialVerifier()
		{
		}
		
		bool
		SequentialVerifier::isColliding(const ::rl::math::Vector& u, const ::rl::math::Vector& v, const ::rl::math::Real& d)
		{
			assert(u.size() == this->model->getDofPosition());
			assert(v.size() == this->model->getDofPosition());
			
			::std::size_t steps = static_cast< ::std::size_t>(::std::ceil(d / this->delta)) - 1;
			
			::rl::math::Vector inter(u.size());
			
			for (::std::size_t i = 0; i < steps; ++i)
			{
				this->model->interpolate(u, v, (i + 1.0f) / (steps + 1.0f), inter);
				
				if (!this->model->isValid(inter))
				{
					return true;
				}
				
				this->model->setPosition(inter);
				this->model->updateFrames();
				
				if (this->model->isColliding())
				{
					return true;
				}
			}
			
			return false;
		}
	}
}
