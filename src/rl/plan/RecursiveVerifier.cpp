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

#include "RecursiveVerifier.h"
#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		RecursiveVerifier::RecursiveVerifier() :
			Verifier()
		{
		}
		
		RecursiveVerifier::~RecursiveVerifier()
		{
		}
		
		bool
		RecursiveVerifier::isColliding(const ::rl::math::Vector& u, const ::rl::math::Vector& v, const ::rl::math::Real& d)
		{
			assert(u.size() == this->model->getDofPosition());
			assert(v.size() == this->model->getDofPosition());
			
			::rl::math::Real exponent = ::std::ceil(
				::std::log(d / this->delta) /
				::std::log(static_cast< ::rl::math::Real>(2))
			);
			
			::rl::math::Real steps = ::std::pow(2, exponent);
			
			::rl::math::Vector inter(u.size());
			
			for (int i = 0; i < steps - 1; ++i)
			{
				// reverse bits
				
				::std::size_t tmp = i + 1;
				::std::size_t tmp2 = tmp & 1;
				
				for (int j = 0; j < exponent - 1; ++j)
				{
					tmp >>= 1;
					tmp2 <<= 1;
					tmp2 |= tmp & 1;
				}
				
				this->model->interpolate(u, v, tmp2 / steps, inter);
				
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
