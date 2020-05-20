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

#include "BridgeSampler.h"
#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		BridgeSampler::BridgeSampler() :
			GaussianSampler(),
			ratio(static_cast<::rl::math::Real>(5) / static_cast<::rl::math::Real>(6))
		{
		}
		
		BridgeSampler::~BridgeSampler()
		{
		}
		
		::rl::math::Vector
		BridgeSampler::generateCollisionFree()
		{
			if (this->rand() > this->ratio)
			{
				return this->generate();
			}
			else
			{
				::rl::math::Vector q(this->model->getDof());
				::rl::math::Vector gauss(this->model->getDof());
				
				while (true)
				{
					::rl::math::Vector q2 = this->generate();
					
					if (this->model->isColliding(q2))
					{
						for (::std::size_t i = 0; i < this->model->getDof(); ++i)
						{
							gauss(i) = this->gauss();
						}
						
						::rl::math::Vector q3 = this->model->generatePositionGaussian(gauss, q2, *this->sigma);
						
						if (this->model->isColliding(q3))
						{
							this->model->interpolate(q2, q3, static_cast<::rl::math::Real>(0.5), q);
							
							if (!this->model->isColliding(q))
							{
								return q;
							}
						}
					}
				}
			}
		}
	}
}
