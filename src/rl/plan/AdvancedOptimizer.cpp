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

#include "AdvancedOptimizer.h"
#include "SimpleModel.h"
#include "Verifier.h"
#include "Viewer.h"

namespace rl
{
	namespace plan
	{
		AdvancedOptimizer::AdvancedOptimizer() :
			SimpleOptimizer(),
			length(1),
			ratio(static_cast<::rl::math::Real>(0.1))
		{
		}
		
		AdvancedOptimizer::~AdvancedOptimizer()
		{
		}
		
		::rl::math::Real
		AdvancedOptimizer::getLength() const
		{
			return this->length;
		}
		
		::rl::math::Real
		AdvancedOptimizer::getRatio() const
		{
			return this->ratio;
		}
		
		void
		AdvancedOptimizer::process(VectorList& path)
		{
			bool changed = true;
			::rl::math::Vector inter(this->getModel()->getDofPosition());
			
			while (changed && path.size() > 2)
			{
				while (changed && path.size() > 2)
				{
					changed = false;
					
					for (VectorList::iterator i = path.begin(), j = ::std::next(i), k = ::std::next(j); i != path.end() && j != path.end() && k != path.end();)
					{
						::rl::math::Real ik = this->getModel()->distance(*i, *k);
						
						if (!this->getVerifier()->isColliding(*i, *k, ik))
						{
							::rl::math::Real ij = this->getModel()->distance(*i, *j);
							::rl::math::Real jk = this->getModel()->distance(*j, *k);
							::rl::math::Real alpha = ij / (ij + jk);
							this->getModel()->interpolate(*i, *k, alpha, inter);
							::rl::math::Real ratio = this->getModel()->distance(*j, inter) / ik;
							
							if (ratio > this->ratio)
							{
								VectorList::iterator l = j;
								++j;
								++k;
								path.erase(l);
								
								if (nullptr != this->getViewer())
								{
									this->getViewer()->drawConfigurationPath(path);
								}
								
								changed = true;
							}
							else
							{
								++i;
								++j;
								++k;
							}
						}
						else
						{
							++i;
							++j;
							++k;
						}
					}
				}
				
				if (this->subdivide(path, this->length))
				{
					changed = true;
				}
			}
		}
		
		void
		AdvancedOptimizer::setLength(const ::rl::math::Real& length)
		{
			this->length = length;
		}
		
		void
		AdvancedOptimizer::setRatio(const ::rl::math::Real& ratio)
		{
			this->ratio = ratio;
		}
	}
}
