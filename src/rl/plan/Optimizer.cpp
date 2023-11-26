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

#include "Optimizer.h"
#include "SimpleModel.h"
#include "Viewer.h"

namespace rl
{
	namespace plan
	{
		Optimizer::Optimizer() :
			model(nullptr),
			verifier(nullptr),
			viewer(nullptr)
		{
		}
		
		Optimizer::~Optimizer()
		{
		}
		
		SimpleModel*
		Optimizer::getModel() const
		{
			return this->model;
		}
		
		Verifier*
		Optimizer::getVerifier() const
		{
			return this->verifier;
		}
		
		Viewer*
		Optimizer::getViewer() const
		{
			return this->viewer;
		}
		
		void
		Optimizer::setModel(SimpleModel* model)
		{
			this->model = model;
		}
		
		void
		Optimizer::setVerifier(Verifier* verifier)
		{
			this->verifier = verifier;
		}
		
		void
		Optimizer::setViewer(Viewer* viewer)
		{
			this->viewer = viewer;
		}
		
		bool
		Optimizer::subdivide(VectorList& path, const ::rl::math::Real& length)
		{
			bool changed = false;
			::rl::math::Vector inter(this->getModel()->getDofPosition());
			
			VectorList::iterator i = path.begin();
			VectorList::iterator j = ++path.begin();
				
			while (i != path.end() && j != path.end())
			{
				if (length > 0 && this->getModel()->distance(*i, *j) > length)
				{
					this->getModel()->interpolate(*i, *j, static_cast<::rl::math::Real>(0.5), inter);
					
					j = path.insert(j, inter);
					
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
				}
			}
			
			return changed;
		}
	}
}
