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

#include "Body.h"
#include "Model.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace solid
		{
			Body::Body(Model* model) :
				::rl::sg::Body(model),
				frame(::rl::math::Transform::Identity())
			{
				this->getModel()->add(this);
			}
			
			Body::~Body()
			{
				while (this->shapes.size() > 0)
				{
					delete this->shapes[0];
				}
				
				this->getModel()->remove(this);
			}
			
			::rl::sg::Shape*
			Body::create(SoVRMLShape* shape)
			{
				return new Shape(shape, this);
			}
			
			void
			Body::getFrame(::rl::math::Transform& frame)
			{
				frame = this->frame;
			}
			
			void
			Body::setFrame(const ::rl::math::Transform& frame)
			{
				this->frame = frame;
				
				for (Iterator i = this->begin(); i != this->end(); ++i)
				{
					static_cast<Shape*>(*i)->update();
				}
			}
			
			void
			Body::setMargin(const ::rl::math::Real& margin)
			{
				for (Iterator i = this->begin(); i != this->end(); ++i)
				{
					static_cast<Shape*>(*i)->setMargin(margin);
				}
			}
		}
	}
}
