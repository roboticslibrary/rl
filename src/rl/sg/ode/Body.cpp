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
#include "Scene.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace ode
		{
			Body::Body(Model* model) :
				::rl::sg::Body(model),
				body(::dBodyCreate(dynamic_cast<Scene*>(getModel()->getScene())->world)),
				space(::dSimpleSpaceCreate(static_cast<Model*>(getModel())->space))
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
				
				::dBodyDestroy(this->body);
				::dSpaceDestroy(this->space);
			}
			
			::rl::sg::Shape*
			Body::create(::SoVRMLShape* shape)
			{
				return new Shape(shape, this);
			}
			
			void
			Body::getFrame(::rl::math::Transform& frame)
			{
				const ::dReal* position = ::dBodyGetPosition(this->body);
				
				frame(0, 3) = position[0];
				frame(1, 3) = position[1];
				frame(2, 3) = position[2];
				
				const ::dReal* rotation = ::dBodyGetRotation(this->body);
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					for (::std::size_t j = 0; j < 3; ++j)
					{
						frame(i, j) = rotation[i * 4 + j];
					}
				}
			}
			
			void
			Body::setFrame(const ::rl::math::Transform& frame)
			{
				::dBodySetPosition(
					this->body,
					static_cast<dReal>(frame(0, 3)),
					static_cast<dReal>(frame(1, 3)),
					static_cast<dReal>(frame(2, 3))
				);
				
				::dMatrix3 rotation;
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					for (::std::size_t j = 0; j < 3; ++j)
					{
						rotation[i * 4 + j] = static_cast< ::dReal>(frame(i, j));
					}
				}
				
				rotation[3] = 0.0f;
				rotation[7] = 0.0f;
				rotation[11] = 0.0f;
				
				::dBodySetRotation(this->body, rotation);
			}
		}
	}
}
