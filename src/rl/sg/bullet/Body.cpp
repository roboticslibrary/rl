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
		namespace bullet
		{
			Body::Body(Model* model) :
				::rl::sg::Body(model),
				object(),
				shape()
			{
				this->getModel()->add(this);
				
				this->object.setCollisionShape(&this->shape);
				this->object.setUserPointer(this);
				
				dynamic_cast<Scene*>(this->getModel()->getScene())->world.addCollisionObject(&this->object);
			}
			
			Body::~Body()
			{
				dynamic_cast<Scene*>(this->getModel()->getScene())->world.removeCollisionObject(&this->object);
				
				while (this->shapes.size() > 0)
				{
					delete this->shapes[0];
				}
				
				this->getModel()->remove(this);
			}
			
			::rl::sg::Shape*
			Body::create(::SoVRMLShape* shape)
			{
				return new Shape(shape, this);
			}
			
			void
			Body::getFrame(::rl::math::Transform& frame)
			{
				for (int i = 0; i < 3; ++i)
				{
					frame(0, i) = this->object.getWorldTransform().getBasis().getRow(i).getX();
					frame(1, i) = this->object.getWorldTransform().getBasis().getRow(i).getY();
					frame(2, i) = this->object.getWorldTransform().getBasis().getRow(i).getZ();
				}
				
				frame(0, 3) = this->object.getWorldTransform().getOrigin().getX();
				frame(1, 3) = this->object.getWorldTransform().getOrigin().getY();
				frame(2, 3) = this->object.getWorldTransform().getOrigin().getZ();
				
				frame(3, 0) = 0;
				frame(3, 1) = 0;
				frame(3, 2) = 0;
				frame(3, 3) = 1;
			}
			
			void
			Body::setFrame(const ::rl::math::Transform& frame)
			{
				this->object.getWorldTransform().getOrigin().setValue(
					static_cast< ::btScalar>(frame(0, 3)),
					static_cast< ::btScalar>(frame(1, 3)),
					static_cast< ::btScalar>(frame(2, 3))
				);
				
				this->object.getWorldTransform().getBasis().setValue(
					static_cast< ::btScalar>(frame(0, 0)), static_cast< ::btScalar>(frame(0, 1)), static_cast< ::btScalar>(frame(0, 2)),
					static_cast< ::btScalar>(frame(1, 0)), static_cast< ::btScalar>(frame(1, 1)), static_cast< ::btScalar>(frame(1, 2)),
					static_cast< ::btScalar>(frame(2, 0)), static_cast< ::btScalar>(frame(2, 1)), static_cast< ::btScalar>(frame(2, 2))
				);
			}
		}
	}
}
