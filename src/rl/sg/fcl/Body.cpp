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
		namespace fcl
		{
			Body::Body(Model* model) :
				::rl::sg::Body(model),
				manager(),
				frame(::rl::math::Transform::Identity())
			{
				this->manager.setup();
				this->getModel()->add(this);
			}
			
			Body::~Body()
			{
				for (::std::size_t i = 0; i < this->collisionObjects.size(); ++i)
				{
					manager.unregisterObject(this->collisionObjects[i]);
				}
				
				while (this->shapes.size() > 0)
				{
					delete this->shapes[0];
				}

				this->getModel()->remove(this);
			}
			
			::rl::sg::Shape*
			Body::create(SoVRMLShape* shape)
			{
				Shape* newShape = new Shape(shape, this);
				return newShape;
			}
			
			void
			Body::add(::rl::sg::Shape* shape)
			{
				Shape* fclShape = static_cast<Shape*>(shape);
				this->shapes.push_back(fclShape);
				fclShape->update(frame);
				this->manager.registerObject(fclShape->collisionObject.get());
				static_cast<Model*>(getModel())->addCollisionObject(fclShape->collisionObject.get(), this);
			}
			
			void
			Body::getFrame(::rl::math::Transform& frame)
			{
				frame = this->frame;
			}
			
			void
			Body::remove(::rl::sg::Shape* shape)
			{
				Iterator found = ::std::find(this->shapes.begin(), this->shapes.end(), shape);
			
				if (found != this->shapes.end())
				{
					Shape* fclShape = static_cast<Shape*>(shape);
					this->shapes.erase(found);
					this->manager.unregisterObject(fclShape->collisionObject.get());
					static_cast<Model*>(this->getModel())->removeCollisionObject(fclShape->collisionObject.get());
				}
			}
			
			void
			Body::setFrame(const ::rl::math::Transform& frame)
			{	
				this->frame = frame;
				
				for (Iterator i = this->begin(); i != this->end(); ++i)
				{
					static_cast<Shape*>(*i)->update(this->frame);
				}
			}
		}
	}
}
