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

#include <Inventor/SbLinear.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>

#include "Body.h"
#include "Model.h"
#include "Scene.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace so
		{
			Body::Body(Model* model) :
				::rl::sg::Body(model),
				root(new ::SoVRMLTransform())
			{
				this->root->ref();
				this->root->setUserData(this);
				
				this->getModel()->add(this);
			}
			
			Body::~Body()
			{
				while (this->shapes.size() > 0)
				{
					delete this->shapes[0];
				}
				
				this->getModel()->remove(this);
				
				this->root->unref();
			}
			
			void
			Body::add(::rl::sg::Shape* shape)
			{
				::rl::sg::Body::add(shape);
				
				this->root->addChild(static_cast<Shape*>(shape)->root);
			}
			
			::rl::sg::Shape*
			Body::create(::SoVRMLShape* shape)
			{
				return new Shape(shape, this);
			}
			
			void
			Body::getFrame(::rl::math::Transform& frame)
			{
				::SoSearchAction searchAction;
				searchAction.setNode(this->root);
				searchAction.apply(static_cast<Scene*>(this->getModel()->getScene())->root);
				
				::SbViewportRegion viewportRegion;
				::SoGetMatrixAction getMatrixAction(viewportRegion);
				getMatrixAction.apply(searchAction.getPath());
				::SbMatrix matrix = getMatrixAction.getMatrix();
				
				for (int i = 0; i < 4; ++i)
				{
					for (int j = 0; j < 4; ++j)
					{
						frame(i, j) = matrix[j][i];
					}
				}
			}
			
			::std::string
			Body::getName() const
			{
				return this->root->getName().getString();
			}
			
			void
			Body::remove(::rl::sg::Shape* shape)
			{
				this->root->removeChild(static_cast<Shape*>(shape)->root);
				
				::rl::sg::Body::remove(shape);
			}
			
			void
			Body::setFrame(const ::rl::math::Transform& frame)
			{
				::SbMatrix matrix;
				
				for (int i = 0; i < 4; ++i)
				{
					for (int j = 0; j < 4; ++j)
					{
						matrix[i][j] = static_cast<float>(frame(j, i));
					}
				}
				
				this->root->setMatrix(matrix);
			}
			
			void
			Body::setName(const ::std::string& name)
			{
				this->root->setName(name.c_str());
			}
		}
	}
}
