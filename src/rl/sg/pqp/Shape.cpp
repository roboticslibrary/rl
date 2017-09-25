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

#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoGetPrimitiveCountAction.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include <Inventor/VRMLnodes/SoVRMLCone.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLCylinder.h>
#include <Inventor/VRMLnodes/SoVRMLGeometry.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLSphere.h>

#include "Body.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace pqp
		{
			Shape::Shape(::SoVRMLShape* shape, Body* body) :
				::rl::sg::Shape(shape, body),
				model(),
				frame(::rl::math::Transform::Identity()),
				transform(::rl::math::Transform::Identity())
			{
				for (::std::size_t i = 0; i < 3; ++i)
				{
					for (::std::size_t j = 0; j < 3; ++j)
					{
						this->rotation[i][j] = static_cast<PQP_REAL>(this->frame(i, j));
					}
				}
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					this->translation[i] = static_cast<PQP_REAL>(this->frame(i, 3));
				}
				
				::SoVRMLGeometry* geometry = static_cast< ::SoVRMLGeometry*>(shape->geometry.getValue());
				
				::SoGetPrimitiveCountAction* primitiveCountAction = new ::SoGetPrimitiveCountAction();
				primitiveCountAction->apply(geometry);
				
				this->model.BeginModel(primitiveCountAction->getTriangleCount());
				
				Model model(&this->model, 0);
				
				::SoCallbackAction callbackAction;
				callbackAction.addTriangleCallback(geometry->getTypeId(), Shape::triangleCallback, &model);
				callbackAction.apply(geometry);
				
				this->model.EndModel();
				
				this->getBody()->add(this);
			}
			
			Shape::~Shape()
			{
				this->getBody()->remove(this);
			}
			
			void
			Shape::getTransform(::rl::math::Transform& transform)
			{
				transform = this->transform;
			}
			
			void
			Shape::setTransform(const ::rl::math::Transform& transform)
			{
				this->transform = transform;
				
				this->update();
			}
			
			void
			Shape::transformToWorld(const ::rl::math::Vector3& local, ::rl::math::Vector3& world) const
			{
				world = this->frame * local;
			}
			
			void
			Shape::triangleCallback(void* userData, ::SoCallbackAction* action, const ::SoPrimitiveVertex* v1, const ::SoPrimitiveVertex* v2, const ::SoPrimitiveVertex* v3)
			{
				PQP_REAL p[3][3] = {
					{v1->getPoint()[0], v1->getPoint()[1], v1->getPoint()[2]},
					{v2->getPoint()[0], v2->getPoint()[1], v2->getPoint()[2]},
					{v3->getPoint()[0], v3->getPoint()[1], v3->getPoint()[2]}
				};
				
				Model* model = static_cast<Model*>(userData);
				
				model->first->AddTri(p[0], p[1], p[2], model->second);
				
				++model->second;
			}
			
			void
			Shape::update()
			{
				this->frame = static_cast<Body*>(this->getBody())->frame * this->transform;
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					for (::std::size_t j = 0; j < 3; ++j)
					{
						this->rotation[i][j] = static_cast<PQP_REAL>(this->frame(i, j));
					}
				}
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					this->translation[i] = static_cast<PQP_REAL>(this->frame(i, 3));
				}
			}
		}
	}
}
