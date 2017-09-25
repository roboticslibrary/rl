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

#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include <Inventor/VRMLnodes/SoVRMLCone.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLCylinder.h>
#include <Inventor/VRMLnodes/SoVRMLGeometry.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLSphere.h>

#include "../Exception.h"
#include "Body.h"
#include "Model.h"
#include "Scene.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace solid
		{
			Shape::Shape(SoVRMLShape* shape, Body* body) :
				::rl::sg::Shape(shape, body),
				complex(false),
				encounters(),
				object(),
				proxy(),
				frame(::rl::math::Transform::Identity()),
				max(),
				min(),
				shape(Shape::create(shape)),
				transform(::rl::math::Transform::Identity())
			{
				this->object = DT_CreateObject(this, this->shape);
				DT_AddObject(dynamic_cast<Scene*>(this->getBody()->getModel()->getScene())->scene, this->object);
				
				DT_GetBBox(this->object, this->min, this->max);
				this->proxy = BP_CreateProxy(dynamic_cast<Scene*>(this->getBody()->getModel()->getScene())->broad, this, this->min, this->max);
				
				this->getBody()->add(this);
			}
			
			Shape::~Shape()
			{
				this->getBody()->remove(this);
				
				for (::std::unordered_set<Shape*>::iterator i = this->encounters.begin(); i != this->encounters.end(); ++i)
				{
					(*i)->encounters.erase(this);
				}
				
				if (nullptr != this->proxy)
				{
					BP_DestroyProxy(dynamic_cast<Scene*>(this->getBody()->getModel()->getScene())->broad, this->proxy);
				}
				
				if (nullptr != this->object)
				{
					DT_RemoveObject(dynamic_cast<Scene*>(this->getBody()->getModel()->getScene())->scene, this->object);
					DT_DestroyObject(this->object);
				}
				
				if (nullptr != this->shape)
				{
					DT_DeleteShape(this->shape);
				}
			}
			
			DT_ShapeHandle
			Shape::create(SoVRMLShape* shape)
			{
				SoVRMLGeometry* geometry = static_cast<SoVRMLGeometry*>(shape->geometry.getValue());
				
				if (geometry->isOfType(SoVRMLBox::getClassTypeId()))
				{
					SoVRMLBox* box = static_cast<SoVRMLBox*>(geometry);
					return DT_NewBox(box->size.getValue()[0], box->size.getValue()[1], box->size.getValue()[2]);
				}
				else if (geometry->isOfType(SoVRMLCone::getClassTypeId()))
				{
					SoVRMLCone* cone = static_cast<SoVRMLCone*>(geometry);
					return DT_NewCone(cone->bottomRadius.getValue(), cone->height.getValue());
				}
				else if (geometry->isOfType(SoVRMLCylinder::getClassTypeId()))
				{
					SoVRMLCylinder* cylinder = static_cast<SoVRMLCylinder*>(geometry);
					return DT_NewCylinder(cylinder->radius.getValue(), cylinder->height.getValue());
				}
				else if (geometry->isOfType(SoVRMLIndexedFaceSet::getClassTypeId()))
				{
					SoVRMLIndexedFaceSet* indexedFaceSet = static_cast<SoVRMLIndexedFaceSet*>(geometry);
					
					if (indexedFaceSet->convex.getValue())
					{
						return Shape::create(static_cast<SoVRMLCoordinate*>(indexedFaceSet->coord.getValue())->point);
					}
					else
					{
						this->complex = true;
						
						return Shape::create(static_cast<SoVRMLCoordinate*>(indexedFaceSet->coord.getValue())->point, indexedFaceSet->coordIndex);
					}
				}
				else if (geometry->isOfType(SoVRMLSphere::getClassTypeId()))
				{
					SoVRMLSphere* sphere = static_cast<SoVRMLSphere*>(geometry);
					return DT_NewSphere(sphere->radius.getValue());
				}
				else
				{
					throw Exception("::rl::sg::solid::Shape::create(SoVRMLShape* shape) - geometry not supported");
				}
				
				return nullptr;
			}
			
			DT_ShapeHandle
			Shape::create(const SoMFVec3f& point)
			{
				DT_ShapeHandle shape = DT_NewPolytope(nullptr);
				
				DT_Vector3 vertex;
				
				for (int i = 0; i < point.getNum(); ++i)
				{
					vertex[0] = point[i][0];
					vertex[1] = point[i][1];
					vertex[2] = point[i][2];
					
					DT_Vertex(vertex);
				}
				
				DT_EndPolytope();
				
				return shape;
			}
			
			DT_ShapeHandle
			Shape::create(const SoMFVec3f& point, const SoMFInt32& coordIndex)
			{
				DT_ShapeHandle shape = DT_NewComplexShape(nullptr);
				
				DT_Vector3 vertex;
				
				if (coordIndex.getNum() > 0)
				{
					DT_Begin();
				}
				
				for (int i = 0; i < coordIndex.getNum(); ++i)
				{
					if (SO_END_FACE_INDEX == coordIndex[i])
					{
						DT_End();
						
						if (i < coordIndex.getNum() - 1)
						{
							DT_Begin();
						}
					}
					else
					{
						vertex[0] = point[coordIndex[i]][0];
						vertex[1] = point[coordIndex[i]][1];
						vertex[2] = point[coordIndex[i]][2];
						
						DT_Vertex(vertex);
					}
				}
				
				DT_EndComplexShape();
				
				return shape;
			}
			
			void
			Shape::getTransform(::rl::math::Transform& transform)
			{
				double m[16];
				
				DT_GetMatrixd(this->object, m);
				
				for (::std::size_t i = 0; i < 4; ++i)
				{
					for (::std::size_t j = 0; j < 4; ++j)
					{
						this->frame(i, j) = static_cast< ::rl::math::Real>(m[i + j * 4]);
					}
				}
				
				this->transform = static_cast<Body*>(this->getBody())->frame.inverse() * this->frame;
				
				transform = this->transform;
			}
			
			void
			Shape::setMargin(const ::rl::math::Real& margin)
			{
				DT_SetMargin(this->object, static_cast<DT_Scalar>(margin));
			}
			
			void
			Shape::setTransform(const ::rl::math::Transform& transform)
			{
				this->transform = transform;
				
				this->update();
			}
			
			void
			Shape::update()
			{
				this->frame = static_cast<Body*>(this->getBody())->frame * this->transform;
				
				double m[16];
				
				for (::std::size_t i = 0; i < 4; ++i)
				{
					for (::std::size_t j = 0; j < 4; ++j)
					{
						m[i + j * 4] = static_cast<double>(this->frame(i, j));
					}
				}
				
				DT_SetMatrixd(this->object, m);
				
				DT_GetBBox(this->object, this->min, this->max);
				
				BP_SetBBox(this->proxy, this->min, this->max);
			}
		}
	}
}
