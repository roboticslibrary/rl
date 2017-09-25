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
		namespace bullet
		{
			Shape::Shape(::SoVRMLShape* shape, Body* body) :
				::rl::sg::Shape(shape, body),
				shape(nullptr),
				transform(),
				indices(),
				triangleIndexVertexArray(nullptr),
				vertices()
			{
				::SoVRMLGeometry* geometry = static_cast< ::SoVRMLGeometry*>(shape->geometry.getValue());
				
				if (geometry->isOfType(::SoVRMLBox::getClassTypeId()))
				{
					::SoVRMLBox* box = static_cast< ::SoVRMLBox*>(geometry);
					::btVector3 boxHalfExtents(box->size.getValue()[0] / 2, box->size.getValue()[1] / 2, box->size.getValue()[2] / 2);
					this->shape = new ::btBoxShape(boxHalfExtents);
				}
				else if (geometry->isOfType(::SoVRMLCone::getClassTypeId()))
				{
					::SoVRMLCone* cone = static_cast< ::SoVRMLCone*>(geometry);
					this->shape = new ::btConeShape(cone->bottomRadius.getValue(), cone->height.getValue());
				}
				else if (geometry->isOfType(::SoVRMLCylinder::getClassTypeId()))
				{
					::SoVRMLCylinder* cylinder = static_cast< ::SoVRMLCylinder*>(geometry);
					this->shape = new ::btCylinderShape(::btVector3(cylinder->radius.getValue(), cylinder->height.getValue() / 2, cylinder->radius.getValue()));
				}
				else if (geometry->isOfType(::SoVRMLIndexedFaceSet::getClassTypeId()))
				{
					::SoVRMLIndexedFaceSet* indexedFaceSet = static_cast< ::SoVRMLIndexedFaceSet*>(geometry);
					
					::SoCallbackAction callbackAction;
					callbackAction.addTriangleCallback(geometry->getTypeId(), Shape::triangleCallback, this);
					callbackAction.apply(geometry);
					
					if (indexedFaceSet->convex.getValue())
					{
						this->shape = new ::btConvexHullShape(
							&this->vertices[0],
							this->vertices.size() / 3,
							3 * sizeof(btScalar)
						);
					}
					else
					{
						this->triangleIndexVertexArray = new ::btTriangleIndexVertexArray(
							this->indices.size() / 3,
							&this->indices[0],
							3 * sizeof(int),
							this->vertices.size() / 3,
							&this->vertices[0],
							3 * sizeof(btScalar)
						);
						
						this->shape = new ::btBvhTriangleMeshShape(this->triangleIndexVertexArray, true);
					}
				}
				else if (geometry->isOfType(::SoVRMLSphere::getClassTypeId()))
				{
					::SoVRMLSphere* sphere = static_cast< ::SoVRMLSphere*>(geometry);
					this->shape = new ::btSphereShape(sphere->radius.getValue());
				}
				else
				{
					throw Exception("::rl::sg::bullet::Shape() - geometry not supported");
				}
				
				this->getBody()->add(this);
				
				if (nullptr != this->shape)
				{
					this->transform.setIdentity();
					dynamic_cast<Body*>(this->getBody())->shape.addChildShape(this->transform, this->shape);
					this->shape->setMargin(0);
					this->shape->setUserPointer(this);
				}
			}
			
			Shape::~Shape()
			{
				if (nullptr != this->shape)
				{
					dynamic_cast<Body*>(this->getBody())->shape.removeChildShape(this->shape);
				}
				
				this->getBody()->remove(this);
				
				delete this->triangleIndexVertexArray;
				delete this->shape;
			}
			
			void
			Shape::getTransform(::rl::math::Transform& transform)
			{
				Body* body = static_cast<Body*>(this->getBody());
				
				for (int i = 0; i < body->shape.getNumChildShapes(); ++i)
				{
					if (this->shape == body->shape.getChildList()[i].m_childShape)
					{
						for (int j = 0; j < 3; ++j)
						{
							transform(0, j) = body->shape.getChildTransform(i).getBasis().getRow(j).getX();
							transform(1, j) = body->shape.getChildTransform(i).getBasis().getRow(j).getY();
							transform(2, j) = body->shape.getChildTransform(i).getBasis().getRow(j).getZ();
						}
						
						transform(0, 3) = body->shape.getChildTransform(i).getOrigin().getX();
						transform(1, 3) = body->shape.getChildTransform(i).getOrigin().getY();
						transform(2, 3) = body->shape.getChildTransform(i).getOrigin().getZ();
						
						transform(3, 0) = 0;
						transform(3, 1) = 0;
						transform(3, 2) = 0;
						transform(3, 3) = 1;
						
						break;
					}
				}
			}
			
			void
			Shape::setTransform(const ::rl::math::Transform& transform)
			{
				Body* body = static_cast<Body*>(this->getBody());
				
				for (int i = 0; i < body->shape.getNumChildShapes(); ++i)
				{
					if (this->shape == body->shape.getChildList()[i].m_childShape)
					{
						this->transform.getOrigin().setValue(
							static_cast< ::btScalar>(transform(0, 3)),
							static_cast< ::btScalar>(transform(1, 3)),
							static_cast< ::btScalar>(transform(2, 3))
						);
						
						this->transform.getBasis().setValue(
							static_cast< ::btScalar>(transform(0, 0)), static_cast< ::btScalar>(transform(0, 1)), static_cast< ::btScalar>(transform(0, 2)),
							static_cast< ::btScalar>(transform(1, 0)), static_cast< ::btScalar>(transform(1, 1)), static_cast< ::btScalar>(transform(1, 2)),
							static_cast< ::btScalar>(transform(2, 0)), static_cast< ::btScalar>(transform(2, 1)), static_cast< ::btScalar>(transform(2, 2))
						);
						
						body->shape.updateChildTransform(i, this->transform);
						
						break;
					}
				}
			}
			
			void
			Shape::triangleCallback(void* userData, ::SoCallbackAction* action, const ::SoPrimitiveVertex* v1, const ::SoPrimitiveVertex* v2, const ::SoPrimitiveVertex* v3)
			{
				Shape* shape = static_cast<Shape*>(userData);
				
				shape->indices.push_back(shape->vertices.size() / 3);
				
				shape->vertices.push_back(v1->getPoint()[0]);
				shape->vertices.push_back(v1->getPoint()[1]);
				shape->vertices.push_back(v1->getPoint()[2]);
				
				shape->indices.push_back(shape->vertices.size() / 3);
				
				shape->vertices.push_back(v2->getPoint()[0]);
				shape->vertices.push_back(v2->getPoint()[1]);
				shape->vertices.push_back(v2->getPoint()[2]);
				
				shape->indices.push_back(shape->vertices.size() / 3);
				
				shape->vertices.push_back(v3->getPoint()[0]);
				shape->vertices.push_back(v3->getPoint()[1]);
				shape->vertices.push_back(v3->getPoint()[2]);
			}
		}
	}
}
