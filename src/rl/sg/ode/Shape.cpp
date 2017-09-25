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
		namespace ode
		{
			Shape::Shape(::SoVRMLShape* shape, Body* body) :
				::rl::sg::Shape(shape, body),
				geom(nullptr),
				baseTransform(::rl::math::Transform::Identity()),
				indices(),
				transform(::rl::math::Transform::Identity()),
				vertices()
			{
				::SoVRMLGeometry* geometry = static_cast< ::SoVRMLGeometry*>(shape->geometry.getValue());
				
				if (geometry->isOfType(::SoVRMLBox::getClassTypeId()))
				{
					::SoVRMLBox* box = static_cast< ::SoVRMLBox*>(geometry);
					this->geom = ::dCreateBox(static_cast<Body*>(this->getBody())->space, box->size.getValue()[0], box->size.getValue()[1], box->size.getValue()[2]);
				}
				else if (geometry->isOfType(::SoVRMLCone::getClassTypeId()))
				{
					throw Exception("::rl::sg::ode::Shape - SoVRMLCone not supported");
				}
				else if (geometry->isOfType(::SoVRMLCylinder::getClassTypeId()))
				{
					::SoVRMLCylinder* cylinder = static_cast< ::SoVRMLCylinder*>(geometry);
					this->geom = ::dCreateCylinder(static_cast<Body*>(this->getBody())->space, cylinder->radius.getValue(), cylinder->height.getValue());
					
					this->baseTransform(0, 0) = 1;
					this->baseTransform(0, 1) = 0;
					this->baseTransform(0, 2) = 0;
					this->baseTransform(0, 3) = 0;
					this->baseTransform(1, 0) = 0;
					this->baseTransform(1, 1) = 0;
					this->baseTransform(1, 2) = 1;
					this->baseTransform(1, 3) = 0;
					this->baseTransform(2, 0) = 0;
					this->baseTransform(2, 1) = -1;
					this->baseTransform(2, 2) = 0;
					this->baseTransform(2, 3) = 0;
					this->baseTransform(3, 0) = 0;
					this->baseTransform(3, 1) = 0;
					this->baseTransform(3, 2) = 0;
					this->baseTransform(3, 3) = 1;
				}
				else if (geometry->isOfType(::SoVRMLIndexedFaceSet::getClassTypeId()))
				{
					::SoCallbackAction callbackAction;
					callbackAction.addTriangleCallback(geometry->getTypeId(), Shape::triangleCallback, this);
					callbackAction.apply(geometry);
					
					::dTriMeshDataID data = ::dGeomTriMeshDataCreate();
					::dGeomTriMeshDataBuildSimple(data, &this->vertices[0], this->vertices.size() / 4, &this->indices[0], this->indices.size());
					this->geom = ::dCreateTriMesh(static_cast<Body*>(this->getBody())->space, data, nullptr, nullptr, nullptr);
				}
				else if (geometry->isOfType(::SoVRMLSphere::getClassTypeId()))
				{
					::SoVRMLSphere* sphere = static_cast< ::SoVRMLSphere*>(geometry);
					this->geom = ::dCreateSphere(static_cast<Body*>(this->getBody())->space, sphere->radius.getValue());
				}
				else
				{
					throw Exception("::rl::sg::ode::Shape() - geometry not supported");
				}
				
				if (nullptr != this->geom)
				{
					::dGeomSetBody(this->geom, static_cast<Body*>(this->getBody())->body);
					::dGeomSetData(this->geom, this);
				}
				
				this->getBody()->add(this);

				this->setTransform(::rl::math::Transform::Identity());
			}
			
			Shape::~Shape()
			{
				this->getBody()->remove(this);
				::dGeomDestroy(this->geom);
			}
			
			void
			Shape::getTransform(::rl::math::Transform& transform)
			{
				transform = this->transform;
			}
			
			void
			Shape::triangleCallback(void* userData, SoCallbackAction* action, const SoPrimitiveVertex* v1, const SoPrimitiveVertex* v2, const SoPrimitiveVertex* v3)
			{
				Shape* shape = static_cast<Shape*>(userData);
				
				shape->indices.push_back(shape->vertices.size() / 4);
				
				shape->vertices.push_back(v1->getPoint()[0]);
				shape->vertices.push_back(v1->getPoint()[1]);
				shape->vertices.push_back(v1->getPoint()[2]);
				shape->vertices.push_back(0);
				
				shape->indices.push_back(shape->vertices.size() / 4);
				
				shape->vertices.push_back(v2->getPoint()[0]);
				shape->vertices.push_back(v2->getPoint()[1]);
				shape->vertices.push_back(v2->getPoint()[2]);
				shape->vertices.push_back(0);
				
				shape->indices.push_back(shape->vertices.size() / 4);
				
				shape->vertices.push_back(v3->getPoint()[0]);
				shape->vertices.push_back(v3->getPoint()[1]);
				shape->vertices.push_back(v3->getPoint()[2]);
				shape->vertices.push_back(0);
			}
			
			void
			Shape::setTransform(const ::rl::math::Transform& transform)
			{
				this->transform = transform;
				
				::rl::math::Transform totalTransform = this->transform * this->baseTransform;
				
				::dGeomSetOffsetPosition(
					this->geom,
					static_cast< ::dReal>(totalTransform(0, 3)),
					static_cast< ::dReal>(totalTransform(1, 3)),
					static_cast< ::dReal>(totalTransform(2, 3))
				);
				
				::dMatrix3 rotation;
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					for (::std::size_t j = 0; j < 3; ++j)
					{
						rotation[i * 4 + j] = static_cast< ::dReal>(totalTransform(i, j));
					}
				}
				
				rotation[3] = 0.0f;
				rotation[7] = 0.0f;
				rotation[11] = 0.0f;
				
				::dGeomSetOffsetRotation(this->geom, rotation);
			}
		}
	}
}
