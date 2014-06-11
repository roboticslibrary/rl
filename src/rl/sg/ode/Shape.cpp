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

#include <Inventor/actions/SoGetPrimitiveCountAction.h>
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
			Shape::Shape(SoVRMLShape* shape, Body* body) :
				::rl::sg::Shape(shape, body),
				geom(NULL),
				indices(NULL),
				vertices(NULL)
			{
				SoVRMLGeometry* geometry = static_cast< SoVRMLGeometry* >(shape->geometry.getValue());
				
				SoGetPrimitiveCountAction* primitiveCountAction = new SoGetPrimitiveCountAction();
				primitiveCountAction->apply(geometry);
				
				if (geometry->isOfType(SoVRMLBox::getClassTypeId()))
				{
					SoVRMLBox* box = static_cast< SoVRMLBox* >(geometry);
					this->geom = dCreateBox(static_cast< Body* >(this->getBody())->space, box->size.getValue()[0], box->size.getValue()[1], box->size.getValue()[2]);
				}
				else if (geometry->isOfType(SoVRMLCone::getClassTypeId()))
				{
					throw Exception("::rl::sg::ode::Shape - SoVRMLCone not supported");
				}
				else if (geometry->isOfType(SoVRMLCylinder::getClassTypeId()))
				{
					SoVRMLCylinder* cylinder = static_cast< SoVRMLCylinder* >(geometry);
					this->geom = dCreateCylinder(static_cast< Body* >(this->getBody())->space, cylinder->radius.getValue(), cylinder->height.getValue());
				}
				else if (geometry->isOfType(SoVRMLIndexedFaceSet::getClassTypeId()))
				{
					SoVRMLIndexedFaceSet* indexedFaceSet = static_cast< SoVRMLIndexedFaceSet* >(geometry);
					
					SoVRMLCoordinate* coord = static_cast< SoVRMLCoordinate* >(indexedFaceSet->coord.getValue());
					
					int vertexCount = coord->point.getNum();
					this->vertices = new dReal[vertexCount * 4];
					
					int indexCount = primitiveCountAction->getTriangleCount() * 3;
					this->indices = new dTriIndex[indexCount];
					
					this->create(coord->point, indexedFaceSet->coordIndex, this->vertices, this->indices);
					
					dTriMeshDataID data = dGeomTriMeshDataCreate();
					dGeomTriMeshDataBuildSimple(data, this->vertices, vertexCount, this->indices, indexCount);
					this->geom = dCreateTriMesh(static_cast< Body* >(this->getBody())->space, data, NULL, NULL, NULL);
				}
				else if (geometry->isOfType(SoVRMLSphere::getClassTypeId()))
				{
					SoVRMLSphere* sphere = static_cast< SoVRMLSphere* >(geometry);
					this->geom = dCreateSphere(static_cast< Body* >(this->getBody())->space, sphere->radius.getValue());
				}
				
				if (NULL != this->geom)
				{
					dGeomSetBody(this->geom, static_cast< Body* >(this->getBody())->body);
					dGeomSetData(this->geom, this);
				}
				
				this->getBody()->add(this);
			}
			
			Shape::~Shape()
			{
				this->getBody()->remove(this);
				
				dGeomDestroy(this->geom);
				
				delete[] this->indices;
				delete[] this->vertices;
			}
			
			void
			Shape::create(const SoMFVec3f& point, const SoMFInt32& coordIndex, dReal* vertices, dTriIndex* indices) const
			{
				::std::size_t index = 0;
				
				for (int i = 0; i < coordIndex.getNum(); ++i)
				{
					if (SO_END_FACE_INDEX != coordIndex[i])
					{
						indices[index] = coordIndex[i];
						++index;
					}
				}
				
				for (int i = 0; i < point.getNum(); ++i)
				{
					vertices[i * 4] = point[i][0];
					vertices[i * 4 + 1] = point[i][1];
					vertices[i * 4 + 2] = point[i][2];
				}
			}
			
			void
			Shape::getTransform(::rl::math::Transform& transform)
			{
				const dReal* position = dGeomGetOffsetPosition(this->geom);
				
				transform(0, 3) = position[0];
				transform(1, 3) = position[1];
				transform(2, 3) = position[2];
				
				const dReal* rotation = dGeomGetOffsetRotation(this->geom);
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					for (::std::size_t j = 0; j < 3; ++j)
					{
						transform(i, j) = rotation[i * 4 + j];
					}
				}
			}
			
			void
			Shape::setTransform(const ::rl::math::Transform& transform)
			{
				dGeomSetOffsetPosition(
					this->geom,
					static_cast< dReal >(transform(0, 3)),
					static_cast< dReal >(transform(1, 3)),
					static_cast< dReal >(transform(2, 3))
				);
				
				dMatrix3 rotation;
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					for (::std::size_t j = 0; j < 3; ++j)
					{
						rotation[i * 4 + j] = static_cast< dReal >(transform(i, j));
					}
				}
				
				rotation[3] = 0.0f;
				rotation[7] = 0.0f;
				rotation[11] = 0.0f;
				
				dGeomSetOffsetRotation(this->geom, rotation);
			}
		}
	}
}
