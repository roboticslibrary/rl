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

#include <fcl/broadphase/broadphase.h>
#include <fcl/BVH/BVH_model.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include <Inventor/VRMLnodes/SoVRMLCone.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLCylinder.h>
#include <Inventor/VRMLnodes/SoVRMLGeometry.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLSphere.h>

#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
#include <boost/make_shared.hpp>
#endif

#include "Body.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace fcl
		{
			Shape::Shape(SoVRMLShape* shape, ::rl::sg::Body* body) :
				::rl::sg::Shape(shape, body),
				baseTransform(::rl::math::Transform::Identity()),
				currentFrame(::rl::math::Transform::Identity()),
				transform(::rl::math::Transform::Identity())
			{
				SoVRMLGeometry* vrmlGeometry = static_cast<SoVRMLGeometry*>(shape->geometry.getValue());
				
				if (vrmlGeometry->isOfType(SoVRMLBox::getClassTypeId()))
				{
					SoVRMLBox* box = static_cast<SoVRMLBox*>(vrmlGeometry);
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
					this->geometry = ::boost::make_shared< ::fcl::Box>(box->size.getValue()[0], box->size.getValue()[1], box->size.getValue()[2]);
#else
					this->geometry = ::std::make_shared< ::fcl::Box>(box->size.getValue()[0], box->size.getValue()[1], box->size.getValue()[2]);
#endif
				}
				else if (vrmlGeometry->isOfType(SoVRMLCone::getClassTypeId()))
				{
					SoVRMLCone* cone = static_cast<SoVRMLCone*>(vrmlGeometry);
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
					this->geometry = ::boost::make_shared< ::fcl::Cone>(cone->bottomRadius.getValue(), cone->height.getValue());
#else
					this->geometry = ::std::make_shared< ::fcl::Cone>(cone->bottomRadius.getValue(), cone->height.getValue());
#endif
					
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
				else if (vrmlGeometry->isOfType(SoVRMLCylinder::getClassTypeId()))
				{
					SoVRMLCylinder* cylinder = static_cast<SoVRMLCylinder*>(vrmlGeometry);
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
					this->geometry = ::boost::make_shared< ::fcl::Cylinder>(cylinder->radius.getValue(), cylinder->height.getValue());
#else
					this->geometry = ::std::make_shared< ::fcl::Cylinder>(cylinder->radius.getValue(), cylinder->height.getValue());
#endif
					
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
				else if (vrmlGeometry->isOfType(SoVRMLIndexedFaceSet::getClassTypeId()))
				{
					SoVRMLIndexedFaceSet* indexedFaceSet = static_cast<SoVRMLIndexedFaceSet*>(vrmlGeometry);
					
					SoCallbackAction callbackAction;
					callbackAction.addTriangleCallback(vrmlGeometry->getTypeId(), Shape::triangleCallback, this);
					callbackAction.apply(vrmlGeometry);
					
					if (indexedFaceSet->convex.getValue() && !indexedFaceSet->convex.isDefault())
					{
						this->normals = ::std::vector< ::fcl::Vec3f>(this->indices.size() / 3);
						this->distances = ::std::vector< ::fcl::FCL_REAL>(this->indices.size() / 3);
						this->polygons = ::std::vector<int>(this->indices.size() + this->indices.size() / 3);
						
						for (::std::size_t i = 0; i < this->indices.size() / 3; ++i)
						{
							::fcl::Vec3f normal = (this->vertices[this->indices[i * 3 + 1]] - this->vertices[this->indices[i * 3]]).cross(this->vertices[this->indices[i * 3 + 2]] - this->vertices[this->indices[i * 3]]);
							this->normals[i] = normal.normalize();
							this->distances[i] = this->vertices[this->indices[i * 3 + 1]].dot(normal);
							this->polygons[i * 4] = 3;
							this->polygons[i * 4 + 1] = this->indices[i * 3];
							this->polygons[i * 4 + 2] = this->indices[i * 3 + 1];
							this->polygons[i * 4 + 3] = this->indices[i * 3 + 2];
						}
						
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
						this->geometry = ::boost::make_shared< ::fcl::Convex>(
#else
						this->geometry = ::std::make_shared< ::fcl::Convex>(
#endif
							this->normals.data(),
							this->distances.data(),
							this->indices.size() / 3,
							this->vertices.empty() ? 0 : &this->vertices.front(),
							this->vertices.size(),
							this->polygons.data()
						);
					}
					else
					{
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
						::boost::shared_ptr< ::fcl::BVHModel< ::fcl::OBBRSS>> mesh = ::boost::make_shared< ::fcl::BVHModel< ::fcl::OBBRSS>>();
#else
						::std::shared_ptr< ::fcl::BVHModel< ::fcl::OBBRSS>> mesh = ::std::make_shared< ::fcl::BVHModel< ::fcl::OBBRSS>>();
#endif
						mesh->beginModel(this->indices.size() / 3, this->vertices.size());
						
						for (::std::size_t i = 0; i < this->indices.size() / 3; ++i)
						{
							mesh->addTriangle(
								this->vertices[this->indices[3 * i]],
								this->vertices[this->indices[3 * i + 1]],
								this->vertices[this->indices[3 * i + 2]]
							);
						}
						
						mesh->endModel();
						this->geometry = mesh;
					}
				}
				else if (vrmlGeometry->isOfType(SoVRMLSphere::getClassTypeId()))
				{
					SoVRMLSphere* sphere = static_cast<SoVRMLSphere*>(vrmlGeometry);
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
					this->geometry = ::boost::make_shared< ::fcl::Sphere>(sphere->radius.getValue());
#else
					this->geometry = ::std::make_shared< ::fcl::Sphere>(sphere->radius.getValue());
#endif
				}
				
				this->collisionObject = ::std::make_shared< ::fcl::CollisionObject>(this->geometry, ::fcl::Transform3f());
				
				this->getBody()->add(this);
				setTransform(::rl::math::Transform::Identity());
			}
			
			Shape::~Shape()
			{
				static_cast<Body*>(this->getBody())->remove(this);
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
				this->update(this->currentFrame);
			}
			
			void
			Shape::triangleCallback(void* userData, SoCallbackAction* action, const SoPrimitiveVertex* v1, const SoPrimitiveVertex* v2, const SoPrimitiveVertex* v3)
			{
				Shape* shape = static_cast<Shape*>(userData);
				
				shape->indices.push_back(shape->vertices.size());
				::fcl::Vec3f fclVertex1(v1->getPoint()[0], v1->getPoint()[1], v1->getPoint()[2]);
				shape->vertices.push_back(fclVertex1);
				
				shape->indices.push_back(shape->vertices.size());
				::fcl::Vec3f fclVertex2(v2->getPoint()[0], v2->getPoint()[1], v2->getPoint()[2]);
				shape->vertices.push_back(fclVertex2);
				
				shape->indices.push_back(shape->vertices.size());
				::fcl::Vec3f fclVertex3(v3->getPoint()[0], v3->getPoint()[1], v3->getPoint()[2]);
				shape->vertices.push_back(fclVertex3);
			}
			
			void
			Shape::update(const ::rl::math::Transform& frame)
			{
				this->currentFrame = frame;
				
				::rl::math::Transform fullTransform = this->currentFrame * this->transform * this->baseTransform;
				
				::fcl::Vec3f translation(fullTransform(0, 3), fullTransform(1, 3), fullTransform(2, 3));
				
				::fcl::Matrix3f rotation(
					fullTransform(0, 0), fullTransform(0, 1), fullTransform(0, 2),
					fullTransform(1, 0), fullTransform(1, 1), fullTransform(1, 2),
					fullTransform(2, 0), fullTransform(2, 1), fullTransform(2, 2)
				);
				
				this->collisionObject->setTransform(rotation, translation);
				this->collisionObject->computeAABB();
			}
		}
	}
}
