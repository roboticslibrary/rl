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

#include <fcl/config.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include <Inventor/VRMLnodes/SoVRMLCone.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLCylinder.h>
#include <Inventor/VRMLnodes/SoVRMLGeometry.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLSphere.h>

#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
#include <fcl/BVH/BVH_model.h>
#else
#include <fcl/geometry/bvh/BVH_model.h>
#endif

#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
#include <boost/make_shared.hpp>
#endif

#include "../Exception.h"
#include "Body.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace fcl
		{
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
			typedef ::fcl::Box Box;
			typedef ::fcl::Cone Cone;
			typedef ::fcl::Convex Convex;
			typedef ::fcl::Cylinder Cylinder;
			typedef ::fcl::OBBRSS OBBRSS;
			typedef ::fcl::Sphere Sphere;
			typedef ::fcl::Transform3f Transform3;
#else
			typedef ::fcl::Box<::rl::math::Real> Box;
			typedef ::fcl::Cone<::rl::math::Real> Cone;
			typedef ::fcl::Convex<::rl::math::Real> Convex;
			typedef ::fcl::Cylinder<::rl::math::Real> Cylinder;
			typedef ::fcl::OBBRSS<::rl::math::Real> OBBRSS;
			typedef ::fcl::Sphere<::rl::math::Real> Sphere;
			typedef ::fcl::Transform3<::rl::math::Real> Transform3;
#endif
			
			Shape::Shape(::SoVRMLShape* shape, ::rl::sg::Body* body) :
				::rl::sg::Shape(body),
				base(::rl::math::Transform::Identity()),
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
				distances(),
#endif
				frame(::rl::math::Transform::Identity()),
				geometry(),
				indices(),
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
				normals(),
#endif
				object(),
				polygons(),
				transform(::rl::math::Transform::Identity()),
				vertices()
			{
				::SoVRMLGeometry* vrmlGeometry = static_cast<::SoVRMLGeometry*>(shape->geometry.getValue());
				
				if (vrmlGeometry->isOfType(::SoVRMLBox::getClassTypeId()))
				{
					::SoVRMLBox* box = static_cast<::SoVRMLBox*>(vrmlGeometry);
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
					this->geometry = ::boost::make_shared<Box>(box->size.getValue()[0], box->size.getValue()[1], box->size.getValue()[2]);
#else
					this->geometry = ::std::make_shared<Box>(box->size.getValue()[0], box->size.getValue()[1], box->size.getValue()[2]);
#endif
				}
				else if (vrmlGeometry->isOfType(::SoVRMLCone::getClassTypeId()))
				{
					::SoVRMLCone* cone = static_cast<::SoVRMLCone*>(vrmlGeometry);
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
					this->geometry = ::boost::make_shared<Cone>(cone->bottomRadius.getValue(), cone->height.getValue());
#else
					this->geometry = ::std::make_shared<Cone>(cone->bottomRadius.getValue(), cone->height.getValue());
#endif
					
					this->base(0, 0) = 1;
					this->base(0, 1) = 0;
					this->base(0, 2) = 0;
					this->base(0, 3) = 0;
					this->base(1, 0) = 0;
					this->base(1, 1) = 0;
					this->base(1, 2) = 1;
					this->base(1, 3) = 0;
					this->base(2, 0) = 0;
					this->base(2, 1) = -1;
					this->base(2, 2) = 0;
					this->base(2, 3) = 0;
					this->base(3, 0) = 0;
					this->base(3, 1) = 0;
					this->base(3, 2) = 0;
					this->base(3, 3) = 1;
				}
				else if (vrmlGeometry->isOfType(::SoVRMLCylinder::getClassTypeId()))
				{
					::SoVRMLCylinder* cylinder = static_cast<::SoVRMLCylinder*>(vrmlGeometry);
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
					this->geometry = ::boost::make_shared<Cylinder>(cylinder->radius.getValue(), cylinder->height.getValue());
#else
					this->geometry = ::std::make_shared<Cylinder>(cylinder->radius.getValue(), cylinder->height.getValue());
#endif
					
					this->base(0, 0) = 1;
					this->base(0, 1) = 0;
					this->base(0, 2) = 0;
					this->base(0, 3) = 0;
					this->base(1, 0) = 0;
					this->base(1, 1) = 0;
					this->base(1, 2) = 1;
					this->base(1, 3) = 0;
					this->base(2, 0) = 0;
					this->base(2, 1) = -1;
					this->base(2, 2) = 0;
					this->base(2, 3) = 0;
					this->base(3, 0) = 0;
					this->base(3, 1) = 0;
					this->base(3, 2) = 0;
					this->base(3, 3) = 1;
				}
				else if (vrmlGeometry->isOfType(::SoVRMLIndexedFaceSet::getClassTypeId()))
				{
					::SoVRMLIndexedFaceSet* indexedFaceSet = static_cast<::SoVRMLIndexedFaceSet*>(vrmlGeometry);
					
					::SoCallbackAction callbackAction;
					callbackAction.addTriangleCallback(vrmlGeometry->getTypeId(), Shape::triangleCallback, this);
					callbackAction.apply(vrmlGeometry);
					
					if (indexedFaceSet->convex.getValue() && !indexedFaceSet->convex.isDefault())
					{
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
						this->normals = ::std::vector<Vector3>(this->indices.size() / 3);
						this->distances = ::std::vector<Real>(this->indices.size() / 3);
#endif
						this->polygons = ::std::vector<int>(this->indices.size() + this->indices.size() / 3);
						
						for (::std::size_t i = 0; i < this->indices.size() / 3; ++i)
						{
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
							Vector3 normal = (this->vertices[this->indices[i * 3 + 1]] - this->vertices[this->indices[i * 3]]).cross(this->vertices[this->indices[i * 3 + 2]] - this->vertices[this->indices[i * 3]]);
							this->normals[i] = normal.normalize();
							this->distances[i] = this->vertices[this->indices[i * 3 + 1]].dot(normal);
#endif
							this->polygons[i * 4] = 3;
							this->polygons[i * 4 + 1] = this->indices[i * 3];
							this->polygons[i * 4 + 2] = this->indices[i * 3 + 1];
							this->polygons[i * 4 + 3] = this->indices[i * 3 + 2];
						}
						
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
						this->geometry = ::boost::make_shared<Convex>(
#else
						this->geometry = ::std::make_shared<Convex>(
#endif
							this->normals.data(),
							this->distances.data(),
							this->indices.size() / 3,
							this->vertices.data(),
							this->vertices.size(),
							this->polygons.data()
						);
#else
						this->geometry = ::std::make_shared<Convex>(
							::std::shared_ptr<const ::std::vector<Vector3>>(&this->vertices),
							this->indices.size() / 3,
							::std::shared_ptr<const ::std::vector<int>>(&this->polygons)
						);
#endif
					}
					else
					{
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
						::boost::shared_ptr<::fcl::BVHModel<OBBRSS>> mesh = ::boost::make_shared<::fcl::BVHModel<OBBRSS>>();
#else
						::std::shared_ptr<::fcl::BVHModel<OBBRSS>> mesh = ::std::make_shared<::fcl::BVHModel<OBBRSS>>();
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
				else if (vrmlGeometry->isOfType(::SoVRMLSphere::getClassTypeId()))
				{
					::SoVRMLSphere* sphere = static_cast<::SoVRMLSphere*>(vrmlGeometry);
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
					this->geometry = ::boost::make_shared<Sphere>(sphere->radius.getValue());
#else
					this->geometry = ::std::make_shared<Sphere>(sphere->radius.getValue());
#endif
				}
				else
				{
					throw Exception("rl::sg::fcl::Shape::() - geometry not supported");
				}
				
				this->object = ::std::make_shared<CollisionObject>(this->geometry, Transform3());
				
				this->getBody()->add(this);
				this->setTransform(::rl::math::Transform::Identity());
			}
			
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
			Shape::Shape(const ::boost::shared_ptr<CollisionGeometry>& geometry, ::rl::sg::Body* body) :
#else
			Shape::Shape(const ::std::shared_ptr<CollisionGeometry>& geometry, ::rl::sg::Body* body) :
#endif
				::rl::sg::Shape(body),
				base(::rl::math::Transform::Identity()),
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
				distances(),
#endif
				frame(::rl::math::Transform::Identity()),
				geometry(geometry),
				indices(),
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
				normals(),
#endif
				object(),
				polygons(),
				transform(::rl::math::Transform::Identity()),
				vertices()
			{
				this->object = ::std::make_shared<CollisionObject>(this->geometry, Transform3());
				
				this->getBody()->add(this);
				this->setTransform(::rl::math::Transform::Identity());
			}
			
			Shape::~Shape()
			{
				static_cast<Body*>(this->getBody())->remove(this);
			}
			
			CollisionObject*
			Shape::getCollisionObject() const
			{
				return this->object.get();
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
				this->update(this->frame);
			}
			
			void
			Shape::triangleCallback(void* userData, ::SoCallbackAction* action, const ::SoPrimitiveVertex* v1, const ::SoPrimitiveVertex* v2, const ::SoPrimitiveVertex* v3)
			{
				Shape* shape = static_cast<Shape*>(userData);
				
				shape->indices.push_back(shape->vertices.size());
				Vector3 vertex1(v1->getPoint()[0], v1->getPoint()[1], v1->getPoint()[2]);
				shape->vertices.push_back(vertex1);
				
				shape->indices.push_back(shape->vertices.size());
				Vector3 vertex2(v2->getPoint()[0], v2->getPoint()[1], v2->getPoint()[2]);
				shape->vertices.push_back(vertex2);
				
				shape->indices.push_back(shape->vertices.size());
				Vector3 vertex3(v3->getPoint()[0], v3->getPoint()[1], v3->getPoint()[2]);
				shape->vertices.push_back(vertex3);
			}
			
			void
			Shape::update(const ::rl::math::Transform& frame)
			{
				this->frame = frame;
				
				::rl::math::Transform transform = this->frame * this->transform;
				
				if (::fcl::GEOM_CONE == this->geometry->getNodeType() || ::fcl::GEOM_CYLINDER == this->geometry->getNodeType())
				{
					transform = transform * this->base;
				}
				
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
				::fcl::Matrix3f rotation(
					transform(0, 0), transform(0, 1), transform(0, 2),
					transform(1, 0), transform(1, 1), transform(1, 2),
					transform(2, 0), transform(2, 1), transform(2, 2)
				);
				Vector3 translation(transform(0, 3), transform(1, 3), transform(2, 3));
				this->object->setTransform(rotation, translation);
#else
				this->object->setTransform(transform.linear(), transform.translation());
#endif
				
				this->object->computeAABB();
			}
		}
	}
}
