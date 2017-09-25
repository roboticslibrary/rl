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

#include <fcl/distance.h>
#include <fcl/BVH/BVH_model.h>

#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
#include <boost/make_shared.hpp>
#endif

#include "../Exception.h"
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
			Scene::Scene() :
				::rl::sg::Scene(),
				::rl::sg::SimpleScene(),
				manager()
			{
				this->isScalingSupported = false;
				this->manager.setup();
			}
			
			Scene::~Scene()
			{
				while (this->models.size() > 0)
				{
					delete this->models[0];
				}
			}
			
			void
			Scene::add(::rl::sg::Model* model)
			{
				Model* modelFcl = static_cast<Model*>(model);
				this->models.push_back(model);
				::std::vector< ::fcl::CollisionObject*> objects;
				modelFcl->manager.getObjects(objects);
				
				if (objects.size() > 0)
				{
					this->manager.registerObjects(objects);
				}
			}
			
			void
			Scene::addCollisionObject(::fcl::CollisionObject* collisionObject, Body* body)
			{
				this->bodyForObj[collisionObject] = body;
				this->manager.registerObject(collisionObject);
			}
			
			bool
			Scene::areColliding(::rl::sg::Body* first, ::rl::sg::Body* second)
			{
				Body* body1 = static_cast<Body*>(first);
				Body* body2 = static_cast<Body*>(second);
				
				body1->manager.update();
				body2->manager.update();
				CollisionData collisionData(this->bodyForObj);
				body1->manager.collide(&body2->manager, &collisionData, Scene::defaultCollisionFunction);
				
				return collisionData.result.numContacts() > 0;
			}
			
			bool
			Scene::areColliding(::rl::sg::Model* first, ::rl::sg::Model* second)
			{
				Model* model1 = static_cast<Model*>(first);
				Model* model2 = static_cast<Model*>(second);
				
				model1->manager.update();
				model2->manager.update();
				CollisionData collisionData(this->bodyForObj);
				model1->manager.collide(&model2->manager, &collisionData, Scene::defaultCollisionFunction);
				
				return collisionData.result.numContacts() > 0;
			}
			
			bool
			Scene::areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second)
			{
				Shape* shape1 = static_cast<Shape*>(first);
				Shape* shape2 = static_cast<Shape*>(second);
				
				::fcl::CollisionRequest request;
				::fcl::CollisionResult result;
				::fcl::collide(shape1->collisionObject.get(), shape2->collisionObject.get(), request, result);
				
				return result.isCollision();
			}
			
			::rl::sg::Model*
			Scene::create()
			{
				return new Model(this);
			}
			
			bool
			Scene::defaultCollisionFunction(::fcl::CollisionObject* o1, ::fcl::CollisionObject* o2, void* data)
			{
				CollisionData* collisionData = static_cast<CollisionData*>(data);
				const ::fcl::CollisionRequest& request = collisionData->request;
				::fcl::CollisionResult& result = collisionData->result;
				
				if (collisionData->done)
				{
					return true;
				}
				
				if (collisionData->bodyForObj.find(o1)->second == collisionData->bodyForObj.find(o2)->second)
				{
					return false;
				}
				
				::fcl::collide(o1, o2, request, result);
				
				if (!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
				{
					collisionData->done = true;
				}
				
				return collisionData->done;
			}
			
			bool
			Scene::defaultDistanceFunction(::fcl::CollisionObject* o1, ::fcl::CollisionObject* o2, void* data, ::fcl::FCL_REAL& dist)
			{
				DistanceData* distanceData = static_cast<DistanceData*>(data);
				const ::fcl::DistanceRequest& request = distanceData->request;
				::fcl::DistanceResult& result = distanceData->result;
				
				if (distanceData->done)
				{
					dist = result.min_distance;
					return true;
				}
				
				if (distanceData->bodyForObj.find(o1)->second == distanceData->bodyForObj.find(o2)->second)
				{
					return false;
				}
				
				::fcl::distance(o1, o2, request, result);
				
				dist = result.min_distance;
				result.nearest_points[0] = o1->getTransform().transform(result.nearest_points[0]);
				result.nearest_points[1] = o2->getTransform().transform(result.nearest_points[1]);
				
				if (dist <= 0)
				{
					return true;
				}
				
				return distanceData->done;
			}
			
			::rl::math::Real
			Scene::depth(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Shape* shape1 = static_cast<Shape*>(first);
				Shape* shape2 = static_cast<Shape*>(second);
				
				::fcl::CollisionRequest request(1, true);
				::fcl::CollisionResult result;
				::fcl::collide(shape1->collisionObject.get(), shape2->collisionObject.get(), request, result);
				
				if (0 == result.numContacts())
				{
					return 0;
				}
				
				const ::fcl::Contact& contact = result.getContact(0);
				
				::fcl::Vec3f pos1 = contact.pos;
				::fcl::Vec3f pos2 = contact.pos + contact.normal * contact.penetration_depth;
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					point1(i) = pos1[i];
					point2(i) = pos2[i];
				}
				
				return result.isCollision() ? ::std::abs(contact.penetration_depth) : 0;
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Body* first, ::rl::sg::Body* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Body* body1 = static_cast<Body*>(first);
				Body* body2 = static_cast<Body*>(second);
				
				body1->manager.update();
				body2->manager.update();
				DistanceData distanceData(this->bodyForObj);
				body1->manager.distance(&body2->manager, &distanceData, Scene::defaultDistanceFunction);
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					point1(i) = distanceData.result.nearest_points[0][i];
					point2(i) = distanceData.result.nearest_points[1][i];
				}
				
				return distanceData.result.min_distance; 
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Model* first, ::rl::sg::Model* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Model* model1 = static_cast<Model*>(first);
				Model* model2 = static_cast<Model*>(second);
				
				model1->manager.update();
				model2->manager.update();
				DistanceData distanceData(this->bodyForObj);
				model1->manager.distance(&model2->manager, &distanceData, Scene::defaultDistanceFunction);
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					point1(i) = distanceData.result.nearest_points[0][i];
					point2(i) = distanceData.result.nearest_points[1][i];
				}
				
				return distanceData.result.min_distance; 
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Shape* shape1 = static_cast<Shape*>(first);
				Shape* shape2 = static_cast<Shape*>(second);
				
				::fcl::DistanceRequest request(true);
				::fcl::DistanceResult result;
				::fcl::distance(shape1->collisionObject.get(), shape2->collisionObject.get(), request, result);
				
				::fcl::Vec3f nearestPoint1 = shape1->collisionObject->getTransform().transform(result.nearest_points[0]);
				::fcl::Vec3f nearestPoint2 = shape2->collisionObject->getTransform().transform(result.nearest_points[1]);
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					point1(i) = nearestPoint1[i];
					point2(i) = nearestPoint2[i];
				}
				
				return result.min_distance;
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Shape* shape, const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
				::boost::shared_ptr< ::fcl::CollisionGeometry> geometry = ::boost::make_shared< ::fcl::Sphere>(0);
#else
				::std::shared_ptr< ::fcl::CollisionGeometry> geometry = ::std::make_shared< ::fcl::Sphere>(0);
#endif
				::fcl::Vec3f translation(point(0), point(1), point(2));
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
				::boost::shared_ptr< ::fcl::CollisionObject> collisionObject = ::boost::make_shared< ::fcl::CollisionObject>(geometry, ::fcl::Transform3f(translation));
#else
				::std::shared_ptr< ::fcl::CollisionObject> collisionObject = ::std::make_shared< ::fcl::CollisionObject>(geometry, ::fcl::Transform3f(translation));
#endif
				
				::fcl::DistanceRequest request;
				::fcl::DistanceResult result;
				::fcl::distance(static_cast<Shape*>(shape)->collisionObject.get(), collisionObject.get(), request, result);
				
				::fcl::Vec3f nearestPoint1 = static_cast<Shape*>(shape)->collisionObject->getTransform().transform(result.nearest_points[0]);
				::fcl::Vec3f nearestPoint2 = collisionObject->getTransform().transform(result.nearest_points[1]);
				
				for (::std::size_t i = 0; i < 3; ++i)
				{
					point1(i) = nearestPoint1[i];
					point2(i) = nearestPoint2[i];
				}
				
				return result.min_distance;
			}
			
			bool
			Scene::isColliding()
			{
				this->manager.update();
				CollisionData collisionData(bodyForObj);
				this->manager.collide(&collisionData, Scene::defaultCollisionFunction);
				return collisionData.result.numContacts() > 0;
			}
			
			void
			Scene::remove(::rl::sg::Model* model)
			{
				Iterator found = ::std::find(this->models.begin(), this->models.end(), model);
				
				if (found != this->models.end())
				{
					this->models.erase(found);
					::std::vector< ::fcl::CollisionObject*> objects;
					static_cast<Model*>(model)->manager.getObjects(objects);
					
					for (::std::size_t i = 0; i < objects.size(); ++i)
					{
						this->manager.unregisterObject(objects[i]);
					}
				}
			}
			
			void
			Scene::removeCollisionObject(::fcl::CollisionObject* collisionObject)
			{
				this->bodyForObj.erase(collisionObject);
				this->manager.unregisterObject(collisionObject);
			}
		}
	}
}
