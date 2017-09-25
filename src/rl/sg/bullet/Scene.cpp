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

#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>

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
			Scene::Scene() :
				::rl::sg::Scene(),
				::rl::sg::DistanceScene(),
				::rl::sg::RaycastScene(),
				::rl::sg::SimpleScene(),
				broadphase(),
				configuration(),
				dispatcher(&configuration),
				world(&dispatcher, &broadphase, &configuration)
			{
				this->isScalingSupported = false;
			}
			
			Scene::~Scene()
			{
				while (this->models.size() > 0)
				{
					delete this->models[0];
				}
			}
			
			bool
			Scene::areColliding(::rl::sg::Body* first, ::rl::sg::Body* second)
			{
				Body* body1 = static_cast<Body*>(first);
				Body* body2 = static_cast<Body*>(second);
				
				ContactResultCallback resultCallback;
				this->world.contactPairTest(&body1->object, &body2->object, resultCallback);
				
				return resultCallback.collision;
			}
			
			bool
			Scene::areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second)
			{
				throw Exception("::rl::sg::bullet::Scene::areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second) - not supported");
			}
			
			::rl::sg::Model*
			Scene::create()
			{
				return new Model(this);
			}
			
			::rl::math::Real
			Scene::depth(::rl::sg::Body* first, ::rl::sg::Body* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Body* body1 = static_cast<Body*>(first);
				Body* body2 = static_cast<Body*>(second);
				
				ContactResultCallback resultCallback;
				this->world.contactPairTest(&body1->object, &body2->object, resultCallback);
				
				point1.x() = resultCallback.positionWorldOnA.getX();
				point1.y() = resultCallback.positionWorldOnA.getY();
				point1.z() = resultCallback.positionWorldOnA.getZ();
				
				point2.x() = resultCallback.positionWorldOnB.getX();
				point2.y() = resultCallback.positionWorldOnB.getY();
				point2.z() = resultCallback.positionWorldOnB.getZ();
				
				return resultCallback.collision ? ::std::abs(resultCallback.distance) : 0;
			}
			
			::rl::math::Real
			Scene::depth(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				throw Exception("::rl::sg::bullet::Scene::depth(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2) - not supported");
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Shape* shape1 = static_cast<Shape*>(first);
				Shape* shape2 = static_cast<Shape*>(second);
				
				if (!shape1->shape->isConvex() || !shape2->shape->isConvex())
				{
					throw Exception("::rl::sg::bullet::Scene::distance() - distance calculation only supported between convex shapes");
				}
				
				Body* body1 = static_cast<Body*>(shape1->getBody());
				Body* body2 = static_cast<Body*>(shape2->getBody());
				
				::btVoronoiSimplexSolver simplexSolver;
				::btGjkEpaPenetrationDepthSolver penetrationDepthSolver;
				::btGjkPairDetector pairDetector(
					dynamic_cast< ::btConvexShape*>(shape1->shape),
					dynamic_cast< ::btConvexShape*>(shape2->shape),
					&simplexSolver,
					&penetrationDepthSolver
				);
				
				::btPointCollector pointCollector;
				::btGjkPairDetector::ClosestPointInput input;
				input.m_transformA = body1->object.getWorldTransform() * shape1->transform;
				input.m_transformB = body2->object.getWorldTransform() * shape2->transform;
				pairDetector.getClosestPoints(input, pointCollector, 0);
				
				point1.x() = pointCollector.m_pointInWorld.x();
				point1.y() = pointCollector.m_pointInWorld.y();
				point1.z() = pointCollector.m_pointInWorld.z();
				
				btVector3 endPt = pointCollector.m_pointInWorld + pointCollector.m_normalOnBInWorld * pointCollector.m_distance;
				
				point2.x() = endPt.x();
				point2.y() = endPt.y();
				point2.z() = endPt.z();
				
				return pointCollector.m_distance;
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Shape* shape, const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				::btSphereShape sphere(0);
				
				::btVoronoiSimplexSolver sGjkSimplexSolver;
				
				::btGjkEpaPenetrationDepthSolver epa;
				::btGjkPairDetector convexConvex(
					dynamic_cast< ::btConvexShape*>(static_cast<Shape*>(shape)->shape),
					dynamic_cast< ::btConvexShape*>(&sphere),
					&sGjkSimplexSolver,
					&epa
				);
				
				Body* body = static_cast<Body*>(shape->getBody());
				
				::btPointCollector gjkOutput;
				::btGjkPairDetector::ClosestPointInput input;
				input.m_transformA = body->object.getWorldTransform() * static_cast<Shape*>(shape)->transform;
				input.m_transformB.setIdentity();
				input.m_transformB.getOrigin().setValue(
					static_cast< ::btScalar>(point.x()),
					static_cast< ::btScalar>(point.y()),
					static_cast< ::btScalar>(point.z())
				);
				
				convexConvex.getClosestPoints(input, gjkOutput, 0);
				
				point1.x() = gjkOutput.m_pointInWorld.x();
				point1.y() = gjkOutput.m_pointInWorld.y();
				point1.z() = gjkOutput.m_pointInWorld.z();
				
				::btVector3 endPt = gjkOutput.m_pointInWorld + gjkOutput.m_normalOnBInWorld * gjkOutput.m_distance;
				
				point2.x() = endPt.x();
				point2.y() = endPt.y();
				point2.z() = endPt.z();
				
				return gjkOutput.m_distance;
			}
			
			bool
			Scene::isColliding()
			{
				this->world.performDiscreteCollisionDetection();
				
				for (int i = 0; i < this->world.getDispatcher()->getNumManifolds(); ++i)
				{
					if (this->world.getDispatcher()->getManifoldByIndexInternal(i)->getNumContacts() > 0)
					{
						return true;
					}
				}
				
				return false;
			}
			
			::rl::sg::Shape*
			Scene::raycast(const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance)
			{
				::btVector3 rayFromWorld(
					static_cast< ::btScalar>(source.x()),
					static_cast< ::btScalar>(source.y()),
					static_cast< ::btScalar>(source.z())
				);
				
				::btVector3 rayToWorld(
					static_cast< ::btScalar>(target.x()),
					static_cast< ::btScalar>(target.y()),
					static_cast< ::btScalar>(target.z())
				);
				
				RayResultCallback resultCallback;
				
				this->world.rayTest(rayFromWorld, rayToWorld, resultCallback); 
				
				if (nullptr != resultCallback.collisionShape)
				{
					::btVector3 hitPointWorld;
					hitPointWorld.setInterpolate3(rayFromWorld, rayToWorld, resultCallback.m_closestHitFraction);
					distance = rayFromWorld.distance(hitPointWorld);
					return static_cast<Shape*>(resultCallback.collisionShape->getUserPointer());
				}
				else
				{
					distance = ::std::numeric_limits< ::btScalar>::quiet_NaN();
					return nullptr;
				}
			}
			
			bool
			Scene::raycast(::rl::sg::Shape* shape, const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance)
			{
				Body* body = static_cast<Body*>(shape->getBody());
				
				::btVector3 rayFromWorld(
					static_cast< ::btScalar>(source.x()),
					static_cast< ::btScalar>(source.y()),
					static_cast< ::btScalar>(source.z())
				);
				
				::btTransform rayFromTrans;
				rayFromTrans.setIdentity();
				rayFromTrans.setOrigin(rayFromWorld);
				
				::btVector3 rayToWorld(
					static_cast< ::btScalar>(target.x()),
					static_cast< ::btScalar>(target.y()),
					static_cast< ::btScalar>(target.z())
				);
				
				::btTransform rayToTrans;
				rayToTrans.setIdentity();
				rayToTrans.setOrigin(rayToWorld);
				
				RayResultCallback resultCallback;
				
				this->world.rayTestSingle(
					rayFromTrans,
					rayToTrans,
					&body->object,
					static_cast<Shape*>(shape)->shape,
					body->object.getWorldTransform() * static_cast<Shape*>(shape)->transform, // TODO
					resultCallback
				);
				
				if (nullptr != resultCallback.collisionShape)
				{
					::btVector3 hitPointWorld;
					hitPointWorld.setInterpolate3(rayFromWorld, rayToWorld, resultCallback.m_closestHitFraction);
					distance = rayFromWorld.distance(hitPointWorld);
					return true;
				}
				else
				{
					distance = ::std::numeric_limits< ::btScalar>::quiet_NaN();
					return false;
				}
			}
			
			Scene::ContactResultCallback::ContactResultCallback() :
				collision(false),
				distance(0),
				positionWorldOnA(),
				positionWorldOnB()
			{
			}
			
			btScalar
#if (BT_BULLET_VERSION < 281)
			Scene::ContactResultCallback::addSingleResult(::btManifoldPoint& cp, const ::btCollisionObject* colObj0, int partId0, int index0, const ::btCollisionObject* colObj1, int partId1, int index1)
#else
			Scene::ContactResultCallback::addSingleResult(::btManifoldPoint& cp, const ::btCollisionObjectWrapper* colObj0, int partId0, int index0, const ::btCollisionObjectWrapper* colObj1, int partId1, int index1)
#endif
			{
				this->collision = true;
				
				if (cp.getDistance() <= this->distance)
				{
					this->distance = cp.getDistance();
					this->positionWorldOnA = cp.getPositionWorldOnA();
					this->positionWorldOnB = cp.getPositionWorldOnB();
				}
				
				return 0;
			}
			
			Scene::RayResultCallback::RayResultCallback() :
				collisionShape(nullptr),
				hitPointWorld()
			{
			}
			
			btScalar
			Scene::RayResultCallback::addSingleResult(::btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
			{
				this->collisionShape = rayResult.m_collisionObject->getCollisionShape();
				this->m_closestHitFraction = rayResult.m_hitFraction;
				return 0;
			}
		}
	}
}
