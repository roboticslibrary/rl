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

#ifndef RL_SG_FCL_SCENE_H
#define RL_SG_FCL_SCENE_H

#include <unordered_map>
#include <fcl/config.h>

#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#else
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/narrowphase/collision.h>
#endif

#include "../DepthScene.h"
#include "../DistanceScene.h"
#include "../SimpleScene.h"

namespace rl
{
	namespace sg
	{
		/**
		 * Flexible Collision Library.
		 *
		 * https://github.com/flexible-collision-library/fcl
		 */
		namespace fcl
		{
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
			typedef ::fcl::CollisionObject CollisionObject;
			typedef ::fcl::CollisionRequest CollisionRequest;
			typedef ::fcl::CollisionResult CollisionResult;
			typedef ::fcl::DistanceRequest DistanceRequest;
			typedef ::fcl::DistanceResult DistanceResult;
			typedef ::fcl::DynamicAABBTreeCollisionManager DynamicAABBTreeCollisionManager;
			typedef ::fcl::FCL_REAL Real;
#else
			typedef ::fcl::CollisionObject<::rl::math::Real> CollisionObject;
			typedef ::fcl::CollisionRequest<::rl::math::Real> CollisionRequest;
			typedef ::fcl::CollisionResult<::rl::math::Real> CollisionResult;
			typedef ::fcl::DistanceRequest<::rl::math::Real> DistanceRequest;
			typedef ::fcl::DistanceResult<::rl::math::Real> DistanceResult;
			typedef ::fcl::DynamicAABBTreeCollisionManager<::rl::math::Real> DynamicAABBTreeCollisionManager;
			typedef ::rl::math::Real Real;
#endif
			
			class RL_SG_EXPORT Scene : public ::rl::sg::DepthScene, public ::rl::sg::DistanceScene, public ::rl::sg::SimpleScene
			{
			public:
				Scene();
				
				virtual ~Scene();
				
				void add(::rl::sg::Model* model);
				
				void addCollisionObject(CollisionObject* collisionObject, Body* body);
				
				bool areColliding(::rl::sg::Body* first, ::rl::sg::Body* second);
				
				bool areColliding(::rl::sg::Model* first, ::rl::sg::Model* second);
				
				bool areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second);
				
				::rl::sg::Model* create();
				
				using ::rl::sg::DepthScene::depth;
				
				::rl::math::Real depth(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::math::Real distance(::rl::sg::Body* first, ::rl::sg::Body* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::math::Real distance(::rl::sg::Model* first, ::rl::sg::Model* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::math::Real distance(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::math::Real distance(::rl::sg::Shape* shape, const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				bool isColliding();
				
				bool isScalingSupported() const;
				
				void remove(::rl::sg::Model* model);
				
				void removeCollisionObject(CollisionObject* collisionObject);
				
				DynamicAABBTreeCollisionManager manager;
				
			protected:
				
			private:
				struct CollisionData
				{
					CollisionData(const ::std::unordered_map<CollisionObject*, Body*>& bodyForObj);
					
					const ::std::unordered_map<CollisionObject*, Body*>& bodyForObj;
					
					bool done;
					
					CollisionRequest request;
					
					CollisionResult result;
				};
				
				struct DistanceData
				{
					DistanceData(const ::std::unordered_map<CollisionObject*, Body*>& bodyForObj);
					
					const ::std::unordered_map<CollisionObject*, Body*>& bodyForObj;
					
					bool done;
					
					DistanceRequest request;
					
					DistanceResult result;
				};
				
				static bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2, void* data);
				
				static bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2, void* data, Real& dist);
				
				::std::unordered_map<CollisionObject*, Body*> bodyForObj;
			};
		}
	}
}

#endif // RL_SG_FCL_SCENE_H
