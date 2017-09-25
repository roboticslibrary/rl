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
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>

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
			class Scene : public ::rl::sg::DepthScene, public ::rl::sg::DistanceScene, public ::rl::sg::SimpleScene
			{
			public:
				Scene();
				
				virtual ~Scene();
				
				void add(::rl::sg::Model* model);
				
				void addCollisionObject(::fcl::CollisionObject* collisionObject, Body* body);
				
				bool areColliding(::rl::sg::Body* first, ::rl::sg::Body* second);
				
				bool areColliding(::rl::sg::Model* first, ::rl::sg::Model* second);
				
				bool areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second);
				
				::rl::sg::Model* create();
				
				::rl::math::Real depth(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::math::Real distance(::rl::sg::Body* first, ::rl::sg::Body* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::math::Real distance(::rl::sg::Model* first, ::rl::sg::Model* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::math::Real distance(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::math::Real distance(::rl::sg::Shape* shape, const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				bool isColliding();
				
				void remove(::rl::sg::Model* model);
				
				void removeCollisionObject(::fcl::CollisionObject* collisionObject);
				
				::fcl::DynamicAABBTreeCollisionManager manager;
				
			protected:
				
			private:
				struct CollisionData
				{
					CollisionData(const ::std::unordered_map< ::fcl::CollisionObject*, Body*>& bodyForObj) :
						bodyForObj(bodyForObj),
						done(false),
						request(),
						result()
					{
					}
					
					const ::std::unordered_map< ::fcl::CollisionObject*, Body*>& bodyForObj;
					
					bool done;
					
					::fcl::CollisionRequest request;
					
					::fcl::CollisionResult result;
				};
				
				struct DistanceData
				{
					DistanceData(const ::std::unordered_map< ::fcl::CollisionObject*, Body*>& bodyForObj) :
						bodyForObj(bodyForObj),
						done(false),
						request(true),
						result()
					{
					}
					
					const ::std::unordered_map< ::fcl::CollisionObject*, Body*>& bodyForObj;
					
					bool done;
					
					::fcl::DistanceRequest request;
					
					::fcl::DistanceResult result;
				};
				
				static bool defaultCollisionFunction(::fcl::CollisionObject* o1, ::fcl::CollisionObject* o2, void* data);
				
				static bool defaultDistanceFunction(::fcl::CollisionObject* o1, ::fcl::CollisionObject* o2, void* data, ::fcl::FCL_REAL& dist);
				
				::std::unordered_map< ::fcl::CollisionObject*, Body*> bodyForObj;
			};
		}
	}
}

#endif // RL_SG_FCL_SCENE_H
