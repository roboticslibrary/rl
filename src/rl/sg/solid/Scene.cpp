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

#include <algorithm>
#include <cmath>

#include "Model.h"
#include "Scene.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace solid
		{
			Scene::Scene() :
				::rl::sg::Scene(),
				::rl::sg::DepthScene(),
				::rl::sg::DistanceScene(),
				::rl::sg::RaycastScene(),
				::rl::sg::SimpleScene(),
				scene(DT_CreateScene())
			{
				this->broad = BP_CreateScene(this, Scene::beginOverlap, Scene::endOverlap);
			}
			
			Scene::~Scene()
			{
				while (this->models.size() > 0)
				{
					delete this->models[0];
				}
				
				if (nullptr != this->scene)
				{
					BP_DestroyScene(this->broad);
				}
				
				if (nullptr != this->scene)
				{
					DT_DestroyScene(this->scene);
				}
			}
			
			bool
			Scene::areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second)
			{
				Shape* shape1 = static_cast<Shape*>(first);
				Shape* shape2 = static_cast<Shape*>(second);
				
				if (shape1->encounters.count(shape2) > 0 || shape2->encounters.count(shape1) > 0)
				{
					DT_Vector3 point;
					
					return (DT_TRUE == DT_GetCommonPoint(
						shape1->complex ? shape1->object : shape2->object,
						shape1->complex ? shape2->object : shape1->object,
						point
					)) ? true : false;
				}
				else
				{
					return false;
				}
			}
			
			void
			Scene::beginOverlap(void* clientData, void* object1, void* object2)
			{
				Shape* shape1 = static_cast<Shape*>(object1);
				Shape* shape2 = static_cast<Shape*>(object2);
				
				shape1->encounters.insert(shape2);
				shape2->encounters.insert(shape1);
			}
			
			::rl::sg::Model*
			Scene::create()
			{
				return new Model(this);
			}
			
			::rl::math::Real
			Scene::depth(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Shape* shape1 = static_cast<Shape*>(first);
				Shape* shape2 = static_cast<Shape*>(second);
				
				DT_Vector3 vector1;
				DT_Vector3 vector2;
				
				DT_Bool intersect = DT_GetPenDepth(
					shape1->complex ? shape1->object : shape2->object,
					shape1->complex ? shape2->object : shape1->object,
					shape1->complex ? vector1 : vector2,
					shape1->complex ? vector2 : vector1
				);
				
				point1(0) = vector1[0];
				point1(1) = vector1[1];
				point1(2) = vector1[2];
				
				point2(0) = vector2[0];
				point2(1) = vector2[1];
				point2(2) = vector2[2];
				
				return (DT_TRUE == intersect) ? (point2 - point1).norm() : 0;
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Shape* shape1 = static_cast<Shape*>(first);
				Shape* shape2 = static_cast<Shape*>(second);
				
				DT_Vector3 vector1;
				DT_Vector3 vector2;
				
				DT_Scalar distance = DT_GetClosestPair(
					shape1->complex ? shape1->object : shape2->object,
					shape1->complex ? shape2->object : shape1->object,
					shape1->complex ? vector1 : vector2,
					shape1->complex ? vector2 : vector1
				);
				
				point1(0) = vector1[0];
				point1(1) = vector1[1];
				point1(2) = vector1[2];
				
				point2(0) = vector2[0];
				point2(1) = vector2[1];
				point2(2) = vector2[2];
				
				return distance;
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Shape* shape, const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				DT_Vector3 position = {
					static_cast<DT_Scalar>(point(0)),
					static_cast<DT_Scalar>(point(1)),
					static_cast<DT_Scalar>(point(2))
				};
				
				DT_ShapeHandle shapeHandle = DT_NewPoint(position);
				DT_ObjectHandle objectHandle = DT_CreateObject(nullptr, shapeHandle);
				
				DT_Vector3 vector1;
				DT_Vector3 vector2;
				
				DT_Scalar distance = DT_GetClosestPair(
					static_cast<Shape*>(shape)->object,
					objectHandle,
					vector1,
					vector2
				);
				
				point1(0) = vector1[0];
				point1(1) = vector1[1];
				point1(2) = vector1[2];
				
				point2(0) = vector2[0];
				point2(1) = vector2[1];
				point2(2) = vector2[2];
				
				DT_DestroyObject(objectHandle);
				DT_DeleteShape(shapeHandle);
				
				return distance;
			}
			
			void
			Scene::endOverlap(void* clientData, void* object1, void* object2)
			{
				Shape* shape1 = static_cast<Shape*>(object1);
				Shape* shape2 = static_cast<Shape*>(object2);
				
				shape1->encounters.erase(shape2);
				shape2->encounters.erase(shape1);
			}
			
			::rl::sg::Shape*
			Scene::raycast(const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance)
			{
				DT_Vector3 vector1;
				vector1[0] = static_cast<DT_Scalar>(source(0));
				vector1[1] = static_cast<DT_Scalar>(source(1));
				vector1[2] = static_cast<DT_Scalar>(source(2));
				
				DT_Vector3 vector2;
				vector2[0] = static_cast<DT_Scalar>(target(0));
				vector2[1] = static_cast<DT_Scalar>(target(1));
				vector2[2] = static_cast<DT_Scalar>(target(2));
				
				DT_Scalar param;
				DT_Vector3 normal;
				
				void* object = DT_RayCast(
					this->scene,
					nullptr,
					vector1,
					vector2,
					std::numeric_limits<DT_Scalar>::max(),
					&param,
					normal
				);
				
				DT_Vector3 hit;
				hit[0] = vector1[0] + (vector2[0] - vector1[0]) * param;
				hit[1] = vector1[1] + (vector2[1] - vector1[1]) * param;
				hit[2] = vector1[2] + (vector2[2] - vector1[2]) * param;
				
				distance = std::sqrt(std::pow(hit[0] - source[0], 2) + std::pow(hit[1] - source[1], 2) + std::pow(hit[2] - source[2], 2));;
				
				return static_cast< ::rl::sg::Shape*>(object);
			}
			
			bool
			Scene::raycast(::rl::sg::Shape* shape, const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance)
			{
				DT_Vector3 vector1;
				vector1[0] = static_cast<DT_Scalar>(source(0));
				vector1[1] = static_cast<DT_Scalar>(source(1));
				vector1[2] = static_cast<DT_Scalar>(source(2));
				
				DT_Vector3 vector2;
				vector2[0] = static_cast<DT_Scalar>(target(0));
				vector2[1] = static_cast<DT_Scalar>(target(1));
				vector2[2] = static_cast<DT_Scalar>(target(2));
				
				DT_Scalar param;
				DT_Vector3 normal;
				
				DT_Bool object = DT_ObjectRayCast(
					static_cast<Shape*>(shape)->object,
					vector1,
					vector2,
					std::numeric_limits<DT_Scalar>::max(),
					&param,
					normal
				);
				
				DT_Vector3 hit;
				hit[0] = vector1[0] + (vector2[0] - vector1[0]) * param;
				hit[1] = vector1[1] + (vector2[1] - vector1[1]) * param;
				hit[2] = vector1[2] + (vector2[2] - vector1[2]) * param;
				
				distance = std::sqrt(std::pow(hit[0] - source[0], 2) + std::pow(hit[1] - source[1], 2) + std::pow(hit[2] - source[2], 2));;
				
				return (DT_TRUE == object) ? true : false;
			}
			
			void
			Scene::setMargin(const ::rl::math::Real& margin)
			{
				for (Iterator i = this->begin(); i != this->end(); ++i)
				{
					static_cast<Model*>(*i)->setMargin(margin);
				}
			}
		}
	}
}
