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

#include "Model.h"
#include "Scene.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace pqp
		{
			Scene::Scene() :
				::rl::sg::Scene(),
				::rl::sg::DistanceScene(),
				::rl::sg::SimpleScene()
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
			Scene::areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second)
			{
				Shape* shape1 = static_cast<Shape*>(first);
				Shape* shape2 = static_cast<Shape*>(second);
				
				::PQP_CollideResult result;
				
				::PQP_Collide(
					&result,
					shape1->rotation,
					shape1->translation,
					&shape1->model,
					shape2->rotation,
					shape2->translation,
					&shape2->model,
					PQP_FIRST_CONTACT
				);
				
				return (result.Colliding() == 1 ? true : false);
			}
			
			::rl::sg::Model*
			Scene::create()
			{
				return new Model(this);
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Shape* shape1 = static_cast<Shape*>(first);
				Shape* shape2 = static_cast<Shape*>(second);
				
				::PQP_DistanceResult result;
				
				::PQP_Distance(
					&result,
					shape1->rotation,
					shape1->translation,
					&shape1->model,
					shape2->rotation,
					shape2->translation,
					&shape2->model,
					::std::numeric_limits< ::rl::math::Real>::epsilon(),
					::std::numeric_limits< ::rl::math::Real>::epsilon()
				);
				
				point1(0) = result.P1()[0];
				point1(1) = result.P1()[1];
				point1(2) = result.P1()[2];
				
				shape1->transformToWorld(point1, point1);
				
				point2(0) = result.P2()[0];
				point2(1) = result.P2()[1];
				point2(2) = result.P2()[2];
				
				shape2->transformToWorld(point2, point2);
				
				return result.Distance();
			}
			
			::rl::math::Real
			Scene::distance(::rl::sg::Shape* shape, const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
			{
				Shape* shape1 = static_cast<Shape*>(shape);
				
				::rl::math::Real p[3] = {0.0f, 0.0f, 0.0f};
				
				::PQP_Model model;
				model.BeginModel(1);
				model.AddTri(&p[0], &p[0], &p[0], 0);
				model.EndModel();
				
				PQP_REAL rotation[3][3] = {
					{1.0f, 0.0f, 0.0f},
					{0.0f, 1.0f, 0.0f},
					{0.0f, 0.0f, 1.0f}
				};
				
				PQP_REAL translation[3] = {point(0), point(1), point(2)};
				
				::PQP_DistanceResult result;
				
				::PQP_Distance(
					&result,
					shape1->rotation,
					shape1->translation,
					&shape1->model,
					rotation,
					translation,
					&model,
					::std::numeric_limits< ::rl::math::Real>::epsilon(),
					::std::numeric_limits< ::rl::math::Real>::epsilon()
				);
				
				point1(0) = result.P1()[0];
				point1(1) = result.P1()[1];
				point1(2) = result.P1()[2];
				
				shape1->transformToWorld(point1, point1);
				
				point2(0) = result.P2()[0] + translation[0];
				point2(1) = result.P2()[1] + translation[1];
				point2(2) = result.P2()[2] + translation[2];
				
				return result.Distance();
			}
		}
	}
}
