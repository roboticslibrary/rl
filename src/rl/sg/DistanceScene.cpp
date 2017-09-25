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

#include "Body.h"
#include "DistanceScene.h"
#include "Model.h"

namespace rl
{
	namespace sg
	{
		DistanceScene::DistanceScene() :
			Scene()
		{
		}
		
		DistanceScene::~DistanceScene()
		{
		}
		
		::rl::math::Real
		DistanceScene::distance(const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
		{
			::rl::math::Real distance = ::std::numeric_limits< ::rl::math::Real>::max();
			
			for (Scene::Iterator i = this->begin(); i != this->end(); ++i)
			{
				::rl::math::Vector3 tmpPoint1;
				::rl::math::Vector3 tmpPoint2;
				
				::rl::math::Real tmpDistance = this->distance(
					*i,
					point,
					tmpPoint1,
					tmpPoint2
				);
				
				if (tmpDistance < distance)
				{
					distance = tmpDistance;
					point1 = tmpPoint1;
					point2 = tmpPoint2;
				}
			}
			
			return distance;
		}
		
		::rl::math::Real
		DistanceScene::distance(Body* first, Body* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
		{
			::rl::math::Real distance = ::std::numeric_limits< ::rl::math::Real>::max();
			
			for (Body::Iterator i = first->begin(); i != first->end(); ++i)
			{
				for (Body::Iterator j = second->begin(); j != second->end(); ++j)
				{
					::rl::math::Vector3 tmpPoint1;
					::rl::math::Vector3 tmpPoint2;
					
					::rl::math::Real tmpDistance = this->distance(
						*i,
						*j,
						tmpPoint1,
						tmpPoint2
					);
					
					if (tmpDistance < distance)
					{
						distance = tmpDistance;
						point1 = tmpPoint1;
						point2 = tmpPoint2;
					}
				}
			}
			
			return distance;
		}
		
		::rl::math::Real
		DistanceScene::distance(Body* body, const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
		{
			::rl::math::Real distance = ::std::numeric_limits< ::rl::math::Real>::max();
			
			for (Body::Iterator i = body->begin(); i != body->end(); ++i)
			{
				::rl::math::Vector3 tmpPoint1;
				::rl::math::Vector3 tmpPoint2;
				
				::rl::math::Real tmpDistance = this->distance(
					*i,
					point,
					tmpPoint1,
					tmpPoint2
				);
				
				if (tmpDistance < distance)
				{
					distance = tmpDistance;
					point1 = tmpPoint1;
					point2 = tmpPoint2;
				}
			}
			
			return distance;
		}
		
		::rl::math::Real
		DistanceScene::distance(Model* first, Model* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
		{
			::rl::math::Real distance = ::std::numeric_limits< ::rl::math::Real>::max();
			
			for (Model::Iterator i = first->begin(); i != first->end(); ++i)
			{
				for (Model::Iterator j = second->begin(); j != second->end(); ++j)
				{
					::rl::math::Vector3 tmpPoint1;
					::rl::math::Vector3 tmpPoint2;
					
					::rl::math::Real tmpDistance = this->distance(
						*i,
						*j,
						tmpPoint1,
						tmpPoint2
					);
					
					if (tmpDistance < distance)
					{
						distance = tmpDistance;
						point1 = tmpPoint1;
						point2 = tmpPoint2;
					}
				}
			}
			
			return distance;
		}
		
		::rl::math::Real
		DistanceScene::distance(Model* model, const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
		{
			::rl::math::Real distance = ::std::numeric_limits< ::rl::math::Real>::max();
			
			for (Model::Iterator i = model->begin(); i != model->end(); ++i)
			{
				::rl::math::Vector3 tmpPoint1;
				::rl::math::Vector3 tmpPoint2;
				
				::rl::math::Real tmpDistance = this->distance(
					*i,
					point,
					tmpPoint1,
					tmpPoint2
				);
				
				if (tmpDistance < distance)
				{
					distance = tmpDistance;
					point1 = tmpPoint1;
					point2 = tmpPoint2;
				}
			}
			
			return distance;
		}
	}
}
