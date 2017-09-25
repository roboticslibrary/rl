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

#include <rl/sg/Body.h>
#include <rl/sg/DistanceScene.h>

#include "DistanceModel.h"

namespace rl
{
	namespace plan
	{
		DistanceModel::DistanceModel() :
			SimpleModel()
		{
		}
		
		DistanceModel::~DistanceModel()
		{
		}
		
		::rl::math::Real
		DistanceModel::distance(const ::rl::math::Vector3& point)
		{
			::rl::math::Real distance = ::std::numeric_limits< ::rl::math::Real>::max();
			::rl::math::Vector3 point1;
			::rl::math::Vector3 point2;
			
			for (::rl::sg::Scene::Iterator i = this->scene->begin(); i != this->scene->end(); ++i)
			{
				if (*i != this->model)
				{
					distance = ::std::min(distance, dynamic_cast< ::rl::sg::DistanceScene*>(this->scene)->distance(*i, point, point1, point2));
				}
			}
			
			return distance;
		}
		
		::rl::math::Real
		DistanceModel::distance(const ::std::size_t& body, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2)
		{
			assert(body < this->model->getNumBodies());
			
			::rl::math::Real distance = ::std::numeric_limits< ::rl::math::Real>::max();
			
			::rl::math::Vector3 tmpPoint1;
			::rl::math::Vector3 tmpPoint2;
			
			if (this->isColliding(body))
			{
				for (::rl::sg::Scene::Iterator i = this->scene->begin(); i != this->scene->end(); ++i)
				{
					if (this->model != *i)
					{
						for (::rl::sg::Model::Iterator j = (*i)->begin(); j != (*i)->end(); ++j)
						{
							::rl::math::Real tmpDistance = dynamic_cast< ::rl::sg::DistanceScene*>(this->scene)->distance(this->model->getBody(body), *j, tmpPoint1, tmpPoint2);
							
							if (tmpDistance < distance)
							{
								distance = tmpDistance;
								point1 = tmpPoint1;
								point2 = tmpPoint2;
							}
						}
					}
				}
				
				for (::std::size_t i = 0; i < this->model->getNumBodies(); ++i)
				{
					if (body != i)
					{
						if (this->areColliding(body, i))
						{
							::rl::math::Real tmpDistance = dynamic_cast< ::rl::sg::DistanceScene*>(this->scene)->distance(this->model->getBody(body), this->model->getBody(i), tmpPoint1, tmpPoint2);
							
							if (tmpDistance < distance)
							{
								distance = tmpDistance;
								point1 = tmpPoint1;
								point2 = tmpPoint2;
							}
						}
					}
				}
			}
			
			return distance;
		}
		
		void
		DistanceModel::distance(const ::std::size_t& body, RealList& distances, Vector3List& points1, Vector3List& points2)
		{
			assert(body < this->model->getNumBodies());
			
			distances.clear();
			points1.clear();
			points2.clear();
			
			::rl::math::Vector3 tmpPoint1;
			::rl::math::Vector3 tmpPoint2;
			
			if (this->isColliding(body))
			{
				for (::rl::sg::Scene::Iterator i = this->scene->begin(); i != this->scene->end(); ++i)
				{
					if (this->model != *i)
					{
						for (::rl::sg::Model::Iterator j = (*i)->begin(); j != (*i)->end(); ++j)
						{
							for (::rl::sg::Body::Iterator k = this->model->getBody(body)->begin(); k != this->model->getBody(body)->end(); ++k)
							{
								for (::rl::sg::Body::Iterator l = (*j)->begin(); l != (*j)->end(); ++l)
								{
									::rl::math::Vector3 tmpPoint1;
									::rl::math::Vector3 tmpPoint2;
									
									::rl::math::Real tmpDistance = dynamic_cast< ::rl::sg::DistanceScene*>(this->scene)->distance(
										*k,
										*l,
										tmpPoint1,
										tmpPoint2
									);
									
									distances.push_back(tmpDistance);
									points1.push_back(tmpPoint1);
									points2.push_back(tmpPoint2);
								}
							}
						}
					}
				}
				
				for (::std::size_t i = 0; i < this->model->getNumBodies(); ++i)
				{
					if (body != i)
					{
						if (this->areColliding(body, i))
						{
							for (::rl::sg::Body::Iterator k = this->model->getBody(body)->begin(); k != this->model->getBody(body)->end(); ++k)
							{
								for (::rl::sg::Body::Iterator l = this->model->getBody(i)->begin(); l != this->model->getBody(i)->end(); ++l)
								{
									::rl::math::Vector3 tmpPoint1;
									::rl::math::Vector3 tmpPoint2;
									
									::rl::math::Real tmpDistance = dynamic_cast< ::rl::sg::DistanceScene*>(this->scene)->distance(
										*k,
										*l,
										tmpPoint1,
										tmpPoint2
									);
									
									distances.push_back(tmpDistance);
									points1.push_back(tmpPoint1);
									points2.push_back(tmpPoint2);
								}
							}
						}
					}
				}
			}
		}
	}
}
