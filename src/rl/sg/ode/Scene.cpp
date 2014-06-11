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
#include "Model.h"
#include "Scene.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		namespace ode
		{
			Scene::Scene() :
				::rl::sg::Scene(),
				::rl::sg::RaycastScene(),
				::rl::sg::SimpleScene(),
				space(dHashSpaceCreate(NULL)),
				world(dWorldCreate())
			{
				dInitODE();
			}
			
			Scene::~Scene()
			{
				while (this->models.size() > 0)
				{
					delete this->models[0];
				}
				
				dSpaceDestroy(this->space);
				dWorldDestroy(this->world);
				dCloseODE();
			}
			
			bool
			Scene::areColliding(::rl::sg::Body* first, ::rl::sg::Body* second)
			{
				bool data = false;
				
				dSpaceCollide2(
					reinterpret_cast< dGeomID >(static_cast< Body* >(first)->space),
					reinterpret_cast< dGeomID >(static_cast< Body* >(second)->space),
					&data,
					&Scene::shapeNearCallback
				);
				
				return data;
			}
			
			bool
			Scene::areColliding(::rl::sg::Model* first, ::rl::sg::Model* second)
			{
				bool data = false;
				
				dSpaceCollide2(
					reinterpret_cast< dGeomID >(static_cast< Model* >(first)->space),
					reinterpret_cast< dGeomID >(static_cast< Model* >(second)->space),
					&data,
					&Scene::bodyNearCallback
				);
				
				return data;
			}
			
			bool
			Scene::areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second)
			{
				bool data = false;
				
				this->bodyNearCallback(
					&data,
					static_cast< Shape* >(first)->geom,
					static_cast< Shape* >(second)->geom
				);
				
				return data;
			}
			
			void
			Scene::bodyNearCallback(void* data, dGeomID o1, dGeomID o2)
			{
				dSpaceCollide2(
					o1,
					o2,
					data,
					&Scene::shapeNearCallback
				);
			}
			
			bool
			Scene::isColliding()
			{
				bool data = false;
				
				dSpaceCollide(this->space, &data, &Scene::modelNearCallback);
				
				return data;
			}
			
			::rl::sg::Model*
			Scene::create()
			{
				return new Model(this);
			}
			
			void
			Scene::modelNearCallback(void* data, dGeomID o1, dGeomID o2)
			{
				dSpaceCollide2(
					o1,
					o2,
					data,
					&Scene::bodyNearCallback
				);
				
				if (*static_cast< bool* >(data))
				{
					return;
				}
				
				dSpaceCollide(
					reinterpret_cast< dSpaceID >(o1),
					data,
					&Scene::bodyNearCallback
				);
				
				if (*static_cast< bool* >(data))
				{
					return;
				}
				
				dSpaceCollide(
					reinterpret_cast< dSpaceID >(o2),
					data,
					&Scene::bodyNearCallback
				);
			}
			
			::rl::sg::Shape*
			Scene::raycast(const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance)
			{
				dGeomID ray = dCreateRay(NULL, std::numeric_limits< dReal >::max());
				
				dGeomRaySet(
					ray,
					static_cast< dReal >(source.x()),
					static_cast< dReal >(source.y()),
					static_cast< dReal >(source.z()),
					static_cast< dReal >(target.x() - source.x()),
					static_cast< dReal >(target.y() - source.y()),
					static_cast< dReal >(target.z() - source.z())
				);
				
				dContactGeom contact;
				contact.depth = std::numeric_limits< dReal >::max();
				contact.g1 = NULL;
				contact.g2 = NULL;
				
				dSpaceCollide2(ray, reinterpret_cast< dGeomID >(this->space), &contact, &Scene::rayNearCallback);
				
				distance = contact.depth;
				
				if (NULL != contact.g1 && ray != contact.g1)
				{
					return static_cast< ::rl::sg::Shape* >(dGeomGetData(contact.g1));
				}
				else if (NULL != contact.g2 && ray != contact.g2)
				{
					return static_cast< ::rl::sg::Shape* >(dGeomGetData(contact.g2));
				}
				else
				{
					return NULL;
				}
			}
			
			bool
			Scene::raycast(::rl::sg::Shape* shape, const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance)
			{
				dGeomID ray = dCreateRay(NULL, std::numeric_limits< dReal >::max());
				
				dGeomRaySet(
					ray,
					static_cast< dReal >(source.x()),
					static_cast< dReal >(source.y()),
					static_cast< dReal >(source.z()),
					static_cast< dReal >(target.x() - source.x()),
					static_cast< dReal >(target.y() - source.y()),
					static_cast< dReal >(target.z() - source.z())
				);
				
				dContactGeom contacts[1];
				
				if (dCollide(ray, static_cast< Shape* >(shape)->geom, 1, contacts, sizeof(dContactGeom)) > 0)
				{
					distance = contacts[0].depth;
					return true;
				}
				else
				{
					distance = std::numeric_limits< dReal >::quiet_NaN();
					return false;
				}
			}
			
			void
			Scene::rayNearCallback(void* data, dGeomID o1, dGeomID o2)
			{
				if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
				{ 
					dSpaceCollide2(o1, o2, data, &Scene::rayNearCallback); 
					
					if (dGeomIsSpace(o1))
					{
						dSpaceCollide(reinterpret_cast< dSpaceID >(o1), data, &Scene::rayNearCallback);
					}
					
					if (dGeomIsSpace(o2))
					{
						dSpaceCollide(reinterpret_cast< dSpaceID >(o2), data, &Scene::rayNearCallback);
					}
				}
				else
				{
					dContactGeom contacts[1];
					
					if (dCollide(o1, o2, 1, contacts, sizeof(dContactGeom)) > 0)
					{
						if (contacts[0].depth < static_cast< dContactGeom* >(data)->depth)
						{
							static_cast< dContactGeom* >(data)->depth = contacts[0].depth;
							static_cast< dContactGeom* >(data)->g1 = o1;
							static_cast< dContactGeom* >(data)->g2 = o2;
						}
					}
				}
			}
			
			void
			Scene::shapeNearCallback(void* data, dGeomID o1, dGeomID o2)
			{
				dContactGeom contacts[1];
				
				if (dCollide(o1, o2, 1, contacts, sizeof(dContactGeom)) > 0)
				{
					*static_cast< bool* >(data) = true;
				}
			}
		}
	}
}
