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

#ifndef _RL_SG_ODE_SCENE_H_
#define _RL_SG_ODE_SCENE_H_

#include <ode/ode.h>

#include "../RaycastScene.h"
#include "../SimpleScene.h"

namespace rl
{
	namespace sg
	{
		namespace ode
		{
			class Scene : public ::rl::sg::RaycastScene, public ::rl::sg::SimpleScene
			{
			public:
				Scene();
				
				virtual ~Scene();
				
				bool areColliding(::rl::sg::Body* first, ::rl::sg::Body* second);
				
				bool areColliding(::rl::sg::Model* first, ::rl::sg::Model* second);
				
				bool areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second);
				
				::rl::sg::Model* create();
				
				bool isColliding();
				
				::rl::sg::Shape* raycast(const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance);
				
				bool raycast(::rl::sg::Shape* shape, const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance);
				
				dSpaceID space;
				
				dWorldID world;
				
			protected:
				
			private:
				static void bodyNearCallback(void* data, dGeomID o1, dGeomID o2);
				
				static void modelNearCallback(void* data, dGeomID o1, dGeomID o2);
				
				static void rayNearCallback(void* data, dGeomID o1, dGeomID o2);
				
				static void shapeNearCallback(void* data, dGeomID o1, dGeomID o2);
			};
		}
	}
}

#endif // _RL_SG_ODE_SCENE_H_
