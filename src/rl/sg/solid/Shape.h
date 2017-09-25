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

#ifndef RL_SG_SOLID_SHAPE_H
#define RL_SG_SOLID_SHAPE_H

#include <unordered_set>
#include <Inventor/fields/SoMFInt32.h>
#include <Inventor/fields/SoMFVec3f.h>
#include <SOLID/SOLID.h>
#include <SOLID/SOLID_broad.h>

#include "../Shape.h"

namespace rl
{
	namespace sg
	{
		namespace solid
		{
			class Body;
			
			class Shape : public ::rl::sg::Shape
			{
			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				
				Shape(SoVRMLShape* shape, Body* body);
				
				virtual ~Shape();
				
				void getTransform(::rl::math::Transform& transform);
				
				void setMargin(const ::rl::math::Real& margin);
				
				void setTransform(const ::rl::math::Transform& transform);
				
				void update();
				
				bool complex;
				
				::std::unordered_set<Shape*> encounters;
				
				DT_ObjectHandle object;
				
				BP_ProxyHandle proxy;
				
			protected:
				
			private:
				DT_ShapeHandle create(SoVRMLShape* shape);
				
				DT_ShapeHandle create(const SoMFVec3f& point);
				
				DT_ShapeHandle create(const SoMFVec3f& point, const SoMFInt32& coordIndex);
				
				::rl::math::Transform frame;
				
				DT_Vector3 max;
				
				DT_Vector3 min;
				
				DT_ShapeHandle shape;
				
				::rl::math::Transform transform;
			};
		}
	}
}

#endif // RL_SG_SOLID_SHAPE_H
