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

#ifndef RL_SG_FCL_SHAPE_H
#define RL_SG_FCL_SHAPE_H

#include <memory>
#include <fcl/config.h>
#include <Inventor/actions/SoCallbackAction.h>

#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
#include <fcl/collision.h>
#else
#include <fcl/narrowphase/collision.h>
#endif

#include "../Shape.h"

namespace rl
{
	namespace sg
	{
		namespace fcl
		{
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
			typedef ::fcl::CollisionGeometry CollisionGeometry;
			typedef ::fcl::CollisionObject CollisionObject;
			typedef ::fcl::FCL_REAL Real;
			typedef ::fcl::Vec3f Vector3;
#else
			typedef ::fcl::CollisionGeometry<::rl::math::Real> CollisionGeometry;
			typedef ::fcl::CollisionObject<::rl::math::Real> CollisionObject;
			typedef ::rl::math::Real Real;
			typedef ::fcl::Vector3<::rl::math::Real> Vector3;
#endif
			
			class RL_SG_EXPORT Shape : public ::rl::sg::Shape
			{
			public:
				Shape(SoVRMLShape* shape, ::rl::sg::Body* body);
				
				virtual ~Shape();
				
				void getTransform(::rl::math::Transform& transform);
				
				void setTransform(const ::rl::math::Transform& transform);
				
				void update(const ::rl::math::Transform& frame);
				
				::std::shared_ptr<CollisionObject> collisionObject;
				
			protected:
				
			private:
				static void triangleCallback(void* userData, SoCallbackAction* action, const SoPrimitiveVertex* v1, const SoPrimitiveVertex* v2, const SoPrimitiveVertex* v3);
				
				::rl::math::Transform base;
				
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
				::std::vector<Real> distances;
#endif
				
				::rl::math::Transform frame;
				
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 5
				::boost::shared_ptr<CollisionGeometry> geometry;
#else
				::std::shared_ptr<CollisionGeometry> geometry;
#endif
				
				::std::vector<int> indices;
				
#if FCL_MAJOR_VERSION < 1 && FCL_MINOR_VERSION < 6
				::std::vector<Vector3> normals;
#endif
				
				::std::vector<int> polygons;
				
				::rl::math::Transform transform;
				
				::std::vector<Vector3> vertices;
			};
		}
	}
}

#endif // RL_SG_FCL_SHAPE_H
