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

#ifndef _RL_PLAN_VIEWER_H_
#define _RL_PLAN_VIEWER_H_

#include <rl/math/Transform.h>
#include <rl/math/Vector.h>

#include "VectorList.h"

namespace rl
{
	namespace plan
	{
		class Viewer
		{
		public:
			Viewer();
			
			virtual ~Viewer();
			
			virtual void drawConfiguration(const ::rl::math::Vector& q) = 0;
			
			virtual void drawConfigurationEdge(const ::rl::math::Vector& q0, const ::rl::math::Vector& q1, const bool& free = true) = 0;
			
			virtual void drawConfigurationPath(const VectorList& path) = 0;
			
			virtual void drawConfigurationVertex(const ::rl::math::Vector& q, const bool& free = true) = 0;
			
			virtual void drawLine(const ::rl::math::Vector& xyz0, const ::rl::math::Vector& xyz1) = 0;
			
			virtual void drawPoint(const ::rl::math::Vector& xyz) = 0;
			
			virtual void drawSphere(const ::rl::math::Vector& center, const ::rl::math::Real& radius) = 0;
			
			virtual void drawWork(const ::rl::math::Transform& t) = 0;
			
			virtual void drawWorkEdge(const ::rl::math::Vector& q0, const ::rl::math::Vector& q1) = 0;
			
			virtual void drawWorkPath(const VectorList& path) = 0;
			
			virtual void drawWorkVertex(const ::rl::math::Vector& q) = 0;
			
			virtual void reset() = 0;
			
			virtual void resetEdges() = 0;
			
			virtual void resetLines() = 0;
			
			virtual void resetPoints() = 0;
			
			virtual void resetSpheres() = 0;
			
			virtual void resetVertices() = 0;
			
		protected:
			
		private:
			
		};
	}
}

#endif // VIEWER_H_
