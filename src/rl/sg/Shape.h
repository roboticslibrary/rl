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

#ifndef RL_SG_SHAPE_H
#define RL_SG_SHAPE_H

#include <string>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <rl/math/Transform.h>

namespace rl
{
	namespace sg
	{
		class Body;
		
		class Shape
		{
		public:
			Shape(::SoVRMLShape* shape, Body* body);
			
			virtual ~Shape();
			
			Body* getBody() const;
			
			virtual ::std::string getName() const;
			
			virtual void getTransform(::rl::math::Transform& transform) = 0;
			
			virtual void setName(const ::std::string& name);
			
			virtual void setTransform(const ::rl::math::Transform& transform) = 0;
			
		protected:
			Body* body;
			
		private:
			::std::string name;
		};
	}
}

#endif // RL_SG_SHAPE_H
