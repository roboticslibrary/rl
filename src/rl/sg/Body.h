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

#ifndef RL_SG_BODY_H
#define RL_SG_BODY_H

#include <string>
#include <vector>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>

namespace rl
{
	namespace sg
	{
		class Model;
		class Shape;
		
		class Body
		{
		public:
			typedef ::std::vector<Shape*>::iterator Iterator;
			
			Body(Model* model);
			
			virtual ~Body();
			
			virtual void add(Shape* shape);
			
			Iterator begin();
			
			virtual Shape* create(::SoVRMLShape* shape) = 0;
			
			Iterator end();
			
			void getBoundingBoxPoints(const ::rl::math::Transform& frame, ::std::vector< ::rl::math::Vector3>& p) const;
			
			Model* getModel() const;
			
			virtual ::std::string getName() const;
			
			::std::size_t getNumShapes() const;
			
			void getPoints(const ::rl::math::Transform& frame, ::std::vector< ::rl::math::Vector3>& p) const;
			
			Shape* getShape(const ::std::size_t& i) const;
			
			virtual void getFrame(::rl::math::Transform& frame) = 0;
			
			virtual void remove(Shape* shape);
			
			virtual void setFrame(const ::rl::math::Transform& frame) = 0;
			
			virtual void setName(const ::std::string& name);
			
			::rl::math::Vector3 center;
			
			::rl::math::Vector3 max;
			
			::rl::math::Vector3 min;
			
			::std::vector< ::rl::math::Vector3> points;
			
		protected:
			Model* model;
			
			::std::vector<Shape*> shapes;
			
		private:
			::std::string name;
		};
	}
}

#endif // RL_SG_BODY_H
