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

#ifndef RL_SG_MODEL_H
#define RL_SG_MODEL_H

#include <string>
#include <vector>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <rl/math/Transform.h>

namespace rl
{
	namespace sg
	{
		class Body;
		class Scene;
		
		class Model
		{
		public:
			typedef ::std::vector<Body*>::iterator Iterator;
			
			Model(Scene* scene);
			
			virtual ~Model();
			
			virtual void add(Body* body);
			
			Iterator begin();
			
			virtual Body* create() = 0;
			
			Iterator end();
			
			Body* getBody(const ::std::size_t& i) const;
			
			virtual ::std::string getName() const;
			
			::std::size_t getNumBodies() const;
			
			Scene* getScene() const;
			
			virtual void remove(Body* body);
			
			virtual void setName(const ::std::string& name);
			
		protected:
			::std::vector<Body*> bodies;
			
			Scene* scene;
			
		private:
			::std::string name;
		};
	}
}

#endif // RL_SG_MODEL_H
