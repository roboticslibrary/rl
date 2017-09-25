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

#ifndef RL_SG_SCENE_H
#define RL_SG_SCENE_H

#include <string>
#include <vector>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>

namespace rl
{
	/**
	 * Scene graph abstraction.
	 */
	namespace sg
	{
		class Model;
		
		class Scene
		{
		public:
			typedef ::std::vector<Model*>::iterator Iterator;
			
			Scene();
			
			virtual ~Scene();
			
			virtual void add(Model* model);
			
			Iterator begin();
			
			virtual Model* create() = 0;
			
			Iterator end();
			
			Model* getModel(const ::std::size_t& i) const;
			
			virtual ::std::string getName() const;
			
			::std::size_t getNumModels() const;
			
			void load(const ::std::string& filename, const bool& doBoundingBoxPoints = false, const bool& doPoints = false);
			
			virtual void remove(Model* model);
			
			virtual void setName(const ::std::string& name);
			
		protected:
			bool isScalingSupported;
			
			::std::vector<Model*> models;
			
		private:
			static void triangleCallback(void* userData, ::SoCallbackAction* action, const ::SoPrimitiveVertex* v1, const ::SoPrimitiveVertex* v2, const ::SoPrimitiveVertex* v3);
			
			::std::string name;
		};
	}
}

#endif // RL_SG_SCENE_H
