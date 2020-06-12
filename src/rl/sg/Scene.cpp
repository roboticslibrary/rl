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

#include <algorithm>

#include "Scene.h"
#include "XmlFactory.h"

namespace rl
{
	namespace sg
	{
		Scene::Scene() :
			Base(),
			models(),
			name()
		{
		}
		
		Scene::~Scene()
		{
		}
		
		void
		Scene::add(Model* model)
		{
			this->models.push_back(model);
		}
		
		Scene::Iterator
		Scene::begin()
		{
			return this->models.begin();
		}
		
		Scene::Iterator
		Scene::end()
		{
			return this->models.end();
		}
		
		Model*
		Scene::getModel(const ::std::size_t& i) const
		{
			return this->models[i];
		}
		
		::std::string
		Scene::getName() const
		{
			return this->name;
		}
		
		::std::size_t
		Scene::getNumModels() const
		{
			return this->models.size();
		}
		
		bool
		Scene::isScalingSupported() const
		{
			return true;
		}
		
		void
		Scene::load(const ::std::string& filename, const bool& doBoundingBoxPoints, const bool& doPoints)
		{
			XmlFactory factory;
			factory.load(filename, this, doBoundingBoxPoints, doPoints);
		}
		
		void
		Scene::remove(Model* model)
		{
			Iterator found = ::std::find(this->models.begin(), this->models.end(), model);
			
			if (found != this->models.end())
			{
				this->models.erase(found);
			}
		}
		
		void
		Scene::setName(const ::std::string& name)
		{
			this->name = name;
		}
	}
}
