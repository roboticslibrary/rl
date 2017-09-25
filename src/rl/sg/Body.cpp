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
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/lists/SoPathList.h>
#include <Inventor/VRMLnodes/SoVRMLInline.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>

#include "Body.h"
#include "Model.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		Body::Body(Model* model) :
			center(::rl::math::Vector3::Zero()),
			max(::rl::math::Vector3::Zero()),
			min(::rl::math::Vector3::Zero()),
			points(),
			model(model),
			shapes(),
			name()
		{
		}
		
		Body::~Body()
		{
		}
		
		void
		Body::add(Shape* shape)
		{
			this->shapes.push_back(shape);
		}
		
		Body::Iterator
		Body::begin()
		{
			return this->shapes.begin();
		}
		
		Body::Iterator
		Body::end()
		{
			return this->shapes.end();
		}
		
		void
		Body::getBoundingBoxPoints(const ::rl::math::Transform& frame, ::std::vector< ::rl::math::Vector3>& p) const
		{
			p.resize(8);
			
			for (::std::size_t i = 0; i < 8; ++i)
			{
				for (::std::size_t j = 0, k = 1; j < 3; ++j, k *= 2)
				{
					p[i](j) = i & k ? this->max(j) : this->min(j);
				}
			}
			
			for (::std::size_t i = 0; i < p.size(); ++i)
			{
				p[i] = frame.linear() * p[i] + frame.translation();
			}
		}
		
		Model*
		Body::getModel() const
		{
			return this->model;
		}
		::std::string
		Body::getName() const
		{
			return this->name;
		}
		
		::std::size_t
		Body::getNumShapes() const
		{
			return this->shapes.size();
		}
		
		void
		Body::getPoints(const ::rl::math::Transform& frame, ::std::vector< ::rl::math::Vector3>& p) const
		{
			p.resize(this->points.size());
			
			for (::std::size_t i = 0; i < p.size(); ++i)
			{
				p[i] = frame.linear() * this->points[i] + frame.translation();
			}
		}
		
		Shape*
		Body::getShape(const ::std::size_t& i) const
		{
			return this->shapes[i];
		}
		
		void
		Body::remove(Shape* shape)
		{
			Iterator found = ::std::find(this->shapes.begin(), this->shapes.end(), shape);
			
			if (found != this->shapes.end())
			{
				this->shapes.erase(found);
			}
		}
		
		void
		Body::setName(const ::std::string& name)
		{
			this->name = name;
		}
	}
}
