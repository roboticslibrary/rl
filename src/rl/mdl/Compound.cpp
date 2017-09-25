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

#include "Compound.h"
#include "Frame.h"
#include "Transform.h"

namespace rl
{
	namespace mdl
	{
		Compound::Compound(Model* model) :
			inFrame(nullptr),
			inTransform(nullptr),
			outFrame(nullptr),
			outTransform(nullptr),
			model(model),
			tree(
				model->tree,
				::boost::get(::boost::edge_weight, model->tree),
				::boost::get(::boost::vertex_color, model->tree)
			)
		{
		}
		
		Compound::~Compound()
		{
		}
		
		void
		Compound::add(Frame* frame)
		{
			frame->compound = this;
		}
		
		void
		Compound::add(Transform* transform, const Frame* a, const Frame* b)
		{
			transform->compound = this;
		}
		
		void
		Compound::remove(Frame* frame)
		{
			frame->compound = nullptr;
		}
		
		void
		Compound::remove(Transform* transform)
		{
			transform->compound = nullptr;
		}
	}
}
