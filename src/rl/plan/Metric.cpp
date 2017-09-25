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

#include "Metric.h"
#include "Model.h"

namespace rl
{
	namespace plan
	{
		Metric::Metric(Model* model, const bool& transformed) :
			model(model),
			transformed(transformed)
		{
		}
		
		Metric::~Metric()
		{
		}
		
		Metric::Distance
		Metric::operator()(const Value& lhs, const Value& rhs) const
		{
			if (this->transformed)
			{
				return this->model->transformedDistance(*lhs.first, *rhs.first);
			}
			else
			{
				return this->model->distance(*lhs.first, *rhs.first);
			}
		}
		
		Metric::Distance
		Metric::operator()(const Distance& lhs, const Distance& rhs, const ::std::size_t& index) const
		{
			return this->model->transformedDistance(lhs, rhs, index);
		}
		
		Metric::Value::Value() :
			first(),
			second()
		{
		}
		
		Metric::Value::Value(const ::rl::math::Vector* first, void* second) :
			first(first),
			second(second)
		{
		}
		
		const ::rl::math::Real*
		Metric::Value::begin() const
		{
			return this->first->data();
		}
		
		const ::rl::math::Real*
		Metric::Value::end() const
		{
			return this->first->data() + this->first->size();
		}
		
		::std::size_t
		Metric::Value::size() const
		{
			return this->first->size();
		}
	}
}
