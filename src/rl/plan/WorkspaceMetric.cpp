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

#include "WorkspaceMetric.h"

namespace rl
{
	namespace plan
	{
		WorkspaceMetric::WorkspaceMetric(::rl::math::Real* distanceWeight, bool* alternativeDistanceComputation) :
			alternativeDistanceComputation(alternativeDistanceComputation),
			distanceWeight(distanceWeight)
		{
		}
		
		WorkspaceMetric::~WorkspaceMetric()
		{
		}
		
		WorkspaceMetric::Distance
		WorkspaceMetric::operator()(const Value& lhs, const Value& rhs) const
		{
			if (*this->alternativeDistanceComputation)
			{
				::rl::math::Vector6 delta = lhs.first->toDelta(*rhs.first, true);
				return delta.norm();
			}
			else
			{
				return lhs.first->distance(*rhs.first, *this->distanceWeight);
			}
		}
		
		WorkspaceMetric::Value::Value() :
			first(),
			second()
		{
		}
		
		WorkspaceMetric::Value::Value(const ::rl::math::Transform* first, void* second) :
			first(first),
			second(second)
		{
		}
	}
}
