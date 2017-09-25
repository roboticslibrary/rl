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

#ifndef RL_PLAN_METRIC_H
#define RL_PLAN_METRIC_H

#include <rl/math/Real.h>
#include <rl/math/Vector.h>

namespace rl
{
	namespace plan
	{
		class Model;
		
		class Metric
		{
		public:
			typedef ::rl::math::Real Distance;
			
			typedef ::std::size_t Size;
			
			struct Value
			{
				typedef const ::rl::math::Real* const_iterator;
				
				Value();
				
				Value(const ::rl::math::Vector* first, void* second);
				
				const ::rl::math::Real* begin() const;
				
				const ::rl::math::Real* end() const;
				
				::std::size_t size() const;
				
				const ::rl::math::Vector* first;
				
				void* second;
			};
			
			Metric(Model* model, const bool& transformed);
			
			virtual ~Metric();
			
			Distance operator()(const Value& lhs, const Value& rhs) const;
			
			Distance operator()(const Distance& lhs, const Distance& rhs, const ::std::size_t& index) const;
			
		protected:
			
		private:
			Model* model;
			
			bool transformed;
		};
	}
}

#endif // RL_PLAN_METRIC_H
