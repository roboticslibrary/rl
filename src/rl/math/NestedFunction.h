//
// Copyright (c) 2009, Andre Gaschler, Markus Rickert
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

#ifndef RL_MATH_NESTEDFUNCTION_H
#define RL_MATH_NESTEDFUNCTION_H

#include <stdexcept>

#include "Function.h"

namespace rl
{
	namespace math
	{
		template<typename T2, typename T>
		class NestedFunction : public Function<T>
		{
		public:
			NestedFunction(const Function<T2>& inner, const Function<T>& outer) :
				Function<T>(),
				inner(inner),
				outer(outer)
			{
				this->lower() = this->inner.lower();
				this->upper() = this->inner.upper();
			}
			
			virtual ~NestedFunction()
			{
			}
			
			NestedFunction* clone() const
			{
				return new NestedFunction(*this);
			}

			T operator()(const Real& x, const ::std::size_t& derivative = 0) const
			{
				switch (derivative)
				{
				case 0:
					return this->outer(this->inner(x));
					break;
				case 1:
					return this->outer(this->inner(x), 1) * this->inner(x, 1);
					break;
				default:
					throw ::std::runtime_error("rl::math::NestedFunction: Derivatives > 1 not supported");
					break;
				}
			}
			
		protected:

		private:
			const Function<T2>& inner;
			
			const Function<T>& outer;
		};
	}
}

#endif // RL_MATH_NESTEDFUNCTION_H
