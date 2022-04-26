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

#ifndef RL_MATH_COMPOSITEFUNCTION_H
#define RL_MATH_COMPOSITEFUNCTION_H

#include <cmath>
#include <stdexcept>

#include "Function.h"

namespace rl
{
	namespace math
	{
		/**
		 * A composite function that takes two functions \f$f\f$ and \f$g\f$ and produces
		 * a function \f$h = f \circ g\f$ such that \f$h(x) = f(g(x))\f$.
		 */
		template<typename T, typename T2>
		class CompositeFunction : public Function<T>
		{
		public:
			CompositeFunction(const Function<T>& f, const Function<T2>& g) :
				Function<T>(g.lower(), g.upper()),
				f(f),
				g(g)
			{
			}
			
			virtual ~CompositeFunction()
			{
			}
			
			CompositeFunction* clone() const
			{
				return new CompositeFunction(*this);
			}
			
			T operator()(const Real& x, const ::std::size_t& derivative = 0) const
			{
				using ::std::pow;
				
				switch (derivative)
				{
				case 0:
					return this->f(this->g(x));
					break;
				case 1:
					return this->f(this->g(x), 1) * this->g(x, 1);
					break;
				case 2:
					{
						T2 g0 = this->g(x);
						return pow(this->g(x, 1), 2) * this->f(g0, 2) + this->f(g0, 1) * this->g(x, 2);
					}
					break;
				case 3:
					{
						T2 g0 = this->g(x);
						T2 g1 = this->g(x, 1);
						return 3 * g1 * this->f(g0, 2) * this->g(x, 2) + pow(g1, 3) * this->f(g0, 3) + this->f(g0, 1) * this->g(x, 3);
					}
					break;
				default:
					throw ::std::runtime_error("rl::math::CompositeFunction: Derivatives > 3 not supported");
					break;
				}
			}
			
		protected:
			
		private:
			const Function<T>& f;
			
			const Function<T2>& g;
		};
	}
}

#endif // RL_MATH_COMPOSITEFUNCTION_H
