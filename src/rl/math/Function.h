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

#ifndef RL_MATH_FUNCTION_H
#define RL_MATH_FUNCTION_H

#include "Real.h"

namespace rl
{
	namespace math
	{
		/**
		 * A mathematical mapping from Real -> ArrayX.
		 * 
		 * A Function is guaranteed to be defined in the interval [lower() upper()],
		 * and may be defined outside this interval. Its computation is expected
		 * to be numerically stable, accurate and efficient.
		 */
		template<typename T>
		class Function
		{
		public:
			Function() :
				x0(0),
				x1(1)
			{
			}
			
			virtual ~Function()
			{
			}
			
			virtual Function* clone() const = 0;
			
			Real duration() const
			{
				return this->upper() - this->lower();
			}
			
			Real& lower()
			{
				return this->x0;
			}
			
			const Real& lower() const
			{
				return this->x0;
			}
			
			Real& upper()
			{
				return this->x1;
			}
			
			const Real& upper() const
			{
				return this->x1;
			}
			
			/**
			 * Evaluates the function or a derivative thereof for a given value x.
			 * 
			 * Some functions are only defined in the interval [lower(), upper()],
			 * and fail to evaluate outside of 
			 * [lower() - FUNCTION_BOUNDARY, upper() + FUNCTION_BOUNDARY].
			 * In Debug mode, this is signaled by failing asserts.
			 * In Release mode, the function is evaluated if algebraically possible,
			 * or will return an empty ArrayX otherwise. 
			 * Some functions are not indefinitely often differentiable, 
			 * and will return a NaN array for all higher orders.
			 * 
			 * @param[in] x Input value of the function or derivative
			 * @param[in] derivative Order of the derivative to be evaluated
			 */
			virtual T operator()(const Real& x, const ::std::size_t& derivative = 0) const = 0;
			
		protected:
			Real x0;
			
			Real x1;
			
		private:
			
		};
		
		static const Real FUNCTION_BOUNDARY = 1.0e-8f;
	}
}

#endif // RL_MATH_FUNCTION_H
