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

#ifndef _RL_MATH_CUBIC_H_
#define _RL_MATH_CUBIC_H_

#include <cmath>
#include <limits>

#include "Real.h"

namespace rl
{
	namespace math
	{
		template< typename T >
		class Cubic
		{
		public:
			Cubic() :
				te(::std::numeric_limits<Real>::max()),
				x0(),
				xe(),
				v0(),
				ve(),
				c0(),
				c1(),
				c2(),
				c3()
			{
			}
			
			virtual ~Cubic()
			{
			}
			
			T a(const Real& t) const
			{
				return 2 * c2 + 6 * c3 * t;
			}
			
			void interpolate()
			{
				c0 = x0;
				c1 = v0;
				c2 = -(2 * te * v0 + 3 * x0 - 3 * xe + te * ve) / ::std::pow(te, 2);
				c3 = (2 * x0 + te * v0 - 2 * xe + te * ve) / ::std::pow(te, 3);
			}
			
			T v(const Real& t) const
			{
				return c1 + 2 * c2 * t + 3 * c3 * ::std::pow(t, 2);
			}
			
			T x(const Real& t) const
			{
				return c0 + c1 * t + c2 * ::std::pow(t, 2) + c3 * ::std::pow(t, 3);
			}
			
			Real te;
			
			T x0;
			
			T xe;
			
			T v0;
			
			T ve;
			
		protected:
			
		private:
			T c0;
			
			T c1;
			
			T c2;
			
			T c3;
		};
	}
}

#endif // _RL_MATH_CUBIC_H_
