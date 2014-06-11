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

#ifndef _RL_MATH_QUINTIC_H_
#define _RL_MATH_QUINTIC_H_

#include <cmath>
#include <limits>

#include "Real.h"

namespace rl
{
	namespace math
	{
		template< typename T >
		class Quintic
		{
		public:
			Quintic() :
				a0(),
				ae(),
				te(::std::numeric_limits<Real>::max()),
				x0(),
				xe(),
				v0(),
				ve(),
				c0(),
				c1(),
				c2(),
				c3(),
				c4(),
				c5()
			{
			}
			
			virtual ~Quintic()
			{
			}
			
			T a(const Real& t) const
			{
				return 2 * c2 + 6 * c3 * t + 12 * c4 * ::std::pow(t, 2) + 20 * c5 * ::std::pow(t, 3);
			}
			
			void interpolate()
			{
				c0 = x0;
				c1 = v0;
				c2 = a0 / 2;
				c3 = -(3 * ::std::pow(te, 2) * a0 + 12 * te * v0 - ::std::pow(te, 2) * ae + 20 * x0 + 8 * te * ve - 20 * xe) / (2 * ::std::pow(te, 3));
				c4 = (16 * te * v0 - 2 * ::std::pow(te, 2) * ae + 30 * x0 + 14 * te * ve - 30 * xe + 3 * ::std::pow(te, 2) * a0) / (2 * ::std::pow(te, 4));
				c5 = -(12 * x0 + 6 * te * v0 + 6 * te * ve - 12 * xe + ::std::pow(te, 2) * a0 - ::std::pow(te, 2) * ae) / (2 * ::std::pow(te, 5));
			}
			
			T j(const Real& t) const
			{
				return 6 * c3 + 24 * c4 * t + 60 * c5 * ::std::pow(t, 2);
			}
			
			T v(const Real& t) const
			{
				return c1 + 2 * c2 * t + 3 * c3 * ::std::pow(t, 2) + 4 * c4 * ::std::pow(t, 3) + 5 * c5 * ::std::pow(t, 4);
			}
			
			T x(const Real& t) const
			{
				return c0 + c1 * t + c2 * ::std::pow(t, 2) + c3 * ::std::pow(t, 3) + c4 * ::std::pow(t, 4) + c5 * ::std::pow(t, 5);
			}
			
			T a0;
			
			T ae;
			
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
			
			T c4;
			
			T c5;
		};
	}
}

#endif // _RL_MATH_QUINTIC_H_
