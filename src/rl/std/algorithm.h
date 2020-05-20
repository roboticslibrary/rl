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

#ifndef RL_STD_ALGORITHM_H
#define RL_STD_ALGORITHM_H

#include <algorithm>

#ifndef __cpp_lib_clamp
#include <cassert>
#include <functional>
#endif

namespace rl
{
	namespace std17
	{
#ifdef __cpp_lib_clamp
		using ::std::clamp;
#else
		template<typename T, typename Compare>
		inline const T& clamp(const T& v, const T& lo, const T& hi, Compare comp)
		{
			assert(!comp(hi, lo));
			return comp(v, lo) ? lo : comp(hi, v) ? hi : v;
		}
		
		template<typename T>
		inline const T& clamp(const T& v, const T& lo, const T& hi)
		{
			return clamp(v, lo, hi, ::std::less<T>());
		}
#endif
	}
}

#endif // RL_MATH_ALGORITHM_H
