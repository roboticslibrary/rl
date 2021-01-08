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

#ifndef RL_STD_MEMORY_H
#define RL_STD_MEMORY_H

#include <memory>

#ifndef __cpp_lib_make_unique
#include <type_traits>
#endif

namespace rl
{
	namespace std14
	{
#ifdef __cpp_lib_make_unique
		using ::std::make_unique;
#else
		template<typename T>
		typename ::std::enable_if<::std::is_array<T>::value, ::std::unique_ptr<T>>::type
		make_unique(::std::size_t n)
		{
			return ::std::unique_ptr<T>(new typename ::std::remove_extent<T>::type[n]());
		}
		
		template<typename T, typename... Args>
		typename ::std::enable_if<!::std::is_array<T>::value, ::std::unique_ptr<T>>::type
		make_unique(Args&&... args)
		{
			return ::std::unique_ptr<T>(new T(::std::forward<Args>(args)...));
		}
		
		template<typename T, typename... Args>
		typename ::std::enable_if<::std::extent<T>::value != 0, ::std::unique_ptr<T>>::type
		make_unique(Args&&...) = delete;
#endif
	}
}

#endif // RL_STD_MEMORY_H
