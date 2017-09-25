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

#ifndef RL_UTIL_THREAD_H
#define RL_UTIL_THREAD_H

#ifdef WIN32
#include <windows.h>
#else // WIN32
#include <pthread.h>
#endif // WIN32

#include <system_error>

namespace rl
{
	namespace util
	{
		namespace this_thread
		{
			inline int get_priority()
			{
#ifdef WIN32
				return ::GetThreadPriority(::GetCurrentThread());
#else // WIN32
				int policy;
				::sched_param param;
				
				if (-1 == ::pthread_getschedparam(::pthread_self(), &policy, &param))
				{
					throw ::std::system_error(::std::error_code(errno, ::std::generic_category()));
				}
				
				return param.sched_priority;
#endif // WIN32
			}
			
			inline int get_priority_max()
			{
#ifdef WIN32
				return THREAD_PRIORITY_TIME_CRITICAL;
#else // WIN32
				return ::sched_get_priority_max(SCHED_FIFO);
#endif // WIN32
			}
			
			inline int get_priority_min()
			{
#ifdef WIN32
				return THREAD_PRIORITY_IDLE;
#else // WIN32
				return ::sched_get_priority_min(SCHED_FIFO);
#endif // WIN32
			}
			
			inline void set_priority(const int& priority)
			{
#ifdef WIN32
				if (!::SetThreadPriority(::GetCurrentThread(), priority))
				{
					throw ::std::system_error(::std::error_code(::GetLastError(), ::std::generic_category()));
				}
#else // WIN32
				::sched_param param;
				param.sched_priority = priority;
				
				if (-1 == ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &param))
				{
					throw ::std::system_error(::std::error_code(errno, ::std::generic_category()));
				}
#endif // WIN32
			}
		}
	}
}

#endif // RL_UTIL_THREAD_H
