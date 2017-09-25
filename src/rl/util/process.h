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

#ifndef RL_UTIL_PROCESS_H
#define RL_UTIL_PROCESS_H

#ifdef WIN32
#include <windows.h>
#else // WIN32
#include <cerrno>
#include <unistd.h>
#include <sys/mman.h>
#endif // WIN32

#include <iostream>
#include <system_error>

namespace rl
{
	namespace util
	{
		namespace this_process
		{
			inline int get_priority_max()
			{
#ifdef WIN32
				return REALTIME_PRIORITY_CLASS;
#else // WIN32
				return ::sched_get_priority_max(SCHED_FIFO);
#endif // WIN32
			}
			
			inline int get_priority_min()
			{
#ifdef WIN32
				return IDLE_PRIORITY_CLASS;
#else // WIN32
				return ::sched_get_priority_min(SCHED_FIFO);
#endif // WIN32
			}
			
			inline void memory_lock_all()
			{
#if !defined(WIN32) && !defined(__APPLE__)
				if (-1 == ::mlockall(MCL_CURRENT | MCL_FUTURE))
				{
					throw ::std::system_error(::std::error_code(errno, ::std::generic_category()));
				}
#endif // !WIN32 && !__APPLE__
			}
			
			inline void memory_reserve(const ::std::size_t& size)
			{
#ifdef WIN32
#else // WIN32
				char* buffer = new char[size];
				
				for (::std::size_t i = 0; i < size; i += ::sysconf(_SC_PAGESIZE))
				{
					buffer[i] = 0;
				}
				
				delete[] buffer;
#endif // WIN32
			}
			
			inline void memory_unlock_all()
			{
#if !defined(WIN32) && !defined(__APPLE__)
				if (-1 == ::munlockall())
				{
					throw ::std::system_error(::std::error_code(errno, ::std::generic_category()));
				}
#endif // !WIN32 && !__APPLE__
			}
			
			inline void set_priority(const int& priority)
			{
#ifdef WIN32
				if (!::SetPriorityClass(::GetCurrentProcess(), priority))
				{
					throw ::std::system_error(::std::error_code(::GetLastError(), ::std::generic_category()));
				}
#else // WIN32
				::sched_param param;
				param.sched_priority = priority;
				
#ifdef __APPLE__
#warning "rl/util/process.h: set_priority not implemented for Mac OS"
				//FIXME https://developer.apple.com/library/mac/documentation/Darwin/Conceptual/KernelProgramming/scheduler/scheduler.html
				::std::cerr << "set_priority not implemented for Mac OS" << ::std::endl;
#else // __APPLE__
				if (-1 == ::sched_setscheduler(::getpid(), SCHED_FIFO, &param))
				{
					throw ::std::system_error(::std::error_code(errno, ::std::generic_category()));
				}
#endif // __APPLE__
#endif // WIN32
			}
		}
	}
}

#endif // RL_UTIL_PROCESS_H
