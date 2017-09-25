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

#ifndef RL_UTIL_XENOMAI_CHRONO_H
#define RL_UTIL_XENOMAI_CHRONO_H

#include <chrono>
#include <native/timer.h>

namespace rl
{
	namespace util
	{
		namespace xenomai
		{
			namespace chrono
			{
				struct steady_clock
				{
					typedef ::std::chrono::nanoseconds duration;
					
					typedef duration::period period;
					
					typedef duration::rep rep;
					
					typedef ::std::chrono::time_point<steady_clock, duration> time_point;
					
					static constexpr bool is_monotonic = true;
					
					static constexpr bool is_steady = true;
					
					static time_point now()
					{
						return time_point(duration(::rt_timer_tsc2ns(::rt_timer_tsc())));
					}
				};
				
				struct system_clock
				{
					typedef ::std::chrono::nanoseconds duration;
					
					typedef duration::period period;
					
					typedef duration::rep rep;
					
					typedef ::std::chrono::time_point<system_clock, duration> time_point;
					
					static constexpr bool is_monotonic = false;
					
					static constexpr bool is_steady = false;
					
					static time_point now()
					{
						return time_point(duration(::rt_timer_ticks2ns(::rt_timer_read())));
					}
					
					static ::std::time_t to_time_t(const time_point& t)
					{
						return ::std::time_t(
							::std::chrono::duration_cast< ::std::chrono::seconds>(
								t.time_since_epoch()
							).count()
						);
					}
					
					static time_point from_time_t(std::time_t t)
					{
						return ::std::chrono::time_point_cast<system_clock::duration>(
							::std::chrono::time_point<system_clock, ::std::chrono::seconds>(
								::std::chrono::seconds(t)
							)
						);
					}
				};
			}
		}
	}
}

#endif // RL_UTIL_XENOMAI_CHRONO_H
