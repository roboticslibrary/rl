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

#ifndef RL_UTIL_XENOMAI_MUTEX_H
#define RL_UTIL_XENOMAI_MUTEX_H

#include <chrono>
#include <system_error>
#include <native/mutex.h>

#include "chrono.h"

namespace rl
{
	namespace util
	{
		namespace xenomai
		{
			class recursive_mutex_base
			{
			public:
				
			protected:
				typedef RT_MUTEX native_type;
				
				recursive_mutex_base() :
					M_mutex()
				{
					int e = ::rt_mutex_create(&this->M_mutex, nullptr);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				recursive_mutex_base(const recursive_mutex_base&) = delete;
				
				recursive_mutex_base& operator=(const recursive_mutex_base&) = delete;
				
				~recursive_mutex_base()
				{
					int e = ::rt_mutex_delete(&this->M_mutex);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				native_type M_mutex;
				
			private:
				
			};
			
			class recursive_mutex : private recursive_mutex_base
			{
			public:
				typedef native_type* native_handle_type;
				
				recursive_mutex() = default;
				
				recursive_mutex(const recursive_mutex&) = delete;
				
				recursive_mutex& operator=(const recursive_mutex&) = delete;
				
				~recursive_mutex() = default;
				
				void lock()
				{
					int e = ::rt_mutex_acquire(&this->M_mutex, TM_INFINITE);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				native_handle_type native_handle()
				{
					return &this->M_mutex;
				}
				
				bool try_lock()
				{
					int e = ::rt_mutex_acquire(&this->M_mutex, TM_NONBLOCK);
					
					switch (e)
					{
					case 0:
						return true;
						break;
					case -EWOULDBLOCK:
						return false;
						break;
					default:
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
						break;
					}
				}
				
				void unlock()
				{
					int e = ::rt_mutex_release(&this->M_mutex);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
			protected:
				
			private:
				
			};
			
			template<typename Derived>
			class timed_mutex_impl
			{
			public:
				
			protected:
				typedef chrono::system_clock clock_t;
				
				template<typename Rep, typename Period>
				bool M_try_lock_for(const ::std::chrono::duration< Rep, Period>& rtime)
				{
					clock_t::duration rt = ::std::chrono::duration_cast<clock_t::duration>(rtime);
					
					if (::std::ratio_greater<clock_t::period, Period>())
					{
						++rt;
					}
					
					::std::chrono::nanoseconds ns = ::std::chrono::duration_cast< ::std::chrono::nanoseconds>(rt);
					int e = ::rt_mutex_acquire(static_cast<Derived*>(this)->native_handle(), ::rt_timer_ns2ticks(ns.count()));
					
					switch (e)
					{
					case 0:
						return true;
						break;
					case -ETIMEDOUT:
						return false;
						break;
					default:
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
						break;
					}
				}
				
				template<typename Duration>
				bool M_try_lock_until(const ::std::chrono::time_point<clock_t, Duration>& atime)
				{
					::std::chrono::time_point<clock_t, ::std::chrono::nanoseconds> ns = ::std::chrono::time_point_cast< ::std::chrono::nanoseconds>(atime);
					int e = ::rt_mutex_acquire_until(static_cast<Derived*>(this)->native_handle(), ::rt_timer_ns2ticks(ns.time_since_epoch().count()));
					
					switch (e)
					{
					case 0:
						return true;
						break;
					case -ETIMEDOUT:
						return false;
						break;
					default:
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
						break;
					}
				}
				
				template<typename Clock, typename Duration>
				bool M_try_lock_until(const ::std::chrono::time_point<Clock, Duration>& atime)
				{
					Duration rtime = atime - Clock::now();
					return this->M_try_lock_until(clock_t::now() + rtime);
				}
				
			private:
				
			};
			
			class recursive_timed_mutex : private recursive_mutex_base, public timed_mutex_impl<recursive_timed_mutex>
			{
			public:
				typedef native_type* native_handle_type;
				
				recursive_timed_mutex() = default;
				
				recursive_timed_mutex(const recursive_timed_mutex&) = delete;
				
				recursive_timed_mutex& operator=(const recursive_timed_mutex&) = delete;
				
				~recursive_timed_mutex() = default;
				
				void lock()
				{
					int e = ::rt_mutex_acquire(&this->M_mutex, TM_INFINITE);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				native_handle_type native_handle()
				{
					return &this->M_mutex;
				}
				
				bool try_lock()
				{
					int e = ::rt_mutex_acquire(&this->M_mutex, TM_NONBLOCK);
					
					switch (e)
					{
					case 0:
						return true;
						break;
					case -EWOULDBLOCK:
						return false;
						break;
					default:
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
						break;
					}
				}
				
				template <class Rep, class Period>
				bool try_lock_for(const ::std::chrono::duration<Rep, Period>& rtime)
				{
					return this->M_try_lock_for(rtime);
				}
				
				template <class Clock, class Duration>
				bool try_lock_until(const ::std::chrono::time_point<Clock, Duration>& atime)
				{
					return this->M_try_lock_until(atime);
				}
				
				void unlock()
				{
					int e = ::rt_mutex_release(&this->M_mutex);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
			protected:
				
			private:
				
			};
		}
	}
}

#endif // RL_UTIL_XENOMAI_MUTEX_H
