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

#ifndef RL_UTIL_XENOMAI_CONDITION_VARIABLE_H
#define RL_UTIL_XENOMAI_CONDITION_VARIABLE_H

#include <chrono>
#include <memory>
#include <mutex>
#include <native/cond.h>

#include "chrono.h"
#include "mutex.h"

namespace rl
{
	namespace util
	{
		namespace xenomai
		{
			typedef recursive_mutex mutex; // TODO
			
			class condition_variable
			{
			public:
				typedef RT_COND native_type;
				
				typedef native_type* native_handle_type;
				
				condition_variable() :
					M_cond()
				{
					int e = ::rt_cond_create(&this->M_cond, nullptr);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				~condition_variable()
				{
					int e = ::rt_cond_delete(&this->M_cond);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				condition_variable(const condition_variable&) = delete;
				
				condition_variable& operator=(const condition_variable&) = delete;
				
				native_handle_type native_handle()
				{
					return &this->M_cond;
				}
				
				void notify_one()
				{
					int e = ::rt_cond_signal(&this->M_cond);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				void notify_all()
				{
					int e = ::rt_cond_broadcast(&this->M_cond);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				void wait(::std::unique_lock<mutex>& lock)
				{
					int e = ::rt_cond_wait(&this->M_cond, lock.mutex()->native_handle(), TM_INFINITE);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				template<typename Predicate>
				void wait(::std::unique_lock<mutex>& lock, Predicate p)
				{
					while (!p())
					{
						this->wait(lock);
					}
				}
				
				template<typename Duration>
				::std::cv_status wait_until(::std::unique_lock<mutex>& lock, const ::std::chrono::time_point<clock_t, Duration>& atime)
				{
					::std::chrono::time_point<clock_t, ::std::chrono::nanoseconds> ns = ::std::chrono::time_point_cast< ::std::chrono::nanoseconds>(atime);
					int e = ::rt_cond_wait_until(&this->M_cond, lock.mutex()->native_handle(), ::rt_timer_ns2ticks(ns.time_since_epoch().count()));
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
					
					return (clock_t::now() < atime ? ::std::cv_status::no_timeout : ::std::cv_status::timeout);
				}
				
				template<typename Clock, typename Duration>
				::std::cv_status wait_until(::std::unique_lock<mutex>& lock, const ::std::chrono::time_point<Clock, Duration>& atime)
				{
					const typename Clock::time_point c_entry = Clock::now();
					const clock_t::time_point s_entry = clock_t::now();
					const ::std::chrono::nanoseconds delta = atime - c_entry;
					const clock_t::time_point s_atime = s_entry + delta;
					return this->wait_until(lock, s_atime);
				}
				
				template<typename Clock, typename Duration, typename Predicate>
				bool wait_until(::std::unique_lock<mutex>& lock, const ::std::chrono::time_point<Clock, Duration>& atime, Predicate p)
				{
					while (!p())
					{
						if (this->wait_until(lock, atime) == ::std::cv_status::timeout)
						{
							return p();
						}
					}
					
					return true;
				}
				
				template<typename Rep, typename Period>
				::std::cv_status wait_for(::std::unique_lock<mutex>& lock, const ::std::chrono::duration<Rep, Period>& rtime)
				{
					return this->wait_until(lock, clock_t::now() + rtime);
				}
				
				template<typename Rep, typename Period, typename Predicate>
				bool wait_for(::std::unique_lock<mutex>& lock, const ::std::chrono::duration<Rep, Period>& rtime, Predicate p)
				{
					return this->wait_until(lock, clock_t::now() + rtime, ::std::move(p));
				}
				
			private:
				typedef chrono::system_clock clock_t;
				
				native_type M_cond;
			};
			
			class condition_variable_any
			{
			public:
				typedef condition_variable::native_handle_type native_handle_type;
				
				condition_variable_any() :
					M_cond(),
					M_mutex(std::make_shared<mutex>())
				{
				}
				
				~condition_variable_any()
				{
				}
				
				condition_variable_any(const condition_variable_any&) = delete;
				
				condition_variable_any& operator=(const condition_variable_any&) = delete;
				
				native_handle_type native_handle()
				{
					return this->M_cond.native_handle();
				}
				
				void notify_one()
				{
					::std::lock_guard<mutex> lock(*this->M_mutex);
					this->M_cond.notify_one();
				}
				
				void notify_all()
				{
					::std::lock_guard<mutex> lock(*this->M_mutex);
					this->M_cond.notify_all();
				}
				
				template<typename Lock>
				void wait(Lock& lock)
				{
					::std::shared_ptr<mutex> my_mutex = this->M_mutex;
					::std::unique_lock<mutex> my_lock(*my_mutex);
					Unlock<Lock> unlock(lock);
					::std::unique_lock<mutex> my_lock2(::std::move(my_lock));
					this->M_cond.wait(my_lock2);
				}
				
				
				template<typename Lock, typename Predicate>
				void wait(Lock& lock, Predicate p)
				{
					while (!p())
					{
						this->wait(lock);
					}
				}
				
				template<typename Lock, typename Clock, typename Duration>
				::std::cv_status wait_until(Lock& lock, const ::std::chrono::time_point<Clock, Duration>& atime)
				{
					::std::shared_ptr<mutex> my_mutex = this->M_mutex;
					::std::unique_lock<mutex> my_lock(*my_mutex);
					Unlock<Lock> unlock(lock);
					::std::unique_lock<mutex> my_lock2(::std::move(my_lock));
					return this->M_cond.wait_until(my_lock2, atime);
				}
				
				template<typename Lock, typename Clock, typename Duration, typename Predicate>
				bool wait_until(Lock& lock, const ::std::chrono::time_point<Clock, Duration>& atime, Predicate p)
				{
					while (!p())
					{
						if (this->wait_until(lock, atime) == ::std::cv_status::timeout)
						{
							return p();
						}
					}
					
					return true;
				}
				
				template<typename Lock, typename Rep, typename Period>
				::std::cv_status wait_for(Lock& lock, const ::std::chrono::duration<Rep, Period>& rtime)
				{
					return this->wait_until(lock, clock_t::now() + rtime);
				}
				
				template<typename Lock, typename Rep, typename Period, typename Predicate>
				bool wait_for(Lock& lock, const ::std::chrono::duration<Rep, Period>& rtime, Predicate p)
				{
					return this->wait_until(lock, clock_t::now() + rtime, ::std::move(p));
				}
				
			protected:
				
			private:
				typedef chrono::system_clock clock_t; // TODO
				
				template<typename Lock>
				struct Unlock
				{
					explicit Unlock(Lock& lk) :
						M_lock(lk)
					{
						this->lk.unlock();
					}
					
					Unlock(const Unlock&) = delete;
					
					Unlock& operator=(const Unlock&) = delete;
					
					~Unlock() noexcept(false)
					{
						if (::std::uncaught_exception())
						{
							try
							{
								this->M_lock.lock();
							}
							catch (...)
							{
							}
						}
						else
						{
							this->M_lock.lock();
						}
					}
					
					Lock& M_lock;
				};
				
				condition_variable M_cond;
				
				::std::shared_ptr<mutex> M_mutex;
			};
		}
	}
}

#endif // RL_UTIL_XENOMAI_CONDITION_VARIABLE_H
