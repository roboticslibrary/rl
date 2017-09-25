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

#ifndef RL_UTIL_RTAI_THREAD
#define RL_UTIL_RTAI_THREAD

#include <chrono>
#include <memory>
#include <rtai_lxrt.h>
#include <system_error>

#include "chrono.h"

namespace rl
{
	namespace util
	{
		namespace rtai
		{
			class thread
			{
			public:
				typedef pthread_t native_handle_type;
				
				class id
				{
				public:
					id() :
						M_task(nullptr),
						M_thread()
					{
					}
					
					explicit id(native_handle_type id, RT_TASK* task) :
						M_task(task),
						M_thread(id)
					{
					}
					
				protected:
					
				private:
					friend class thread;
					
					friend class ::std::hash<thread::id>;
					
					friend bool operator==(thread::id x, thread::id y)
					{
						return ::pthread_equal(x.M_thread, y.M_thread);
					}
					
					friend bool operator!=(thread::id x, thread::id y)
					{
						return !(x == y);
					}
					
					friend bool operator<(thread::id x, thread::id y)
					{
						return x.M_thread < y.M_thread;
					}
					
					friend bool operator<=(thread::id x, thread::id y)
					{
						return !(y < x);
					}
					
					friend bool operator>(thread::id x, thread::id y)
					{
						return y < x;
					}
					
					friend bool operator>=(thread::id x, thread::id y)
					{
						return !(x < y);
					}
					
					template<class CharT, class Traits>
					friend ::std::basic_ostream<CharT, Traits>& operator<<(::std::basic_ostream<CharT, Traits>& out, thread::id id)
					{
						if (id == thread::id())
						{
							return out << "thread::id of a non-executing thread";
						}
						else
						{
							return out << id.M_thread;
						}
					}
					
					RT_TASK* M_task;
					
					native_handle_type M_thread;
				};
				
				thread() = default;
				
				thread(thread&) = delete;
				
				thread(const thread&) = delete;
				
				thread(thread&& other)
				{
					this->swap(other);
				}
				
				template<typename Callable, typename... Args>
				explicit thread(Callable&& f, Args&&... args)
				{
					this->start_thread(
						this->make_routine(
							::std::bind<void>(
								::std::forward<Callable>(f),
								::std::forward<Args>(args)...
							)
						)
					);
				}
				
				~thread()
				{
					if (this->joinable())
					{
						::std::terminate();
					}
				}
				
				void detach()
				{
					int e = EINVAL;
					
					if (this->M_id != id())
					{
						e = ::pthread_detach(this->M_id.M_thread);
					}
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(e, ::std::generic_category()));
					}
					
					this->M_id = id();
				}
				
				thread::id get_id() const
				{
					return this->M_id;
				}
				
				static unsigned int hardware_concurrency()
				{
					return 0;
				}
				
				void join()
				{
					int e = EINVAL;
					
					if (this->M_id != id())
					{
						e = ::pthread_join(this->M_id.M_thread, 0);
					}
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(e, ::std::generic_category()));
					}
					
					this->M_id = id();
				}
				
				bool joinable() const
				{
					return !(this->M_id == id());
				}
				
				native_handle_type native_handle()
				{
					return this->M_id.M_thread;
				}
				
				thread& operator=(const thread&) = delete;
				
				thread& operator=(thread&& t)
				{
					if (this->joinable())
					{
						::std::terminate();
					}
					
					this->swap(t);
					return *this;
				}
				
				void resume()
				{
					int e = ::rt_task_resume(this->M_id.M_task);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				template<typename Rep, typename Period>
				void set_periodic(const ::std::chrono::duration<Rep, Period>& period)
				{
					::std::chrono::nanoseconds period_ns = ::std::chrono::duration_cast< ::std::chrono::nanoseconds>(period);
					
					int e = ::rt_task_make_periodic(this->M_id.M_task, ::rt_get_time(), ::nano2count(period_ns.count()));
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				template<typename Duration, typename Rep, typename Period>
				void set_periodic(const ::std::chrono::time_point<chrono::system_clock, Duration>& idate, const ::std::chrono::duration<Rep, Period>& period)
				{
					::std::chrono::time_point<chrono::system_clock, ::std::chrono::nanoseconds> idate_ns = ::std::chrono::time_point_cast< ::std::chrono::nanoseconds>(idate);
					::std::chrono::nanoseconds period_ns = ::std::chrono::duration_cast< ::std::chrono::nanoseconds>(period);
					
					int e = ::rt_task_make_periodic(this->M_id.M_task, ::nano2count(idate_ns.time_since_epoch().count()), ::nano2count(period_ns.count()));
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				template<typename Clock, typename Duration, typename Rep, typename Period>
				void set_periodic(const ::std::chrono::time_point<Clock, Duration>& idate, const ::std::chrono::duration<Rep, Period>& period)
				{
					const typename Clock::time_point c_entry = Clock::now();
					const chrono::system_clock::time_point s_entry = chrono::system_clock::now();
					const ::std::chrono::nanoseconds delta = idate - c_entry;
					const chrono::system_clock::time_point s_idate = s_entry + delta;
					this->set_periodic(s_idate, period);
				}
				
				void set_priority(const int& prio)
				{
					int e = ::rt_change_prio(this->M_id.M_task, prio);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				void suspend()
				{
					int e = ::rt_task_suspend(this->M_id.M_task);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				void swap(thread& other)
				{
					using ::std::swap;
					swap(this->M_id, other.M_id);
				}
				
				friend void swap(thread& lhs, thread& rhs)
				{
					lhs.swap(rhs);
				}
				
			protected:
				
			private:
				struct Impl_base;
				
				typedef ::std::shared_ptr<Impl_base> shared_base_type;
				
				struct Impl_base
				{
					inline virtual ~Impl_base();
					
					virtual void run() = 0;
					
					id M_id;
					
					shared_base_type this_ptr;
				};
				
				template<typename Callable>
				struct Impl : public Impl_base
				{
					Impl(Callable&& f) :
						func(::std::forward<Callable>(f))
					{
					}
					
					void run()
					{
						this->func();
					}
					
					Callable func;
				};
				
				static void* execute_native_thread_routine(void* p)
				{
					thread::Impl_base* t = static_cast<thread::Impl_base*>(p);
					thread::shared_base_type local;
					local.swap(t->this_ptr);
					
					try
					{
						t->M_id.M_task = ::rt_task_init(0, 0, 0, 0);
						::rt_make_hard_real_time();
						t->run();
						::rt_make_soft_real_time();
						::rt_task_delete(t->M_id.M_task);
					}
					catch (...)
					{
						::std::terminate();
					}
					
					return 0;
				}
				
				template<typename Callable>
				::std::shared_ptr<Impl<Callable>> make_routine(Callable&& f)
				{
					return ::std::make_shared<Impl<Callable>>(::std::forward<Callable>(f));
				}
				
				void start_thread(shared_base_type b)
				{
					if (!__gthread_active_p())
					{
#if __EXCEPTIONS
						throw ::std::system_error(::std::make_error_code(::std::errc::operation_not_permitted), "Enable multithreading to use std::thread");
#else
						throw ::std::system_error(::std::error_code(::std::errc::operation_not_permitted, ::std::generic_category()));
#endif
					}
					
					b->M_id = this->M_id;
					b->this_ptr = b;
					int e = ::pthread_create(&this->M_id.M_thread, nullptr, &thread::execute_native_thread_routine, b.get());
					
					if (e)
					{
						b->this_ptr.reset();
						throw ::std::system_error(::std::error_code(e, ::std::generic_category()));
					}
				}
				
				id M_id;
			};
			
			inline thread::Impl_base::~Impl_base() = default;
			
			namespace this_thread
			{
				inline thread::id get_id()
				{
					return thread::id(::pthread_self(), ::rt_buddy());
				}
				
				inline int get_priority()
				{
					return ::rt_get_prio(::rt_buddy());
				}
				
				inline int get_priority_max()
				{
					return 0;
				}
				
				inline int get_priority_min()
				{
					return RT_SCHED_LOWEST_PRIORITY;
				}
				
				inline void set_priority(const int& priority)
				{
					int e = ::rt_change_prio(::rt_buddy(), priority);
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				template<typename Rep, typename Period>
				inline void sleep_for(const ::std::chrono::duration<Rep, Period>& rtime)
				{
					::std::chrono::nanoseconds ns = ::std::chrono::duration_cast< ::std::chrono::nanoseconds>(rtime);
					int e = ::rt_sleep(::nano2count(ns.count()));
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				template<typename Duration>
				inline void sleep_until(const ::std::chrono::time_point<chrono::system_clock, Duration>& atime)
				{
					::std::chrono::time_point<chrono::system_clock, ::std::chrono::nanoseconds> ns = ::std::chrono::time_point_cast< ::std::chrono::nanoseconds>(atime);
					int e = ::rt_sleep_until(::nano2count(ns.time_since_epoch().count()));
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				template<typename Clock, typename Duration>
				inline void sleep_until(const ::std::chrono::time_point<Clock, Duration>& atime)
				{
					sleep_for(atime - Clock::now());
				}
				
				inline void start_timer()
				{
					::rt_set_oneshot_mode();
					::start_rt_timer(0);
				}
				
				inline void stop_timer()
				{
					::stop_rt_timer();
				}
				
				inline void wait_period()
				{
					int e = ::rt_task_wait_period();
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
				
				inline void yield()
				{
					int e = ::rt_task_yield();
					
					if (e)
					{
						throw ::std::system_error(::std::error_code(-e, ::std::generic_category()));
					}
				}
			}
		}
	}
}

namespace std
{
	template<>
	struct hash< ::rl::util::rtai::thread::id>
	{
		size_t operator()(const ::rl::util::rtai::thread::id& id) const
		{
			return hash< ::rl::util::rtai::thread::native_handle_type>()(id.M_thread);
		}
	};
	
	inline void swap(::rl::util::rtai::thread& x, ::rl::util::rtai::thread& y)
	{
		x.swap(y);
	}
}

#endif // RL_UTIL_RTAI_THREAD
