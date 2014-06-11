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

#ifndef _RL_UTIL_THREAD_H_
#define _RL_UTIL_THREAD_H_

#ifdef WIN32
#include <process.h>
#include <windows.h>
#else // WIN32
#include <pthread.h>
#include <unistd.h>
#endif // WIN32

#include <cstdlib>

namespace rl
{
	namespace util
	{
		class Thread
		{
		public:
			explicit Thread() :
#ifdef WIN32
				id(0),
				thread(NULL)
#else // WIN32
				thread()
#endif // WIN32
			{
			}
			
			virtual ~Thread()
			{
#ifdef WIN32
				CloseHandle((*this).thread);
#endif // WIN32
			}
			
			bool operator==(const Thread& rhs) const
			{
#ifdef WIN32
				return (*this).id == rhs.id;
#else // WIN32
				return pthread_equal((*this).thread, rhs.thread);
#endif // WIN32
			}
			
			bool operator!=(const Thread& rhs) const
			{
				return !operator==(rhs);
			}
			
			void join()
			{
#ifdef WIN32
				WaitForSingleObject((*this).thread, INFINITE);
#else // WIN32
				pthread_join((*this).thread, NULL);
#endif // WIN32
			}
			
			virtual void run() = 0;
			
			static void sleep(const double& seconds)
			{
#ifdef WIN32
				Sleep(static_cast< unsigned int >(seconds * 1000.0f));
#else // WIN32
				usleep(static_cast< ::std::size_t >(seconds * 1000.0f * 1000.0f));
#endif // WIN32
			}
			
			void start()
			{
#ifdef WIN32
				(*this).thread = reinterpret_cast< void* >(_beginthreadex(NULL, 0, Thread::start, this, 0, &(*this).id));
#else // WIN32
				pthread_create(&(*this).thread, NULL, Thread::start, this);
#endif // WIN32
			}
			
			static void yield()
			{
#ifdef WIN32
				Sleep(0);
#else // WIN32
				sched_yield();
#endif // WIN32
			}
			
		protected:
			
		private:
#ifdef WIN32
			static unsigned int __stdcall start(void* arg)
			{
				static_cast< Thread* >(arg)->run();
				return 0;
			}
			
			unsigned int id;
			
			void* thread;
#else // WIN32
			static void* start(void* arg)
			{
				static_cast< Thread* >(arg)->run();
				return NULL;
			}
			
			pthread_t thread;
#endif // WIN32
		};
	}
}

#endif // _RL_UTIL_THREAD_H_
