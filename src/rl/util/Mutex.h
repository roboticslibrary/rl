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

#ifndef _RL_UTIL_MUTEX_H_
#define _RL_UTIL_MUTEX_H_

#ifdef WIN32
#include <windows.h>
#else // WIN32
#include <errno.h>
#include <pthread.h>
#endif // WIN32

namespace rl
{
	namespace util
	{
		class Mutex
		{
		public:
			explicit Mutex() :
				mutex()
			{
#ifdef WIN32
				InitializeCriticalSection(&(*this).mutex);
#else // WIN32
				pthread_mutex_init(&(*this).mutex, NULL);
#endif // WIN32
			}
			
			virtual ~Mutex()
			{
#ifdef WIN32
				DeleteCriticalSection(&(*this).mutex);
#else // WIN32
				pthread_mutex_destroy(&(*this).mutex);
#endif // WIN32
			}
			
			Mutex& operator++()
			{
				(*this).unlock();
				return (*this);
			}
			
			Mutex& operator--()
			{
				(*this).lock();
				return (*this);
			}
			
			void lock()
			{
#ifdef WIN32
				EnterCriticalSection(&(*this).mutex);
#else // WIN32
				pthread_mutex_lock(&(*this).mutex);
#endif // WIN32
			}
			
			bool tryLock()
			{
#ifdef WIN32
				return TryEnterCriticalSection(&(*this).mutex) ? true : false; 
#else // WIN32
				return pthread_mutex_trylock(&(*this).mutex) == EBUSY ? false : true;
#endif // WIN32
			}
			
			void unlock()
			{
#ifdef WIN32
				LeaveCriticalSection(&(*this).mutex);
#else // WIN32
				pthread_mutex_unlock(&(*this).mutex);
#endif // WIN32
			}
			
		protected:
			
		private:
#ifdef WIN32
			CRITICAL_SECTION mutex;
#else // WIN32
			pthread_mutex_t mutex;
#endif // WIN32
		};
	}
}

#endif // _RL_UTIL_MUTEX_H_
