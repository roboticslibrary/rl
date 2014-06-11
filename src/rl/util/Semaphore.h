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

#ifndef _RL_UTIL_SEMAPHORE_H_
#define _RL_UTIL_SEMAPHORE_H_

#include <cstdlib>

#ifdef WIN32
#include <windows.h>
#else // WIN32
#include <semaphore.h>
#endif // WIN32

namespace rl
{
	namespace util
	{
		class Semaphore
		{
		public:
			explicit Semaphore(const ::std::size_t& n) :
#ifdef WIN32
				semaphore(CreateSemaphore(NULL, n, n, NULL))
#else // WIN32
				semaphore()
#endif // WIN32
			{
#ifndef WIN32
				sem_init(&(*this).semaphore, 0, n);
#endif // WIN32
			}
			
			virtual ~Semaphore()
			{
#ifdef WIN32
				CloseHandle((*this).semaphore);
#else // WIN32
				sem_destroy(&(*this).semaphore);
#endif // WIN32
			}
			
			Semaphore& operator++()
			{
				(*this).release();
				return (*this);
			}
			
			Semaphore& operator--()
			{
				(*this).acquire();
				return (*this);
			}
			
			void acquire()
			{
#ifdef WIN32
				WaitForSingleObject((*this).semaphore, INFINITE);
#else // WIN32
				sem_wait(&(*this).semaphore);
#endif // WIN32
			}
			
			void release()
			{
#ifdef WIN32
				ReleaseSemaphore((*this).semaphore, 1, NULL);
#else // WIN32
				sem_post(&(*this).semaphore);
#endif // WIN32
			}
			
			bool tryAcquire()
			{
#ifdef WIN32
				return WaitForSingleObject((*this).semaphore, 0) == WAIT_OBJECT_0 ? false : true;
#else // WIN32
				return sem_trywait(&(*this).semaphore) == 0 ? true : false;
#endif // WIN32
			}
			
		protected:
			
		private:
#ifdef WIN32
			HANDLE semaphore;
#else // WIN32
			sem_t semaphore;
#endif // WIN32
		};
	}
}

#endif // _RL_UTIL_SEMAPHORE_H_
