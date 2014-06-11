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

#ifndef _RL_UTIL_TIMER_H_
#define _RL_UTIL_TIMER_H_

#include <ctime>

#ifdef WIN32
#include <windows.h>
#else // WIN32
#include <unistd.h>
#include <sys/time.h>
#endif // WIN32

#include <rl/math/Unit.h>

namespace rl
{
	namespace util
	{
		class Timer
		{
		public:
			explicit Timer() :
				begin(),
				end()
			{
#ifdef WIN32
				QueryPerformanceFrequency(&(*this).frequency);
#endif // WIN32
			}
			
			virtual ~Timer()
			{
			}
			
			/**
			 * @return elapsed time [s]
			 */
			double elapsed() const
			{
#ifdef WIN32
				return static_cast< double >((*this).end.QuadPart - (*this).begin.QuadPart) / static_cast< double >((*this).frequency.QuadPart); 
#else // WIN32
				return static_cast< double >((*this).end.tv_sec - (*this).begin.tv_sec) + static_cast< double >((*this).end.tv_usec - (*this).begin.tv_usec) / 1000000.0;
#endif // WIN32
			}
			
			/**
			 * @return current time [s]
			 */
			static double now()
			{
#ifdef WIN32
				LARGE_INTEGER now;
				QueryPerformanceCounter(&now);
				LARGE_INTEGER frequency;
				QueryPerformanceFrequency(&frequency);
				return static_cast< double >(now.QuadPart) / static_cast< double >(frequency.QuadPart); 
#else // WIN32
				timeval now;
				gettimeofday(&now, NULL);
				return static_cast< double >(now.tv_sec) + static_cast< double >(now.tv_usec) / 1000000.0;
#endif // WIN32
			}
			
			/**
			 * @param seconds sleep time [s]
			 */
			static void sleep(const double& seconds)
			{
#ifdef WIN32
				Sleep(static_cast< unsigned int >(seconds * ::rl::math::UNIT2MILLI));
#else // WIN32
				usleep(static_cast< ::std::size_t >(seconds * ::rl::math::UNIT2MICRO));
#endif // WIN32
			}
			
			void start()
			{
#ifdef WIN32
				QueryPerformanceCounter(&(*this).begin);
#else // WIN32
				gettimeofday(&(*this).begin, NULL);
#endif // WIN32
			}
			
			void stop()
			{
#ifdef WIN32
				QueryPerformanceCounter(&(*this).end);
#else // WIN32
				gettimeofday(&(*this).end, NULL);
#endif // WIN32
			}
			
		protected:
			
		private:
#ifdef WIN32
			LARGE_INTEGER begin;
			
			LARGE_INTEGER end;
			
			LARGE_INTEGER frequency;
#else // WIN32
			timeval begin;
			
			timeval end;
#endif // WIN32
		};
	}
}

#endif // _RL_UTIL_TIMER_H_
