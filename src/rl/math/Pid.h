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

#ifndef RL_MATH_PID_H
#define RL_MATH_PID_H

#include "Real.h"

namespace rl
{
	namespace math
	{
		/**
		 * Proportional-Integral-Derivative controller.
		 */
		template<typename T>
		class Pid
		{
		public:
			Pid() :
				e(0),
				kd(0),
				ki(0),
				kp(0),
				i(0)
			{
				
			}
			
			virtual ~Pid()
			{
				
			}
			
			/**
			 * Calculate next step.
			 * 
			 * \f[ k_{\mathrm{p}} \, e(t) + k_{\mathrm{i}} \int_{0}^{t} e(\tau) \, \mathrm{d}\tau + k_{\mathrm{d}} \, \frac{\mathrm{d}}{\mathrm{d}t} \, e(t) \f]
			 * 
			 * @param[in] dt \f$\mathrm{d}t\f$
			 */
			T operator()(const T& x, const Real& dt)
			{
				T e = this->x - x;
				T p = this->kp * e;
				this->i += this->ki * e * dt;
				T d = this->kd * (e - this->e) / dt;
				this->e = e;
				return x + p + this->i + d;
			}
			
			void reset()
			{
				this->e = 0;
				this->i = 0;
			}
			
			/**
			 * Derivative gain.
			 * \f[ k_{\mathrm{d}} \f]
			 * */
			T kd;
			
			/**
			 * Integral gain.
			 * \f[ k_{\mathrm{i}} \f]
			 * */
			T ki;
			
			/**
			 * Proportional gain.
			 * \f[ k_{\mathrm{p}} \f]
			 * */
			T kp;
			
			/** Setpoint. */
			T x;
			
		protected:
			
		private:
			/** Previous error. */
			T e;
			
			/**
			 * Integral output.
			 * \f[ k_{\mathrm{i}} \int_{0}^{t} e(\tau) \, \mathrm{d}\tau \f]
			 * */
			T i;
		};
	}
}

#endif // RL_MATH_PID_H
