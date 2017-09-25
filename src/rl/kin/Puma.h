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

#ifndef RL_KIN_PUMA_H
#define RL_KIN_PUMA_H

#include "Kinematics.h"

namespace rl
{
	namespace kin
	{
		/**
		 * TRR base and TRT wrist.
		 */
		class Puma : public Kinematics
		{
		public:
			enum Arm
			{
				ARM_LEFT = -1,
				ARM_RIGHT = 1
			};
			
			enum Elbow
			{
				ELBOW_ABOVE = 1,
				ELBOW_BELOW = -1
			};
			
			enum Wrist
			{
				WRIST_FLIP = 1,
				WRIST_NONFLIP = -1
			};
			
			Puma();
			
			virtual ~Puma();
			
			virtual Kinematics* clone() const;
			
			Arm getArm() const;
			
			Elbow getElbow() const;
			
			Wrist getWrist() const;
			
			bool inversePosition(
				const ::rl::math::Transform& x,
				::rl::math::Vector& q,
				const ::std::size_t& leaf = 0,
				const ::rl::math::Real& delta = ::std::numeric_limits< ::rl::math::Real>::infinity(),
				const ::rl::math::Real& epsilon = 1.0e-3f,
				const ::std::size_t& iterations = 1000
			);
			
			bool isSingular() const;
			
			void parameters(const ::rl::math::Vector& q, Arm& arm, Elbow& elbow, Wrist& wrist) const;
			
			void setArm(const Arm& arm);
			
			void setElbow(const Elbow& elbow);
			
			void setWrist(const Wrist& wrist);
			
		protected:
			
		private:
			template<typename T> T atan2(const T& y, const T& x) const
			{
				T a = ::std::atan2(y, x);
				return (::std::abs(a) <= 1.0e-6f) ? 0.0f : a;
			}
			
			template<typename T> T cos(const T& x) const
			{
				T c = ::std::cos(x);
				return (::std::abs(c) <= 1.0e-6f) ? 0.0f : c;
			}
			
			template<typename T> T sin(const T& x) const
			{
				T s = ::std::sin(x);
				return (::std::abs(s) <= 1.0e-6f) ? 0.0f : s;
			}
			
			Arm arm;
			
			Elbow elbow;
			
			Wrist wrist;
		};
	}
}

#endif // RL_KIN_PUMA_H
	
