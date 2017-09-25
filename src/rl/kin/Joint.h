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

#ifndef RL_KIN_JOINT_H
#define RL_KIN_JOINT_H

#include <unordered_set>
#include <rl/math/Unit.h>

#include "Transform.h"

namespace rl
{
	namespace kin
	{
		class Joint : public Transform
		{
		public:
			Joint();
			
			virtual ~Joint();
			
			::rl::math::Real getPosition() const;
			
			virtual ::rl::math::Unit getPositionUnit() const = 0;
			
			virtual ::rl::math::Unit getSpeedUnit() const = 0;
			
			virtual void jacobian(const ::rl::math::Transform& tcp, ::rl::math::MatrixBlock& j) = 0;
			
			virtual void setPosition(const ::rl::math::Real& q);
			
			::rl::math::Real a;
			
			::rl::math::Real alpha;
			
			::rl::math::Real d;
			
			::std::unordered_set< ::std::size_t> leaves;
			
			::rl::math::Real max;
			
			::rl::math::Real min;
			
			::rl::math::Real offset;
			
			::rl::math::Real speed;
			
			::rl::math::Real theta;
			
			bool wraparound;
			
		protected:
			::rl::math::Real q;
			
		private:
			
		};
	}
}

#endif // RL_KIN_JOINT_H
