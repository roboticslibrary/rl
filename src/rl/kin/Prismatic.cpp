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

#include "Link.h"
#include "Prismatic.h"

namespace rl
{
	namespace kin
	{
		Prismatic::Prismatic() :
			Joint()
		{
		}
		
		Prismatic::~Prismatic()
		{
		}
		
		::rl::math::Unit
		Prismatic::getPositionUnit() const
		{
			return ::rl::math::UNIT_METER;
		}
		
		::rl::math::Unit
		Prismatic::getSpeedUnit() const
		{
			return ::rl::math::UNIT_METER_PER_SECOND;
		}
		
		void
		Prismatic::jacobian(const ::rl::math::Transform& tcp, ::rl::math::MatrixBlock& j)
		{
			j(0, 0) = this->in->frame(0, 2);
			j(1, 0) = this->in->frame(1, 2);
			j(2, 0) = this->in->frame(2, 2);
			j(3, 0) = 0;
			j(4, 0) = 0;
			j(5, 0) = 0;
		}
		
		void
		Prismatic::setPosition(const ::rl::math::Real& q)
		{
			this->transform.fromDenavitHartenbergPaul(
				this->d + q + this->offset,
				this->theta,
				this->a,
				this->alpha
			);
			
			Joint::setPosition(q);
		}
	}
}
