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

#include <rl/math/Vector.h>

#include "Link.h"
#include "Revolute.h"

namespace rl
{
	namespace kin
	{
		Revolute::Revolute() :
			Joint()
		{
		}
		
		Revolute::~Revolute()
		{
		}
		
		::rl::math::Unit
		 Revolute::getPositionUnit() const
		{
			return ::rl::math::UNIT_RADIAN;
		}
		
		::rl::math::Unit
		 Revolute::getSpeedUnit() const
		{
			return ::rl::math::UNIT_RADIAN_PER_SECOND;
		}
		
		void
		Revolute::jacobian(const ::rl::math::Transform& tcp, ::rl::math::MatrixBlock& j)
		{
			::rl::math::Vector3 p;
			::rl::math::Vector3 z;
			::rl::math::Vector3 zxp;
			
			z(0) = this->in->frame(0, 2);
			z(1) = this->in->frame(1, 2);
			z(2) = this->in->frame(2, 2);
			p(0) = tcp(0, 3) - this->in->frame(0, 3);
			p(1) = tcp(1, 3) - this->in->frame(1, 3);
			p(2) = tcp(2, 3) - this->in->frame(2, 3);
			zxp = z.cross(p);
			j(0, 0) = zxp(0);
			j(1, 0) = zxp(1);
			j(2, 0) = zxp(2);
			j(3, 0) = z(0);
			j(4, 0) = z(1);
			j(5, 0) = z(2);
		}
		
		void
		Revolute::setPosition(const ::rl::math::Real& q)
		{
			this->transform.fromDenavitHartenbergPaul(
				this->d,
				this->theta + q + this->offset,
				this->a,
				this->alpha
			);
			
			Joint::setPosition(q);
		}
	}
}
