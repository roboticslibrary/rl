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

#include <rl/math/Rotation.h>

#include "Helical.h"

namespace rl
{
	namespace mdl
	{
		Helical::Helical() :
			Joint(1, 1),
			h(1)
		{
			this->qUnits(0) = ::rl::math::Units::none; // TODO
			this->qdUnits(0) = ::rl::math::Units::none; // TODO
			this->qddUnits(0) = ::rl::math::Units::none; // TODO
			this->S(2, 0) = 1;
			this->S(5, 0) = this->h;
			this->speedUnits(0) = ::rl::math::Units::none; // TODO
			this->tauUnits(0) = ::rl::math::Units::none; // TODO
		}
		
		Helical::~Helical()
		{
		}
		
		::rl::math::Real
		Helical::getPitch() const
		{
			return this->h;
		}
		
		void
		Helical::setPitch(const ::rl::math::Real& h)
		{
			this->h = h;
			this->x.translation() = this->S.block<3, 1>(3, 0) * this->h * (this->q(0) + this->offset(0));
		}
		
		void
		Helical::setPosition(const ::rl::math::ConstVectorRef& q)
		{
			this->q = q;
			this->x.linear() = ::rl::math::AngleAxis(this->q(0), this->S.block<3, 1>(0, 0)).toRotationMatrix();
			this->x.translation() = this->S.block<3, 1>(3, 0) * this->h * (this->q(0) + this->offset(0));
		}
	}
}
