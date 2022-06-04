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

#include "Spherical.h"

namespace rl
{
	namespace mdl
	{
		Spherical::Spherical() :
			Joint(4, 3)
		{
			this->max.setConstant(1); // TODO
			this->min.setConstant(-1); // TODO
			this->qUnits(0) = ::rl::math::Units::none;
			this->qUnits(1) = ::rl::math::Units::none;
			this->qUnits(2) = ::rl::math::Units::none;
			this->qUnits(3) = ::rl::math::Units::none;
			this->qdUnits(0) = ::rl::math::Units::radianPerSecond;
			this->qdUnits(1) = ::rl::math::Units::radianPerSecond;
			this->qdUnits(2) = ::rl::math::Units::radianPerSecond;
			this->qddUnits(0) = ::rl::math::Units::radianPerSecondSquared;
			this->qddUnits(1) = ::rl::math::Units::radianPerSecondSquared;
			this->qddUnits(2) = ::rl::math::Units::radianPerSecondSquared;
			this->S(0, 0) = 1;
			this->S(1, 1) = 1;
			this->S(2, 2) = 1;
			this->speedUnits(0) = ::rl::math::Units::radianPerSecond;
			this->speedUnits(1) = ::rl::math::Units::radianPerSecond;
			this->speedUnits(2) = ::rl::math::Units::radianPerSecond;
			this->tauUnits(0) = ::rl::math::Units::newtonMeter;
			this->tauUnits(1) = ::rl::math::Units::newtonMeter;
			this->tauUnits(2) = ::rl::math::Units::newtonMeter;
		}
		
		Spherical::~Spherical()
		{
		}
		
		void
		Spherical::clamp(::rl::math::VectorRef q) const
		{
			::Eigen::Map<::rl::math::Quaternion>(q.data()).normalize();
		}
		
		::rl::math::Real
		Spherical::distance(const ::rl::math::ConstVectorRef& q1, const ::rl::math::ConstVectorRef& q2) const
		{
			::Eigen::Map<const ::rl::math::Quaternion> quaternion1(q1.data());
			::Eigen::Map<const ::rl::math::Quaternion> quaternion2(q2.data());
			return quaternion1.angularDistance(quaternion2);
		}
		
		void
		Spherical::generatePositionGaussian(const ::rl::math::ConstVectorRef& rand, const ::rl::math::ConstVectorRef& mean, const ::rl::math::ConstVectorRef& sigma, ::rl::math::VectorRef q) const
		{
			q = ::rl::math::Quaternion::Random(rand, ::Eigen::Map<const ::rl::math::Quaternion>(mean.data()), sigma).coeffs();
		}
		
		void
		Spherical::generatePositionUniform(const ::rl::math::ConstVectorRef& rand, ::rl::math::VectorRef q) const
		{
			q = ::rl::math::Quaternion::Random(rand).coeffs();
		}
		
		void
		Spherical::generatePositionUniform(const ::rl::math::ConstVectorRef& rand, const ::rl::math::ConstVectorRef& min, const ::rl::math::ConstVectorRef& max, ::rl::math::VectorRef q) const
		{
			q = ::rl::math::Quaternion::Random(rand).coeffs();
		}
		
		void
		Spherical::interpolate(const ::rl::math::ConstVectorRef& q1, const ::rl::math::ConstVectorRef& q2, const ::rl::math::Real& alpha, ::rl::math::VectorRef q) const
		{
			::Eigen::Map<const ::rl::math::Quaternion> quaternion1(q1.data());
			::Eigen::Map<const ::rl::math::Quaternion> quaternion2(q2.data());
			q = quaternion1.slerp(alpha, quaternion2).coeffs();
		}
		
		bool
		Spherical::isValid(const ::rl::math::ConstVectorRef& q) const
		{
			return ::Eigen::internal::isApprox(q.norm(), static_cast<::rl::math::Real>(1), static_cast<::rl::math::Real>(1.0e-3));
		}
		
		void
		Spherical::normalize(::rl::math::VectorRef q) const
		{
			::Eigen::Map<::rl::math::Quaternion> quaternion(q.data());
			
			if (quaternion.squaredNorm() > 0)
			{
				quaternion.normalize();
			}
			else
			{
				quaternion.setIdentity();
			}
		}
		
		void
		Spherical::setPosition(const ::rl::math::ConstVectorRef& q)
		{
			this->q = q;
			this->x.linear() = ::Eigen::Map<const ::rl::math::Quaternion>(this->q.data()).toRotationMatrix();
		}
		
		void
		Spherical::step(const ::rl::math::ConstVectorRef& q1, const ::rl::math::ConstVectorRef& dq, ::rl::math::VectorRef q2) const
		{
			::Eigen::Map<const ::rl::math::Quaternion> quaternion1(q1.data());
			q2 = (quaternion1 * ::rl::math::AngleAxis(dq.norm(), dq.normalized())).coeffs();
		}
		
		::rl::math::Real
		Spherical::transformedDistance(const ::rl::math::ConstVectorRef& q1, const ::rl::math::ConstVectorRef& q2) const
		{
			return ::std::pow(this->distance(q1, q2), 2);
		}
	}
}
