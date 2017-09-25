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

#include <cassert>
#include <rl/math/Rotation.h>

#include "Frame.h"
#include "Joint.h"
#include "Rhino.h"
#include "Transform.h"

namespace rl
{
	namespace kin
	{
		Rhino::Rhino() :
			Kinematics(),
			arm(ARM_LEFT),
			elbow(ELBOW_ABOVE)
		{
		}
		
		Rhino::~Rhino()
		{
		}
		
		Kinematics*
		Rhino::clone() const
		{
			return new Rhino(*this);
		}
		
		Rhino::Arm
		Rhino::getArm() const
		{
			return this->arm;
		}
		
		Rhino::Elbow
		Rhino::getElbow() const
		{
			return this->elbow;
		}
		
		bool
		Rhino::inversePosition(const ::rl::math::Transform& x, ::rl::math::Vector& q, const ::std::size_t& leaf, const ::rl::math::Real& delta, const ::rl::math::Real& epsilon, const ::std::size_t& iterations)
		{
			assert(q.size() == this->getDof());
			
			int arm = this->arm;
			int elbow = this->elbow;
			
			::rl::math::Real a1 = this->joints[0]->a;
			::rl::math::Real a2 = this->joints[1]->a;
			::rl::math::Real a3 = this->joints[2]->a;
			::rl::math::Real d1 = this->joints[0]->d;
			::rl::math::Real d5 = this->joints[4]->d;
			
			::rl::math::Real a2_2 = ::std::pow(a2, 2);
			::rl::math::Real a3_2 = ::std::pow(a3, 2);
			
			::rl::math::Transform hand = this->transforms[this->transforms.size() - 1]->transform.inverse(::Eigen::Isometry); // TODO
			::rl::math::Transform base = this->frames[1]->frame.inverse(::Eigen::Isometry); // TODO
			
			::rl::math::Transform x_hand = x * hand;
			::rl::math::Transform x_base = base * x_hand;
			
			const ::rl::math::Transform::TranslationPart& p = x_base.translation();
			const ::rl::math::Transform::LinearPart& rotation = x_base.linear();
			
			const ::rl::math::Transform::LinearPart::ConstColXpr& n = rotation.col(0);
			const ::rl::math::Transform::LinearPart::ConstColXpr& s = rotation.col(1);
			const ::rl::math::Transform::LinearPart::ConstColXpr& a = rotation.col(2);
			
			::rl::math::Vector3 p4 = p - d5 * a;
			
			::rl::math::Real px0 = p4(0);
			::rl::math::Real py0 = p4(1);
			::rl::math::Real pz0 = p4(2);
			
			// shoulder
			::rl::math::Real theta1 = this->atan2(-arm * py0, -arm * px0);
			
			::rl::math::Real c1 = this->cos(theta1);
			::rl::math::Real s1 = this->sin(theta1);
			
			px0 -= a1 * c1;
			py0 -= a1 * s1;
			pz0 -= d1;
			
			::rl::math::Real px0_2 = ::std::pow(px0, 2);
			::rl::math::Real py0_2 = ::std::pow(py0, 2);
			::rl::math::Real pz0_2 = ::std::pow(pz0, 2);
			
			::rl::math::Real K = static_cast< ::rl::math::Real>(arm * elbow);
			
			::rl::math::Real r_2 = px0_2 + py0_2;
			::rl::math::Real r = ::std::sqrt(r_2);
			
			::rl::math::Real Q_2 = r_2 + pz0_2;
			::rl::math::Real Q = ::std::sqrt(Q_2);
			
			if (::std::abs(Q) <= ::std::numeric_limits< ::rl::math::Real>::epsilon())
			{
				return false;
			}
			
			::rl::math::Real sinAlpha = -pz0 / Q;
			::rl::math::Real cosAlpha = -r * arm / Q;
			
			::rl::math::Real cosBeta = (Q_2 + a2_2 - a3_2) / (2.0f * a2 * Q);
			
			if (::std::abs(cosBeta) > 1.0f)
			{
				return false;
			}
			
			::rl::math::Real sinBeta = ::std::sqrt(1.0f - ::std::pow(cosBeta, 2));
			
			::rl::math::Real s2 = sinAlpha * cosBeta + K * cosAlpha * sinBeta;
			::rl::math::Real c2 = cosAlpha * cosBeta - K * sinAlpha * sinBeta;
			
			// arm
			::rl::math::Real theta2 = this->atan2(s2, c2) + static_cast< ::rl::math::Real>(M_PI_2);
			
			::rl::math::Real c3 = (a2_2 + a3_2 - Q_2) / (2 * a2 * a3);
			::rl::math::Real s3 = K * ::std::sqrt(1.0f - ::std::pow(c3, 2));
			
			// elbow
			::rl::math::Real theta3 = this->atan2(s3, c3) + static_cast< ::rl::math::Real>(M_PI);
			
			::rl::math::Real c23 = this->cos(theta2 + theta3);
			::rl::math::Real s23 = this->sin(theta2 + theta3);
			
			// wrist
			::rl::math::Real theta4 = this->atan2(
				(c1 * c23) * a(0) +
				(s1 * c23) * a(1) -
							 s23  * a(2),
				 c1 * s23  * a(0) +
				 s1 * s23  * a(1) +
							 c23  * a(2)
			);
			
			// flange
			::rl::math::Real theta5 = this->atan2(
				-s1 * n(0) +
				 c1 * n(1),
				-s1 * s(0) +
				 c1 * s(1)
			);
			
			q(0) = theta1;
			q(1) = theta2;
			q(2) = theta3;
			q(3) = theta4;
			q(4) = theta5;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				q(i) = ::std::fmod(q(i), 2.0f * static_cast< ::rl::math::Real>(M_PI));
				
				if (q(i) < this->joints[i]->min)
				{
					q(i) += 2.0f * static_cast< ::rl::math::Real>(M_PI);
					
					if (q(i) < this->joints[i]->min || q(i) > this->joints[i]->max)
					{
						return false;
					}
				}
				else if (q(i) > this->joints[i]->max)
				{
					q(i) -= 2.0f * static_cast< ::rl::math::Real>(M_PI);
					
					if (q(i) < this->joints[i]->min || q(i) > this->joints[i]->max)
					{
						return false;
					}
				}
			}
			
			return true;
		}
		
		void
		Rhino::parameters(const ::rl::math::Vector& q, Arm& arm, Elbow& elbow) const
		{
			assert(q.size() == this->getDof());
			
			::rl::math::Real c2 = this->cos(q(1));
			::rl::math::Real s2 = this->sin(q(1));
			
			::rl::math::Real c3 = this->cos(q(2));
			::rl::math::Real s3 = this->sin(q(2));
			
			::rl::math::Real a2 = this->joints[1]->a;
			::rl::math::Real a3 = this->joints[2]->a;
			
			::rl::math::Real tmp = s2 * a3 * c3 + c2 * a3 * s3 + a2 * s2;
			::rl::math::Real tmp2 = a3 * s3;
			
			if (tmp < 0.0f)
			{
				arm = ARM_RIGHT;
				
				if (tmp2 < 0.0f)
				{
					elbow = ELBOW_ABOVE;
				}
				else
				{
					elbow = ELBOW_BELOW;
				}
			}
			else
			{
				arm = ARM_LEFT;
				
				if (tmp2 < 0.0f)
				{
					elbow = ELBOW_BELOW;
				}
				else
				{
					elbow = ELBOW_ABOVE;
				}
			}
		}
		
		void
		Rhino::setArm(const Arm& arm)
		{
			this->arm = arm;
		}
		
		void
		Rhino::setElbow(const Elbow& elbow)
		{
			this->elbow = elbow;
		}
	}
}
