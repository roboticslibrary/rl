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
#include "Puma.h"
#include "Transform.h"

namespace rl
{
	namespace kin
	{
		Puma::Puma() :
			Kinematics(),
			arm(ARM_LEFT),
			elbow(ELBOW_ABOVE),
			wrist(WRIST_NONFLIP)
		{
		}
		
		Puma::~Puma()
		{
		}
		
		Kinematics*
		Puma::clone() const
		{
			return new Puma(*this);
		}
		
		Puma::Arm
		Puma::getArm() const
		{
			return this->arm;
		}
		
		Puma::Elbow
		Puma::getElbow() const
		{
			return this->elbow;
		}
		
		Puma::Wrist
		Puma::getWrist() const
		{
			return this->wrist;
		}
		
		bool
		Puma::inversePosition(const ::rl::math::Transform& x, ::rl::math::Vector& q, const ::std::size_t& leaf, const ::rl::math::Real& delta, const ::rl::math::Real& epsilon, const ::std::size_t& iterations)
		{
			assert(q.size() == this->getDof());
			
			int arm = this->arm;
			int elbow = this->elbow;
			int wrist = this->wrist;
			
			::rl::math::Real myeps = 1e-9;
			
			//TODO throw exception instead??
			assert(this->joints[3]->a == 0
				&& this->joints[4]->a == 0
				&& this->joints[5]->a == 0
				&& this->joints[4]->d == 0);
			assert(::std::abs(this->joints[0]->alpha - (-static_cast< ::rl::math::Real>(M_PI)/2)) < 1e-3
				&& ::std::abs(this->joints[1]->alpha                                              ) < 1e-3
				&& ::std::abs(this->joints[2]->alpha - (-static_cast< ::rl::math::Real>(M_PI)/2)) < 1e-3
				&& ::std::abs(this->joints[3]->alpha - ( static_cast< ::rl::math::Real>(M_PI)/2)) < 1e-3
				&& ::std::abs(this->joints[4]->alpha - ( static_cast< ::rl::math::Real>(M_PI)/2)) < 1e-3
				&& ::std::abs(this->joints[5]->alpha                                              ) < 1e-3);
			
			::rl::math::Real a1 = this->joints[0]->a;
			::rl::math::Real a2 = this->joints[1]->a;
			::rl::math::Real a3 = this->joints[2]->a;
			::rl::math::Real d1 = this->joints[0]->d;
			::rl::math::Real d3 = this->joints[1]->d + this->joints[2]->d; // d2 and d3 are parallel, so we choose d3 := d2 + d3 and d2 := 0
			::rl::math::Real d4 = this->joints[3]->d;
			::rl::math::Real d6 = this->joints[5]->d;
			
			::rl::math::Real a3_2 = ::std::pow(a3, 2);
			::rl::math::Real d3_2 = ::std::pow(d3, 2);
			::rl::math::Real d4_2 = ::std::pow(d4, 2);
			
			::rl::math::Transform hand = this->transforms[this->transforms.size() - 1]->transform.inverse(::Eigen::Isometry); // TODO
			::rl::math::Transform base = this->frames[1]->frame.inverse(::Eigen::Isometry); // TODO
			
			::rl::math::Transform x_hand = x * hand;
			::rl::math::Transform x_base = base * x_hand;
			
			const ::rl::math::Transform::TranslationPart& p = x_base.translation();
			const ::rl::math::Transform::LinearPart& rotation = x_base.linear();
			
			const ::rl::math::Transform::LinearPart::ConstColXpr& n = rotation.col(0);
			const ::rl::math::Transform::LinearPart::ConstColXpr& s = rotation.col(1);
			const ::rl::math::Transform::LinearPart::ConstColXpr& a = rotation.col(2);
			
			// wrist center location in 3D
			::rl::math::Vector3 p4 = p - d6 * a;
			
			::rl::math::Real px0 = p4(0);
			::rl::math::Real py0 = p4(1);
			::rl::math::Real pz0 = p4(2);
			
			// Distance from arm to wrist in x_1 y_1 plane (see Siegert p79, p81)
			if (::std::pow(px0, 2) + ::std::pow(py0, 2) - d3_2 < myeps)
			{
				// degenerate case: too close to itself
				return false;
			}
			
			::rl::math::Real r = ::std::pow(::std::pow(px0, 2) + ::std::pow(py0, 2) - d3_2 , 0.5);
			
			// shoulder (see Siegert p82)
			// FIXME: For a1 != 0, the configuration model deviates from the Puma 560
			// In this case, Siegert's formula may give a (correct) solution with wrong arm handedness
			// For instance, for the Mitsubishi RV-6SL robot with a1=50mm, poses near 
			// the boundaries of one configuration will yield an inverse solution in the wrong
			// configuration space.
			// A fix would be to enumerate all (eight) solutions and pick the one thats best
			::rl::math::Real theta1 = this->atan2(-arm * r * py0 - d3 * px0, -arm * r * px0 + d3 * py0);
			
			::rl::math::Real c1 = this->cos(theta1);
			::rl::math::Real s1 = this->sin(theta1);
			
			// note: wrist center projected on x_1 z_1 plane is [ c1*px0 + s1*py0; pz0 ]
			
			// wrist location in x_2 -y_2
			// note: all theta2 and theta3 calculations take place in this 2D plane
			::rl::math::Real adx = c1*px0 + s1*py0 - a1;
			::rl::math::Real ady = pz0 - d1;
			
			// distance of elbow to wrist
			::rl::math::Real c34 = ::std::pow(a3_2 + d4_2, 0.5);
			
			// angle from x_2 to wrist
			::rl::math::Real deltaWrist = -this->atan2(ady, adx);
			
			// distance from arm to wrist
			::rl::math::Real adlength = ::std::pow(::std::pow(adx, 2) + ::std::pow(ady, 2), 0.5);
			
			if (adlength < myeps)
			{
				// arm too close to itself
				return false;
			}
			
			// angle from x_2 to wrist
			::rl::math::Real cosalpha = (::std::pow(a2, 2) + ::std::pow(adlength, 2) - ::std::pow(c34, 2)) / (2 * a2 * adlength);
			
			if (1 - ::std::abs(cosalpha) < myeps)
			{
				// too close to working boundaries or out of working range
				return false;
			}
			
			// arm elbow configuration
			::rl::math::Real K = static_cast< ::rl::math::Real>(arm * elbow);
			
			// arm
			::rl::math::Real theta2 = K * ::std::acos(cosalpha) + deltaWrist;
			
			// angle from x_3 to wrist
			::rl::math::Real beta = this->atan2(d4, a3);
			
			// angle from elbow-arm to elbow-wrist
			::rl::math::Real costheta3plusbeta = (::std::pow(a2, 2) + ::std::pow(c34, 2) - ::std::pow(adlength, 2)) / (2 * a2 * c34);
			
			if (1 - ::std::abs(costheta3plusbeta) < myeps)
			{
				// too close to working boundaries or out of working range
				return false;
			}
			
			::rl::math::Real theta3plusbeta = ::std::acos(costheta3plusbeta);
			
			// elbow
			::rl::math::Real theta3  = K * theta3plusbeta - beta - static_cast< ::rl::math::Real>(M_PI);
			
			::rl::math::Real c23 = this->cos(theta2 + theta3 + static_cast< ::rl::math::Real>(M_PI));
			::rl::math::Real s23 = this->sin(theta2 + theta3 + static_cast< ::rl::math::Real>(M_PI));
			
			// forearm
			
			::rl::math::Real theta4 = this->atan2(
				wrist * (
					-s1       * a(0) +
					 c1       * a(1)
				),
				wrist * (
					 c1 * c23 * a(0) +
					 s1 * c23 * a(1) -
						  s23 * a(2)
				)
			);
			
			::rl::math::Real c4 = this->cos(theta4);
			::rl::math::Real s4 = this->sin(theta4);
			
			// wrist
			
			::rl::math::Real theta5 = this->atan2(
				(c1 * c23 * c4 - s1 * s4) * a(0) +
				(s1 * c23 * c4 + c1 * s4) * a(1) -
					  s23 * c4            * a(2),
				 c1 * s23                 * a(0) +
				 s1 * s23                 * a(1) +
					  c23                 * a(2)
			) + static_cast< ::rl::math::Real>(M_PI);
			
			// flange
			
			::rl::math::Real theta6 = this->atan2(
				-(s4 * c1 * c23 + c4 * s1) * n(0) -
				 (s4 * s1 * c23 - c4 * c1) * n(1) +
				  s4      * s23            * n(2),
				-(s4 * c1 * c23 + c4 * s1) * s(0) -
				 (s4 * s1 * c23 - c4 * c1) * s(1) +
				  s4      * s23            * s(2)
			);
			
			// apply thetas and offsets
			q(0) = theta1 - this->joints[0]->theta - this->joints[0]->offset;
			q(1) = theta2 - this->joints[1]->theta - this->joints[1]->offset;
			q(2) = theta3 - this->joints[2]->theta - this->joints[2]->offset;
			q(3) = theta4 - this->joints[3]->theta - this->joints[3]->offset;
			q(4) = theta5 - this->joints[4]->theta - this->joints[4]->offset;
			q(5) = theta6 - this->joints[5]->theta - this->joints[5]->offset;
			
			// fit into min max angles
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
		
		bool
		Puma::isSingular() const
		{
			return (::std::abs(this->jacobian.determinant()) > 1.0e-9f) ? false : true ;
		}
		
		void
		Puma::parameters(const ::rl::math::Vector& q, Arm& arm, Elbow& elbow, Wrist& wrist) const
		{
			assert(q.size() == this->getDof());
			
			::rl::math::Vector myq(q);
			
			for (::std::size_t i = 0; i < 6; ++i)
			{
				myq(i) = q(i) + this->joints[i]->theta + this->joints[i]->offset;
				myq(i) = ::std::fmod(
					myq(i) + static_cast< ::rl::math::Real>(M_PI),
					2.0f * static_cast< ::rl::math::Real>(M_PI)
				) - static_cast< ::rl::math::Real>(M_PI);
			}
			
			if (myq(4) < 0.0f)
			{
				wrist = WRIST_FLIP;
			}
			else
			{
				wrist = WRIST_NONFLIP;
			}
			
			::rl::math::Real c2 = this->cos(myq(1));
			::rl::math::Real s2 = this->sin(myq(1));
			
			::rl::math::Real c3 = this->cos(myq(2));
			::rl::math::Real s3 = this->sin(myq(2));
			
			::rl::math::Real a2 = this->joints[1]->a;
			::rl::math::Real a3 = this->joints[2]->a;
			::rl::math::Real d4 = this->joints[3]->d;
			
			// z_1 component of the derivative of wrist location wrt to theta2
			::rl::math::Real tmp = d4 * c2 * s3 + d4 * s2 * c3 + a3 * s2 * s3 - a3 * c2 * c3 - a2 * c2; 
			
			// y_2 component of wrist location
			::rl::math::Real tmp2 = a3 * s3 + d4 * c3;
			
			if (tmp < 0.0f)
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
			else
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
		}
		
		void
		Puma::setArm(const Arm& arm)
		{
			this->arm = arm;
		}
		
		void
		Puma::setElbow(const Elbow& elbow)
		{
			this->elbow = elbow;
		}
		
		void
		Puma::setWrist(const Wrist& wrist)
		{
			this->wrist = wrist;
		}
	}
}
