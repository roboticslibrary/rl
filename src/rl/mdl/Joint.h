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

#ifndef RL_MDL_JOINT_H
#define RL_MDL_JOINT_H

#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

#include "Transform.h"

namespace rl
{
	namespace mdl
	{
		class Joint : public Transform
		{
		public:
			Joint(const ::std::size_t& dofPosition, const ::std::size_t& dofVelocity);
			
			virtual ~Joint();
			
			virtual void clip(::rl::math::Vector& q) const;
			
			virtual ::rl::math::Real distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;
			
			void forwardAcceleration();
			
			void forwardDynamics1();
			
			void forwardDynamics2();
			
			void forwardDynamics3();
			
			void forwardVelocity();
			
			virtual ::rl::math::Vector generatePositionGaussian(const ::rl::math::ConstVectorRef& rand, const ::rl::math::ConstVectorRef& mean, const ::rl::math::ConstVectorRef& sigma) const;
			
			virtual ::rl::math::Vector generatePositionUniform(const ::rl::math::ConstVectorRef& rand) const;
			
			const ::rl::math::Vector& getAcceleration() const;
			
			const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>& getAccelerationUnits() const;
			
			::std::size_t getDof() const;
			
			::std::size_t getDofPosition() const;
			
			const ::rl::math::Vector& getMaximum() const;
			
			const ::rl::math::Vector& getMinimum() const;
			
			const ::rl::math::Vector& getPosition() const;
			
			const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>& getPositionUnits() const;
			
			const ::rl::math::Vector& getTorque() const;
			
			const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>& getTorqueUnits() const;
			
			const ::rl::math::Vector& getSpeed() const;
			
			const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>& getSpeedUnits() const;
			
			const ::rl::math::Vector& getVelocity() const;
			
			const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>& getVelocityUnits() const;
			
			virtual void interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const;
			
			void inverseForce();
			
			virtual bool isValid(const ::rl::math::Vector& q) const;
			
			virtual void normalize(::rl::math::Vector& q) const;
			
			void setAcceleration(const ::rl::math::Vector& qdd);
			
			virtual void setPosition(const ::rl::math::Vector& q) = 0;
			
			void setTorque(const ::rl::math::Vector& tau);
			
			void setVelocity(const ::rl::math::Vector& qd);
			
			virtual void step(const ::rl::math::Vector& q1, const ::rl::math::Vector& qdot, ::rl::math::Vector& q2) const;
			
			virtual ::rl::math::Real transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;
			
			::rl::math::MotionVector a;
			
			::rl::math::MotionVector c;
			
			::rl::math::Matrix D;
			
			::rl::math::Vector max;
			
			::rl::math::Vector min;
			
			::rl::math::Vector offset;
			
			::rl::math::Vector q;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> qUnits;
			
			::rl::math::Vector qd;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> qdUnits;
			
			::rl::math::Vector qdd;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> qddUnits;
			
			::rl::math::Matrix S;
			
			::rl::math::Vector speed;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> speedUnits;
			
			::rl::math::Vector tau;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> tauUnits;
			
			::rl::math::Vector u;
			
			::rl::math::Matrix U;
			
			::rl::math::MotionVector v;
			
			::Eigen::Matrix<bool, ::Eigen::Dynamic, 1> wraparound;
			
		protected:
			
		private:
			
		};
	}
}

#endif // RL_MDL_JOINT_H
