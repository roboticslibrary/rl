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

#include "Frame.h"
#include "Joint.h"

namespace rl
{
	namespace mdl
	{
		Joint::Joint(const ::std::size_t& dofPosition, const ::std::size_t& dofVelocity) :
			a(),
			c(),
			D(dofVelocity, dofVelocity),
			max(dofPosition),
			min(dofPosition),
			offset(dofPosition),
			q(dofPosition),
			qUnits(dofPosition),
			qd(dofVelocity),
			qdUnits(dofVelocity),
			qdd(dofVelocity),
			qddUnits(dofVelocity),
			S(6, dofVelocity),
			speed(dofVelocity),
			speedUnits(dofVelocity),
			tau(dofVelocity),
			tauUnits(dofVelocity),
			u(dofVelocity),
			U(6, dofVelocity),
			v(),
			wraparound(dofPosition)
		{
			this->a.setZero(); // TODO
			this->c.setZero(); // TODO
			this->D.setZero(); // TODO
			this->max.setConstant(::std::numeric_limits< ::rl::math::Real>::max()); // TODO
			this->min.setConstant(-::std::numeric_limits< ::rl::math::Real>::max()); // TODO
			this->offset.setZero(); // TODO
			this->q.setZero(); // TODO
			this->qd.setZero(); // TODO
			this->qdd.setZero(); // TODO
			this->S.setZero(); // TODO
			this->speed.setConstant(::std::numeric_limits< ::rl::math::Real>::max()); // TODO
			this->tau.setZero(); // TODO
			this->u.setZero(); // TODO
			this->U.setZero(); // TODO
			this->v.setZero(); // TODO
			this->wraparound.setConstant(false); // TODO
		}
		
		Joint::~Joint()
		{
		}
		
		void
		Joint::clip(::rl::math::Vector& q) const
		{
			for (::std::size_t i = 0; i < this->getDofPosition(); ++i)
			{
				if (this->wraparound(i))
				{
					::rl::math::Real range = ::std::abs(this->max(i) - this->min(i));
					
					while (q(i) > this->max(i))
					{
						q(i) -= range;
					}
					
					while (q(i) < this->min(i))
					{
						q(i) += range;
					}
				}
				else if (q(i) > this->max(i))
				{
					q(i) = this->max(i);
				}
				else if (q(i) < this->min(i))
				{
					q(i) = this->min(i);
				}
			}
		}
		
		::rl::math::Real
		Joint::distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			return (q2 - q1).norm();
		}
		
		void
		Joint::forwardAcceleration()
		{
			// X * a + aj + cj + v x vj
			this->out->a = this->x * this->in->a + this->a + this->c + this->out->v.cross(this->v);
		}
		
		void
		Joint::forwardDynamics1()
		{
			this->forwardVelocity();
			// cj + v x vj
			this->out->c = this->c + this->out->v.cross(this->v);
		}
		
		void
		Joint::forwardDynamics2()
		{
			// I^A * S
			this->U = this->out->iA.matrix() * this->S;
			// S^T * U
			this->D = this->S.transpose() * this->U;
			// tau - S^T * p^A
			this->u = this->tau - this->S.transpose() * this->out->pA.matrix();
			// I^A - U * D^-1 * U^T
			::rl::math::ArticulatedBodyInertia ia;
			ia = this->out->iA.matrix() - this->U * this->D.inverse() * this->U.transpose();
			// p^A + I^a * c + U * D^-1 * u
			::rl::math::ForceVector pa;
			pa = this->out->pA + ia * this->out->c + this->U.matrix() * this->D.inverse() * this->u;
			// I^A + X^* * I^a * X
			this->in->iA = this->in->iA + this->x / ia;
			// p^A + X^* * p^a
			this->in->pA = this->in->pA + this->x / pa;
		}
		
		void
		Joint::forwardDynamics3()
		{
			// X * a + c
			::rl::math::MotionVector a;
			a = this->x * this->in->a + this->out->c;
			// D^-1 * (u - U^T * a')
			this->qdd = this->D.inverse() * (this->u - this->U.transpose() * a.matrix());
			// S * qdd
			this->a = this->S * this->qdd;
			// a' + S * qdd
			this->out->a = a + this->a;
		}
		
		void
		Joint::forwardVelocity()
		{
			// X * v + vj
			this->out->v = this->x * this->in->v + this->v;
		}
		
		::rl::math::Vector
		Joint::generatePositionGaussian(const ::rl::math::ConstVectorRef& rand, const ::rl::math::ConstVectorRef& mean, const ::rl::math::ConstVectorRef& sigma) const
		{
			::rl::math::Vector q(this->getDofPosition());
			
			for (::std::size_t i = 0; i < this->getDofPosition(); ++i)
			{
				q(i) = mean(i) + rand(i) * sigma(i);
			}
			
			this->clip(q);
			
			return q;
		}
		
		::rl::math::Vector
		Joint::generatePositionUniform(const ::rl::math::ConstVectorRef& rand) const
		{
			::rl::math::Vector q(this->getDofPosition());
			
			for (::std::size_t i = 0; i < this->getDofPosition(); ++i)
			{
				q(i) = this->min(i) + rand(i) * (this->max(i) - this->min(i));
			}
			
			return q;
		}
		
		const ::rl::math::Vector&
		Joint::getAcceleration() const
		{
			return this->qdd;
		}
		
		const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>&
		Joint::getAccelerationUnits() const
		{
			return this->qddUnits;
		}
		
		::std::size_t
		Joint::getDof() const
		{
			return this->qd.size();
		}
		
		::std::size_t
		Joint::getDofPosition() const
		{
			return this->q.size();
		}
		
		const ::rl::math::Vector&
		Joint::getMaximum() const
		{
			return this->max;
		}
		
		const ::rl::math::Vector&
		Joint::getMinimum() const
		{
			return this->min;
		}
		
		const ::rl::math::Vector&
		Joint::getPosition() const
		{
			return this->q;
		}
		
		const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>&
		Joint::getPositionUnits() const
		{
			return this->qUnits;
		}
		
		const ::rl::math::Vector&
		Joint::getTorque() const
		{
			return this->tau;
		}
		
		const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>&
		Joint::getTorqueUnits() const
		{
			return this->tauUnits;
		}
		
		const ::rl::math::Vector&
		Joint::getSpeed() const
		{
			return this->speed;
		}
		
		const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>&
		Joint::getSpeedUnits() const
		{
			return this->speedUnits;
		}
		
		const ::rl::math::Vector&
		Joint::getVelocity() const
		{
			return this->qd;
		}
		
		const ::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>&
		Joint::getVelocityUnits() const
		{
			return this->qdUnits;
		}
		
		void
		Joint::interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const
		{
			q = (1.0f - alpha) * q1 + alpha * q2;
		}
		
		void
		Joint::inverseForce()
		{
			// S^T * f
			this->tau = this->S.transpose() * this->out->f.matrix();
			
			// f + X * f
			this->in->f = this->in->f + this->x / this->out->f;
		}
		
		bool
		Joint::isValid(const ::rl::math::Vector& q) const
		{
			for (::std::size_t i = 0; i < this->getDofPosition(); ++i)
			{
				if (q(i) < this->min(i) || q(i) > this->max(i))
				{
					return false;
				}
			}
			
			return true;
		}
		
		void
		Joint::normalize(::rl::math::Vector& q) const
		{
		}
		
		void
		Joint::setAcceleration(const ::rl::math::Vector& qdd)
		{
			this->qdd = qdd;
			
			// S * qdd
			this->a = this->S * this->qdd;
		}
		
		void
		Joint::setTorque(const ::rl::math::Vector& tau)
		{
			this->tau = tau;
		}
		
		void
		Joint::setVelocity(const ::rl::math::Vector& qd)
		{
			this->qd = qd;
			
			// S * qd
			this->v = this->S * this->qd;
		}
		
		void
		Joint::step(const ::rl::math::Vector& q1, const ::rl::math::Vector& qdot, ::rl::math::Vector& q2) const
		{
			q2 = q1 + qdot;
			this->clip(q2);
		}
		
		::rl::math::Real
		Joint::transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			return (q2 - q1).squaredNorm();
		}
	}
}
