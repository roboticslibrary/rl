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

#include "Kinematic.h"
#include "JacobianInverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		JacobianInverseKinematics::JacobianInverseKinematics(Kinematic* kinematic) :
			InverseKinematics(kinematic),
			delta(::std::numeric_limits< ::rl::math::Real>::infinity()),
			duration(::std::chrono::milliseconds(100)),
			epsilon(1.0e-3f),
			iterations(1000),
			svd(true),
			transpose(false),
			randDistribution(0, 1),
			randEngine(::std::random_device()())
		{
		}
		
		JacobianInverseKinematics::~JacobianInverseKinematics()
		{
		}
		
		bool
		JacobianInverseKinematics::solve()
		{
			::std::chrono::steady_clock::time_point start = ::std::chrono::steady_clock::now();
			double remaining = ::std::chrono::duration<double>(this->duration).count();
			
			::rl::math::Vector q = this->kinematic->getPosition();
			::rl::math::Vector q2(this->kinematic->getDofPosition());
			::rl::math::Vector dq(this->kinematic->getDofPosition());
			::rl::math::Vector dx(6 * this->kinematic->getOperationalDof());
			
			::rl::math::Vector rand(this->kinematic->getDof());
			
			do
			{
				::rl::math::Real delta = 1;
				
				for (::std::size_t i = 0; i < this->iterations && delta > this->epsilon; ++i)
				{
					this->kinematic->forwardPosition();
					dx.setZero();
					
					for (::std::size_t i = 0; i < this->goals.size(); ++i)
					{
						::rl::math::VectorBlock dxi = dx.segment(6 * this->goals[i].second, 6);
						dxi = this->kinematic->getOperationalPosition(this->goals[i].second).toDelta(this->goals[i].first);
					}
					
					this->kinematic->calculateJacobian();
					
					if (this->transpose)
					{
						::rl::math::Vector tmp = this->kinematic->getJacobian() * this->kinematic->getJacobian().transpose() * dx;
						dq = dx.dot(tmp) / tmp.dot(tmp) * this->kinematic->getJacobian().transpose() * dx;
					}
					else
					{
						this->kinematic->calculateJacobianInverse(0, this->svd);
						dq = this->kinematic->getJacobianInverse() * dx;
					}
					
					this->kinematic->step(q, dq, q2);
					delta = this->kinematic->distance(q, q2);
					
					if (delta > this->delta)
					{
						this->kinematic->interpolate(q, q2, this->delta, q2);
					}
					
					q = q2;
					this->kinematic->setPosition(q);
				}
				
				if (dx.squaredNorm() < this->epsilon)
				{
					this->kinematic->normalize(q);
					this->kinematic->setPosition(q);
					
					if (this->kinematic->isValid(q))
					{
						return true;
					}
				}
				
				for (::std::size_t i = 0; i < this->kinematic->getDof(); ++i)
				{
					rand(i) = this->randDistribution(this->randEngine);
				}
				
				q = this->kinematic->generatePositionUniform(rand);
				this->kinematic->setPosition(q);
				
				remaining = ::std::chrono::duration<double>(this->duration - (::std::chrono::steady_clock::now() - start)).count();
			}
			while (remaining > 0);
			
			return false;
		}
	}
}
