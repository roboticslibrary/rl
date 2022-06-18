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

#include "JacobianInverseKinematics.h"
#include "Kinematic.h"

namespace rl
{
	namespace mdl
	{
		constexpr JacobianInverseKinematics::Method JacobianInverseKinematics::METHOD_DLS;
		constexpr JacobianInverseKinematics::Method JacobianInverseKinematics::METHOD_SVD;
		constexpr JacobianInverseKinematics::Method JacobianInverseKinematics::METHOD_TRANSPOSE;
		
		JacobianInverseKinematics::JacobianInverseKinematics(Kinematic* kinematic) :
			IterativeInverseKinematics(kinematic),
			delta(::std::numeric_limits<::rl::math::Real>::infinity()),
			method(Method::svd),
			randDistribution(0, 1),
			randEngine(::std::random_device()()),
			steps(100)
		{
		}
		
		JacobianInverseKinematics::~JacobianInverseKinematics()
		{
		}
		
		const ::rl::math::Real&
		JacobianInverseKinematics::getDelta() const
		{
			return this->delta;
		}
		
		const JacobianInverseKinematics::Method&
		JacobianInverseKinematics::getMethod() const
		{
			return this->method;
		}
		
		const ::std::size_t&
		JacobianInverseKinematics::getSteps() const
		{
			return this->steps;
		}
		
		void
		JacobianInverseKinematics::seed(const ::std::mt19937::result_type& value)
		{
			this->randEngine.seed(value);
		}
		
		void
		JacobianInverseKinematics::setDelta(const ::rl::math::Real& delta)
		{
			this->delta = delta;
		}
		
		void
		JacobianInverseKinematics::setMethod(const Method& method)
		{
			this->method = method;
		}
		
		void
		JacobianInverseKinematics::setSteps(const ::std::size_t& steps)
		{
			this->steps = steps;
		}
		
		bool
		JacobianInverseKinematics::solve()
		{
			::std::chrono::steady_clock::time_point start = ::std::chrono::steady_clock::now();
			double remaining = ::std::chrono::duration<double>(this->getDuration()).count();
			::std::size_t attempt = 0;
			::std::size_t iteration = 0;
			
			::rl::math::Vector q = this->kinematic->getPosition();
			::rl::math::Vector q2(this->kinematic->getDofPosition());
			::rl::math::Vector dq(this->kinematic->getDofPosition());
			::rl::math::Vector dx(6 * this->kinematic->getOperationalDof());
			
			::rl::math::Vector rand(this->kinematic->getDof());
			
			do
			{
				if (attempt > 0)
				{
					for (::std::size_t i = 0; i < this->kinematic->getDof(); ++i)
					{
						rand(i) = this->randDistribution(this->randEngine);
					}
					
					q = this->kinematic->generatePositionUniform(rand);
					this->kinematic->setPosition(q);
				}
				
				for (::std::size_t i = 0; i < this->steps && remaining > 0 && iteration < this->getIterations(); ++i, ++iteration)
				{
					this->kinematic->forwardPosition();
					dx.setZero();
					
					for (::std::size_t j = 0; j < this->goals.size(); ++j)
					{
						::rl::math::VectorBlock dxi = dx.segment(6 * this->goals[j].second, 6);
						dxi = this->kinematic->getOperationalPosition(this->goals[j].second).toDelta(this->goals[j].first);
					}
					
					if (dx.squaredNorm() < ::std::pow(this->getEpsilon(), 2))
					{
						this->kinematic->normalize(q);
						this->kinematic->setPosition(q);
						
						if (this->kinematic->isValid(q))
						{
							return true;
						}
					}
					
					this->kinematic->calculateJacobian();
					
					switch (this->method)
					{
					case Method::dls:
						this->kinematic->calculateJacobianInverse(0, false);
						dq = this->kinematic->getJacobianInverse() * dx;
						break;
					case Method::svd:
						this->kinematic->calculateJacobianInverse(0, true);
						dq = this->kinematic->getJacobianInverse() * dx;
						break;
					case Method::transpose:
						{
							::rl::math::Vector tmp = this->kinematic->getJacobian() * this->kinematic->getJacobian().transpose() * dx;
							::rl::math::Real alpha = dx.dot(tmp) / tmp.dot(tmp);
							dq = alpha * this->kinematic->getJacobian().transpose() * dx;
						}
						break;
					default:
						break;
					}
					
					this->kinematic->step(q, dq, q2);
					
					if (this->kinematic->transformedDistance(q, q2) > ::std::pow(this->delta, 2))
					{
						this->kinematic->interpolate(q, q2, this->delta, q2);
					}
					
					q = q2;
					this->kinematic->setPosition(q);
					
					remaining = ::std::chrono::duration<double>(this->getDuration() - (::std::chrono::steady_clock::now() - start)).count();
				}
			}
			while (attempt++ < this->getRandomRestarts() && remaining > 0 && iteration < this->getIterations());
			
			return false;
		}
	}
}
