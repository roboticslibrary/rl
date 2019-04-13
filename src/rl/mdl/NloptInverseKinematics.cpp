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

#include <memory>

#include "Kinematic.h"
#include "NloptInverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		NloptInverseKinematics::NloptInverseKinematics(Kinematic* kinematic) :
			IterativeInverseKinematics(kinematic),
			delta(static_cast< ::rl::math::Real>(1.0e-8)),
			epsilonRotation(static_cast< ::rl::math::Real>(1.0e-6)),
			epsilonTranslation(static_cast< ::rl::math::Real>(1.0e-6)),
			randDistribution(0, 1),
			randEngine(::std::random_device()()),
			tolerance(static_cast< ::rl::math::Real>(1.0e-8))
		{
			this->lb = this->kinematic->getMinimum();
			this->ub = this->kinematic->getMaximum();
		}
		
		NloptInverseKinematics::~NloptInverseKinematics()
		{
		}
		
		::rl::math::Real
		NloptInverseKinematics::error(const ::rl::math::Vector& q)
		{
			this->kinematic->setPosition(q);
			this->kinematic->forwardPosition();
			
			::rl::math::Vector dx = ::rl::math::Vector::Zero(6 * this->kinematic->getOperationalDof());
			
			for (::std::size_t i = 0; i < this->goals.size(); ++i)
			{
				::rl::math::VectorBlock dxi = dx.segment(6 * this->goals[i].second, 6);
				dxi = this->kinematic->getOperationalPosition(this->goals[i].second).toDelta(this->goals[i].first);
				
				if (dxi.segment(0, 3).cwiseAbs().maxCoeff() < this->epsilonTranslation)
				{
					dxi.segment(0, 3).setZero();
				}
				
				if (dxi.segment(3, 3).cwiseAbs().maxCoeff() < this->epsilonRotation)
				{
					dxi.segment(3, 3).setZero();
				}
			}
			
			return dx.squaredNorm();
		}
		
		::rl::math::Real
		NloptInverseKinematics::f(unsigned n, const double* x, double* grad, void* data)
		{
			NloptInverseKinematics* ik = static_cast<NloptInverseKinematics*>(data);
			
			::Eigen::Map< const ::rl::math::Vector> q(x, n, 1);
			
			if (!q.allFinite())
			{
				return ::std::numeric_limits< ::rl::math::Real>::infinity();
			}
			
			::rl::math::Real result = ik->error(q);
			
			::rl::math::Vector q2(q);
			
			if (nullptr != grad)
			{
				for (::std::ptrdiff_t i = 0; i < q.size(); ++i)
				{
					q2(i) = q(i) + ik->delta;
					::rl::math::Real dq1 = ik->error(q2);
					
					q2(i) = q(i) - ik->delta;
					::rl::math::Real dq2 = ik->error(q2);
					
					q2(i) = q(i);
					grad[i] = (dq1 - dq2) / (2 * ik->delta);
				}
			}
			
			return result;
		}
		
		const ::rl::math::Real&
		NloptInverseKinematics::getEpsilonRotation() const
		{
			return this->epsilonRotation;
		}
		
		const ::rl::math::Real&
		NloptInverseKinematics::getEpsilonTranslation() const
		{
			return this->epsilonTranslation;
		}
		
		const ::rl::math::Real&
		NloptInverseKinematics::getDelta() const
		{
			return this->delta;
		}
		
		const double&
		NloptInverseKinematics::getTolerance() const
		{
			return this->tolerance;
		}
		
		void
		NloptInverseKinematics::setDelta(const::rl::math::Real& delta)
		{
			this->delta = delta;
		}
		
		void NloptInverseKinematics::setEpsilonRotation(const::rl::math::Real& epsilonRotation)
		{
			this->epsilonRotation = epsilonRotation;
		}
		
		void NloptInverseKinematics::setEpsilonTranslation(const::rl::math::Real& epsilonTranslation)
		{
			this->epsilonTranslation = epsilonTranslation;
		}
		
		void
		NloptInverseKinematics::setLowerBound(const ::rl::math::Vector& lb)
		{
			this->lb = lb;
		}
		
		void
		NloptInverseKinematics::setTolerance(const double& tolerance)
		{
			this->tolerance = tolerance;
		}
		
		void
		NloptInverseKinematics::setUpperBound(const ::rl::math::Vector& ub)
		{
			this->ub = ub;
		}
		
		bool
		NloptInverseKinematics::solve()
		{
			::std::chrono::steady_clock::time_point start = ::std::chrono::steady_clock::now();
			double remaining = ::std::chrono::duration<double>(this->duration).count();
			
			::std::unique_ptr< ::nlopt_opt_s, decltype(&::nlopt_destroy)> opt(
				::nlopt_create(::NLOPT_LD_SLSQP, this->kinematic->getDofPosition()),
				::nlopt_destroy
			);
			
			::std::vector<double> tolerance(this->kinematic->getDofPosition(), this->tolerance);
			
			if (::nlopt_set_xtol_abs(opt.get(), tolerance.data()) < 0)
			{
				return false;
			}
			
			if (::nlopt_set_min_objective(opt.get(), &NloptInverseKinematics::f, this) < 0)
			{
				return false;
			}
			
			if (::nlopt_set_lower_bounds(opt.get(), this->lb.data()) < 0)
			{
				return false;
			}
			
			if (::nlopt_set_upper_bounds(opt.get(), this->ub.data()) < 0)
			{
				return false;
			}
			
			::rl::math::Vector rand(this->kinematic->getDof());
			::rl::math::Vector q = this->kinematic->getPosition();
			double optF;
			
			do
			{
				if (::nlopt_set_maxtime(opt.get(), remaining) < 0)
				{
					return false;
				}
				
				if (::nlopt_optimize(opt.get(), q.data(), &optF) < -1)
				{
					return false;
				}
				
				if (this->error(q) < ::std::numeric_limits< ::rl::math::Real>::epsilon())
				{
					return true;
				}
				
				for (::std::size_t i = 0; i < this->kinematic->getDof(); ++i)
				{
					rand(i) = this->randDistribution(this->randEngine);
				}
				
				q = this->kinematic->generatePositionUniform(rand, this->lb, this->ub);
				
				remaining = ::std::chrono::duration<double>(this->duration - (::std::chrono::steady_clock::now() - start)).count();
			}
			while (remaining > 0);
			
			return false;
		}
	}
}
