#include <memory>

#include "Kinematic.h"
#include "NloptInverseKinematics.h"

namespace rl
{
	namespace mdl
	{
		NloptInverseKinematics::NloptInverseKinematics(Kinematic* kinematic) :
			InverseKinematics(kinematic),
			delta(1.0e-8f),
			duration(::std::chrono::milliseconds(100)),
			epsilonRotation(1.0e-6f),
			epsilonTranslation(1.0e-6f),
			tolerance(1.0e-8f),
			randDistribution(0, 1),
			randEngine(::std::random_device()())
		{
		}
		
		NloptInverseKinematics::~NloptInverseKinematics()
		{
		}
		
		void
		NloptInverseKinematics::check(const nlopt_result& ret)
		{
			switch (ret)
			{
			case ::NLOPT_FAILURE:
				throw ::std::runtime_error("nlopt failure");
				break;
			case ::NLOPT_INVALID_ARGS:
				throw ::std::invalid_argument("nlopt invalid argument");
				break;
			case ::NLOPT_OUT_OF_MEMORY:
				throw ::std::bad_alloc();
				break;
			default:
				break;
			}
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
		
		bool
		NloptInverseKinematics::solve()
		{
			::std::chrono::steady_clock::time_point start = ::std::chrono::steady_clock::now();
			double remaining = ::std::chrono::duration<double>(this->duration).count();
			
			::std::unique_ptr< ::nlopt_opt_s, decltype(&::nlopt_destroy)> opt(
				::nlopt_create(::NLOPT_LD_SLSQP, this->kinematic->getDofPosition()),
				::nlopt_destroy
			);
			
			::rl::math::Vector lb = this->kinematic->getMinimum();
			::rl::math::Vector ub = this->kinematic->getMaximum();
			
			::std::vector<double> tolerance(this->kinematic->getDofPosition(), this->tolerance);
			
			check(::nlopt_set_xtol_abs(opt.get(), tolerance.data()));
			check(::nlopt_set_min_objective(opt.get(), &NloptInverseKinematics::f, this));
			check(::nlopt_set_lower_bounds(opt.get(), lb.data()));
			check(::nlopt_set_upper_bounds(opt.get(), ub.data()));
			
			::rl::math::Vector q = this->kinematic->getPosition();
			double optF;
			
			do
			{
				check(::nlopt_set_maxtime(opt.get(), remaining));
				check(::nlopt_optimize(opt.get(), q.data(), &optF));
				
				if (this->error(q) < ::std::numeric_limits< ::rl::math::Real>::epsilon())
				{
					return true;
				}
				
				for (::std::size_t i = 0; i < this->kinematic->getDofPosition(); ++i)
				{
					q(i) = lb(i) + this->randDistribution(this->randEngine) * (ub(i) - lb(i));
				}
				
				remaining = ::std::chrono::duration<double>(this->duration - (::std::chrono::steady_clock::now() - start)).count();
			}
			while (remaining > 0);
			
			return false;
		}
	}
}
