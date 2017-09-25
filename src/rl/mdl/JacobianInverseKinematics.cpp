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
			::rl::math::Vector dq(this->kinematic->getDofPosition());
			::rl::math::Vector dx(6 * this->kinematic->getOperationalDof());
			
			::rl::math::Vector lb = this->kinematic->getMinimum();
			::rl::math::Vector ub = this->kinematic->getMaximum();
			
			do
			{
				::rl::math::Real norm = 1;
				
				for (::std::size_t i = 0; i < this->iterations && norm > this->epsilon; ++i)
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
					
					norm = dq.norm();
					
					if (norm > this->delta)
					{
						dq *= this->delta / norm;
						norm = dq.norm();
					}
					
					q += dq;
					
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
				
				for (::std::size_t i = 0; i < this->kinematic->getDofPosition(); ++i)
				{
					q(i) = lb(i) + this->randDistribution(this->randEngine) * (ub(i) - lb(i));
				}
				
				this->kinematic->setPosition(q);
				
				remaining = ::std::chrono::duration<double>(this->duration - (::std::chrono::steady_clock::now() - start)).count();
			}
			while (remaining > 0);
			
			return false;
		}
	}
}
