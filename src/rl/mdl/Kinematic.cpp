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

#include <algorithm>
#include <stack>
#include <boost/graph/graphviz.hpp>
#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>
#include <rl/math/Spatial.h>
#include <rl/math/Unit.h>

#include "Exception.h"
#include "Kinematic.h"
#include "Prismatic.h"
#include "Revolute.h"

namespace rl
{
	namespace mdl
	{
		Kinematic::Kinematic() :
			Metric(),
			invJ(),
			J(),
			Jdqd()
		{
		}
		
		Kinematic::~Kinematic()
		{
		}
		
		void
		Kinematic::calculateJacobian(const bool& inWorldFrame)
		{
			this->calculateJacobian(this->J, inWorldFrame);
		}
		
		void
		Kinematic::calculateJacobian(::rl::math::Matrix& J, const bool& inWorldFrame)
		{
			assert(J.rows() == this->getOperationalDof() * 6);
			assert(J.cols() == this->getDof());
			
			::rl::math::Vector tmp(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				for (::std::size_t j = 0; j < this->getDof(); ++j)
				{
					tmp(j) = i == j ? 1 : 0;
				}
				
				this->setVelocity(tmp);
				this->forwardVelocity();
				
				for (::std::size_t j = 0; j < this->getOperationalDof(); ++j)
				{
					if (inWorldFrame)
					{
						J.block(j * 6, i, 3, 1) = this->getOperationalPosition(j).linear() * this->getOperationalVelocity(j).linear();
						J.block(j * 6 + 3, i, 3, 1) = this->getOperationalPosition(j).linear() * this->getOperationalVelocity(j).angular();
					}
					else
					{
						J.block(j * 6, i, 3, 1) = this->getOperationalVelocity(j).linear();
						J.block(j * 6 + 3, i, 3, 1) = this->getOperationalVelocity(j).angular();
					}
				}
			}
		}
		
		void
		Kinematic::calculateJacobianDerivative(const bool& inWorldFrame)
		{
			this->calculateJacobianDerivative(this->Jdqd, inWorldFrame);
		}
		
		void
		Kinematic::calculateJacobianDerivative(::rl::math::Vector& Jdqd, const bool& inWorldFrame)
		{
			::rl::math::Vector tmp = ::rl::math::Vector::Zero(this->getDof());
			
			this->setAcceleration(tmp);
			this->forwardVelocity();
			this->forwardAcceleration();
			
			for (::std::size_t j = 0; j < this->getOperationalDof(); ++j)
			{
				if (inWorldFrame)
				{
					rl::math::Matrix33 wR = this->getOperationalVelocity(j).angular().cross33() * this->getOperationalPosition(j).linear();
					Jdqd.segment(j * 6, 3) = this->getOperationalPosition(j).linear() * this->getOperationalAcceleration(j).linear() + wR * this->getOperationalVelocity(j).linear();
					Jdqd.segment(j * 6 + 3, 3) = this->getOperationalPosition(j).linear() * this->getOperationalAcceleration(j).angular() + wR * this->getOperationalVelocity(j).angular();
				}
				else
				{
					Jdqd.segment(j * 6, 3) = this->getOperationalAcceleration(j).linear();
					Jdqd.segment(j * 6 + 3, 3) = this->getOperationalAcceleration(j).angular();
				}
			}
		}
		
		void
		Kinematic::calculateJacobianInverse(const ::rl::math::Real& lambda, const bool& doSvd)
		{
			this->calculateJacobianInverse(this->J, this->invJ, lambda, doSvd);
		}
		
		void
		Kinematic::calculateJacobianInverse(const ::rl::math::Matrix& J, ::rl::math::Matrix& invJ, const ::rl::math::Real& lambda, const bool& doSvd) const
		{
			if (doSvd)
			{
				invJ.setZero();
				
				::Eigen::JacobiSVD< ::rl::math::Matrix> svd(J, ::Eigen::ComputeFullU | ::Eigen::ComputeFullV);
				
				::rl::math::Real wMin = svd.singularValues().minCoeff();
				::rl::math::Real lambdaSqr = wMin < static_cast< ::rl::math::Real>(1.0e-9) ? (1 - ::std::pow((wMin / static_cast< ::rl::math::Real>(1.0e-9)), 2)) * ::std::pow(lambda, 2) : 0;
				
				for (::std::ptrdiff_t i = 0; i < svd.nonzeroSingularValues(); ++i)
				{
					invJ.noalias() += (
						svd.singularValues()(i) / (::std::pow(svd.singularValues()(i), 2) + lambdaSqr) *
						svd.matrixV().col(i) * svd.matrixU().col(i).transpose()
					);
				}
			}
			else
			{
				invJ = J.transpose() * (
					J * J.transpose() + ::std::pow(lambda, 2) *
					::rl::math::Matrix::Identity(this->getOperationalDof() * 6, this->getOperationalDof() * 6)
				).inverse();
			}
		}
		
		::rl::math::Real
		Kinematic::calculateManipulabilityMeasure() const
		{
			return calculateManipulabilityMeasure(this->J);
		}
		
		::rl::math::Real
		Kinematic::calculateManipulabilityMeasure(const ::rl::math::Matrix& J) const
		{
			return ::std::sqrt((J * J.transpose()).determinant());
		}
		
		void
		Kinematic::forwardAcceleration()
		{
			for (::std::vector<Element*>::iterator i = this->elements.begin(); i != this->elements.end(); ++i)
			{
				(*i)->forwardAcceleration();
			}
		}
		
		void
		Kinematic::forwardPosition()
		{
			for (::std::vector<Element*>::iterator i = this->elements.begin(); i != this->elements.end(); ++i)
			{
				(*i)->forwardPosition();
			}
		}
		
		void
		Kinematic::forwardVelocity()
		{
			for (::std::vector<Element*>::iterator i = this->elements.begin(); i != this->elements.end(); ++i)
			{
				(*i)->forwardVelocity();
			}
		}
		
		const ::rl::math::Matrix&
		Kinematic::getJacobian() const
		{
			return this->J;
		}
		
		const ::rl::math::Vector&
		Kinematic::getJacobianDerivative() const
		{
			return this->Jdqd;
		}
		
		const ::rl::math::Matrix&
		Kinematic::getJacobianInverse() const
		{
			return this->invJ;
		}
		
		bool
		Kinematic::isSingular() const
		{
			return this->isSingular(this->J);
		}
		
		bool
		Kinematic::isSingular(const ::rl::math::Matrix& J) const
		{
			::Eigen::JacobiSVD< ::rl::math::Matrix> svd(J);
			return (::std::abs(svd.singularValues()(svd.singularValues().size() - 1)) > ::std::numeric_limits< ::rl::math::Real>::epsilon()) ? false : true;
		}
		
		void
		Kinematic::update()
		{
			Metric::update();
			
			this->invJ = ::rl::math::Matrix::Identity(this->getDof(), 6 * this->getOperationalDof());
			this->J = ::rl::math::Matrix::Identity(6 * this->getOperationalDof(), this->getDof());
			this->Jdqd = ::rl::math::Vector::Zero(6 * this->getOperationalDof());
		}
	}
}
