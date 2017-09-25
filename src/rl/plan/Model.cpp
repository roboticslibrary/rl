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

#include <rl/sg/Body.h>

#include "Model.h"

namespace rl
{
	namespace plan
	{
		Model::Model() :
			kin(nullptr),
			mdl(nullptr),
			model(nullptr),
			scene(nullptr)
		{
		}
		
		Model::~Model()
		{
		}
		
		bool
		Model::areColliding(const ::std::size_t& i, const ::std::size_t& j) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->areColliding(i, j);
			}
			else
			{
				return this->mdl->areColliding(i, j);
			}
		}
		
		void
		Model::clip(::rl::math::Vector& q) const
		{
			if (nullptr != this->kin)
			{
				this->kin->clip(q);
			}
			else
			{
				this->mdl->clip(q);
			}
		}
		
		::rl::math::Real
		Model::distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->distance(q1, q2);
			}
			else
			{
				return this->mdl->distance(q1, q2);
			}
		}
		
		void
		Model::forwardForce(const ::rl::math::Vector& tau, ::rl::math::Vector& f) const
		{
			if (nullptr != this->kin)
			{
				this->kin->forwardForce(tau, f);
			}
			else
			{
				f = this->mdl->getJacobianInverse().transpose() * tau; // TODO
			}
		}
		
		const ::rl::math::Transform&
		Model::forwardPosition(const ::std::size_t& i) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->forwardPosition(i);
			}
			else
			{
				return this->mdl->getOperationalPosition(i);
			}
		}
		
		void
		Model::forwardVelocity(const ::rl::math::Vector& qdot, ::rl::math::Vector& xdot) const
		{
			if (nullptr != this->kin)
			{
				this->kin->forwardVelocity(qdot, xdot);
			}
			else
			{
				xdot = this->mdl->getJacobian() * qdot; // TODO
			}
		}
		
		::rl::math::Vector
		Model::generatePositionGaussian(const ::rl::math::Vector& rand, const ::rl::math::Vector& mean, const ::rl::math::Vector& sigma) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->generatePositionGaussian(rand, mean, sigma);
			}
			else
			{
				return this->mdl->generatePositionGaussian(rand, mean, sigma);
			}
		}
		
		::rl::math::Vector
		Model::generatePositionUniform(const ::rl::math::Vector& rand) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->generatePositionUniform(rand);
			}
			else
			{
				return this->mdl->generatePositionUniform(rand);
			}
		}
		
		::std::size_t
		Model::getBodies() const
		{
			if (nullptr != this->kin)
			{
				return this->kin->getBodies();
			}
			else
			{
				return this->mdl->getBodies();
			}
		}
		
		::rl::sg::Body*
		Model::getBody(const ::std::size_t& i) const
		{
			return this->model->getBody(i);
		}
		
		::rl::math::Vector3&
		Model::getCenter(const ::std::size_t& i) const
		{
			return this->model->getBody(i)->center;
		}
		
		::std::size_t
		Model::getDof() const
		{
			if (nullptr != this->kin)
			{
				return this->kin->getDof();
			}
			else
			{
				return this->mdl->getDof();
			}
		}
		
		::std::size_t
		Model::getDofPosition() const
		{
			if (nullptr != this->kin)
			{
				return this->kin->getDof();
			}
			else
			{
				return this->mdl->getDofPosition();
			}
		}
		
		const ::rl::math::Transform&
		Model::getFrame(const ::std::size_t& i) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->getFrame(i);
			}
			else
			{
				return this->mdl->getFrame(i);
			}
		}
		
		const ::rl::math::Matrix&
		Model::getJacobian() const
		{
			if (nullptr != this->kin)
			{
				return this->kin->getJacobian();
			}
			else
			{
				return this->mdl->getJacobian();
			}
		}
		
		::rl::math::Real
		Model::getManipulabilityMeasure() const
		{
			if (nullptr != this->kin)
			{
				return this->kin->getManipulabilityMeasure();
			}
			else
			{
				return this->mdl->calculateManipulabilityMeasure();
			}
		}
		
		::std::string
		Model::getManufacturer() const
		{
			if (nullptr != this->kin)
			{
				return this->kin->getManufacturer();
			}
			else
			{
				return this->mdl->getManufacturer();
			}
		}
		
		::rl::math::Vector
		Model::getMaximum() const
		{
			if (nullptr != this->kin)
			{
				::rl::math::Vector maximum(this->getDofPosition());
				this->kin->getMaximum(maximum);
				return maximum;
			}
			else
			{
				return this->mdl->getMaximum();
			}
		}
		
		::rl::math::Vector
		Model::getMinimum() const
		{
			if (nullptr != this->kin)
			{
				::rl::math::Vector minimum(this->getDofPosition());;
				this->kin->getMinimum(minimum);
				return minimum;
			}
			else
			{
				return this->mdl->getMinimum();
			}
		}
		
		::std::string
		Model::getName() const
		{
			if (nullptr != this->kin)
			{
				return this->kin->getName();
			}
			else
			{
				return this->mdl->getName();
			}
		}
		
		::std::size_t
		Model::getOperationalDof() const
		{
			if (nullptr != this->kin)
			{
				return this->kin->getOperationalDof();
			}
			else
			{
				return this->mdl->getOperationalDof();
			}
		}
		
		::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>
		Model::getPositionUnits() const
		{
			if (nullptr != this->kin)
			{
				::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> units(this->getDofPosition());;
				this->kin->getPositionUnits(units);
				return units;
			}
			else
			{
				return this->mdl->getPositionUnits();
			}
		}
		
		void
		Model::inverseForce(const ::rl::math::Vector& f, ::rl::math::Vector& tau) const
		{
			if (nullptr != this->kin)
			{
				this->kin->inverseForce(f, tau);
			}
			else
			{
				tau = this->mdl->getJacobian().transpose() * f; // TODO
			}
		}
		
		::rl::math::Real
		Model::inverseOfTransformedDistance(const ::rl::math::Real& d) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->inverseOfTransformedDistance(d);
			}
			else
			{
				return this->mdl->inverseOfTransformedDistance(d);
			}
		}
		
		void
		Model::inverseVelocity(const ::rl::math::Vector& xdot, ::rl::math::Vector& qdot) const
		{
			if (nullptr != this->kin)
			{
				this->kin->inverseVelocity(xdot, qdot);
			}
			else
			{
				qdot = this->mdl->getJacobianInverse() * xdot; // TODO
			}
		}
		
		void
		Model::interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const
		{
			if (nullptr != this->kin)
			{
				this->kin->interpolate(q1, q2, alpha, q);
			}
			else
			{
				this->mdl->interpolate(q1, q2, alpha, q);
			}
		}
		
		bool
		Model::isColliding(const ::std::size_t& i) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->isColliding(i);
			}
			else
			{
				return this->mdl->isColliding(i);
			}
		}
		
		bool
		Model::isSingular() const
		{
			if (nullptr != this->kin)
			{
				return this->kin->isSingular();
			}
			else
			{
				return this->mdl->isSingular();
			}
		}
		
		bool
		Model::isValid(const ::rl::math::Vector& q) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->isValid(q);
			}
			else
			{
				return this->mdl->isValid(q);
			}
		}
		
		::rl::math::Real
		Model::maxDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->maxDistanceToRectangle(q, min, max);
			}
			else
			{
				return this->mdl->maxDistanceToRectangle(q, min, max);
			}
		}
		
		::rl::math::Real
		Model::minDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->minDistanceToRectangle(q, min, max);
			}
			else
			{
				return this->mdl->minDistanceToRectangle(q, min, max);
			}
		}
		
		::rl::math::Real
		Model::minDistanceToRectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cuttingDimension) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->minDistanceToRectangle(q, min, max, cuttingDimension);
			}
			else
			{
				return this->mdl->minDistanceToRectangle(q, min, max, cuttingDimension);
			}
		}
		
		::rl::math::Real
		Model::newDistance(const ::rl::math::Real& dist, const ::rl::math::Real& oldOff, const ::rl::math::Real& newOff, const int& cuttingDimension) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->newDistance(dist, oldOff, newOff, cuttingDimension);
			}
			else
			{
				return this->mdl->newDistance(dist, oldOff, newOff, cuttingDimension);
			}
		}
		
		void
		Model::reset()
		{
		}
		
		void
		Model::setPosition(const ::rl::math::Vector& q)
		{
			if (nullptr != this->kin)
			{
				assert(q.size() == this->kin->getDof());
				this->kin->setPosition(q);
			}
			else
			{
				assert(q.size() == this->mdl->getDofPosition());
				this->mdl->setPosition(q);
			}
		}
		
		void
		Model::step(const ::rl::math::Vector& q1, const ::rl::math::Vector& qdot, ::rl::math::Vector& q2) const
		{
			if (nullptr != this->kin)
			{
				this->kin->step(q1, qdot, q2);
			}
			else
			{
				this->mdl->step(q1, qdot, q2);
			}
		}
		
		::rl::math::Real
		Model::transformedDistance(const ::rl::math::Real& d) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->transformedDistance(d);
			}
			else
			{
				return this->mdl->transformedDistance(d);
			}
		}
		
		::rl::math::Real
		Model::transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->transformedDistance(q1, q2);
			}
			else
			{
				return this->mdl->transformedDistance(q1, q2);
			}
		}
		
		::rl::math::Real
		Model::transformedDistance(const ::rl::math::Real& q1, const ::rl::math::Real& q2, const ::std::size_t& i) const
		{
			if (nullptr != this->kin)
			{
				return this->kin->transformedDistance(q1, q2, i);
			}
			else
			{
				return this->mdl->transformedDistance(q1, q2, i);
			}
		}
		
		void
		Model::updateFrames(const bool& doUpdateModel)
		{
			if (nullptr != this->kin)
			{
				assert(this->model->getNumBodies() == this->kin->getBodies());
				
				this->kin->updateFrames();
				
				if (doUpdateModel)
				{
					for (::std::size_t i = 0; i < this->model->getNumBodies(); ++i)
					{
						this->model->getBody(i)->setFrame(this->kin->getFrame(i));
					}
				}
			}
			else
			{
				assert(this->model->getNumBodies() == this->mdl->getBodies());
				
				this->mdl->forwardPosition();
				
				if (doUpdateModel)
				{
					for (::std::size_t i = 0; i < this->model->getNumBodies(); ++i)
					{
						this->model->getBody(i)->setFrame(this->mdl->getFrame(i));
					}
				}
			}
		}
		
		void
		Model::updateJacobian()
		{
			if (nullptr != this->kin)
			{
				this->kin->updateJacobian();
			}
			else
			{
				this->mdl->calculateJacobian();
			}
		}
		
		void
		Model::updateJacobianInverse(const ::rl::math::Real& lambda, const bool& doSvd)
		{
			if (nullptr != this->kin)
			{
				this->kin->updateJacobianInverse(lambda, doSvd);
			}
			else
			{
				this->mdl->calculateJacobianInverse(lambda, doSvd);
			}
		}
	}
}
