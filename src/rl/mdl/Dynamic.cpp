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
#include "Dynamic.h"
#include "Prismatic.h"
#include "Revolute.h"
#include "World.h"

namespace rl
{
	namespace mdl
	{
		Dynamic::Dynamic() :
			Kinematic(),
			G(),
			invM(),
			invMx(),
			M(),
			V()
		{
		}
		
		Dynamic::~Dynamic()
		{
		}
		
		void
		Dynamic::calculateCentrifugalCoriolis()
		{
			this->calculateCentrifugalCoriolis(this->V);
		}
		
		void
		Dynamic::calculateCentrifugalCoriolis(::rl::math::Vector& V)
		{
			::rl::math::Vector g(3);
			this->getWorldGravity(g);
			
			::rl::math::Vector tmp(this->getDof());
			tmp.setZero(); //TODO
			
			this->setAcceleration(tmp);
			this->setWorldGravity(0, 0, 0);
			
			this->inverseDynamics();
			V = this->getTorque();
			
			this->setWorldGravity(g);
		}
		
		void
		Dynamic::calculateGravity()
		{
			this->calculateGravity(this->G);
		}
		
		void
		Dynamic::calculateGravity(::rl::math::Vector& G)
		{
			::rl::math::Vector tmp(this->getDof());
			tmp.setZero(); //TODO
			
			this->setVelocity(tmp);
			this->setAcceleration(tmp);
			
			this->inverseDynamics();
			G = this->getTorque();
		}
		
		void
		Dynamic::calculateMassMatrix()
		{
			this->calculateMassMatrix(this->M);
		}
		
		void
		Dynamic::calculateMassMatrix(::rl::math::Matrix& M)
		{
			::rl::math::Vector g(3);
			this->getWorldGravity(g);
			
			::rl::math::Vector tmp(this->getDof());
			tmp.setZero(); //TODO
			
			this->setVelocity(tmp);
			this->setWorldGravity(0, 0, 0);
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				for (::std::size_t j = 0; j < this->getDof(); ++j)
				{
					tmp(j) = i == j ? 1 : 0;
				}
				
				this->setAcceleration(tmp);
				this->inverseDynamics();
				
				M.col(i) = this->getTorque();
			}
			
			this->setWorldGravity(g);
		}
		
		void
		Dynamic::calculateMassMatrixInverse()
		{
			this->calculateMassMatrixInverse(this->invM);
		}
		
		void
		Dynamic::calculateMassMatrixInverse(::rl::math::Matrix& invM)
		{
			::rl::math::Vector g(3);
			this->getWorldGravity(g);
			
			::rl::math::Vector tmp(this->getDof());
			tmp.setZero(); //TODO
			
			this->setVelocity(tmp);
			this->setWorldGravity(0, 0, 0);
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				for (::std::size_t j = 0; j < this->getDof(); ++j)
				{
					tmp(j) = i == j ? 1 : 0;
				}
				
				this->setTorque(tmp);
				this->forwardDynamics();
				
				invM.col(i) = this->getAcceleration();
			}
			
			this->setWorldGravity(g);
		}
		
		void
		Dynamic::calculateOperationalMassMatrixInverse()
		{
			this->calculateOperationalMassMatrixInverse(this->J, this->invM, this->invMx);
		}
		
		void
		Dynamic::calculateOperationalMassMatrixInverse(const ::rl::math::Matrix& J, const ::rl::math::Matrix& invM, ::rl::math::Matrix& invMx) const
		{
			invMx = J * invM * J.transpose(); // TODO
		}
		
		Model*
		Dynamic::clone() const
		{
			return new Dynamic(*this);
		}
		
		void
		Dynamic::eulerCauchy(const ::rl::math::Real& dt)
		{
			::rl::math::Vector y = this->getPosition();
			::rl::math::Vector dy = this->getVelocity();
			
			this->forwardDynamics();
			
			// f
			::rl::math::Vector f = this->getAcceleration();
			
			// y_0 + dy_0 * dt
			y += dt * dy; // TODO
			// dy_0 + f * dt
			dy += dt * f; // TODO
			
			this->setPosition(y);
			this->setVelocity(dy);
			this->setAcceleration(f);
		}
		
		void
		Dynamic::forwardDynamics()
		{
			for (::std::vector<Element*>::iterator i = this->elements.begin(); i != this->elements.end(); ++i)
			{
				(*i)->forwardDynamics1();
			}
			
			for (::std::vector<Element*>::reverse_iterator i = this->elements.rbegin(); i != this->elements.rend(); ++i)
			{
				(*i)->forwardDynamics2();
			}
			
			for (::std::vector<Element*>::iterator i = this->elements.begin(); i != this->elements.end(); ++i)
			{
				(*i)->forwardDynamics3();
			}
		}
		
		const ::rl::math::Vector&
		Dynamic::getCentrifugalCoriolis() const
		{
			return this->V;
		}
		
		const ::rl::math::Vector&
		Dynamic::getGravity() const
		{
			return this->G;
		}
		
		const ::rl::math::Matrix&
		Dynamic::getMassMatrixInverse() const
		{
			return this->invM;
		}
		
		const ::rl::math::Matrix&
		Dynamic::getMassMatrix() const
		{
			return this->M;
		}
		
		const ::rl::math::Matrix&
		Dynamic::getOperationalMassMatrixInverse() const
		{
			return this->invMx;
		}
		
		void
		Dynamic::getWorldGravity(::rl::math::Real& x, ::rl::math::Real& y, ::rl::math::Real& z) const
		{
			dynamic_cast<World*>(this->tree[this->root].get())->getGravity(x, y, z);
		}
		
		void
		Dynamic::getWorldGravity(::rl::math::Vector& xyz) const
		{
			this->getWorldGravity(xyz(0), xyz(1), xyz(2));
		}
		
		void
		Dynamic::inverseDynamics()
		{
			for (::std::vector<Element*>::iterator i = this->elements.begin(); i != this->elements.end(); ++i)
			{
				(*i)->inverseDynamics1();
			}
			
			for (::std::vector<Element*>::reverse_iterator i = this->elements.rbegin(); i != this->elements.rend(); ++i)
			{
				(*i)->inverseDynamics2();
			}
		}
		
		void
		Dynamic::inverseForce()
		{
			for (::std::vector<Element*>::reverse_iterator i = this->elements.rbegin(); i != this->elements.rend(); ++i)
			{
				(*i)->inverseForce();
			}
		}
		
		void
		Dynamic::rungeKuttaNystrom(const ::rl::math::Real& dt)
		{
			::rl::math::Vector y0 = this->getPosition();
			::rl::math::Vector dy0 = this->getVelocity();
			
			this->forwardDynamics();
			
			::rl::math::Vector f = this->getAcceleration();
			
			// k1 = dt / 2 * f
			::rl::math::Vector k1 = dt / 2 * f;
			
			// y_0 + dt / 2 * dy_0 + dt / 4 * k_1
			::rl::math::Vector y = y0 + dt / 2 * dy0 + dt / 4 * k1; // TODO
			// dy_0 + k1
			::rl::math::Vector dy = dy0 + k1;
			
			this->setPosition(y);
			this->setVelocity(dy);
			this->forwardDynamics();
			
			// k2 = dt / 2 * f
			::rl::math::Vector k2 = dt / 2 * this->getAcceleration();
			
			// dy_0 + k_2
			dy = dy0 + k2;
			
			this->setVelocity(dy);
			this->forwardDynamics();
			
			// k3 = dt / 2 * f
			::rl::math::Vector k3 = dt / 2 * this->getAcceleration();
			
			// y_0 + dt * dy_0 + dt * k_3
			y = y0 + dt * dy0 + dt * k3; // TODO
			// dy_0 + 2 * k_3
			dy = dy0 + 2 * k3;
			
			this->setPosition(y);
			this->setVelocity(dy);
			this->forwardDynamics();
			
			// k4 = dt / 2 * f
			::rl::math::Vector k4 = dt / 2 * this->getAcceleration();
			
			// y_0 + dy_0 * dt + dt / 3 * (k_1 + k_2 + k_3)
			y = y0 + dy0 * dt + dt / 3 * (k1 + k2 + k3); // TODO
			// dy_0 + 1 / 3 * (k_1 + 2 * k_2 + 2 * k_3 + k_4)
			dy = dy0 + 1.0f / 3.0f * (k1 + 2 * k2 + 2 * k3 + k4); // TODO
			
			this->setPosition(y);
			this->setVelocity(dy);
			this->setAcceleration(f);
		}
		
		void
		Dynamic::setWorldGravity(const ::rl::math::Real& x, const ::rl::math::Real& y, const ::rl::math::Real& z)
		{
			dynamic_cast<World*>(this->tree[this->root].get())->setGravity(x, y, z);
		}
		
		void
		Dynamic::setWorldGravity(const ::rl::math::Vector& xyz)
		{
			this->setWorldGravity(xyz(0), xyz(1), xyz(2));
		}
		
		void
		Dynamic::update()
		{
			Kinematic::update();
			
			this->M.resize(this->getDof(), this->getDof());
			this->V.resize(this->getDof());
			this->G.resize(this->getDof());
			this->invM.resize(this->getDof(), this->getDof());
			this->invMx.resize(6 * this->getOperationalDof(), 6 * this->getOperationalDof());
		}
	}
}
