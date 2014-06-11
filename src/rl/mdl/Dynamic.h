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

#ifndef _RL_MDL_DYNAMIC_H_
#define _RL_MDL_DYNAMIC_H_

#include "Kinematic.h"

namespace rl
{
	namespace mdl
	{
		class Dynamic : public Kinematic
		{
		public:
			Dynamic();
			
			virtual ~Dynamic();
			
			void calculateCentrifugalCoriolis();
			
			/**
			 * Calculate centrifugal and Coriolis vector.
			 * 
			 * \pre setPosition()
			 * \pre setVelocity()
			 * 
			 * \par Side Effects:
			 * inverseDynamics()\n\n
			 * setAcceleration()\n\n
			 * setGravity()
			 */
			void calculateCentrifugalCoriolis(::rl::math::Vector& V);
			
			void calculateGravity();
			
			/**
			 * Calculate gravity vector.
			 * 
			 * \pre setPosition()
			 * \pre setWorldGravity()
			 * 
			 * \par Side Effects:
			 * inverseDynamics()\n\n
			 * setVelocity()\n\n
			 * setAcceleration()
			 */
			void calculateGravity(::rl::math::Vector& G);
			
			void calculateMassMatrix();
			
			/**
			 * Calculate mass matrix.
			 * 
			 * \pre setPosition()
			 * 
			 * \par Side Effects:
			 * inverseDynamics()\n\n
			 * setAcceleration()\n\n
			 * setVelocity()\n\n
			 * setWorldGravity()
			 */
			void calculateMassMatrix(::rl::math::Matrix& M);
			
			void calculateMassMatrixInverse();
			
			/**
			 * Calculate mass matrix inverse.
			 * 
			 * \pre setPosition()
			 * 
			 * \par Side Effects:
			 * forwardDynamics()\n\n
			 * setTorque()\n\n
			 * setVelocity()\n\n
			 * setWorldGravity()
			 */
			void calculateMassMatrixInverse(::rl::math::Matrix& invM);
			
			void calculateOperationalMassMatrixInverse();
			
			/**
			 * Calculate operational space mass matrix inverse.
			 */
			void calculateOperationalMassMatrixInverse(const ::rl::math::Matrix& J, const ::rl::math::Matrix& invM, ::rl::math::Matrix& invMx) const;
			
			Model* clone() const;
			
			/**
			 * Integrate with Euler-Cauchy.
			 * 
			 * \pre setPosition()
			 * \pre setVelocity()
			 * \post getAcceleration()
			 * \post getPosition()
			 * \post getVelocity()
			 */
			void eulerCauchy(const ::rl::math::Real& dt);
			
			/**
			 * \pre setPosition()
			 * \pre setTorque()
			 * \pre setVelocity()
			 * \post getAcceleration()
			 */
			void forwardDynamics();
			
			const ::rl::math::Vector& getCentrifugalCoriolis() const;
			
			const ::rl::math::Vector& getGravity() const;
			
			const ::rl::math::Matrix& getMassMatrixInverse() const;
			
			const ::rl::math::Matrix& getMassMatrix() const;
			
			const ::rl::math::Matrix& getOperationalMassMatrixInverse() const;
			
			void getWorldGravity(::rl::math::Real& x, ::rl::math::Real& y, ::rl::math::Real& z) const;
			
			void getWorldGravity(::rl::math::Vector& xyz) const;
			
			/**
			 * \pre setAcceleration()
			 * \pre setPosition()
			 * \pre setVelocity()
			 * \post getTorque()
			 */
			void inverseDynamics();
			
			void inverseForce();
			
			/**
			 * Integrate with Runge-Kutta-Nystrom.
			 * 
			 * \pre setPosition()
			 * \pre setVelocity()
			 * \post getAcceleration()
			 * \post getPosition()
			 * \post getVelocity()
			 */
			void rungeKuttaNystrom(const ::rl::math::Real& dt);
			
			void setWorldGravity(const ::rl::math::Real& x, const ::rl::math::Real& y, const ::rl::math::Real& z);
			
			void setWorldGravity(const ::rl::math::Vector& xyz);
			
			virtual void update();
			
		protected:
			::rl::math::Vector G;
			
			::rl::math::Matrix invM;
			
			::rl::math::Matrix invMx;
			
			::rl::math::Matrix M;
			
			::rl::math::Vector V;
			
		private:
			
		};
	}
}

#endif // _RL_MDL_DYNAMIC_H_
