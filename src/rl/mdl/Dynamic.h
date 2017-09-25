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

#ifndef RL_MDL_DYNAMIC_H
#define RL_MDL_DYNAMIC_H

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
			
			/**
			 * Calculate centrifugal and Coriolis vector.
			 * 
			 * @pre setPosition()
			 * @pre setVelocity()
			 * @post getCentrifugalCoriolis()
			 * 
			 * @see inverseDynamics()
			 */
			void calculateCentrifugalCoriolis();
			
			/**
			 * Calculate centrifugal and Coriolis vector.
			 * 
			 * @param[out] V Centrifugal and Coriolis vector \f$\vec{V}(\vec{q}, \dot{\vec{q}})\f$
			 * 
			 * @pre setPosition()
			 * @pre setVelocity()
			 * 
			 * @see inverseDynamics()
			 */
			void calculateCentrifugalCoriolis(::rl::math::Vector& V);
			
			/**
			 * Calculate gravity vector.
			 * 
			 * @pre setPosition()
			 * @post getGravity()
			 * 
			 * @see inverseDynamics()
			 */
			void calculateGravity();
			
			/**
			 * Calculate gravity vector.
			 * 
			 * @param[out] G Gravity vector \f$\vec{G}(\vec{q})\f$
			 * 
			 * @pre setPosition()
			 * 
			 * @see inverseDynamics()
			 */
			void calculateGravity(::rl::math::Vector& G);
			
			/**
			 * Calculate joint space mass matrix.
			 * 
			 * @pre setPosition()
			 * @post getMassMatrix()
			 * 
			 * @see inverseDynamics()
			 */
			void calculateMassMatrix();
			
			/**
			 * Calculate joint space mass matrix.
			 * 
			 * @param[out] M Joint space mass matrix \f$\matr{M}^{-1}(\vec{q})\f$
			 * 
			 * @pre setPosition()
			 * 
			 * @see inverseDynamics()
			 */
			void calculateMassMatrix(::rl::math::Matrix& M);
			
			/**
			 * Calculate joint space mass matrix inverse.
			 * 
			 * @pre setPosition()
			 * @post getMassMatrixInverse()
			 * 
			 * @see forwardDynamics()
			 */
			void calculateMassMatrixInverse();
			
			/**
			 * Calculate joint space mass matrix inverse.
			 * 
			 * @param[out] invM Joint space mass matrix inverse \f$\matr{M}^{-1}(\vec{q})\f$
			 * 
			 * @pre setPosition()
			 * 
			 * @see forwardDynamics()
			 */
			void calculateMassMatrixInverse(::rl::math::Matrix& invM);
			
			/**
			 * Calculate operational space mass matrix inverse.
			 * 
			 * @pre setPosition()
			 * @pre calculateJacobian()
			 * @pre calculateMassMatrix()
			 * @pre calculateMassMatrixInverse()
			 * @post getOperationalMassMatrixInverse()
			 */
			void calculateOperationalMassMatrixInverse();
			
			/**
			 * Calculate operational space mass matrix inverse.
			 * 
			 * @param[in] J Jacobian matrix \f$\matr{J}(\vec{q})\f$
			 * @param[in] invM Joint space mass matrix inverse \f$\matr{M}^{-1}(\vec{q})\f$
			 * @param[out] invMx Operational space mass matrix inverse \f$\matr{M}_{\mathrm{x}}^{-1}(\vec{q})\f$
			 */
			void calculateOperationalMassMatrixInverse(const ::rl::math::Matrix& J, const ::rl::math::Matrix& invM, ::rl::math::Matrix& invMx) const;
			
			Model* clone() const;
			
			/**
			 * Integration via Euler-Cauchy.
			 * 
			 * \f[ \vec{q}_{i + 1} = \vec{q}_{i} + \Delta t \, \dot{\vec{q}}_{i} \f]
			 * \f[ \dot{\vec{q}}_{i + 1} = \dot{\vec{q}}_{i} +  \Delta t \, f(t_{i}, \vec{q}_{i}, \dot{\vec{q}}_{i}) \f]
			 * \f[ \ddot{\vec{q}} = f(t, \vec{q}, \dot{\vec{q}}) \f]
			 * \f[ t_{i + 1} = t_{i} + \Delta t \f]
			 * 
			 * @param[in] dt Integration time step \f$\Delta t\f$
			 * 
			 * @pre setPosition()
			 * @pre setVelocity()
			 * @pre setTorque()
			 * @post getPosition()
			 * @post getVelocity()
			 * @post getAcceleration()
			 * 
			 * @see forwardDynamics
			 */
			void eulerCauchy(const ::rl::math::Real& dt);
			
			/**
			 * Forward dynamics via articulated-body algorithm.
			 * 
			 * \f[ \ddot{\vec{q}} = \matr{M}^{-1}(\vec{q}) \, \bigl( \vec{\tau} - \vec{C}(\vec{q}, \dot{\vec{q}}) - \vec{V}(\vec{q}) \bigr) \f]
			 * 
			 * @pre setPosition()
			 * @pre setVelocity()
			 * @pre setTorque()
			 * @post getAcceleration()
			 */
			void forwardDynamics();
			
			/**
			 * Access calculated centrifugal and Coriolis vector.
			 * 
			 * @return Centrifugal and Coriolis vector \f$\vec{V}(\vec{q}, \dot{\vec{q}})\f$
			 * 
			 * @pre setPosition()
			 * @pre setVelocity()
			 */
			const ::rl::math::Vector& getCentrifugalCoriolis() const;
			
			/**
			 * Access calculated gravity vector.
			 * 
			 * @return Gravity vector \f$\vec{G}(\vec{q})\f$
			 * 
			 * @pre setPosition()
			 * @pre setWorldGravity()
			 */
			const ::rl::math::Vector& getGravity() const;
			
			/**
			 * Access calculated joint space mass matrix inverse.
			 * 
			 * @return Joint space mass matrix inverse \f$\matr{M}^{-1}(\vec{q})\f$
			 * 
			 * @pre setPosition()
			 * @pre calculateJacobian()
			 * @pre calculateMassMatrix()
			 * @pre calculateMassMatrixInverse()
			 */
			const ::rl::math::Matrix& getMassMatrixInverse() const;
			
			/**
			 * Access calculated joint space mass matrix.
			 * 
			 * @return Joint space mass matrix \f$\matr{M}(\vec{q})\f$
			 * 
			 * @pre setPosition()
			 */
			const ::rl::math::Matrix& getMassMatrix() const;
			
			/**
			 * Access calculated operational space mass matrix inverse.
			 * 
			 * @return Operational space mass matrix inverse \f$\matr{M}_{\mathrm{x}}^{-1}(\vec{q})\f$
			 * 
			 * @pre setPosition()
			 * @pre calculateJacobian()
			 * @pre calculateMassMatrix()
			 * @pre calculateMassMatrixInverse()
			 * @pre calculateOperationalMassMatrixInverse()
			 */
			const ::rl::math::Matrix& getOperationalMassMatrixInverse() const;
			
			void getWorldGravity(::rl::math::Real& x, ::rl::math::Real& y, ::rl::math::Real& z) const;
			
			void getWorldGravity(::rl::math::Vector& xyz) const;
			
			/**
			 * Inverse dynamics via recursive Newton-Euler algorithm.
			 * 
			 * \f[ \vec{\tau} = \matr{M}(\vec{q}) \, \ddot{\vec{q}} + \vec{C}(\vec{q}, \dot{\vec{q}}) + \vec{G}(\vec{q}) \f]
			 * 
			 * @pre setPosition()
			 * @pre setVelocity()
			 * @pre setAcceleration()
			 * @post getTorque()
			 */
			void inverseDynamics();
			
			void inverseForce();
			
			/**
			 * Integration via Runge-Kutta-Nystr&ouml;m.
			 * 
			 * \f[ \vec{q}_{i + 1} = \vec{q}_{i} + \Delta t \, \dot{\vec{q}}_{i} + \frac{\Delta t}{3} (\vec{k}_{1} + \vec{k}_{2} + \vec{k}_{3}) \f]
			 * \f[ \dot{\vec{q}}_{i + 1} = \dot{\vec{q}}_{i} + \frac{1}{3} (\vec{k}_{1} + 2 \vec{k}_{2} + 2 \vec{k}_{3} + \vec{k}_{4}) \f]
			 * \f[ \ddot{\vec{q}} = f(t, \vec{q}, \dot{\vec{q}}) \f]
			 * \f[ t_{i + 1} = t_{i} + \Delta t \f]
			 * \f[ \vec{k}_{1} = \frac{\Delta t}{2} \, f(t_{i}, \vec{q}_{i}, \dot{\vec{q}}_{i}) \f]
			 * \f[ \vec{k}_{2} = \frac{\Delta t}{2} \, f(t_{i} + \frac{\Delta t}{2}, \vec{q}_{i} + \frac{\Delta t}{2} \, \dot{\vec{q}}_{i} + \frac{\Delta t}{4} \, \vec{k}_{1}, \dot{\vec{q}}_{i} + \vec{k}_{1}) \f]
			 * \f[ \vec{k}_{3} = \frac{\Delta t}{2} \, f(t_{i} + \frac{\Delta t}{2}, \vec{q}_{i} + \frac{\Delta t}{2} \, \dot{\vec{q}}_{i} + \frac{\Delta t}{4} \, \vec{k}_{1}, \dot{\vec{q}}_{i} + \vec{k}_{2}) \f]
			 * \f[ \vec{k}_{4} = \frac{\Delta t}{2} \, f(t_{i} + \Delta t, \vec{q}_{i} + \Delta t \, \dot{\vec{q}}_{i} + \Delta t \, \vec{k}_{3}, \dot{\vec{q}}_{i} + 2 \vec{k}_{3}) \f]
			 * 
			 * @param[in] dt Integration time step \f$\Delta t\f$
			 * 
			 * @pre setPosition()
			 * @pre setVelocity()
			 * @pre setTorque()
			 * @post getPosition()
			 * @post getVelocity()
			 * @post getAcceleration()
			 * 
			 * @see forwardDynamics()
			 */
			void rungeKuttaNystrom(const ::rl::math::Real& dt);
			
			void setWorldGravity(const ::rl::math::Real& x, const ::rl::math::Real& y, const ::rl::math::Real& z);
			
			void setWorldGravity(const ::rl::math::Vector& xyz);
			
			virtual void update();
			
		protected:
			/**
			 * Gravity vector
			 * 
			 * \f[ \vec{G}(\vec{q}) \f]
			 * 
			 * @pre calculateGravity()
			 * 
			 * @see getGravity()
			 * */
			::rl::math::Vector G;
			
			/**
			 * Joint space mass matrix inverse.
			 * 
			 * \f[ \matr{M}^{-1}(\vec{q}) \f]
			 * 
			 * @pre calculateMassMatrixInverse()
			 * 
			 * @see getMassMatrixInverse()
			 * */
			::rl::math::Matrix invM;
			
			/**
			 * Operational space mass matrix inverse.
			 * 
			 * \f[ \matr{M}_{\mathrm{x}}^{-1}(\vec{q}) \f]
			 * 
			 * @pre calculateOperationalMassMatrixInverse()
			 * 
			 * @see getOperationalMassMatrixInverse()
			 * */
			::rl::math::Matrix invMx;
			
			/**
			 * Joint space mass matrix.
			 * 
			 * \f[ \matr{M}(\vec{q}) \f]
			 * 
			 * @pre calculateMassMatrix()
			 * 
			 * @see getMassMatrix()
			 * */
			::rl::math::Matrix M;
			
			/**
			 * Centrifugal and Coriolis vector.
			 * 
			 * \f[ \vec{V}(\vec{q}, \dot{\vec{q}}) \f]
			 * 
			 * @pre calculateCentrifugalCoriolis()
			 * 
			 * @see getCentrifugalCoriolis()
			 * */
			::rl::math::Vector V;
			
		private:
			
		};
	}
}

#endif // RL_MDL_DYNAMIC_H
