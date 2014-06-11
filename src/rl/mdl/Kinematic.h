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

#ifndef _RL_MDL_KINEMATIC_H_
#define _RL_MDL_KINEMATIC_H_

#include <rl/math/Matrix.h>

#include "Metric.h"

namespace rl
{
	namespace mdl
	{
		class Kinematic : public Metric
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			
			Kinematic();
			
			virtual ~Kinematic();
			
			Model* clone() const;
			
			/**
			 * Resolved motion rate control.
			 * 
			 * \pre setPosition()
			 * \post getPosition()
			 * 
			 * \par Side Effects:
			 * calculateJacobian()\n\n
			 * calculateJacobianInverse()\n\n
			 * forwardPosition()
			 */
			bool calculateInversePosition(
				const ::rl::math::Transform& x,
				const ::std::size_t& leaf = 0,
				const ::rl::math::Real& delta = ::std::numeric_limits< ::rl::math::Real >::infinity(),
				const ::rl::math::Real& epsilon = 1.0e-3f,
				const ::std::size_t& iterations = 1000
			);
			
			void calculateJacobian();
			
			/**
			 * Calculate Jacobian matrix.
			 * 
			 * \pre setPosition()
			 * 
			 * \par Side Effects:
			 * forwardVelocity()\n\n
			 * setVelocity()
			 */
			void calculateJacobian(::rl::math::Matrix& J);
			
			void calculateJacobianDerivative();
			
			/**
			 * Calculate Jacobian derivative matrix.
			 * 
			 * \pre setPosition()
			 * \pre setVelocity()
			 * 
			 * \par Side Effects:
			 * forwardAcceleration()\n\n
			 * forwardVelocity()\n\n
			 * setAcceleration()
			 */
			void calculateJacobianDerivative(::rl::math::Vector& Jdqd);
			
			void calculateJacobianInverse(const ::rl::math::Real& lambda = 0.0f, const bool& doSvd = true);
			
			/**
			 * Calculate Jacobian matrix inverse.
			 * 
			 * \pre calculateJacobian()
			 */
			void calculateJacobianInverse(const ::rl::math::Matrix& J, ::rl::math::Matrix& invJ, const ::rl::math::Real& lambda = 0.0f, const bool& doSvd = true) const;
			
			/**
			 * Calculate manipulability measure.
			 * 
			 * \pre calculateJacobian()
			 */
			::rl::math::Real calculateManipulabilityMeasure() const;
			
			::rl::math::Real calculateManipulabilityMeasure(const ::rl::math::Matrix& J) const;
			
			/**
			 * \pre setAcceleration()
			 * \pre setPosition()
			 * \pre setVelocity()
			 * \post getOperationalAcceleration()
			 */
			void forwardAcceleration();
			
			/**
			 * \pre setPosition()
			 * \post getOperationalPosition()
			 */
			void forwardPosition();
			
			/**
			 * \pre setPosition()
			 * \pre setVelocity()
			 * \post getOperationalVelocity()
			 */
			void forwardVelocity();
			
			const ::rl::math::Matrix& getJacobian() const;
			
			const ::rl::math::Vector& getJacobianDerivative() const;
			
			const ::rl::math::Matrix& getJacobianInverse() const;
			
			/**
			 * Check if current configuration is singular.
			 * 
			 * \pre calculateJacobian()
			 */
			bool isSingular() const;
			
			bool isSingular(const ::rl::math::Matrix& J) const;
			
			virtual void update();
			
		protected:
			::rl::math::Matrix invJ;
			
			::rl::math::Matrix J;
			
			::rl::math::Vector Jdqd;
			
		private:
			
		};
	}
}

#endif // _RL_MDL_KINEMATIC_H_
