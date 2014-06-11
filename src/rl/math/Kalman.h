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

#ifndef _RL_MATH_KALMAN_H_
#define _RL_MATH_KALMAN_H_

#include <Eigen/LU>

#include "Matrix.h"
#include "Vector.h"

namespace rl
{
	namespace math
	{
		class Kalman
		{
		public:
			Kalman()
			{
			};
			
			virtual ~Kalman()
			{
			};
			
			/**
			 * \f[ K_{k} = P^{-}_{k} H^{T} \left( H P^{-}_{k} H^{T} + R \right)^{-1} \f]
			 * \f[ \hat{x}_{k} = \hat{x}^{-}_{k} + K_{k} \left( z_{k} - H \hat{x}^{-}_{k} \right) \f]
			 * \f[ P_{k} = \left( I - K_{k} H \right) P^{-}_{k} \f]
			 * 
			 * @param xPrior \f$ \hat{x}^{-}_{k} \f$
			 * @param pPrior \f$ P^{-}_{k} \f$
			 * @param h \f$ H \f$
			 * @param r \f$ R \f$
			 * @param z \f$ z_{k} \f$
			 * @param xPost \f$ \hat{x}_{k} \f$
			 * @param pPost \f$ P_{k} \f$
			 */
			template< typename Vector1, typename Matrix2, typename Matrix3, typename Matrix4, typename Vector5, typename Vector6, typename Matrix7 >
			static void correct(
				const Vector1& xPrior,
				const Matrix2& pPrior,
				const Matrix3& h,
				const Matrix4& r,
				const Vector5& z,
				Vector6& xPost,
				Matrix7& pPost
			)
			{
				// \f[ K_{k} = P^{-}_{k} H^{T} \left( H P^{-}_{k} H^{T} + R \right)^{-1} \f]
				Matrix k = pPrior * h.transpose() * (h * pPrior * h.transpose() + r).inverse();
				// \f[ \hat{x}_{k} = \hat{x}^{-}_{k} + K_{k} \left( z_{k} - H \hat{x}^{-}_{k} \right) \f]
				xPost = xPrior + k * (z - h * xPrior);
				// P_{k} = \left( I - K_{k} H \right) P^{-}_{k} \f]
				pPost = (Matrix::Identity(k.rows(), h.cols()) - k * h) * pPrior;
			}
			
			/**
			 * \f[ \hat{x}^{-}_{k} = A \hat{x}_{k - 1} \f]
			 * \f[ P^{-}_{k} = A P_{k - 1} A^{T} + Q \f]
			 * 
			 * @param xPost \f$ \hat{x}_{k - 1} \f$
			 * @param pPost \f$ P_{k - 1} \f$
			 * @param a \f$ A \f$
			 * @param q \f$ Q \f$
			 * @param xPrior \f$ \hat{x}^{-}_{k} \f$
			 * @param pPrior \f$ P^{-}_{k} \f$
			 */
			template< typename Vector1, typename Matrix2, typename Matrix3, typename Matrix4, typename Vector5, typename Matrix6 >
			static void predict(
				const Vector1& xPost,
				const Matrix2& pPost,
				const Matrix3& a,
				const Matrix4& q,
				Vector5& xPrior,
				Matrix6& pPrior
			)
			{
				// \f[ \hat{x}^{-}_{k} = A \hat{x}_{k - 1} \f]
				xPrior = a * xPost;
				// \f[ P^{-}_{k} = A P_{k - 1} A^{T} + Q \f]
				pPrior = a * pPost * a.transpose() + q;
			}
			
			/**
			 * \f[ \hat{x}^{-}_{k} = A \hat{x}_{k - 1} + B u_{k - 1} \f]
			 * \f[ P^{-}_{k} = A P_{k - 1} A^{T} + Q \f]
			 * 
			 * @param xPost \f$ \hat{x}_{k - 1} \f$
			 * @param pPost \f$ P_{k - 1} \f$
			 * @param a \f$ A \f$
			 * @param b \f$ B \f$
			 * @param u \f$ U \f$
			 * @param q \f$ Q \f$
			 * @param xPrior \f$ \hat{x}^{-}_{k} \f$
			 * @param pPrior \f$ P^{-}_{k} \f$
			 */
			template< typename Vector1, typename Matrix2, typename Matrix3, typename Matrix4, typename Vector5, typename Matrix6, typename Vector7, typename Matrix8 >
			static void predict(
				const Vector1& xPost,
				const Matrix2& pPost,
				const Matrix3& a,
				const Matrix4& b,
				const Vector5& u,
				const Matrix6& q,
				Vector7& xPrior,
				Matrix8& pPrior
			)
			{
				// \f[ \hat{x}^{-}_{k} = A \hat{x}_{k - 1} + B u_{k - 1} \f]
				xPrior = a * xPost + b * u;
				// \f[ P^{-}_{k} = A P_{k - 1} A^{T} + Q \f]
				pPrior = a * pPost * a.transpose() + q;
			}
			
		protected:
			
		private:
			
		};
	}
}

#endif // _RL_MATH_KALMAN_H_
