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

#ifndef RL_MATH_KALMAN_H
#define RL_MATH_KALMAN_H

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <Eigen/Core>
#include <Eigen/LU>

namespace rl
{
	namespace math
	{
		/**
		 * Kalman filter.
		 * 
		 * Greg Welch and Gary Bishop. An introduction to the Kalman filter. Technical
		 * Report TR 95-041, University of North Carolina at Chapel Hill, Chapel Hill,
		 * NC, USA, July 2006.
		 * 
		 * http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
		 */
		template<typename Scalar>
		class Kalman
		{
		public:
			typedef Scalar ScalarType;
			
			typedef typename ::Eigen::Matrix<Scalar, ::Eigen::Dynamic, ::Eigen::Dynamic> MatrixType;
			
			typedef typename ::Eigen::Matrix<Scalar, ::Eigen::Dynamic, 1> VectorType;
			
			Kalman(const ::std::size_t& states, const ::std::size_t& observations, const ::std::size_t& controls = 0) :
				A(MatrixType::Identity(states, states)),
				B(MatrixType::Zero(states, controls)),
				H(MatrixType::Zero(observations, states)),
				PPosteriori(MatrixType::Zero(states, states)),
				PPriori(MatrixType::Zero(states, states)),
				Q(MatrixType::Identity(states, states)),
				R(MatrixType::Identity(observations, observations)),
				xPosteriori(VectorType::Zero(states)),
				xPriori(VectorType::Zero(states))
			{
			}
			
			virtual ~Kalman()
			{
			}
			
			MatrixType& controlModel()
			{
				return this->B;
			}
			
			const MatrixType& controlModel() const
			{
				return this->B;
			}
			
			/**
			 * Measurement update ("correct").
			 * 
			 * Compute the Kalman gain
			 * \f[ \matr{K}_{k} = \matr{P}^{-}_{k} \matr{H}^{\mathrm{T}} \left( \matr{H} \matr{P}^{-}_{k} \matr{H}^{\mathrm{T}} + \matr{R} \right)^{-1} \f]
			 * Update estimate with measurement \f$\vec{z}_{k}\f$
			 * \f[ \hat{\vec{x}}_{k} = \hat{\vec{x}}^{-}_{k} + \matr{K}_{k} \left( \vec{z}_{k} - \matr{H} \hat{\vec{x}}^{-}_{k} \right) \f]
			 * Update the error covariance
			 * \f[ \matr{P}_{k} = \left( \matr{1} - \matr{K}_{k} \matr{H} \right) \matr{P}^{-}_{k} \f]
			 * 
			 * @param[in] z Measurement \f$\vec{z}_{k}\f$
			 */
			VectorType correct(const VectorType& z)
			{
				assert(z.size() == xPriori.size());
				
				// \f[ \matr{K}_{k} = \matr{P}^{-}_{k} \matr{H}^{\mathrm{T}} \left( \matr{H} \matr{P}^{-}_{k} \matr{H}^{\mathrm{T}} + \matr{R} \right)^{-1} \f]
				MatrixType K = PPriori * H.transpose() * (H * PPriori * H.transpose() + R).inverse();
				// \f[ \hat{\vec{x}}_{k} = \hat{\vec{x}}^{-}_{k} + \matr{K}_{k} \left( \vec{z}_{k} - \matr{H} \hat{\vec{x}}^{-}_{k} \right) \f]
				xPosteriori = xPriori + K * (z - H * xPriori);
				// \f[ \matr{P}_{k} = \left( \matr{1} - \matr{K}_{k} \matr{H} \right) \matr{P}^{-}_{k} \f]
				PPosteriori = (MatrixType::Identity(K.rows(), H.cols()) - K * H) * PPriori;
				return xPosteriori;
			}
			
			MatrixType& errorCovariancePosteriori()
			{
				return this->PPosteriori;
			}
			
			const MatrixType& errorCovariancePosteriori() const
			{
				return this->PPosteriori;
			}
			
			MatrixType& errorCovariancePriori()
			{
				return this->PPriori;
			}
			
			const MatrixType& errorCovariancePriori() const
			{
				return this->PPriori;
			}
			
			MatrixType& measurementModel()
			{
				return this->H;
			}
			
			const MatrixType& measurementModel() const
			{
				return this->H;
			}
			
			MatrixType& measurementNoiseCovariance()
			{
				return this->R;
			}
			
			const MatrixType& measurementNoiseCovariance() const
			{
				return this->R;
			}
			
			/**
			 * Time update ("predict") without control input.
			 * 
			 * Project the state ahead
			 * \f[ \hat{\vec{x}}^{-}_{k} = \matr{A} \hat{\vec{x}}_{k - 1} \f]
			 * Project the error covariance ahead
			 * \f[ \matr{P}^{-}_{k} = \matr{A} \matr{P}_{k - 1} \matr{A}^{\mathrm{T}} + \matr{Q} \f]
			 */
			VectorType predict()
			{
				// \f[ \hat{\vec{x}}^{-}_{k} = \matr{A} \hat{\vec{x}}_{k - 1} \f]
				xPriori = A * xPosteriori;
				// \f[ \matr{P}^{-}_{k} = \matr{A} \matr{P}_{k - 1} \matr{A}^{\mathrm{T}} + \matr{Q} \f]
				PPriori = A * PPosteriori * A.transpose() + Q;
				return xPriori;
			}
			
			/**
			 * Time update ("predict") with control input.
			 * 
			 * Project the state ahead
			 * \f[ \hat{\vec{x}}^{-}_{k} = \matr{A} \hat{\vec{x}}_{k - 1} + \matr{B} \vec{u}_{k - 1} \f]
			 * Project the error covariance ahead
			 * \f[ \matr{P}^{-}_{k} = \matr{A} \matr{P}_{k - 1} \matr{A}^{\mathrm{T}} + \matr{Q} \f]
			 * 
			 * @param[in] u Control input \f$\vec{u}_{k - 1}\f$
			 */
			VectorType predict(const VectorType& u)
			{
				assert(u.size() == B.cols());
				
				// \f[ \hat{\vec{x}}^{-}_{k} = \matr{A} \hat{\vec{x}}_{k - 1} + \matr{B} \vec{u}_{k - 1} \f]
				xPriori = A * xPosteriori + B * u;
				// \matr{P}^{-}_{k} = \matr{A} \matr{P}_{k - 1} \matr{A}^{\mathrm{T}} + \matr{Q} \f]
				PPriori = A * PPosteriori * A.transpose() + Q;
				return xPriori;
			}
			
			MatrixType& processNoiseCovariance()
			{
				return this->Q;
			}
			
			const MatrixType& processNoiseCovariance() const
			{
				return this->Q;
			}
			
			VectorType& statePosteriori()
			{
				return this->xPosteriori;
			}
			
			const VectorType& statePosteriori() const
			{
				return this->xPosteriori;
			}
			
			VectorType& statePriori()
			{
				return this->xPriori;
			}
			
			const VectorType& statePriori() const
			{
				return this->xPriori;
			}
			
			MatrixType& stateTransitionModel()
			{
				return this->A;
			}
			
			const MatrixType& stateTransitionModel() const
			{
				return this->A;
			}
			
		protected:
			
		private:
			/** \f$\matr{A}\f$ relates the state at the previous time step \f$k - 1\f$ to the state at the current step \f$k\f$. */
			MatrixType A;
			
			/** \f$\matr{B}\f$ relates the control input \f$\vec{u}\f$ to the state \f$\vec{x}\f$. */
			MatrixType B;
			
			/** \f$\matr{H}\f$ relates the state to the measurement \f$\vec{z}_{k}\f$. */
			MatrixType H;
			
			/** A posteriori estimate error covariance \f$\matr{P}_{k - 1}\f$. */
			MatrixType PPosteriori;
			
			/** A priori estimate error covariance \f$\matr{P}^{-}_{k}\f$. */
			MatrixType PPriori;
			
			/** Process noise covariance \f$\matr{Q}\f$. */
			MatrixType Q;
			
			/** Measurement error covariance \f$\matr{R}\f$. */
			MatrixType R;
			
			/** A posteriori state estimate \f$\hat{\vec{x}}_{k - 1}\f$. */
			VectorType xPosteriori;
			
			/** A priori state estimate \f$\hat{\vec{x}}^{-}_{k}\f$. */
			VectorType xPriori;
		};
	}
}

#endif // RL_MATH_KALMAN_H
