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

#ifndef RL_MATH_MOTIONVECTOR_H
#define RL_MATH_MOTIONVECTOR_H

#include <Eigen/Core>

namespace rl
{
	namespace math
	{
		namespace spatial
		{
			template<typename Scalar> class ForceVector;
			
			/**
			 * Motion vector.
			 */
			template<typename Scalar>
			class MotionVector
			{
			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				
				typedef Scalar ScalarType;
				
				typedef typename ::Eigen::Matrix<Scalar, 6, 6> CrossType;
				
				typedef typename ::Eigen::Matrix<Scalar, 6, 1> MatrixType;
				
				typedef const MatrixType ConstMatrixType;
				
				typedef ::Eigen::Block<MatrixType, 3, 1> AngularType;
				
				typedef const ::Eigen::Block<ConstMatrixType, 3, 1> ConstAngularType;
				
				typedef ::Eigen::Block<MatrixType, 3, 1> LinearType;
				
				typedef const ::Eigen::Block<ConstMatrixType, 3, 1> ConstLinearType;
				
				MotionVector()
				{
				}
				
				template<typename OtherDerived>
				MotionVector(const ::Eigen::MatrixBase<OtherDerived>& other) :
					data(other)
				{
				}
				
				virtual ~MotionVector()
				{
				}
				
				static MotionVector Zero()
				{
					return MotionVector(MatrixType::Zero());
				}
				
				AngularType angular()
				{
					return data.template segment<3>(0);
				}
				
				ConstAngularType angular() const
				{
					return data.template segment<3>(0);
				}
				
				template<typename OtherScalar>
				ForceVector<OtherScalar> cross(const ForceVector<OtherScalar>& other) const;
				
				MotionVector cross(const MotionVector& other) const
				{
					MotionVector res;
					res.angular() = angular().cross(other.angular());
					res.linear() = angular().cross(other.linear()) + linear().cross(other.angular());
					return res;
				}
				
				CrossType cross66Force() const
				{
					CrossType res;
					res.template topLeftCorner<3, 3>() = angular().cross33();
					res.template topRightCorner<3, 3>() = linear().cross33();
					res.template bottomLeftCorner<3, 3>().setZero();
					res.template bottomRightCorner<3, 3>() = angular().cross33();
					return res;
				}
				
				CrossType cross66Motion() const
				{
					CrossType res;
					res.template topLeftCorner<3, 3>() = angular().cross33();
					res.template topRightCorner<3, 3>().setZero();
					res.template bottomLeftCorner<3, 3>() = linear().cross33();
					res.template bottomRightCorner<3, 3>() = angular().cross33();
					return res;
				}
				
				template<typename OtherScalar>
				Scalar dot(const ForceVector<OtherScalar>& other) const;
				
				LinearType linear()
				{
					return data.template segment<3>(3);
				}
				
				ConstLinearType linear() const
				{
					return data.template segment<3>(3);
				}
				
				template<typename OtherScalar>
				bool isApprox(const MotionVector<OtherScalar>& other, const typename ::Eigen::NumTraits<Scalar>::Real& prec = ::Eigen::NumTraits<Scalar>::dummy_precision()) const
				{
					return matrix().isApprox(other.matrix(), prec);
				}
				
				ConstMatrixType& matrix() const
				{
					return data;
				}
				
				template<typename OtherDerived>
				MotionVector& operator=(const ::Eigen::MatrixBase<OtherDerived>& other)
				{
					data = other;
					return *this;
				}
				
				template<typename OtherScalar>
				MotionVector& operator+=(const MotionVector<OtherScalar>& other)
				{
					angular() += other.angular();
					linear() += other.linear();
					return *this;
				}
				
				template<typename OtherScalar>
				MotionVector& operator-=(const MotionVector<OtherScalar>& other)
				{
					angular() -= other.angular();
					linear() -= other.linear();
					return *this;
				}
				
				template<typename OtherScalar>
				MotionVector& operator*=(const OtherScalar& other)
				{
					angular() *= other;
					linear() *= other;
					return *this;
				}
				
				template<typename OtherScalar>
				MotionVector& operator/=(const OtherScalar& other)
				{
					angular() /= other;
					linear() /= other;
					return *this;
				}
				
				template<typename OtherScalar>
				MotionVector operator+(const MotionVector<OtherScalar>& other) const
				{
					MotionVector res;
					res.angular() = angular() + other.angular();
					res.linear() = linear() + other.linear();
					return res;
				}
				
				MotionVector operator-() const
				{
					MotionVector res;
					res.angular() = -angular();
					res.linear() = -linear();
					return res;
				}
				
				template<typename OtherScalar>
				MotionVector operator-(const MotionVector<OtherScalar>& other) const
				{
					MotionVector res;
					res.angular() = angular() - other.angular();
					res.linear() = linear() - other.linear();
					return res;
				}
				
				template<typename OtherScalar>
				MotionVector operator*(const OtherScalar& other) const
				{
					MotionVector res;
					res.angular() = angular() * other;
					res.linear() = linear() * other;
					return res;
				}
				
				template<typename OtherScalar>
				MotionVector operator/(const OtherScalar& other) const
				{
					MotionVector res;
					res.angular() = angular() / other;
					res.linear() = linear() / other;
					return res;
				}
				
				void setZero()
				{
					angular().setZero();
					linear().setZero();
				}
				
			protected:
				
			private:
				MatrixType data;
			};
		}
	}
}

#endif // RL_MATH_MOTIONVECTOR_H
