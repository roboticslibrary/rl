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

#ifndef RL_MATH_ARTICULATEDBODYINERTIA_H
#define RL_MATH_ARTICULATEDBODYINERTIA_H

#include <Eigen/Core>

namespace rl
{
	namespace math
	{
		namespace spatial
		{
			template<typename Scalar> class ForceVector;
			template<typename Scalar> class MotionVector;
			template<typename Scalar> class RigidBodyInertia;
			
			/**
			 * Articulated-body inertia.
			 */
			template<typename Scalar>
			class ArticulatedBodyInertia
			{
			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				
				typedef Scalar ScalarType;
				
				typedef ::Eigen::Matrix<Scalar, 6, 6> MatrixType;
				
				typedef ::Eigen::Matrix<Scalar, 3, 3> CenterOfGravityType;
				
				typedef const CenterOfGravityType ConstCenterOfGravityType;
				
				typedef ::Eigen::Matrix<Scalar, 3, 3> InertiaType;
				
				typedef const InertiaType ConstInertiaType;
				
				typedef ::Eigen::Matrix<Scalar, 3, 3> MassType;
				
				typedef const MassType ConstMassType;
				
				ArticulatedBodyInertia()
				{
				}
				
				template<typename OtherDerived>
				ArticulatedBodyInertia(const ::Eigen::DenseBase<OtherDerived>& other) :
					centerOfGravityData(other.template topRightCorner<3, 3>()),
					inertiaData(other.template topLeftCorner<3, 3>()),
					massData(other.template bottomRightCorner<3, 3>())
				{
				}
				
				virtual ~ArticulatedBodyInertia()
				{
				}
				
				CenterOfGravityType& cog()
				{
					return centerOfGravityData;
				}
				
				ConstCenterOfGravityType& cog() const
				{
					return centerOfGravityData;
				}
				
				InertiaType& inertia()
				{
					return inertiaData;
				}
				
				ConstInertiaType& inertia() const
				{
					return inertiaData;
				}
				
				MassType& mass()
				{
					return massData;
				}
				
				ConstMassType& mass() const
				{
					return massData;
				}
				
				MatrixType matrix() const
				{
					MatrixType res;
					res.template topLeftCorner<3, 3>() = inertia();
					res.template topRightCorner<3, 3>() = cog();
					res.template bottomLeftCorner<3, 3>() = cog().transpose();
					res.template bottomRightCorner<3, 3>() = mass();
					return res;
				}
				
				template<typename OtherDerived>
				ArticulatedBodyInertia& operator=(const ::Eigen::MatrixBase<OtherDerived>& other)
				{
					cog() = other.template topRightCorner<3, 3>();
					inertia() = other.template topLeftCorner<3, 3>();
					mass() = other.template bottomRightCorner<3, 3>();
					return *this;
				}
				
				template<typename OtherScalar>
				ArticulatedBodyInertia& operator=(const RigidBodyInertia<OtherScalar>& other);
				
				ArticulatedBodyInertia operator+(const ArticulatedBodyInertia& other) const
				{
					ArticulatedBodyInertia res;
					res.cog() = cog() + other.cog();
					res.inertia() = inertia() + other.inertia();
					res.mass() = mass() + other.mass();
					return res;
				}
				
				ArticulatedBodyInertia operator-(const ArticulatedBodyInertia& other) const
				{
					ArticulatedBodyInertia res;
					res.cog() = cog() - other.cog();
					res.inertia() = inertia() - other.inertia();
					res.mass() = mass() - other.mass();
					return res;
				}
				
				template<typename OtherScalar>
				ArticulatedBodyInertia operator*(const OtherScalar& other) const
				{
					ArticulatedBodyInertia res;
					res.cog() = cog() * other;
					res.inertia() = inertia() * other;
					res.mass() = mass() * other;
					return res;
				}
				
				template<typename OtherScalar>
				ForceVector<Scalar> operator*(const MotionVector<OtherScalar>& other) const;
				
				template<typename OtherScalar>
				ArticulatedBodyInertia operator/(const OtherScalar& other) const
				{
					ArticulatedBodyInertia res;
					res.cog() = cog() / other;
					res.inertia() = inertia() / other;
					res.mass() = mass() / other;
					return res;
				}
				
				void setIdentity()
				{
					cog().setZero();
					inertia().setIdentity();
					mass().setIdentity();
				}
				
				void setZero()
				{
					cog().setZero();
					inertia().setZero();
					mass().setZero();
				}
				
			protected:
				
			private:
				CenterOfGravityType centerOfGravityData;
				
				InertiaType inertiaData;
				
				MassType massData;
			};
		}
	}
}

#endif // RL_MATH_ARTICULATEDBODYINERTIA_H
