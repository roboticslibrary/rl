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

#ifndef RL_MATH_ARTICULATEDBODYINERTIA_HXX
#define RL_MATH_ARTICULATEDBODYINERTIA_HXX

namespace rl
{
	namespace math
	{
		namespace spatial
		{
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ArticulatedBodyInertia<Scalar>&
			ArticulatedBodyInertia<Scalar>::operator=(const RigidBodyInertia<OtherScalar>& other)
			{
				cog() = other.cog().cross33();
				inertia() = other.inertia();
				mass() = ::Eigen::Matrix<Scalar, 3, 3>::Identity() * other.mass();
				return *this;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ArticulatedBodyInertia<Scalar>&
			ArticulatedBodyInertia<Scalar>::operator+=(const RigidBodyInertia<OtherScalar>& other)
			{
				cog() += other.cog().cross33();
				inertia() += other.inertia();
				mass() += ::Eigen::Matrix<Scalar, 3, 3>::Identity() * other.mass();
				return *this;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ArticulatedBodyInertia<Scalar>&
			ArticulatedBodyInertia<Scalar>::operator-=(const RigidBodyInertia<OtherScalar>& other)
			{
				cog() -= other.cog().cross33();
				inertia() -= other.inertia();
				mass() -= ::Eigen::Matrix<Scalar, 3, 3>::Identity() * other.mass();
				return *this;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ArticulatedBodyInertia<Scalar>
			ArticulatedBodyInertia<Scalar>::operator+(const RigidBodyInertia<OtherScalar>& other) const
			{
				ArticulatedBodyInertia res;
				res.cog() = cog() + other.cog().cross33();
				res.inertia() = inertia() + other.inertia();
				res.mass() = mass() + ::Eigen::Matrix<Scalar, 3, 3>::Identity() * other.mass();
				return res;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ArticulatedBodyInertia<Scalar>
			ArticulatedBodyInertia<Scalar>::operator-(const RigidBodyInertia<OtherScalar>& other) const
			{
				ArticulatedBodyInertia res;
				res.cog() = cog() - other.cog().cross33();
				res.inertia() = inertia() - other.inertia();
				res.mass() = mass() - ::Eigen::Matrix<Scalar, 3, 3>::Identity() * other.mass();
				return res;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ForceVector<Scalar>
			ArticulatedBodyInertia<Scalar>::operator*(const MotionVector<OtherScalar>& other) const
			{
				ForceVector<Scalar> res;
				res.moment() = inertia() * other.angular() + cog() * other.linear();
				res.force() = mass() * other.linear() + cog().transpose() * other.angular();
				return res;
			}
		}
	}
}

#endif // RL_MATH_ARTICULATEDBODYINERTIA_HXX
