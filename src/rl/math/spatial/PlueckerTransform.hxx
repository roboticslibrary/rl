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

#ifndef RL_MATH_PLUECKERTRANSFORM_HXX
#define RL_MATH_PLUECKERTRANSFORM_HXX

namespace rl
{
	namespace math
	{
		namespace spatial
		{
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ForceVector<OtherScalar>
			PlueckerTransform<Scalar>::operator*(const ForceVector<OtherScalar>& other) const
			{
				ForceVector<OtherScalar> res;
				res.force() = rotation() * other.force();
				res.moment() = rotation() * (other.moment() - translation().cross(other.force()));
				return res;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			MotionVector<OtherScalar>
			PlueckerTransform<Scalar>::operator*(const MotionVector<OtherScalar>& other) const
			{
				MotionVector<OtherScalar> res;
				res.linear() = rotation() * (other.linear() - translation().cross(other.angular()));
				res.angular() = rotation() * other.angular();
				return res;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			RigidBodyInertia<OtherScalar>
			PlueckerTransform<Scalar>::operator*(const RigidBodyInertia<OtherScalar>& other) const
			{
				RigidBodyInertia<OtherScalar> res;
				res.cog() = rotation() * (other.cog() - other.mass() * translation());
				res.inertia() = rotation() * (other.inertia() + translation().cross(other.cog()).cross33() + (other.cog() - other.mass() * translation()).cross(translation()).cross33()) * rotation().transpose();
				res.mass() = other.mass();
				return res;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ArticulatedBodyInertia<OtherScalar>
			PlueckerTransform<Scalar>::operator*(const ArticulatedBodyInertia<OtherScalar>& other) const
			{
				ArticulatedBodyInertia<OtherScalar> res;
				res.cog() = rotation() * (other.cog() - translation().cross33() * other.mass()) * rotation().transpose();
				res.inertia() = rotation() * (other.inertia() - translation().cross33() * other.cog().transpose() + (other.cog() - translation().cross33() * other.mass()) * translation().cross33()) * rotation().transpose();
				res.mass() = rotation() * other.mass() * rotation().transpose();
				return res;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ForceVector<OtherScalar>
			PlueckerTransform<Scalar>::operator/(const ForceVector<OtherScalar>& other) const
			{
				ForceVector<OtherScalar> res;
				res.force() = rotation().transpose() * other.force();
				res.moment() = rotation().transpose() * other.moment() + translation().cross(rotation().transpose() * other.force());
				return res;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			MotionVector<OtherScalar>
			PlueckerTransform<Scalar>::operator/(const MotionVector<OtherScalar>& other) const
			{
				MotionVector<OtherScalar> res;
				res.linear() = rotation().transpose() * other.linear() + translation().cross(rotation().transpose() * other.angular());
				res.angular() = rotation().transpose() * other.angular();
				return res;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			RigidBodyInertia<OtherScalar>
			PlueckerTransform<Scalar>::operator/(const RigidBodyInertia<OtherScalar>& other) const
			{
				RigidBodyInertia<OtherScalar> res;
				res.cog() = rotation().transpose() * other.cog() + other.mass() * translation();
				res.inertia() = rotation().transpose() * other.inertia() * rotation() - translation().cross(rotation().transpose() * other.cog()).cross33() - (rotation().transpose() * other.cog() + other.mass() * translation()).cross(translation()).cross33();
				res.mass() = other.mass();
				return res;
			}
			
			template<typename Scalar>
			template<typename OtherScalar>
			inline
			ArticulatedBodyInertia<OtherScalar>
			PlueckerTransform<Scalar>::operator/(const ArticulatedBodyInertia<OtherScalar>& other) const
			{
				ArticulatedBodyInertia<OtherScalar> res;
				typename ArticulatedBodyInertia<OtherScalar>::CenterOfGravityType cog = rotation().transpose() * other.cog() * rotation();
				typename ArticulatedBodyInertia<OtherScalar>::MassType mass = rotation().transpose() * other.mass() * rotation();
				res.cog() = cog + translation().cross33() * mass;
				res.inertia() = rotation().transpose() * other.inertia() * rotation() + translation().cross33() * cog.transpose() - (cog + translation().cross33() * mass) * translation().cross33();
				res.mass() = mass;
				return res;
			}
		}
	}
}

#endif // RL_MATH_PLUECKERTRANSFORM_HXX
