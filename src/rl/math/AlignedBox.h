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

#ifndef RL_MATH_ALIGNEDBOX_H
#define RL_MATH_ALIGNEDBOX_H

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <Eigen/Geometry>

#include "Real.h"

namespace rl
{
	namespace math
	{
		template<typename _Scalar, int _AmbientDim>
		class AlignedBox : public ::Eigen::AlignedBox<_Scalar, _AmbientDim>
		{
		public:
			typedef typename ::Eigen::AlignedBox<_Scalar, _AmbientDim>::Scalar Scalar;
			
			typedef typename ::Eigen::AlignedBox<_Scalar, _AmbientDim>::ScalarTraits ScalarTraits;
			
			typedef typename ::Eigen::AlignedBox<_Scalar, _AmbientDim>::Index Index;
			
			typedef typename ::Eigen::AlignedBox<_Scalar, _AmbientDim>::RealScalar RealScalar;
			
			typedef typename ::Eigen::AlignedBox<_Scalar, _AmbientDim>::NonInteger NonInteger;
			
			typedef typename ::Eigen::AlignedBox<_Scalar, _AmbientDim>::VectorType VectorType;
			
			using ::Eigen::AlignedBox<_Scalar, _AmbientDim>::AlignedBox;
			
			template<typename Derived>
			inline NonInteger interiorDistance(const ::Eigen::MatrixBase<Derived>& p) const
			{
				using ::std::max;
				using ::std::min;
				
				Scalar dist(::Eigen::NumTraits<Scalar>::highest());
				
				for (Index k = 0; k < this->dim(); ++k)
				{
					dist = min(dist, max(p[k] - this->m_min[k], Scalar(0)));
					dist = min(dist, max(this->m_max[k] - p[k], Scalar(0)));
				}
				
				return dist;
			}
			
			template<typename Derived>
			inline Scalar squaredInteriorDistance(const ::Eigen::MatrixBase<Derived>& p) const
			{
				using ::std::pow;
				return pow(this->squaredInteriorDistance(p), 2);
			}
			
		protected:
			
		private:
			
		};
		
		typedef AlignedBox<Real, ::Eigen::Dynamic> AlignedBoxX;
		
		typedef AlignedBox<Real, 2> AlignedBox2;
		
		typedef AlignedBox<Real, 3> AlignedBox3;
	}
}

#endif // RL_MATH_ALIGNEDBOX_H
