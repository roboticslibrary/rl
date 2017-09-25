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

#ifndef RL_MATH_VECTOR_H
#define RL_MATH_VECTOR_H

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <Eigen/Core>

#include "Real.h"

namespace rl
{
	namespace math
	{
		typedef ::Eigen::Matrix<Real, ::Eigen::Dynamic, 1> Vector;
		
		typedef ::Eigen::Matrix<Real, 2, 1> Vector2;
		
		typedef ::Eigen::Matrix<Real, 3, 1> Vector3;
		
		typedef ::Eigen::Matrix<Real, 4, 1> Vector4;
		
		typedef ::Eigen::Matrix<Real, 6, 1> Vector6;
		
		typedef ::Eigen::VectorBlock<Vector, ::Eigen::Dynamic> VectorBlock;
		
		typedef ::Eigen::VectorBlock<Vector2, ::Eigen::Dynamic> Vector2Block;
		
		typedef ::Eigen::VectorBlock<Vector3, ::Eigen::Dynamic> Vector3Block;
		
		typedef ::Eigen::VectorBlock<Vector4, ::Eigen::Dynamic> Vector4Block;
		
		typedef ::Eigen::VectorBlock<Vector6, ::Eigen::Dynamic> Vector6Block;
		
		typedef ::Eigen::VectorBlock<const Vector, ::Eigen::Dynamic> ConstVectorBlock;
		
		typedef ::Eigen::VectorBlock<const Vector2, ::Eigen::Dynamic> ConstVector2Block;
		
		typedef ::Eigen::VectorBlock<const Vector3, ::Eigen::Dynamic> ConstVector3Block;
		
		typedef ::Eigen::VectorBlock<const Vector4, ::Eigen::Dynamic> ConstVector4Block;
		
		typedef ::Eigen::VectorBlock<const Vector6, ::Eigen::Dynamic> ConstVector6Block;
		
		typedef ::Eigen::Ref<Vector> VectorRef;
		
		typedef ::Eigen::Ref<const Vector> ConstVectorRef;
	}
}

#endif // RL_MATH_VECTOR_H
