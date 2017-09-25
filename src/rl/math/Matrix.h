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

#ifndef RL_MATH_MATRIX_H
#define RL_MATH_MATRIX_H

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <Eigen/Core>

#include "Real.h"

namespace rl
{
	namespace math
	{
		typedef ::Eigen::Matrix<Real, ::Eigen::Dynamic, ::Eigen::Dynamic> Matrix;
		
		typedef ::Eigen::Matrix<Real, 2, 2> Matrix22;
		
		typedef ::Eigen::Matrix<Real, 3, 3> Matrix33;
		
		typedef ::Eigen::Matrix<Real, 4, 4> Matrix44;
		
		typedef ::Eigen::Matrix<Real, 6, 6> Matrix66;
		
		typedef ::Eigen::Block<Matrix> MatrixBlock;
		
		typedef ::Eigen::Block<Matrix22> Matrix22Block;
		
		typedef ::Eigen::Block<Matrix33> Matrix33Block;
		
		typedef ::Eigen::Block<Matrix44> Matrix44Block;
		
		typedef ::Eigen::Block<Matrix66> Matrix66Block;
		
		typedef Matrix::ColXpr MatrixColumn;
		
		typedef Matrix22::ColXpr Matrix22Column;
		
		typedef Matrix33::ColXpr Matrix33Column;
		
		typedef Matrix44::ColXpr Matrix44Column;
		
		typedef Matrix66::ColXpr Matrix66Column;
		
		typedef Matrix::RowXpr MatrixRow;
		
		typedef Matrix22::RowXpr Matrix22Row;
		
		typedef Matrix33::RowXpr Matrix33Row;
		
		typedef Matrix44::RowXpr Matrix44Row;
		
		typedef Matrix66::RowXpr Matrix66Row;
		
		typedef ::Eigen::Block<const Matrix> ConstMatrixBlock;
		
		typedef ::Eigen::Block<const Matrix22> ConstMatrix22Block;
		
		typedef ::Eigen::Block<const Matrix33> ConstMatrix33Block;
		
		typedef ::Eigen::Block<const Matrix44> ConstMatrix44Block;
		
		typedef ::Eigen::Block<const Matrix66> ConstMatrix66Block;
		
		typedef Matrix::ConstColXpr ConstMatrixColumn;
		
		typedef Matrix22::ConstColXpr ConstMatrix22Column;
		
		typedef Matrix33::ConstColXpr ConstMatrix33Column;
		
		typedef Matrix44::ConstColXpr ConstMatrix44Column;
		
		typedef Matrix66::ConstColXpr ConstMatrix66Column;
		
		typedef Matrix::ConstRowXpr ConstMatrixRow;
		
		typedef Matrix22::ConstRowXpr ConstMatrix22Row;
		
		typedef Matrix33::ConstRowXpr ConstMatrix33Row;
		
		typedef Matrix44::ConstRowXpr ConstMatrix44Row;
		
		typedef Matrix66::ConstRowXpr ConstMatrix66Row;
		
		typedef ::Eigen::Ref<Matrix> MatrixRef;
		
		typedef ::Eigen::Ref<const Matrix> ConstMatrixRef;
	}
}

#endif // RL_MATH_MATRIX_H
