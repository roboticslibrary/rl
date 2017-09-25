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

#ifndef RL_MATH_TYPETRAITS_H
#define RL_MATH_TYPETRAITS_H

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <limits>
#include <Eigen/Core>

namespace rl
{
	namespace math
	{
		template<typename T>
		class TypeTraits
		{
		public:
			static T Constant(const ::std::size_t& i, const T& value)
			{
				return value;
			}
			
			static T Zero(const ::std::size_t& i)
			{
				return 0;
			}
			
			static T abs(const T& t)
			{
				return ::std::abs(t);
			}
			
			static bool equal(const T& lhs, const T& rhs, const T& epsilon = ::Eigen::NumTraits<T>::dummy_precision())
			{
				return ::std::abs(lhs - rhs) < epsilon;
			}
			
			static T max_element(const T& t)
			{
				return t;
			}
			
			static T min_element(const T& t)
			{
				return t;
			}
			
			static ::std::size_t size(const T& t)
			{
				return 1;
			}
			
		protected:
			
		private:
			
		};
		
		template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
		class TypeTraits< ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
		{
		public:
			static ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols> Constant(const ::std::size_t& i, const Scalar& value)
			{
				return ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Constant(i, value);
			}
			
			static ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols> Zero(const ::std::size_t& i)
			{
				return ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Zero(i);
			}
			
			static ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols> abs(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t)
			{
				return t.abs();
			}
			
			static bool equal(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& lhs, const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& rhs, const Scalar& epsilon = ::Eigen::NumTraits<Scalar>::dummy_precision())
			{
				return lhs.isApprox(rhs, epsilon);
			}
			
			static Scalar max_element(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t)
			{
				return t.maxCoeff();
			}
			
			static Scalar min_element(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t)
			{
				return t.minCoeff();
			}
			
			static ::std::size_t size(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t)
			{
				return t.size();
			}
			
		protected:
			
		private:
			
		};
		
		template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
		class TypeTraits< ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
		{
		public:
			static ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> Constant(const ::std::size_t& i, const Scalar& value)
			{
				return ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Constant(i, value);
			}
			
			static ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> Zero(const ::std::size_t& i)
			{
				return ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Zero(i);
			}
			
			static ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> abs(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t)
			{
				return t.cwiseAbs();
			}
			
			static bool equal(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& lhs, const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& rhs, const Scalar& epsilon = ::Eigen::NumTraits<Scalar>::dummy_precision())
			{
				return lhs.isApprox(rhs, epsilon);
			}
			
			static Scalar max_element(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t)
			{
				return t.maxCoeff();
			}
			
			static Scalar min_element(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t)
			{
				return t.minCoeff();
			}
			
			static ::std::size_t size(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t)
			{
				return t.size();
			}
			
		protected:
			
		private:
			
		};
	}
}

#endif // RL_MATH_TYPETRAITS_H
