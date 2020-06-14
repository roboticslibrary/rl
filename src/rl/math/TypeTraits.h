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

#include <algorithm>
#include <iterator>
#include <limits>
#include <Eigen/Core>
#include <rl/std/iterator.h>

namespace rl
{
	namespace math
	{
		template<typename T, typename Enable = void>
		class TypeTraits
		{
		public:
			typedef typename T::value_type value_type;
			
			static T Constant(const ::std::size_t& i, const T& value)
			{
				return T(i, value);
			}
			
			static T Zero(const ::std::size_t& i)
			{
				return T(i, 0);
			}
			
			static T abs(const T& t)
			{
				using ::std::abs;
				using ::rl::std17::size;
				using ::std::transform;
				T res(size(t));
				transform(t.begin(), t.end(), res.begin(), static_cast<value_type(*)(value_type)>(&abs));
				return res;
			}
			
			static bool equal(const T& lhs, const T& rhs, const value_type& epsilon = ::Eigen::NumTraits<value_type>::dummy_precision())
			{
				using ::std::abs;
				using ::std::begin;
				using ::std::end;
				using ::std::min;
				
				auto first1 = begin(lhs);
				auto last1 = end(lhs);
				auto first2 = begin(rhs);
				
				value_type norm = value_type();
				value_type norm1 = value_type();
				value_type norm2 = value_type();
				
				while (first1 != last1)
				{
					value_type tmp = abs(*first1 - *first2);
					norm += tmp * tmp;
					value_type tmp1 = abs(*first1);
					norm1 += tmp1 * tmp1;
					value_type tmp2 = abs(*first2);
					norm2 += tmp2 * tmp2;
					++first1;
					++first2;
				}
				
				return norm <= epsilon * epsilon * min(norm1, norm2);
			}
			
			static T max_element(const T& t)
			{
				using ::std::max_element;
				return max_element(t.begin(), t.end());
			}
			
			static T min_element(const T& t)
			{
				using ::std::min_element;
				return min_element(t.begin(), t.end());
			}
			
			static ::std::size_t size(const T& t)
			{
				using ::rl::std17::size;
				return size(t);
			}
			
		protected:
			
		private:
			
		};
		
		template<typename T>
		class TypeTraits<T, typename ::std::enable_if<::std::is_integral<T>::value>::type>
		{
		public:
			typedef T value_type;
			
			static T Constant(const ::std::size_t& i, const T& value)
			{
				return value;
			}
			
			static T Zero(const ::std::size_t& i)
			{
				return 0;
			}
			
			static T abs(const float& t)
			{
				return ::std::abs(t);
			}
			
			static bool equal(const T& lhs, const T& rhs)
			{
				return lhs == rhs;
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
		
		template<typename T>
		class TypeTraits<T, typename ::std::enable_if<::std::is_floating_point<T>::value>::type>
		{
		public:
			typedef T value_type;
			
			static T Constant(const ::std::size_t& i, const T& value)
			{
				return value;
			}
			
			static T Zero(const ::std::size_t& i)
			{
				return 0;
			}
			
			static T abs(const float& t)
			{
				return ::std::abs(t);
			}
			
			static bool equal(const T& lhs, const T& rhs, const T& epsilon = ::Eigen::NumTraits<T>::dummy_precision())
			{
				return ::Eigen::internal::isApprox(lhs, rhs, epsilon);
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
			
			static constexpr T e = T(2.718281828459045235360287471352662498L);
			
			static constexpr T log2e = T(1.442695040888963407359924681001892137L);
			
			static constexpr T log10e = T(0.434294481903251827651128918916605082L);
			
			static constexpr T pi = T(3.141592653589793238462643383279502884L);
			
			static constexpr T inv_pi = T(0.318309886183790671537767526745028724L);
			
			static constexpr T inv_sqrtpi = T(0.564189583547756286948079451560772586L);
			
			static constexpr T ln2 = T(0.693147180559945309417232121458176568L);
			
			static constexpr T ln10 = T(2.302585092994045684017991454684364208L);
			
			static constexpr T sqrt2 = T(1.414213562373095048801688724209698079L);
			
			static constexpr T sqrt3 = T(1.732050807568877293527446341505872367L);
			
			static constexpr T inv_sqrt3 = T(0.577350269189625764509148780501957456L);
			
			static constexpr T egamma = T(0.577215664901532860606512090082402431L);
			
			static constexpr T phi = T(1.618033988749894848204586834365638118L);
			
		protected:
			
		private:
			
		};
		
		template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
		class TypeTraits<::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
		{
		public:
			typedef Scalar value_type;
			
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
		class TypeTraits<::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
		{
		public:
			typedef Scalar value_type;
			
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
