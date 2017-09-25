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

#ifndef RL_MATH_METRICS_L2SQUARED_H
#define RL_MATH_METRICS_L2SQUARED_H

namespace rl
{
	namespace math
	{
		namespace metrics
		{
			template<typename T>
			struct L2Squared
			{
				typedef typename T::value_type Distance;
				
				typedef typename T::size_type Size;
				
				typedef T Value;
				
				Distance operator()(const Value& lhs, const Value& rhs) const
				{
					using ::std::begin;
					using ::std::end;
					
					auto first1 = begin(lhs);
					auto last1 = end(lhs);
					auto first2 = begin(rhs);
					
					Distance distance = Distance();
					
					while (first1 != last1)
					{
						Distance tmp = *first1 - *first2;
						distance += tmp * tmp;
						++first1;
						++first2;
					}
					
					return distance;
				}
				
				Distance operator()(const Distance& lhs, const Distance& rhs, const Size& index) const
				{
					Distance tmp = lhs - rhs;
					return tmp * tmp;
				}
			};
			
			template<typename Scalar, int Rows, int Options, int MaxRows, int MaxCols>
			struct L2Squared< ::Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, MaxCols>>
			{
				typedef Scalar Distance;
				
				typedef int Size;
				
				typedef ::Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, MaxCols> Value;
				
				Distance operator()(const Value& lhs, const Value& rhs) const
				{
					return (lhs - rhs).squaredNorm();
				}
				
				Distance operator()(const Distance& lhs, const Distance& rhs, const Size& index) const
				{
					Distance tmp = lhs - rhs;
					return tmp * tmp;
				}
			};
			
			template<typename Scalar, int Rows, int Options, int MaxRows, int MaxCols>
			struct L2Squared< ::Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, MaxCols>*>
			{
				typedef Scalar Distance;
				
				typedef int Size;
				
				typedef ::Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, MaxCols>* Value;
				
				Distance operator()(const Value& lhs, const Value& rhs) const
				{
					return (*lhs - *rhs).squaredNorm();
				}
				
				Distance operator()(const Distance& lhs, const Distance& rhs, const Size& index) const
				{
					Distance tmp = lhs - rhs;
					return tmp * tmp;
				}
			};
			
			template<typename Scalar, int Rows, int Options, int MaxRows, int MaxCols>
			struct L2Squared<const ::Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, MaxCols>*>
			{
				typedef Scalar Distance;
				
				typedef int Size;
				
				typedef const ::Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, MaxCols>* Value;
				
				Distance operator()(const Value& lhs, const Value& rhs) const
				{
					return (*lhs - *rhs).squaredNorm();
				}
				
				Distance operator()(const Distance& lhs, const Distance& rhs, const Size& index) const
				{
					Distance tmp = lhs - rhs;
					return tmp * tmp;
				}
			};
		}
	}
}

#endif // RL_MATH_METRICS_L2SQUARED_H
