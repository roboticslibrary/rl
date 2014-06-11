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

#ifndef _RL_MATH_TRANSFORM_H_
#define _RL_MATH_TRANSFORM_H_

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>

#include <Eigen/Geometry>

#include "Matrix.h"
#include "Quaternion.h"

namespace rl
{
	namespace math
	{
		typedef ::Eigen::Transform< Real, 3, ::Eigen::Affine > Transform;
		
		typedef ::Eigen::Translation< Real, 3 > Translation;
		
		namespace transform
		{
			template< typename Matrix1, typename Matrix2, typename Real >
			inline
			Real
			distance(const Matrix1& t1, const Matrix2& t2, const Real& weight = 1.0)
			{
				Quaternion q1(t1.rotation());
				Quaternion q2(t2.rotation());
				
				return ::std::sqrt(
					::std::pow(t2(0, 3) - t1(0, 3), 2) +
					::std::pow(t2(1, 3) - t1(1, 3), 2) +
					::std::pow(t2(2, 3) - t1(2, 3), 2) +
					weight * ::std::pow(q1.angularDistance(q2), 2)
				);
			}
			
			template< typename Matrix1, typename Real, typename Matrix2 >
			inline
			void
			fromDelta(const Matrix1& t1, const Real& x, const Real& y, const Real& z, const Real& a, const Real& b, const Real& c, Matrix2& t2)
			{
//				assert(t1.rows() > 3);
//				assert(t1.cols() > 3);
//				assert(t2.rows() > 3);
//				assert(t2.cols() > 3);
				
				t2(0, 0) = t1(0, 0) - c * t1(1, 0) + b * t1(2, 0);
				t2(0, 1) = t1(0, 1) - c * t1(1, 1) + b * t1(2, 1);
				t2(0, 2) = t1(0, 2) - c * t1(1, 2) + b * t1(2, 2);
				t2(0, 3) = t1(0, 3) + x;
				t2(1, 0) = t1(1, 0) + c * t1(0, 0) - a * t1(2, 0);
				t2(1, 1) = t1(1, 1) + c * t1(0, 1) - a * t1(2, 1);
				t2(1, 2) = t1(1, 2) + c * t1(0, 2) - a * t1(2, 2);
				t2(1, 3) = t1(1, 3) + y;
				t2(2, 0) = t1(2, 0) - b * t1(0, 0) + a * t1(1, 0);
				t2(2, 1) = t1(2, 1) - b * t1(0, 1) + a * t1(1, 1);
				t2(2, 2) = t1(2, 2) - b * t1(0, 2) + a * t1(1, 2);
				t2(2, 3) = t1(2, 3) + z;
				t2(3, 0) = 0.0f;
				t2(3, 1) = 0.0f;
				t2(3, 2) = 0.0f;
				t2(3, 3) = 1.0f;
			}
			
			template< typename Matrix1, typename Vector, typename Matrix2 >
			inline
			void
			fromDelta(const Matrix1& t1, const Vector& xyzabc, Matrix2& t2)
			{
//				assert(xyzabc.size() > 5);
				
				fromDelta(t1, xyzabc(0), xyzabc(1), xyzabc(2), xyzabc(3), xyzabc(4), xyzabc(5), t2);
			}
			
			template< typename Matrix, typename Real >
			inline
			void
			getDelta(const Matrix& t, Real& x, Real& y, Real& z, Real& a, Real& b, Real& c)
			{
//				assert(t.rows() > 3);
//				assert(t.cols() > 3);
				
				x = t(0, 3);
				y = t(1, 3);
				z = t(2, 3);
				a = (t(2, 1) - t(1, 2)) / 2.0f;
				b = (t(0, 2) - t(2, 0)) / 2.0f;
				c = (t(1, 0) - t(0, 1)) / 2.0f;
			}
			
			template< typename Matrix, typename Vector >
			inline
			void
			getDelta(const Matrix& t, Vector& xyzabc)
			{
//				assert(xyzabc.size() > 5);
				
				getDelta(t, xyzabc(0), xyzabc(1), xyzabc(2), xyzabc(3), xyzabc(4), xyzabc(5));
			}
			
			template< typename Matrix, typename Real >
			inline
			void
			getDenavitHartenberg(const Matrix& t, Real& d, Real& theta, Real& a, Real& alpha)
			{
//				assert(t.rows() > 3);
//				assert(t.cols() > 3);
				
				assert(::std::abs(t(2, 0)) <= ::std::numeric_limits<Real>::epsilon());
				
				d = t(2, 3);
				
				theta = ::std::atan2(t(1, 0), t(0, 0));
				
				if (::std::abs(t(0, 0)) <= ::std::numeric_limits<Real>::epsilon())
				{
					a = t(1, 3) / t(1, 0);
				}
				else
				{
					a = t(0, 3) / t(0, 0);
				}
				
				alpha = ::std::atan2(t(2, 1), t(2, 2));
			}
			
			template< typename Real, typename Matrix >
			inline
			void
			setDelta(const Real& x, const Real& y, const Real& z, const Real& a, const Real& b, const Real& c, Matrix& t)
			{
//				assert(t.rows() > 3);
//				assert(t.cols() > 3);
				
				t(0, 0) = 0;
				t(0, 1) = -c;
				t(0, 2) = b;
				t(0, 3) = x;
				t(1, 0) = c;
				t(1, 1) = 0;
				t(1, 2) = -a;
				t(1, 3) = y;
				t(2, 0) = -b;
				t(2, 1) = a;
				t(2, 2) = 0;
				t(2, 3) = z;
				t(3, 0) = 0;
				t(3, 1) = 0;
				t(3, 2) = 0;
				t(3, 3) = 1;
			}
			
			template< typename Vector, typename Matrix >
			inline
			void
			setDelta(const Vector& xyzabc, Matrix& t)
			{
//				assert(xyzabc.size() > 5);
				
				setDelta(xyzabc(0), xyzabc(1), xyzabc(2), xyzabc(3), xyzabc(4), xyzabc(5), t);
			}
			
			template< typename Real, typename Matrix >
			inline
			void
			setDenavitHartenberg(const Real& d, const Real& theta, const Real& a, const Real& alpha, Matrix& t)
			{
//				assert(t.rows() > 3);
//				assert(t.cols() > 3);
				
				Real cosAlpha = ::std::cos(alpha);
				Real cosTheta = ::std::cos(theta);
				Real sinAlpha = ::std::sin(alpha);
				Real sinTheta = ::std::sin(theta);
				
				t(0, 0) = cosTheta;
				t(1, 0) = sinTheta;
				t(2, 0) = 0.0f;
				t(3, 0) = 0.0f;
				t(0, 1) = -cosAlpha * sinTheta;
				t(1, 1) = cosAlpha * cosTheta;
				t(2, 1) = sinAlpha;
				t(3, 1) = 0.0f;
				t(0, 2) = sinAlpha * sinTheta;
				t(1, 2) = -sinAlpha * cosTheta;
				t(2, 2) = cosAlpha;
				t(3, 2) = 0.0f;
				t(0, 3) = a * cosTheta;
				t(1, 3) = a * sinTheta;
				t(2, 3) = d;
				t(3, 3) = 1.0f;
			}
			
			template< typename Matrix1, typename Matrix2, typename Real >
			inline
			void
			toDelta(const Matrix1& t1, const Matrix2& t2, Real& x, Real& y, Real& z, Real& a, Real& b, Real& c)
			{
//				assert(t1.rows() > 3);
//				assert(t1.cols() > 3);
//				assert(t2.rows() > 3);
//				assert(t2.cols() > 3);
				
				x = t2(0, 3) - t1(0, 3);
				y = t2(1, 3) - t1(1, 3);
				z = t2(2, 3) - t1(2, 3);
				a = (t1(1, 0) * t2(2, 0) - t1(2, 0) * t2(1, 0) + t1(1, 1) * t2(2, 1) - t1(2, 1) * t2(1, 1) + t1(1, 2) * t2(2, 2) - t1(2, 2) * t2(1, 2)) / 2.0f;
				b = (t1(2, 0) * t2(0, 0) - t1(0, 0) * t2(2, 0) + t1(2, 1) * t2(0, 1) - t1(0, 1) * t2(2, 1) + t1(2, 2) * t2(0, 2) - t1(0, 2) * t2(2, 2)) / 2.0f;
				c = (t1(0, 0) * t2(1, 0) - t1(1, 0) * t2(0, 0) + t1(0, 1) * t2(1, 1) - t1(1, 1) * t2(0, 1) + t1(0, 2) * t2(1, 2) - t1(1, 2) * t2(0, 2)) / 2.0f;
			}
			
			template< typename Matrix1, typename Matrix2, typename Vector >
			inline
			void
			toDelta(const Matrix1& t1, const Matrix2& t2, Vector& xyzabc)
			{
//				assert(xyzabc.size() > 5);
				
				toDelta(t1, t2, xyzabc(0), xyzabc(1), xyzabc(2), xyzabc(3), xyzabc(4), xyzabc(5));
			}
		}
	}
}

#endif // _RL_MATH_TRANSFORM_H_
