//
// Copyright (c) 2009, Andre Gaschler
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

#ifndef RL_MATH_CIRCULARVECTOR2_H
#define RL_MATH_CIRCULARVECTOR2_H

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <cassert>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/LU>

#include "Circular.h"
#include "Function.h"
#include "Matrix.h" 
#include "Vector.h"

namespace rl
{
	namespace math
	{
		/**
		 * Circular segment function that maps from a time x to a point in 2D on
		 * a circular trajectory.
		 * 
		 */
		template<>
		class Circular<Vector2> : public Function<Vector2>
		{
		public:
			Circular<Vector2>() :
				Function<Vector2>(),
				angle(0)
			{
			}
			
			virtual ~Circular<Vector2>()
			{
			}
			
			/**
			 * Generates a circular segment function in 2D through three given points.
			 * 
			 * The given points most not be (numerically close to) colinear.
			 */
			static Circular<Vector2> ThreePoints(const Vector2& y0, const Vector2& yi, const Vector2& y1, const Real& x1 = 1)
			{
				Circular<Vector2> f;
				
				::Eigen::Matrix<Real, 2, 3> points2d;
				points2d << y0, yi, y1;
				
				// Solve substituted system similar to http://www.arndt-bruenner.de/mathe/scripts/kreis3p.htm
				Matrix33 A;
				A << Matrix::Ones(3, 1), -points2d.transpose();
				Vector3 b = - (points2d.transpose() * points2d).diagonal();
				Vector3 x = A.fullPivLu().solve(b); 
				Vector2 center2d = x.bottomRows(2) / 2;
				//Real radiusSquared = center2d.transpose() * center2d - x(0);
				//Real radius = ::std::sqrt(radiusSquared);
				assert((A * x - b).norm() < 1e-8 && "Circular motion: Linear system cannot be solved. (Points must not be colinear.)");
					
				f.center = center2d;
				f.axisX = y0 - f.center;
				f.axisY = Vector2(f.axisX(1), -f.axisX(0));
				Real angleVia = ::std::atan2(f.axisY.transpose() * (yi - f.center), f.axisX.transpose() * (yi - f.center));
				
				if (angleVia < 0)
				{
					f.axisY *= -1.0;
				}
				
				f.angle = ::std::atan2(f.axisY.transpose() * (y1 - f.center), f.axisX.transpose() * (y1 - f.center));
				
				if (f.angle < 0)
				{
					f.angle += 2 * static_cast<Real>(M_PI);
				}
				
				f.x1 = x1;
				
				return f;
			}
			
			/**
			 * Generates a circular segment through three given points in 2D with
			 * a given segment angle.
			 * 
			 * The given points must not be (numerically close to) colinear.
			 * Contrary to ThreePoints, where the circular segment ends in y1,
			 * the circular segments ends after a given angle.
			 * With this, a full circle can be constructed, given an angle of 2*pi.
			 * The given segment angle may be any real number, which allows
			 * multiple rotations.
			 * 
			 */
			static Circular<Vector2> ThreePointsAngle(const Vector2& y0, const Vector2& yi, const Vector2& y1, const Real& angle, const Real& x1 = 1)
			{
				Circular<Vector2> c = ThreePoints(y0, yi, y1, x1);
				c.angle = angle;
				return c;
			}
			
			Circular<Vector2>* clone() const
			{
				return new Circular<Vector2>(*this);
			}
			
			Real getAngle() const
			{
				return this->angle;
			}
			
			Vector2 getAxisX() const
			{
				return this->axisX;
			}
			
			Vector2 getAxisY() const
			{
				return this->axisY;
			}
			
			Vector2 getCenter() const
			{
				return this->center;
			}
			
			/**
			 * Evaluates the circular segment function for a given x.
			 * 
			 * Note that only the first two derivatives are implemented, all higher
			 * orders will return NaN.
			 */
			Vector2 operator()(const Real& x, const ::std::size_t& derivative = 0) const
			{
				assert(x >= this->lower() - FUNCTION_BOUNDARY);
				assert(x <= this->upper() + FUNCTION_BOUNDARY);
				assert(derivative <= 2 && "Circular: higher derivatives not implemented");
				
				Real c = this->angle / this->x1;

				if (derivative == 0)
				{
					return this->center + ::std::cos(c * x) * this->axisX + ::std::sin(c * x) * this->axisY;
				}
				else if (derivative == 1)
				{
					return -c * ::std::sin(c * x) * this->axisX + c * ::std::cos(c * x) * this->axisY;
				}
				else if (derivative == 2)
				{
					return -::std::pow(c, 2) * ::std::cos(c * x) * this->axisX - ::std::pow(c, 2) * ::std::sin(c * x) * this->axisY;
				}
				else
				{
					return this->axisX * ::std::numeric_limits<Real>::signaling_NaN();
				}
			}
			
		protected:
			/** Angle of circular motion. Any real value is allowed. */
			Real angle;
			
			/** First axis of the circular motion. Note that at time 0, the function value is (center + axisX). */
			Vector2 axisX;
			
			/** Second axis of the circular motion. After a motion of 90 degrees, the function value is (center + axisY). */
			Vector2 axisY;
			
			/** Center of the circle. */
			Vector2 center;
			
		private:
			
		};
	}
}

#endif // RL_MATH_CIRCULARVECTOR2_H
