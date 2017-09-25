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

#ifndef RL_MATH_CIRCULARVECTOR3_H
#define RL_MATH_CIRCULARVECTOR3_H

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <cassert>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/LU>

#include "Circular.h"
#include "CircularVector2.h"
#include "Function.h"
#include "Matrix.h" 
#include "Vector.h"

namespace rl
{
	namespace math
	{
		/**
		 * Circular segment function that maps from a time x to a point in 3D on
		 * a circular trajectory.
		 * 
		 */
		template<>
		class Circular<Vector3> : public Function<Vector3>
		{
		public:
			Circular<Vector3>() :
				Function<Vector3>(),
				angle(0)
			{
			}
			
			virtual ~Circular<Vector3>()
			{
			}
			
			/**
			 * Generates a circular segment through three given points in 3D.
			 * 
			 * The given points must not be (numerically close to) colinear.
			 */
			static Circular<Vector3> ThreePoints(const Vector3& y0, const Vector3& yi, const Vector3& y1, const Real& x1 = 1)
			{
				Circular f;
				
				Vector3 planeNormal = (yi - y0).cross(y1 - y0);
				assert(planeNormal.norm() > 1e-8 && "Circular motion is ambiguous. Given three points must not be colinear!");
				planeNormal.normalize();
				Real planeD = planeNormal.transpose() * y0;
				
				// Numerically stable orthonormals
				Vector3 planeDirection1, planeDirection2;
				Vector3 someUnit(1, 0, 0);
				Vector3 someOtherUnit(0, 1, 0);
				
				if ((planeNormal.transpose() * someUnit).norm() < (planeNormal.transpose() * someOtherUnit).norm())
				{
					planeDirection1 = planeNormal.cross(someUnit);
				}
				else
				{
					planeDirection1 = planeNormal.cross(someOtherUnit);
				}
				
				planeDirection1.normalize();
				planeDirection2 = planeNormal.cross(planeDirection1);
				planeDirection2.normalize();
				
				// Project all points to a plane that contains the circular motion
				::Eigen::Matrix<Real, 2, 3> projectionTo2d;
				projectionTo2d << planeDirection1.transpose(), planeDirection2.transpose();
				Vector2 y02d = projectionTo2d * y0;
				Vector2 yi2d = projectionTo2d * yi;
				Vector2 y12d = projectionTo2d * y1;
				
				Circular<Vector2> circ2d = Circular<Vector2>::ThreePoints(y02d, yi2d, y12d);
				f.center = projectionTo2d.transpose() * circ2d.getCenter() + planeNormal * planeD;
				f.axisX = y0 - f.center;
				f.axisY = planeNormal.cross(Vector3(f.axisX));
				f.angle = circ2d.getAngle();
				
				f.x1 = x1;
				
				return f;
			}
			
			/**
			 * Generates a circular segment through three given points in 3D with
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
			static Circular<Vector3> ThreePointsAngle(const Vector3& y0, const Vector3& yi, const Vector3& y1, const Real& angle, const Real& x1 = 1)
			{
				Circular<Vector3> c = ThreePoints(y0, yi, y1, x1);
				c.angle = angle;
				return c;
			}
			
			Circular<Vector3>* clone() const
			{
				return new Circular<Vector3>(*this);
			}
			
			Real getAngle() const
			{
				return this->angle;
			}
			
			Vector3 getAxisX() const
			{
				return this->axisX;
			}
			
			Vector3 getAxisY() const
			{
				return this->axisY;
			}
			
			Vector3 getCenter() const
			{
				return this->center;
			}
			
			/**
			 * Evaluates the circular segment function for a given x.
			 * 
			 * Note that only the first two derivatives are implemented, all higher
			 * orders will return NaN.
			 */
			Vector3 operator()(const Real& x, const ::std::size_t& derivative = 0) const
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
			Vector3 axisX;
			
			/** Second axis of the circular motion. After a motion of 90 degrees, the function value is (center + axisY). */
			Vector3 axisY;
			
			/** Center of the circle. */
			Vector3 center;
			
		private:
			
		};
	}
}

#endif // RL_MATH_CIRCULARVECTOR3_H
