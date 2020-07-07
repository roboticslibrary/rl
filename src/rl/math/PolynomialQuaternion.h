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

#ifndef RL_MATH_QUATERNIONPOLYNOMIAL_H
#define RL_MATH_QUATERNIONPOLYNOMIAL_H

#include <cassert>
#include <vector>

#include "Function.h"
#include "Polynomial.h"
#include "Quaternion.h"
#include "Rotation.h"
#include "Vector.h"

namespace rl
{
	namespace math
	{
		template<>
		class Polynomial<Quaternion> : public Function<Quaternion>
		{
		public:
			Polynomial<Quaternion>() :
				Function<Quaternion>(0, 0),
				y0(),
				c()
			{
			}
			
			Polynomial<Quaternion>(const ::std::size_t& degree) :
				Function<Quaternion>(0, 0),
				y0(),
				c(degree + 1)
			{
			}
			
			virtual ~Polynomial<Quaternion>()
			{
			}
			
			static Polynomial<Quaternion> CubicFirst(const Quaternion& y0, const Quaternion& y1, const Vector3& yd0, const Vector3& yd1, const Real& x1 = 1)
			{
				using ::std::abs;
				using ::std::atan2;
				
				Quaternion dy = y0.conjugate() * y1;
				Real norm = dy.vec().norm();
				Real dtheta = 2 * atan2(norm, abs(dy.w()));
				Vector3 e = dy.vec();
				
				if (norm > 0)
				{
					e /= norm;
				}
				
				Polynomial<Quaternion> f(3);
				
				f.c[0] = Vector3::Zero();
				f.c[1] = x1 * yd0;
				f.c[2] = x1 * invB(e, dtheta, yd1) - 3 * e * dtheta;
				f.c[3] = e * dtheta;
				f.x1 = x1;
				f.y0 = y0;
				
				return f;
			}
			
			static Polynomial<Quaternion> Linear(const Quaternion& y0, const Quaternion& y1, const Real& x1 = 1)
			{
				using ::std::abs;
				using ::std::atan2;
				
				Quaternion dy = y0.conjugate() * y1;
				Real norm = dy.vec().norm();
				Real dtheta = 2 * atan2(norm, abs(dy.w()));
				Vector3 e = dy.vec();
				
				if (norm > 0)
				{
					e /= norm;
				}
				
				Polynomial<Quaternion> f(1);
				
				f.c[0] = Vector3::Zero();
				f.c[1] = e * dtheta;
				f.x1 = x1;
				f.y0 = y0;
				
				return f;
			}
			
			Polynomial<Quaternion>* clone() const
			{
				return new Polynomial<Quaternion>(*this);
			}
			
			Vector3& coefficient(const ::std::size_t& i)
			{
				return this->c[i];
			}
			
			const Vector3& coefficient(const ::std::size_t& i) const
			{
				return this->c[i];
			}
			
			::std::size_t degree() const
			{
				return this->c.size() - 1;
			}
			
			Quaternion operator()(const Real& x, const ::std::size_t& derivative = 0) const
			{
				assert(derivative <= 2 && "Polynomial<Quaternion>: higher derivatives not implemented");
				
				Vector3 u = this->eval(x);
				Real theta = u.norm();
				
				if (theta > 0)
				{
					u /= theta;
				}
				
				Quaternion y = this->y0 * AngleAxis(theta, u);
				
				if (0 == derivative)
				{
					return y;
				}
				
				Polynomial<Quaternion> fd = this->derivative();
				Vector3 axisd = fd.eval(x);
				Real thetad = u.dot(axisd);
				Vector3 w = u.cross(axisd) / theta;
				Vector3 omega;
				
				if (theta > 0)
				{
					omega = u * thetad + ::std::sin(theta) * w.cross(u) - (1 - ::std::cos(theta)) * w;
				}
				else
				{
					omega = axisd;
				}
				
				Vector3 yomega = y._transformVector(omega);
				Quaternion yd = y.firstDerivative(yomega);
				
				if (1 == derivative)
				{
					return yd;
				}
				
				Polynomial<Quaternion> fdd = fd.derivative();
				Vector3 axisdd = fdd.eval(x);
				Real thetadd = w.cross(u).dot(axisd) + u.dot(axisdd);
				Vector3 wd = (u.cross(axisdd) - 2 * thetad * w) / theta;
				Vector3 omegad;
				
				if (theta > 0)
				{
					omegad = u * thetadd + ::std::sin(theta) * wd.cross(u) - (1 - ::std::cos(theta)) * wd + thetad * w.cross(u) + omega.cross(u * thetad - w);
				}
				else
				{
					omegad = axisdd;
				}
				
				Vector3 yomegad = y._transformVector(omegad);
				Quaternion ydd = y.secondDerivative(yd, yomega, yomegad);
				
				if (2 == derivative)
				{
					return ydd;
				}
				
				return Quaternion();
			}
			
			Quaternion y0;
			
		protected:
			::std::vector<Vector3> c;
			
		private:
			Polynomial<Quaternion> derivative() const
			{
				::std::size_t degree = this->degree() > 0 ? this->degree() - 1 : 0;
				
				Polynomial<Quaternion> f(degree);
				f.x0 = this->x0;
				f.x1 = this->x1;
				
				if (0 == this->degree())
				{
					f.c[0] = Vector3::Zero();
				}
				else
				{
					for (::std::size_t i = 0; i < this->degree(); ++i)
					{
						f.c[i] = (static_cast<Real>(this->degree() - (i + 1) + 1) * this->c[i] + static_cast<Real>(i + 1) * this->c[i + 1]) / (this->x1 - this->x0);
					}
				}
				
				return f;
			}
			
			Vector3 eval(const Real& x) const
			{
				assert(x > this->lower() - this->functionBoundary);
				assert(x < this->upper() + this->functionBoundary);
				
				Vector3 axis = Vector3::Zero();
				
				for (::std::size_t i = 0; i < this->degree() + 1; ++i)
				{
					axis += this->c[this->degree() - i] * ::std::pow(x / this->x1, static_cast<int>(this->degree() - i)) * ::std::pow(x / this->x1 - 1, static_cast<int>(i));
				}
				
				return axis;
			}
			
			static Vector3 invB(const Vector3& e, const Real& dtheta, const Vector3& x)
			{
				if (dtheta < ::std::numeric_limits<Real>::epsilon())
				{
					return x;
				}
				
				Real cosdtheta = ::std::cos(dtheta);
				Real sindtheta = ::std::sin(dtheta);
				
				return e.dot(x) * e + static_cast<Real>(0.5) * (dtheta * sindtheta) / (1 - cosdtheta) * e.cross(x).cross(e) + static_cast<Real>(0.5) * dtheta * e.cross(x);
			}
		};
	}
}

#endif // RL_MATH_QUATERNIONPOLYNOMIAL_H
