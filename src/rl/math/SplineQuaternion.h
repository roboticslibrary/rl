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

#ifndef RL_MATH_QUATERNIONSPLINE_H
#define RL_MATH_QUATERNIONSPLINE_H

#include <limits>
#include <vector>

#include "PolynomialQuaternion.h"
#include "Spline.h"

namespace rl
{
	namespace math
	{
		template<>
		class Spline<Quaternion> : public Function<Quaternion>
		{
		public:
			typedef ::std::vector<Polynomial<Quaternion>>::const_iterator ConstIterator;
			
			typedef ::std::vector<Polynomial<Quaternion>>::const_reverse_iterator ConstReverseIterator;
			
			typedef ::std::vector<Polynomial<Quaternion>>::iterator Iterator;
			
			typedef ::std::vector<Polynomial<Quaternion>>::reverse_iterator ReverseIterator;
			
			Spline<Quaternion>() :
				Function<Quaternion>(),
				polynomials(),
				x0(0),
				x1(0)
			{
				this->x0 = 0;
				this->x1 = 0;
			}
			
			virtual ~Spline<Quaternion>()
			{
			}
			
			static Spline<Quaternion> CubicFirst(const ::std::vector<Real>& x, const ::std::vector<Quaternion>& y, const Vector3& yd0, const Vector3& yd1)
			{
				assert(x.size() > 1);
				assert(x.size() == y.size());
				
				::std::size_t n = x.size();
				
				Spline<Quaternion> f;
				
				::std::vector<Real> dtheta(n - 1);
				::std::vector<Real> dx(n - 1);
				::std::vector<Vector3> e(n - 1);
				
				for (::std::size_t i = 0; i < n - 1; ++i)
				{
					dx[i] = x[i + 1] - x[i];
					dtheta[i] = y[i].angularDistance(y[i + 1]);
					e[i] = (y[i].inverse() * y[i + 1]).vec();
					
					if (e[i].norm() <= ::std::numeric_limits<Real>::epsilon())
					{
						e[i].setZero();
					}
					else
					{
						e[i].normalize();
					}
				}
				
				::std::vector<Real> a(n);
				::std::vector<Real> b(n);
				::std::vector<Real> c(n);
				::std::vector<Vector3> d(n);
				Real dOmega;
				::std::vector<Vector3> omega(n, Vector3::Zero());
				::std::vector<Vector3> omegaprev(n);
				
				do
				{
					for (::std::size_t i = 1; i < n - 1; ++i)
					{
						omegaprev[i] = omega[i];
					}
					
					for (::std::size_t i = 1; i < n - 1; ++i)
					{
						a[i] = 2 / dx[i - 1];
						b[i] = 4 / dx[i - 1] + 4 / dx[i];
						c[i] = 2 / dx[i];
						d[i] = (6 * e[i - 1] * dtheta[i - 1]) / ::std::pow(dx[i - 1], 2) + (6 * e[i] * dtheta[i]) / ::std::pow(dx[i], 2) - R(e[i - 1], dtheta[i - 1], omegaprev[i]);
					}
					
					if (n > 2)
					{
						d[1] -= a[1] * B(e[0], dtheta[0], yd0);
						d[n - 2] -= c[n - 2] * invB(e[n - 2], dtheta[n - 2], yd1);
					}
					
					for (::std::size_t i = 2; i < n - 1; ++i)
					{
						 b[i] -= c[i - 1] * a[i] / b[i - 1];
						 d[i] -= (a[i] / b[i - 1]) * B(e[i - 1], dtheta[i - 1], d[i - 1]);
					}
					
					if (n > 2)
					{
						omega[n - 2] = d[n - 2] / b[n - 2];
					}
					
					for (::std::size_t i = 1; i < n - 2; ++i)
					{
						omega[n - 2 - i] = (d[n - 2 - i] - c[n - 2 - i] * invB(e[n - 2 - i], dtheta[n - 2 - i], omega[n - 2 - i + 1])) / b[n - 2 - i];
					}
					
					dOmega = 0;
					
					for (::std::size_t i = 1; i < n - 1; ++i)
					{
						dOmega += (omega[i] - omegaprev[i]).norm();
					}
				}
				while (dOmega > 1.0e-6f);
				
				omega[0] = yd0;
				omega[n - 1] = yd1;
				
				for (::std::size_t i = 0; i < n - 1; ++i)
				{
					Polynomial<Quaternion> fi(3);
					fi.coefficient(0) = Vector3::Zero();
					fi.coefficient(1) = dx[i] * omega[i];
					fi.coefficient(2) = dx[i] * invB(e[i], dtheta[i], omega[i + 1]) - 3 * e[i] * dtheta[i];
					fi.coefficient(3) = e[i] * dtheta[i];
					fi.upper() = dx[i];
					fi.y0 = y[i];
					f.push_back(fi);
				}
				
				return f;
			}
			
			Polynomial<Quaternion>& at(const ::std::size_t& i)
			{
				return this->polynomials.at(i);
			}
			
			const Polynomial<Quaternion>& at(const ::std::size_t& i) const
			{
				return this->polynomials.at(i);
			}
			
			Polynomial<Quaternion>& back()
			{
				return this->polynomials.back();
			}
			
			const Polynomial<Quaternion>& back() const
			{
				return this->polynomials.back();
			}
			
			Iterator begin()
			{
				return this->polynomials.begin();
			}
			
			ConstIterator begin() const
			{
				return this->polynomials.begin();
			}
			
			void clear()
			{
				this->polynomials.clear();
			}
			
			Spline<Quaternion>* clone() const
			{
				return new Spline<Quaternion>(*this);
			}
			
			Real duration() const
			{
				return this->upper() - this->lower();
			}
			
			bool empty()
			{
				return this->polynomials.empty();
			}
			
			Iterator end()
			{
				return this->polynomials.end();
			}
			
			ConstIterator end() const
			{
				return this->polynomials.end();
			}
			
			Polynomial<Quaternion>& front()
			{
				return this->polynomials.front();
			}
			
			const Polynomial<Quaternion>& front() const
			{
				return this->polynomials.front();
			}
			
			Real& lower()
			{
				return this->x0;
			}
			
			const Real& lower() const
			{
				return this->x0;
			}
			
			Quaternion operator()(const Real& x, const ::std::size_t& derivative = 0) const
			{
				assert(x >= this->lower() - FUNCTION_BOUNDARY);
				assert(x <= this->upper() + FUNCTION_BOUNDARY);
				assert(this->polynomials.size() > 0);
				
				Real x0 = this->lower();
				::std::size_t i = 0;

				for (; x > x0 + this->polynomials[i].duration() && i < this->polynomials.size(); ++i)
				{
					x0 += this->polynomials[i].duration();			
				}
				
				return this->polynomials[i](x - x0, derivative);
			}
			
			Polynomial<Quaternion>& operator[](const ::std::size_t& i)
			{
				return this->polynomials[i];
			}
			
			const Polynomial<Quaternion>& operator[](const ::std::size_t& i) const
			{
				return this->polynomials[i];
			}
			
			void pop_back()
			{
				if (!this->polynomials.empty())
				{
					this->x1 -= this->polynomials.back().duration();
					this->polynomials.pop_back();
					
					if (this->polynomials.empty())
					{
						this->x0 = 0;
					}
				}
			}
			
			void push_back(Polynomial<Quaternion>& polynomial)
			{
				if (this->polynomials.empty())
				{
					this->x0 = polynomial.lower();
				}
				
				this->polynomials.push_back(polynomial);
				this->x1 += polynomial.duration();
			}
			
			::std::size_t size() const
			{
				return this->polynomials.size();
			}
			
			ReverseIterator rbegin()
			{
				return this->polynomials.rbegin();
			}
			
			ConstReverseIterator rbegin() const
			{
				return this->polynomials.rbegin();
			}
			
			ReverseIterator rend()
			{
				return this->polynomials.rend();
			}
			
			ConstReverseIterator rend() const
			{
				return this->polynomials.rend();
			}
			
			Real& upper()
			{
				return this->x1;
			}
			
			const Real& upper() const
			{
				return this->x1;
			}
			
		protected:
			::std::vector<Polynomial<Quaternion>> polynomials;
			
			Real x0;
			
			Real x1;
			
		private:
			static Vector3 B(const Vector3& e, const Real& dtheta, const Vector3& x)
			{
				if (dtheta <= ::std::numeric_limits<Real>::epsilon())
				{
					return x;
				}
				
				Real cosdtheta = ::std::cos(dtheta);
				Real sindtheta = ::std::sin(dtheta);
				
				return x.dot(e) * e + sindtheta / dtheta * e.cross(x).cross(e) - (1 - cosdtheta) / dtheta * e.cross(x);
			}
			
			static Vector3 invB(const Vector3& e, const Real& dtheta, const Vector3& x)
			{
				if (dtheta <= ::std::numeric_limits<Real>::epsilon())
				{
					return x;
				}
				
				Real cosdtheta = ::std::cos(dtheta);
				Real sindtheta = ::std::sin(dtheta);
				
				return e.dot(x) * e + 0.5f * (dtheta * sindtheta) / (1 - cosdtheta) * e.cross(x).cross(e) + 0.5f * dtheta * e.cross(x);
			}
			
			static Vector3 R(const Vector3& e, const Real& dtheta, const Vector3& omega)
			{
				if (dtheta <= ::std::numeric_limits<Real>::epsilon())
				{
					return Vector3::Zero();
				}
				
				Real cosdtheta = ::std::cos(dtheta);
				Real sindtheta = ::std::sin(dtheta);
				
				Real r0 = 0.5f * (dtheta - sindtheta) / (1 - cosdtheta);
				Real r1 = (dtheta * sindtheta - 2 * (1 - cosdtheta)) / (dtheta * (1 - cosdtheta));
				
				return r0 * (omega.dot(omega) - ::std::pow(e.dot(omega), 2)) * e + r1 * e.dot(omega) * e.cross(omega).cross(e);
			}
		};
	}
}

#endif // RL_MATH_QUATERNIONSPLINE_H
