//
// Copyright (c) 2009, Markus Rickert, Andre Gaschler
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

#ifndef RL_MATH_POLYNOMIAL_H
#define RL_MATH_POLYNOMIAL_H

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <cassert>
#include <stdexcept>
#include <vector>

#ifdef HAVE_EIGEN_UNSUPPORTED
#include <unsupported/Eigen/Polynomials>
#else // HAVE_EIGEN_UNSUPPORTED
#include <Eigen/Eigenvalues>
#endif // HAVE_EIGEN_UNSUPPORTED

#include "Function.h"
#include "Matrix.h"
#include "TypeTraits.h"
#include "Vector.h"

namespace rl
{
	namespace math
	{
		/**
		 * A vector-valued polynomial function from Real -> T.
		 * 
		 * A Polynomial is indefinitely often differentiable and can be evaluated
		 * efficiently.
		 */
		template<typename T>
		class Polynomial : public Function<T>
		{
		public:
			Polynomial(const ::std::size_t& degree) :
				Function<T>(),
				c(degree + 1)
			{
			}
			
			virtual ~Polynomial()
			{
			}
			
			static Polynomial CubicFirst(const T& y0, const T& y1, const T& yd0, const T& yd1, const Real& x1 = 1)
			{
				Polynomial f(3);
				
				f.c[0] = y0;
				f.c[1] = yd0;
				f.c[2] = -(2 * x1 * yd0 + 3 * y0 - 3 * y1 + x1 * yd1) / ::std::pow(x1, 2);
				f.c[3] = (2 * y0 + x1 * yd0 - 2 * y1 + x1 * yd1) / ::std::pow(x1, 3);
				f.x1 = x1;
				
				return f;
			}
			
			static Polynomial CubicSecond(const T& y0, const T& y1, const T& ydd0, const T& ydd1, const Real& x1 = 1)
			{
				Polynomial f(3);
				
				f.c[0] = y0;
				f.c[1] = -(2 * ::std::pow(x1, 2) * ydd0 + ::std::pow(x1, 2) * ydd1 + 6 * y0 - 6 * y1) / x1 / 6;
				f.c[2] = ydd0 / 2;
				f.c[3] = -(ydd0 - ydd1) / x1 / 6;
				f.x1 = x1;
				
				return f;
			}
			
			static Polynomial Linear(const T& y0, const T& y1, const Real& x1 = 1)
			{
				Polynomial f(1);
				
				f.c[0] = y0;
				f.c[1] = (y1 - y0) / x1;
				f.x1 = x1;
				
				return f;
			}
			
			static Polynomial Quadratic(const T& y0, const T& y1, const T& yd0, const Real& x1 = 1)
			{
				Polynomial f(2);
				
				f.c[0] = y0;
				f.c[1] = yd0;
				f.c[2] = -(x1 * yd0 + y0 - y1) / ::std::pow(x1, 2);
				f.x1 = x1;
				
				return f;
			}
			
			static Polynomial QuarticFirstSecond(const T& y0, const T& y1, const T& yd0, const T& yd1, const T& ydd0, const Real& x1 = 1)
			{
				Polynomial f(4);
				
				f.c[0] = y0;
				f.c[1] = yd0;
				f.c[2] = ydd0 / 2;
				f.c[3] = -(::std::pow(x1, 2) * ydd0 + 3 * x1 * yd0 + x1 * yd1 + 4 * y0 - 4 * y1) / ::std::pow(x1, 3);
				f.c[4] = 0.5f * (::std::pow(x1, 2) * ydd0 + 4 * x1 * yd0 + 2 * x1 * yd1 + 6 * y0 - 6 * y1) / ::std::pow(x1, 4);
				f.x1 = x1;
				
				return f;
			}
			
			static Polynomial QuinticFirstSecond(const T& y0, const T& y1, const T& yd0, const T& yd1, const T& ydd0, const T& ydd1, const Real& x1 = 1)
			{
				Polynomial f(5);
				
				f.c[0] = y0;
				f.c[1] = yd0;
				f.c[2] = ydd0 / 2;
				f.c[3] = -(3 * ::std::pow(x1, 2) * ydd0 + 12 * x1 * yd0 - ::std::pow(x1, 2) * ydd1 + 20 * y0 + 8 * x1 * yd1 - 20 * y1) / (2 * ::std::pow(x1, 3));
				f.c[4] = (16 * x1 * yd0 - 2 * ::std::pow(x1, 2) * ydd1 + 30 * y0 + 14 * x1 * yd1 - 30 * y1 + 3 * ::std::pow(x1, 2) * ydd0) / (2 * ::std::pow(x1, 4));
				f.c[5] = -(12 * y0 + 6 * x1 * yd0 + 6 * x1 * yd1 - 12 * y1 + ::std::pow(x1, 2) * ydd0 - ::std::pow(x1, 2) * ydd1) / (2 * ::std::pow(x1, 5));
				f.x1 = x1;
				
				return f;
			}
			
			static Polynomial SepticFirstSecondThird(const T& y0, const T& y1, const T& yd0, const T& yd1, const T& ydd0, const T& ydd1, const T& yddd0, const T& yddd1, const Real& x1 = 1)
			{
				Polynomial f(7);
				
				f.c[0] = y0;
				f.c[1] = yd0;
				f.c[2] = ydd0 / 2;
				f.c[3] = yddd0 / 6;
				f.c[4] = -(::std::pow(x1, 3) * (yddd1 + 4 * yddd0) + ::std::pow(x1, 2) * (30 * ydd0 - 15 * ydd1) + x1 * (90 * yd1 + 120 * yd0) - 210 * y1 + 210 * y0 ) / (6 * ::std::pow(x1, 4));
				f.c[5] = (::std::pow(x1, 3) * (yddd1 + 2 * yddd0) + ::std::pow(x1, 2) * (20 * ydd0 - 14 * ydd1) + x1 * (78 * yd1 + 90 * yd0) - 168 * y1 + 168 * y0) / (2 * ::std::pow(x1, 5));
				f.c[6] = -(::std::pow(x1, 3) * (3 * yddd1 + 4 * yddd0) + ::std::pow(x1, 2) * (45 * ydd0 - 39 * ydd1) + x1 * (204 * yd1 + 216 * yd0) - 420 * y1 + 420 * y0) / (6 * ::std::pow(x1, 6));
				f.c[7] = (::std::pow(x1, 3) * (yddd1 + yddd0) + ::std::pow(x1, 2) * (12 * ydd0 - 12 * ydd1) + x1 * (60 * yd1 + 60 * yd0) - 120 * y1 + 120 * y0) / (6 * ::std::pow(x1, 7));
				f.x1 = x1;
				
				return f;
			}
			
			static Polynomial SexticFirstSecondThird(const T& y0, const T& y1, const T& yd0, const T& yd1, const T& ydd0, const T& ydd1, const T& yddd0, const Real& x1 = 1)
			{
				Polynomial f(6);
				
				f.c[0] = y0;
				f.c[1] = yd0;
				f.c[2] = ydd0 / 2;
				f.c[3] = yddd0 / 6;
				f.c[4] = -(::std::pow(x1, 3) * yddd0 + ::std::pow(x1, 2) * (6 * ydd0 - ydd1) + x1 * (10 * yd1 + 20 * yd0) - 30 * y1 + 30 * y0 ) / (2 * ::std::pow(x1, 4));
				f.c[5] = (::std::pow(x1, 3) * yddd0 + ::std::pow(x1, 2) * (8 * ydd0 - 2 * ydd1) + x1 * (18 * yd1 + 30 * yd0) - 48 * y1 + 48 * y0) / (2 * ::std::pow(x1, 5));
				f.c[6] = -(::std::pow(x1, 3) * yddd0 + ::std::pow(x1, 2) * (9 * ydd0 - 3 * ydd1) + x1 * (24 * yd1 + 36 * yd0) - 60 * y1 + 60 * y0) / (6 * ::std::pow(x1, 6));
				f.x1 = x1;
				
				return f;
			}
			
			Polynomial* clone() const
			{
				return new Polynomial(*this);
			}
			
			T& coefficient(const ::std::size_t& i)
			{
				return this->c[i];
			}
			
			const T& coefficient(const ::std::size_t& i) const
			{
				return this->c[i];
			}
			
			::std::size_t degree() const
			{
				return this->c.size() - 1;
			}
			
			Polynomial derivative() const
			{
				Polynomial f(this->degree() > 0 ? this->degree() - 1 : 0);
				f.x0 = this->x0;
				f.x1 = this->x1;
				
				if (0 == this->degree())
				{
					f.c[0] = TypeTraits<T>::Zero(TypeTraits<T>::size(this->c[0]));
				}
				else
				{
					for (::std::size_t i = 0; i < this->degree(); ++i)
					{
						f.c[i] = static_cast<Real>(i + 1) * this->c[i + 1];
					}
				}
				
				return f;
			}
			
			/**
			 * Returns the array of the maximum function values of each dimension 
			 * within the definition range, not regarding the sign of the function values.
			 * 
			 * For polynomials higher than cubics, Eigen::PolynomialSolver is required
			 * and calculations become iterative, without guaranteeing convergence.
			 * A common use case is to verify speed limits with the comparison
			 * (trajectory.derivate().getAbsoluteMaximum() < maximumSpeed).all() 
			 */
			T getAbsoluteMaximum() const
			{
				Polynomial derivative = this->derivative();
				
				T maximum = TypeTraits<T>::abs((*this)(this->x0));
				T y1 = TypeTraits<T>::abs((*this)(this->x1));
				
				for (::std::ptrdiff_t row = 0; row < maximum.size(); ++row)
				{
					maximum[row] = ::std::max(maximum[row], y1[row]);
					::std::vector<Real> coefficients(derivative.degree() + 1);
					
					for (::std::size_t i = 0; i < derivative.degree() + 1; ++i)
					{
						coefficients[i] = derivative.coefficient(i)[row];
					}
					
					::std::vector<Real> extrema = this->realRoots(coefficients);
					
					for (::std::vector<Real>::const_iterator x = extrema.begin(); x < extrema.end(); ++x)
					{
						if (this->x0 < *x && *x < this->x1)
						{
							Real y = ::std::abs((*this)(*x)[row]);
							
							if (y > maximum[row])
							{
								maximum[row] = y;
							}
						}
					}
				}
				
				return maximum;
			}
			
			Polynomial integral() const
			{
				Polynomial f(this->degree() + 1);
				f.x0 = this->x0;
				f.x1 = this->x1;
				
				f.c[0] = TypeTraits<T>::Zero(TypeTraits<T>::size(this->c[0]));
				
				for (::std::size_t i = 0; i < this->degree() + 1; ++i)
				{
					f.c[i + 1] = this->c[i] / static_cast<Real>(i + 1);
				}
				
				return f;
			}
			
			T operator()(const Real& x, const ::std::size_t& derivative = 0) const
			{
				if (derivative > 0)
				{
					Polynomial f = this->derivative();
					
					for (::std::size_t i = 1; i < derivative; ++i)
					{
						f = f.derivative();
					}
					
					return f(x);
				}
				else
				{
					assert(x > this->lower() - FUNCTION_BOUNDARY);
					assert(x < this->upper() + FUNCTION_BOUNDARY);
					
					T y = this->c[this->degree()];
					
					for (::std::size_t i = 1; i < this->degree() + 1; ++i)
					{
						y *= x;
						y += this->c[this->degree() - i];
					}
					
					return y;
				}
			}
			
			friend Polynomial operator+(const T& lhs, const Polynomial& rhs)
			{
				return Polynomial(rhs) += lhs;
			}
			
			friend Polynomial operator+(const Polynomial& lhs, const T& rhs)
			{
				return Polynomial(lhs) += rhs;
			}
			
			Polynomial operator+(const Polynomial& rhs) const
			{
				return Polynomial(*this) += rhs;
			}
			
			Polynomial& operator+=(const T& rhs)
			{
				if (this->c.size() < 1)
				{
					this->c.resize(1);
					this->c[0] = rhs;
				}
				else
				{
					this->c[0] += rhs;
				}
				
				return *this;
			}
			
			Polynomial& operator+=(const Polynomial& rhs)
			{
				::std::size_t min = ::std::min(this->c.size(), rhs.c.size());
				
				for (::std::size_t i = 0; i < min; ++i)
				{
					this->c[i] += rhs.c[i];
				}
				
				this->c.resize(::std::max(this->c.size(), rhs.c.size()));
				
				for (::std::size_t i = min; i < rhs.c.size(); ++i)
				{
					this->c[i] = rhs.c[i];
				}
				
				return *this;
			}
			
			Polynomial operator-() const
			{
				return *this * -1;
			}
			
			friend Polynomial operator-(const T& lhs, const Polynomial& rhs)
			{
				return Polynomial(rhs) -= lhs;
			}
			
			friend Polynomial operator-(const Polynomial& lhs, const T& rhs)
			{
				return Polynomial(lhs) -= rhs;
			}
			
			Polynomial operator-(const Polynomial& rhs) const
			{
				return Polynomial(*this) -= rhs;
			}
			
			Polynomial& operator-=(const T& rhs)
			{
				if (this->c.size() < 1)
				{
					this->c.resize(1);
					this->c[0] = -rhs;
				}
				else
				{
					this->c[0] -= rhs;
				}
				
				return *this;
			}
			
			Polynomial& operator-=(const Polynomial& rhs)
			{
				::std::size_t min = ::std::min(this->c.size(), rhs.c.size());
				
				for (::std::size_t i = 0; i < min; ++i)
				{
					this->c[i] -= rhs.c[i];
				}
				
				this->c.resize(::std::max(this->c.size(), rhs.c.size()));
				
				for (::std::size_t i = min; i < rhs.c.size(); ++i)
				{
					this->c[i] = rhs.c[i];
				}
				
				return *this;
			}
			
			friend Polynomial operator*(const Real& lhs, const Polynomial& rhs)
			{
				return Polynomial(rhs) *= lhs;
			}
			
			friend Polynomial operator*(const Polynomial& lhs, const Real& rhs)
			{
				return Polynomial(lhs) *= rhs;
			}
			
			Polynomial& operator*=(const Real& rhs)
			{
				for (::std::size_t i = 0; i < this->c.size(); ++i)
				{
					this->c[i] *= rhs;
				}
				
				return *this;
			}
			
			friend Polynomial operator/(const Real& lhs, const Polynomial& rhs)
			{
				return Polynomial(rhs) /= lhs;
			}
			
			friend Polynomial operator/(const Polynomial& lhs, const Real& rhs)
			{
				return Polynomial(lhs) /= rhs;
			}
			
			Polynomial& operator/=(const Real& rhs)
			{
				return *this *= 1 / rhs;
			}
			
			/**
			 * Calculates the real roots (or, zeros) for given polynomial coefficients.
			 * For degrees higher than 3, Eigen::PolynomialSolver is required
			 * and calculations become iterative, without guaranteeing convergence.
			 */
			static ::std::vector<Real> realRoots(const ::std::vector<Real>& c)
			{
				::std::size_t degree = c.empty() ? 0 : c.size() - 1;
				
				for (::std::size_t i = 0; i < c.size(); ++i)
				{
					if (::std::abs(c[c.size() - 1 - i]) <= ::std::numeric_limits<Real>::epsilon())
					{
						degree = degree > 0 ? degree - 1 : degree;
					}
					else
					{
						break;
					}
				}
				
				if (degree < 1)
				{
					return ::std::vector<Real>(0);
				}
				else if (1 == degree)
				{
					::std::vector<Real> roots(1);
					roots[0] = -c[0] / c[1];
					return roots;
				}
				else if (2 == degree)
				{
					Real det = c[1] * c[1] - 4 * c[0] * c[2];
					
					if (det > 0)
					{
						Real sqrtDet = ::std::sqrt(det); 
						::std::vector<Real> roots(2);
						roots[0] = (-c[1] + sqrtDet) / (2 * c[2]);
						roots[1] = (-c[1] - sqrtDet) / (2 * c[2]);
						return roots;
					}
					else if (::std::abs(det) <= ::std::numeric_limits<Real>::epsilon())
					{
						::std::vector<Real> roots(1);
						roots[0] = -c[1] / (2 * c[2]);
						return roots;
					}
					else
					{
						return ::std::vector<Real>(0);
					}
				}
				else if (3 == degree)
				{
					Real p = (3 * c[3] * c[1] - ::std::pow(c[2], 2)) / (3 * ::std::pow(c[3], 2));
					Real q = (2 * ::std::pow(c[2], 3) - 9 * c[3] * c[2] * c[1] + 27 * ::std::pow(c[3], 2) * c[0]) / (27 * ::std::pow(c[3], 3));
					
					Real back = c[2] / (3 * c[3]);
					
					if (::std::abs(p) <= ::std::numeric_limits<Real>::epsilon() && ::std::abs(q) <= ::std::numeric_limits<Real>::epsilon())
					{
						::std::vector<Real> roots(1);
						roots[0] = 0 - back;
						return roots;
					}
					else if (::std::abs(p) <= ::std::numeric_limits<Real>::epsilon() && ::std::abs(q) > ::std::numeric_limits<Real>::epsilon())
					{
						::std::vector<Real> roots(1);
						roots[0] = cbrt(-q) - back;
						return roots;
					}
					else if (::std::abs(p) > ::std::numeric_limits<Real>::epsilon() && ::std::abs(q) <= ::std::numeric_limits<Real>::epsilon())
					{
						Real sqrtP = ::std::sqrt(-p);
						::std::vector<Real> roots(3);
						roots[0] = 0 - back;
						roots[1] = sqrtP - back;
						roots[2] = -sqrtP - back;
						return roots;
					}
					else if (::std::abs(4 * ::std::pow(p, 3) + 27 * ::std::pow(q, 2)) <= ::std::numeric_limits<Real>::epsilon() && ::std::abs(p) > ::std::numeric_limits<Real>::epsilon())
					{
						::std::vector<Real> roots(2);
						roots[0] = -(3 * q) / (2 * p) - back;
						roots[1] = (3 * q) / p - back;
						return roots;
					}
					else
					{
						Real det = ::std::pow(q, 2) / 4 + ::std::pow(p, 3) / 27;
						
						if (det < 0)
						{
							Real tmp1 = 2 * ::std::sqrt(-p / 3);
							Real tmp2 = ::std::acos((3 * q) / (2 * p) * ::std::sqrt(-3 / p)) / 3;
							::std::vector<Real> roots(3);
							roots[0] = tmp1 * ::std::cos(tmp2) - back;
							roots[1] = tmp1 * ::std::cos(tmp2 - 2 * M_PI / 3) - back;
							roots[2] = tmp1 * ::std::cos(tmp2 - 4 * M_PI / 3) - back;
							return roots;
						}
						else
						{
							Real sqrtDet = ::std::sqrt(det);
							Real u = cbrt(-q / 2 + sqrtDet);
							Real v = cbrt(-q / 2 - sqrtDet);
							::std::vector<Real> roots(1);
							roots[0] = u + v - back;
							return roots;
						}
					}
				}
				else
				{
#ifdef HAVE_EIGEN_UNSUPPORTED
					::std::vector<Real> roots;
					::Eigen::PolynomialSolver<Real, ::Eigen::Dynamic> solver(Vector::Map(c.data(), degree));
					solver.realRoots(roots);
					return roots;
#else
					Matrix companion(degree, degree);
					companion.topLeftCorner(1, degree).setZero();
					companion.bottomLeftCorner(degree - 1, degree).setIdentity();
					companion.rightCols(1) = -1 / c[degree] * Vector::Map(c.data(), degree);
					
					::Eigen::EigenSolver<Matrix> solver(companion);
					
					::std::vector<Real> roots;
					
					for (::std::ptrdiff_t i = 0; i < solver.eigenvalues().size(); ++i)
					{
						if (::std::abs(solver.eigenvalues()[i].imag()) < ::Eigen::NumTraits<Real>::dummy_precision())
						{
							roots.push_back(solver.eigenvalues()[i].real());
						}
					}
					
					return roots;
#endif
				}
			}
			
			/** 
			 * Stretches the x-axis of a polynomial by a given factor.
			 * 
			 * The returned, scaled polynomial p' of a given polynomial p fulfills
			 * p'(x * factor) = p(x), and p'.duration() = factor * p.duration().
			 * This is done by recalculating the underlying polynomial coefficients. 
			 */
			Polynomial scaledX(const Real& factor) const
			{
				assert(this->lower() == 0);
				
				Polynomial scaled(this->degree());
				scaled.upper() = this->upper() * factor;
				
				for (::std::size_t i = 0; i < this->degree() + 1; ++i)
				{
					scaled.coefficient(i) = ::std::pow(factor, -static_cast<int>(i)) * this->coefficient(i);
				}
				
				return scaled;
			}

			/**
			 * Translates a polynomial along the x-axis by a given parameter translation.
			 *
			 * The returned polynomial p' for a given polynomial p fulfills
			 * p'(x - translation) = p(x)
			 * This is done by recalculating the underlying polynomial coefficients.
			 */
			Polynomial translatedX(const Real& translation) const
			{
				Polynomial translated(this->degree());
				translated.lower() = this->lower() - translation;
				translated.upper() = this->upper() - translation;
				
				for (::std::size_t n = 0; n < this->degree() + 1; ++n)
				{
					translated.coefficient(n) = TypeTraits<T>::Zero(TypeTraits<T>::size(this->coefficient(n)));
				}

				for (::std::size_t n = 0; n < this->degree() + 1; ++n)
				{
					T c = this->coefficient(n);
					::std::size_t b = 1;
					
					for (::std::size_t k = 0; k <= n; ++k)
					{
						// b is the binomial coefficient n over k
						translated.coefficient(k) += b * ::std::pow(translation, static_cast<int>(n - k)) * c;

						// compute next binomial coefficient, integer division is ok because b is divisible
						b = b * (n - k) / (k + 1);
					}
				}

				return translated;
			}
			
		protected:
			::std::vector<T> c;
			
		private:
			
		};
	}
}

#endif // RL_MATH_POLYNOMIAL_H
