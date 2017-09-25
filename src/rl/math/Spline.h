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

#ifndef RL_MATH_SPLINE_H
#define RL_MATH_SPLINE_H

#include <limits>
#include <vector>

#include "Function.h"
#include "Polynomial.h"

namespace rl
{
	namespace math
	{
		/**
		 * A piecewise Function of Polynomial functions.
		 * 
		 * A Spline is a function that consists of a list of polynomials, which
		 * may have different degrees. It is indefinitely often differentiable,
		 * and can be scaled algebraically. As all children of Function, its is
		 * mapping Real -> T, but a Spline is only defined in the interval
		 * [lower() upper()].
		 * 
		 * @see Polynomial
		 */
		template<typename T>
		class Spline : public Function<T>
		{
		public:
			typedef typename ::std::vector<Polynomial<T>>::const_iterator ConstIterator;
			
			typedef typename ::std::vector<Polynomial<T>>::const_reverse_iterator ConstReverseIterator;
			
			typedef typename ::std::vector<Polynomial<T>>::iterator Iterator;
			
			typedef typename ::std::vector<Polynomial<T>>::reverse_iterator ReverseIterator;
			
			Spline() :
				Function<T>(),
				polynomials()
			{
				this->x0 = 0;
				this->x1 = 0;
			}
			
			virtual ~Spline()
			{
			}
			
			/**
			 * Generates a piecewise spline with parabolic segments around the
			 * given supporting points y and linear segments in between.
			 * 
			 * Note that the duration of the returned Spline is longer than the
			 * given x, because there is one parabolic interval more than segments
			 * in x.
			 * 
			 * @param[in] parabolicInterval Gives the duration of a parabolic interval,
			 * the following linear interval then has a duration of x(n+1) - x(n) -
			 * parabolicInterval
			 */
			template<typename Container1, typename Container2>
			static Spline LinearParabolic(const Container1& x, const Container2& y, const Real& parabolicInterval)
			{
				assert(x.size() > 1);
				assert(x.size() == y.size());
				
				::std::size_t n = y.size();
				::std::size_t dim = TypeTraits<T>::size(y[0]);
				
				Spline f;
				
				{
					T yd = (y[1] - y[0]) / (x[1] - x[0]);
					T yBeforeLinear = y[0] + yd * (parabolicInterval / 2);
					Real linearInterval = (x[1] - x[0]) - parabolicInterval;
					assert(linearInterval > 0);
					T yAfterLinear = y[0] + yd * (parabolicInterval / 2 + linearInterval);
					
					Polynomial<T> parabolic = Polynomial<T>::Quadratic(y[0], yBeforeLinear, TypeTraits<T>::Zero(dim), parabolicInterval);
					f.push_back(parabolic);
					Polynomial<T> linear = Polynomial<T>::Linear(yBeforeLinear, yAfterLinear, linearInterval);
					f.push_back(linear);
				}
				
				for (::std::size_t i = 1; i < n - 1; ++i)
				{
					Real deltaXPrev = x[i] - x[i - 1];
					Real deltaXNext = x[i + 1] - x[i];
					Real linearInterval = deltaXNext - parabolicInterval;
					assert(deltaXPrev > parabolicInterval && deltaXNext > parabolicInterval);
					T ydPrev = (y[i] - y[i - 1]) / deltaXPrev;
					T ydNext = (y[i + 1] - y[i]) / deltaXNext;
					T yBeforeParabolic = y[i] + (-parabolicInterval / 2 * ydPrev);
					T yBeforeLinear = y[i] + (parabolicInterval / 2 * ydNext);
					T yAfterLinear = y[i] + ((parabolicInterval / 2 + linearInterval) * ydNext);
					
					Polynomial<T> parabolic = Polynomial<T>::Quadratic(yBeforeParabolic, yBeforeLinear, ydPrev, parabolicInterval);
					f.push_back(parabolic);
					Polynomial<T> linear = Polynomial<T>::Linear(yBeforeLinear, yAfterLinear, linearInterval);
					f.push_back(linear);
				}
				
				{
					T yd = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]);
					T yBeforeParabolic = y[n - 1] - yd * (parabolicInterval / 2);

					Polynomial<T> parabolic = Polynomial<T>::Quadratic(yBeforeParabolic, TypeTraits<T>::Zero(dim), yd, parabolicInterval);
					f.push_back(parabolic);
				}
				
				return f;
			}
			
			/**
			 * Generates a piecewise spline with quartic polynomial segments around
			 * the given supporting points y and linear segments in between.
			 * 
			 * Note that the duration of the returned Spline is longer than the
			 * given x, because there is one quartic polynomial interval more than
			 * segments in x.
			 * 
			 * @param[in] quarticInterval Gives the duration of a quartic interval,
			 * the following linear interval then has a duration of x(n+1) - x(n) -
			 * quarticInterval
			 */
			template<typename Container1, typename Container2>
			static Spline LinearQuartic(const Container1& x, const Container2& y, const Real& quarticInterval)
			{
				assert(x.size() > 1);
				assert(x.size() == y.size());
				
				::std::size_t n = y.size();
				::std::size_t dim = TypeTraits<T>::size(y[0]);
				
				Spline f;
				
				{
					T yd = (y[1] - y[0]) / (x[1] - x[0]);
					T yBeforeLinear = y[0] + yd * (quarticInterval / 2);
					Real linearInterval = (x[1] - x[0]) - quarticInterval;
					assert(linearInterval > 0);
					T yAfterLinear = y[0] + yd * (quarticInterval / 2 + linearInterval);
					
					Polynomial<T> quartic = Polynomial<T>::QuarticFirstSecond(y[0], yBeforeLinear, TypeTraits<T>::Zero(dim), yd, TypeTraits<T>::Zero(dim), quarticInterval);
					f.push_back(quartic);
					Polynomial<T> linear = Polynomial<T>::Linear(yBeforeLinear, yAfterLinear, linearInterval);
					f.push_back(linear);
				}
				
				for (::std::size_t i = 1; i < n - 1; ++i)
				{
					Real deltaXPrev = x[i] - x[i - 1];
					Real deltaXNext = x[i + 1] - x[i];
					Real linearInterval = deltaXNext - quarticInterval;
					assert(deltaXPrev > quarticInterval && deltaXNext > quarticInterval);
					T ydPrev = (y[i] - y[i - 1]) / deltaXPrev;
					T ydNext = (y[i + 1] - y[i]) / deltaXNext;
					T yBeforeQuartic = y[i] + (-quarticInterval / 2 * ydPrev);
					T yBeforeLinear = y[i] + (quarticInterval / 2 * ydNext);
					T yAfterLinear = y[i] + ((quarticInterval / 2 + linearInterval) * ydNext); 
					
					Polynomial<T> quartic = Polynomial<T>::QuarticFirstSecond(yBeforeQuartic, yBeforeLinear, ydPrev, ydNext, TypeTraits<T>::Zero(dim), quarticInterval);
					f.push_back(quartic);
					Polynomial<T> linear = Polynomial<T>::Linear(yBeforeLinear, yAfterLinear, linearInterval);
					f.push_back(linear);
				}
				
				{
					T yd = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]);
					T yBeforeQuartic = y[n - 1] - yd * (quarticInterval / 2);
					
					Polynomial<T> quartic = Polynomial<T>::QuarticFirstSecond(yBeforeQuartic, y[n - 1], yd, TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), quarticInterval);
					f.push_back(quartic);
				}
				
				return f;
			}
			
			/**
			 * Generates a piecewise spline with sextic polynomial segments around
			 * the given supporting points y and linear segments in between.
			 * 
			 * Note that the duration of the returned Spline is longer than the
			 * given x, because there is one sextic polynomial interval more than
			 * segments in x.
			 * 
			 * @param[in] sexticInterval Gives the duration of a sextic interval,
			 * the following linear interval then has a duration of x(n+1) - x(n) -
			 * sexticInterval
			 */
			template<typename Container1, typename Container2>
			static Spline LinearSextic(const Container1& x, const Container2& y, const Real& sexticInterval)
			{
				assert(x.size() > 1);
				assert(x.size() == y.size());
				
				::std::size_t n = y.size();
				::std::size_t dim = TypeTraits<T>::size(y[0]);
				
				Spline f;
				
				{
					T yd = (y[1] - y[0]) / (x[1] - x[0]);
					T yBeforeLinear = y[0] + yd * (sexticInterval / 2);
					Real linearInterval = (x[1] - x[0]) - sexticInterval;
					assert(linearInterval > 0);
					T yAfterLinear = y[0] + yd * (sexticInterval / 2 + linearInterval);
					
					Polynomial<T> sextic = Polynomial<T>::SexticFirstSecondThird(y[0], yBeforeLinear, TypeTraits<T>::Zero(dim), yd, TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), sexticInterval);
					f.push_back(sextic);
					Polynomial<T> linear = Polynomial<T>::Linear(yBeforeLinear, yAfterLinear, linearInterval);
					f.push_back(linear);
				}
				
				for (::std::size_t i = 1; i < n - 1; ++i)
				{
					Real deltaXPrev = x[i] - x[i - 1];
					Real deltaXNext = x[i + 1] - x[i];
					Real linearInterval = deltaXNext - sexticInterval;
					assert(deltaXPrev > sexticInterval && deltaXNext > sexticInterval);
					T ydPrev = (y[i] - y[i - 1]) / deltaXPrev;
					T ydNext = (y[i + 1] - y[i]) / deltaXNext;
					T yBeforeSextic = y[i] + (-sexticInterval / 2 * ydPrev);
					T yBeforeLinear = y[i] + (sexticInterval / 2 * ydNext);
					T yAfterLinear = y[i] + ((sexticInterval / 2 + linearInterval) * ydNext); 
					
					Polynomial<T> sextic = Polynomial<T>::SexticFirstSecondThird(yBeforeSextic, yBeforeLinear, ydPrev, ydNext, TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), sexticInterval);
					f.push_back(sextic);
					Polynomial<T> linear = Polynomial<T>::Linear(yBeforeLinear, yAfterLinear, linearInterval);
					f.push_back(linear);
				}
				
				{
					T yd = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]);
					T yBeforeSextic = y[n - 1] - yd * (sexticInterval / 2);
					
					Polynomial<T> sextic = Polynomial<T>::SexticFirstSecondThird(yBeforeSextic, y[n - 1], yd, TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), sexticInterval);
					f.push_back(sextic);
				}
				
				return f;
			}
			
#if !(defined(_MSC_VER) && _MSC_VER < 1800)
			/**
			 * Generates a spline of polynomials of degrees 4-1-4 from rest to rest
			 * for one dimension.
			 * 
			 * Its acceleration and deceleration segments are parabolic and reach amax.
			 * vmax may or may not be reached; in the latter case, the linear segment
			 * is omitted.
			 * The result is the shortest such 4-1-4 spline with continuous acceleration.
			 */
			template<typename U = T>
			static Spline QuarticLinearQuarticAtRest(
				const typename ::std::enable_if< ::std::is_floating_point<U>::value, U>::type& q0,
				const typename ::std::enable_if< ::std::is_floating_point<U>::value, U>::type& q1,
				const typename ::std::enable_if< ::std::is_floating_point<U>::value, U>::type& vmax,
				const typename ::std::enable_if< ::std::is_floating_point<U>::value, U>::type& amax
			)
			{
				T xta = 3 * ::std::pow(vmax, 2) / (4 * amax); // travel distance during acceleration (or, deceleration)
				T distance = ::std::abs(q1 - q0);
				T xl = distance - 2 * xta; // travel distance during linear part
				T sign = (q1 > q0) ? 1 : -1;
				
				Spline f;
				
				if (xl > 0)
				{
					// vmax is reached, linear segment exists
					T tl = xl / vmax;
					T ta = 3 * vmax / (2 * amax);
					Polynomial<T> acc = Polynomial<T>::QuarticFirstSecond(q0, q0 + sign * xta, 0, sign * vmax, 0, ta);
					f.push_back(acc);
					Polynomial<T> lin = Polynomial<T>::Linear(q0 + sign * xta, q1 - sign * xta, tl);
					f.push_back(lin);
					Polynomial<T> dec = Polynomial<T>::QuarticFirstSecond(q1 - sign * xta, q1, sign * vmax, 0, 0, ta);
					f.push_back(dec);
				}
				else
				{
					// vmax is not reached
					T vreached = ::std::sqrt(2 * distance * amax / 3);
					T th = ::std::sqrt(3 * distance / (2 * amax));
					T xh = (q0 + q1) / 2;
					Polynomial<T> acc = Polynomial<T>::QuarticFirstSecond(q0, xh, 0, sign * vreached, 0, th);
					f.push_back(acc);
					Polynomial<T> dec = Polynomial<T>::QuarticFirstSecond(xh, q1, sign * vreached, 0, 0, th);
					f.push_back(dec);
				}
				
				return f;
			}
#endif
			
			/**
			 * Generates a spline of polynomials of degrees 4-1-4 from rest to rest
			 * that is phase-synchronized for multiple degree-of-freedoms.
			 * 
			 * Its acceleration and deceleration segments are parabolic and at least
			 * one degree-of-freedon reaches its maximum acceleration amax.
			 * vmax may or may not be reached; in the latter case, the linear segment
			 * is omitted.
			 * The result is the shortest synchronized 4-1-4 spline with continuous acceleration.
			 */
#if !(defined(_MSC_VER) && _MSC_VER < 1800)
			template<typename U = T>
			static Spline QuarticLinearQuarticAtRest(
				const typename ::std::enable_if< ::std::is_class<U>::value, U>::type& q0,
				const typename ::std::enable_if< ::std::is_class<U>::value, U>::type& q1,
				const typename ::std::enable_if< ::std::is_class<U>::value, U>::type& vmax,
				const typename ::std::enable_if< ::std::is_class<U>::value, U>::type& amax
			)
#else
			static Spline QuarticLinearQuarticAtRest(const T& q0, const T& q1, const T& vmax, const T& amax)
#endif
			{
				assert(q0.size() >= 1 && q0.size() == q1.size() && q0.size() == vmax.size() && q0.size() == amax.size() && "QuarticLinearQuarticAtRest: parameters must have same dimension.");
				
				::std::size_t dim = TypeTraits<T>::size(q0);
				
				// Find minimal durations for each dof independently
				T tAcc(dim);
				T tLin(dim);
				
				for (::std::size_t i = 0; i < dim; ++i)
				{
					Real xta = 3 * ::std::pow(vmax[i], 2) / (4 * amax[i]); // travel distance during acceleration (or, deceleration)
					Real distance = ::std::abs(q1[i] - q0[i]);
					Real xl = distance - 2 * xta; // travel distance during linear part
					
					if (xl > 0)
					{
						// vmax is reached, linear segment exists
						tAcc[i] = 3 * vmax[i] / (2 * amax[i]);
						tLin[i] = xl / vmax[i];
					}
					else
					{
						// vmax is not reached
						//Real vreached = ::std::sqrt(2 * distance * amax(i) / 3);
						tAcc[i] = ::std::sqrt(3 * distance / (2 * amax[i]));
						tLin[i] = 0;
					}
				}
				
				// Use slowest degree-of-freedom to synchronize all
				Real tAccMax = TypeTraits<T>::max_element(tAcc);
				Real tLinMax = TypeTraits<T>::max_element(tLin);
				
				T sign(dim);
				T xAfterAcc(dim);
				T vReached(dim);
				
				for (::std::size_t i = 0; i < dim; ++i)
				{
					sign[i] = (q1[i] > q0[i]) ? 1 : -1;
					Real distance = ::std::abs(q1[i] - q0[i]);
					xAfterAcc[i] = tAccMax * distance / (2 * tLinMax + 2 * tAccMax);
					vReached[i] = distance / (tLinMax + tAccMax);
				}
				
				Spline f;
				
				Polynomial<T> acc = Polynomial<T>::QuarticFirstSecond(q0, q0 + sign * xAfterAcc, TypeTraits<T>::Zero(dim), sign * vReached, TypeTraits<T>::Zero(dim), tAccMax);
				f.push_back(acc);
				
				if (tLinMax > 0)
				{
					Polynomial<T> lin = Polynomial<T>::Linear(q0 + sign * xAfterAcc, q1 - sign * xAfterAcc, tLinMax);
					f.push_back(lin);
				}
				
				Polynomial<T> dec = Polynomial<T>::QuarticFirstSecond(q1 - sign * xAfterAcc, q1, sign * vReached, TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), tAccMax);
				f.push_back(dec);
				
				return f;
			}
			
#if !(defined(_MSC_VER) && _MSC_VER < 1800)
			/**
			 * Generates a spline of polynomials of degrees 6-1-6 from rest to rest
			 * for one dimension.
			 * 
			 * Its acceleration and deceleration segments are quartic and reach amax.
			 * vmax may or may not be reached; in the latter case, the linear segment
			 * is omitted.
			 * Compared to QuarticLinearQuarticAtRest, SexticLinearSexticAtRest
			 * makes sure the jerk is continuous, which results in a smoother and 
			 * slightly longer trajectory.
			 * The result is the shortest such 6-1-6 spline.
			 */
			template<typename U = T>
			static Spline SexticLinearSexticAtRest(
				const typename ::std::enable_if< ::std::is_floating_point<U>::value, U>::type& q0,
				const typename ::std::enable_if< ::std::is_floating_point<U>::value, U>::type& q1,
				const typename ::std::enable_if< ::std::is_floating_point<U>::value, U>::type& vmax,
				const typename ::std::enable_if< ::std::is_floating_point<U>::value, U>::type& amax
			)
			{
				T xta = 15 * ::std::pow(vmax, 2) / (16 * amax); // travel distance during acceleration (or, deceleration)
				T distance = ::std::abs(q1 - q0);
				T xl = distance - 2 * xta; // travel distance during linear part
				T sign = (q1 > q0) ? 1 : -1;
				
				Spline f;
				
				if (xl > 0)
				{
					// vmax is reached, linear segment exists
					T tl = xl / vmax;
					T ta = 15 * vmax / (8 * amax);
					Polynomial<T> acc = Polynomial<T>::SexticFirstSecondThird(q0, q0 + sign * xta, 0, sign * vmax, 0, 0, 0, ta);
					f.push_back(acc);
					Polynomial<T> lin = Polynomial<T>::Linear(q0 + sign * xta, q1 - sign * xta, tl);
					f.push_back(lin);
					Polynomial<T> dec = Polynomial<T>::SexticFirstSecondThird(q1 - sign * xta, q1, sign * vmax, 0, 0, 0, 0, ta);
					f.push_back(dec);
				}
				else
				{
					// vmax is not reached
					T vreached = ::std::sqrt(8 * distance * amax / 15);
					T th = ::std::sqrt(15 * distance / (8 * amax));
					T xh = (q0 + q1) / 2;
					Polynomial<T> acc = Polynomial<T>::SexticFirstSecondThird(q0, xh, 0, sign * vreached, 0, 0, 0, th);
					f.push_back(acc);
					Polynomial<T> dec = Polynomial<T>::SexticFirstSecondThird(xh, q1, sign * vreached, 0, 0, 0, 0, th);
					f.push_back(dec);
				}
				
				return f;
			}
#endif
			
			/**
			 * Generates a spline of polynomials of degrees 6-1-6 from rest to rest
			 * that is phase-synchronized for multiple degree-of-freedoms.
			 * 
			 * Its acceleration and deceleration segments are quartic and at least
			 * one degree-of-freedon reaches its maximum acceleration amax.
			 * vmax may or may not be reached; in the latter case, the linear segment
			 * is omitted.
			 * The result is the shortest synchronized 6-1-6 spline with continuous jerk.
			 */
#if !(defined(_MSC_VER) && _MSC_VER < 1800)
			template<typename U = T>
			static Spline SexticLinearSexticAtRest(
				const typename ::std::enable_if< ::std::is_class<U>::value, U>::type& q0,
				const typename ::std::enable_if< ::std::is_class<U>::value, U>::type& q1,
				const typename ::std::enable_if< ::std::is_class<U>::value, U>::type& vmax,
				const typename ::std::enable_if< ::std::is_class<U>::value, U>::type& amax
			)
#else
			static Spline SexticLinearSexticAtRest(const T& q0, const T& q1, const T& vmax, const T& amax)
#endif
			{
				assert(q0.size() >= 1 && q0.size() == q1.size() && q0.size() == vmax.size() && q0.size() == amax.size() && "SexticLinearSexticAtRest: parameters must have same dimension.");
				
				::std::size_t dim = TypeTraits<T>::size(q0);
				
				// Find minimal durations for each dof independently
				T tAcc(dim);
				T tLin(dim);
				
				for (::std::size_t i = 0; i < dim; ++i)
				{
					Real xta = 15 * ::std::pow(vmax[i], 2) / (16 * amax[i]); // travel distance during acceleration (or, deceleration)
					Real distance = ::std::abs(q1[i] - q0[i]);
					Real xl = distance - 2 * xta; // travel distance during linear part
					
					if (xl > 0)
					{
						// vmax is reached, linear segment exists
						tAcc[i] = 15 * vmax[i] / (8 * amax[i]);
						tLin[i] = xl / vmax[i];
					}
					else
					{
						// vmax is not reached
						//Real vreached = ::std::sqrt(8 * distance * amax(i) / 15);
						tAcc[i] = ::std::sqrt(15 * distance / (8 * amax[i]));
						tLin[i] = 0;
					}
				}
				
				// Use slowest degree-of-freedom to synchronize all
				Real tAccMax = TypeTraits<T>::max_element(tAcc);
				Real tLinMax = TypeTraits<T>::max_element(tLin);
				
				T sign(dim);
				T xAfterAcc(dim);
				T vReached(dim);
				
				for (::std::size_t i = 0; i < dim; ++i)
				{
					sign[i] = (q1[i] > q0[i]) ? 1 : -1;
					Real distance = ::std::abs(q1[i] - q0[i]);
					xAfterAcc[i] = tAccMax * distance / (2 * tLinMax + 2 * tAccMax);
					vReached[i] = distance / (tLinMax + tAccMax);
				}
				
				Spline f;
				
				Polynomial<T> acc = Polynomial<T>::SexticFirstSecondThird(q0, q0 + sign * xAfterAcc,  TypeTraits<T>::Zero(dim), sign * vReached, TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), tAccMax);
				f.push_back(acc);
				
				if (tLinMax > 0)
				{
					Polynomial<T> lin = Polynomial<T>::Linear(q0 + sign * xAfterAcc, q1 - sign * xAfterAcc, tLinMax);
					f.push_back(lin);
				}
				
				Polynomial<T> dec = Polynomial<T>::SexticFirstSecondThird(q1 - sign * xAfterAcc, q1, sign * vReached, TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim),  TypeTraits<T>::Zero(dim), TypeTraits<T>::Zero(dim), tAccMax);
				f.push_back(dec);
				
				return f;
			}
			
			/**
			 * Generates a trapezoidal acceleration trajectory from rest to rest
			 * for multiple dimensions that are phase-synchronized.
			 * 
			 * A trapezoidal acceleration trajectory has up to seven segments of 
			 * constant jerk. Its velocity curve is double-S shaped.
			 * 
			 * see L. Biagiotti, C. Melchiorri (2008) "Trajectory Planning for Automatic
			 * Machines and Robots", pp. 90ff.
			 */
			static Spline TrapeziodalAccelerationAtRest(const T& q0, const T& q1, const T& vmax, const T& amax, const T& jmax)
			{
//				assert((vmax > 0).all() && (amax > 0).all() && (jmax > 0).all() && "TrapeziodalAccelerationAtRest: velocity, acceleration and jerk limits must be positive.");
				
				::std::size_t dim = TypeTraits<T>::size(q0);
				
				// Find minimal durations for each dof independently
				T sign(dim);
				T tJerk(dim);
				T tAcc(dim);
				T tLin(dim);
				T vReached(dim);
				
				for (::std::size_t i = 0; i < dim; ++i)
				{
					sign(i) = (q1(i) > q0(i)) ? 1 : -1;
					Real d = ::std::abs(q1(i) - q0(i));
					Real tj;
					Real ta;

					// assume vmax is reached
					if (vmax(i) * jmax(i) >= amax(i) * amax(i))
					{
						tj = amax(i) / jmax(i); // amax reached
						ta = tj + vmax(i) / amax(i);
					}
					else
					{
						tj = ::std::sqrt(vmax(i) / jmax(i)); // amax not reached
						ta = 2 * tj;
					}
					
					Real tv = d / vmax(i) - ta;
					vReached(i) = sign(i) * vmax(i);
					
					if (tv <= 0)
					{
						tv = 0; // vmax not reached, recalculate
						
						if (d >= 2 * ::std::pow(amax(i), 3) / ::std::pow(jmax(i), 2))
						{
							tj = amax(i) / jmax(i); // amax reached
							ta = tj / 2 + ::std::sqrt(::std::pow(tj / 2, 2) + d / amax(i));
							vReached(i) = sign(i) * ((ta - tj) * amax(i));
						}
						else
						{
							tj = ::std::pow(d / (2 * jmax(i)), 1 / 3.); // amax not reached
							ta = 2 * tj;
							vReached(i) = sign(i) * (tj * tj * jmax(i));
						}
					}
					
					tAcc(i) = ta;
					tLin(i) = tv;
					tJerk(i) = tj;
				}
				
				// Use slowest degree-of-freedom to synchronize all with tjFixed, tAccMax, tLinMax
				Real tjFixed = TypeTraits<T>::max_element(tJerk);
				Real tAccMax = TypeTraits<T>::max_element(tAcc);
				Real tLinMax = TypeTraits<T>::max_element(tLin);
				
				T vLin(dim);
				T qBeforeLin(dim);
				T aAcc(dim);
				T vAfterJerk(dim);
				T xAfterJerk(dim);
				
				for (::std::size_t i = 0; i < dim; ++i)
				{
					// Useful positions and velocities to compute spline
					vLin(i) = (q1(i) - q0(i)) / (tAccMax + tLinMax);
					qBeforeLin(i) = (q1(i) + q0(i)) / 2.0f - (tLinMax / 2.0f) * vLin(i);
					aAcc(i) = vLin(i) / (tAccMax - tjFixed);
					vAfterJerk(i) = aAcc(i) * tjFixed / 2;
					xAfterJerk(i) = aAcc(i) * tjFixed * tjFixed / 6;
				}
				
				Spline f;
				
				Polynomial<T> jerk1 = Polynomial<T>::CubicFirst(q0, q0 + xAfterJerk, 0 * vmax, vAfterJerk, tjFixed);
				f.push_back(jerk1);
				
				T vAfterConstAcc = vAfterJerk;
				T xAfterConstAcc = xAfterJerk;
				
				if (2 * tjFixed < tAccMax)
				{
					Real tConstAcc = tAccMax - 2 * tjFixed;
					vAfterConstAcc += tConstAcc * aAcc;
					xAfterConstAcc += tConstAcc * (vAfterJerk + vAfterConstAcc) / 2;
					Polynomial<T> acc1 = Polynomial<T>::Quadratic(q0 + xAfterJerk, q0 + xAfterConstAcc, vAfterJerk, tConstAcc);
					f.push_back(acc1);
				}
				
				Polynomial<T> jerk2 = Polynomial<T>::CubicFirst(q0 + xAfterConstAcc, qBeforeLin, vAfterConstAcc, vLin, tjFixed);
				f.push_back(jerk2);
				
				if (tLinMax > 0)
				{
					Polynomial<T> lin = Polynomial<T>::Linear(qBeforeLin, qBeforeLin + tLinMax * vLin, tLinMax);
					f.push_back(lin);
				}
				
				Polynomial<T> jerk3 = Polynomial<T>::CubicFirst(qBeforeLin + tLinMax * vLin, q1 - xAfterConstAcc, vLin, vAfterConstAcc, tjFixed);
				f.push_back(jerk3);
				
				if (2 * tjFixed < tAccMax)
				{
					Real tConstAcc = tAccMax - 2 * tjFixed;
					Polynomial<T> acc2 = Polynomial<T>::Quadratic(q1 - xAfterConstAcc, q1 - xAfterJerk, vAfterConstAcc, tConstAcc);
					f.push_back(acc2);
				}
				
				Polynomial<T> jerk4 = Polynomial<T>::CubicFirst(q1 - xAfterJerk, q1, vAfterJerk, 0 * vmax, tjFixed);
				f.push_back(jerk4);
				
				return f;
			}
			
			Polynomial<T>& at(const ::std::size_t& i)
			{
				return this->polynomials.at(i);
			}
			
			const Polynomial<T>& at(const ::std::size_t& i) const
			{
				return this->polynomials.at(i);
			}
			
			Polynomial<T>& back()
			{
				return this->polynomials.back();
			}
			
			const Polynomial<T>& back() const
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
				this->x0 = 0;
				this->x1 = 0;
			}
			
			Spline* clone() const
			{
				return new Spline(*this);
			}
			
			Spline derivative() const
			{
				Spline spline;
				
				for (::std::size_t i = 0; i < this->size(); ++i)
				{
					Polynomial<T> polynomial = this->polynomials[i].derivative();
					spline.push_back(polynomial);
				}
				
				return spline;
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
			
			Polynomial<T>& front()
			{
				return this->polynomials.front();
			}
			
			const Polynomial<T>& front() const
			{
				return this->polynomials.front();
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
				T maximum = TypeTraits<T>::abs((*this)(this->x0));
				
				for (::std::size_t i = 0; i < this->size(); ++i)
				{
					T maximumOfPolynomial = this->polynomials[i].getAbsoluteMaximum();
					
					for (::std::ptrdiff_t row = 0; row < maximum.size(); ++row)
					{
						Real y = maximumOfPolynomial[row];
						
						if (y > maximum[row])
						{
							maximum[row] = y;
						}
					}
				}
			
				return maximum;
			}
			
			/**
			 * Verifies that the spline is smooth and has no jumps 
			 * at the piecewise function boundaries.
			 * 
			 * Mathematically, it checks whether the spline and a certain number
			 * of its derivatives are continuous.
			 *  
			 * @param[in] upToDerivative Sets the number of derivatives that are
			 * also checked for continuity.
			 */
			bool isContinuous(const ::std::size_t& upToDerivative = 1) const
			{
				for (::std::size_t d = 0; d <= upToDerivative; ++d)
				{
					for (::std::size_t i = 1; i < this->polynomials.size(); ++i)
					{
						T xBefore = this->polynomials[i - 1](this->polynomials[i - 1].upper(), d);
						T xAfter = this->polynomials[i](this->polynomials[i].lower(), d);
						return TypeTraits<T>::equal(xBefore, xAfter);
					}
				}
				
				return true;
			}
			
			T operator()(const Real& x, const ::std::size_t& derivative = 0) const
			{
				assert(x >= this->lower() - FUNCTION_BOUNDARY);
				assert(x <= this->upper() + FUNCTION_BOUNDARY);
				assert(this->polynomials.size() > 0);
				
				Real x0 = this->lower();
				::std::size_t i = 0;
				
				for (; x > x0 + this->polynomials[i].duration() && i + 1 < this->polynomials.size(); ++i)
				{
					x0 += this->polynomials[i].duration();
				}
				
				return this->polynomials[i](x - x0, derivative);
			}
			
			Polynomial<T>& operator[](const ::std::size_t& i)
			{
				return this->polynomials[i];
			}
			
			const Polynomial<T>& operator[](const ::std::size_t& i) const
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
			
			void push_back(Polynomial<T>& polynomial)
			{
				if (this->polynomials.empty())
				{
					this->x0 = polynomial.lower();
				}
				
				this->polynomials.push_back(polynomial);
				this->x1 += polynomial.duration();
			}
			
			void push_back(Spline& spline)
			{
				for (::std::size_t i = 0; i < spline.polynomials.size(); ++i)
				{
					this->push_back(spline.polynomials[i]);
				}
			}
			
			/** 
			 * Stretches the x-axis of a spline by a given factor.
			 * 
			 * The returned, scaled spline s' of a given spline s fulfills
			 * s'(x * factor) = s(x), and s'.duration() = factor * s.duration().
			 * This is done by recalculating the underlying polynomial coefficients. 
			 */
			Spline scaledX(const Real& factor) const
			{
				Spline spline;
				
				for (::std::size_t i = 0; i < this->size(); ++i)
				{
					Polynomial<T> polynomial = this->polynomials[i].scaledX(factor);
					spline.push_back(polynomial);
				}
				
				return spline;
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
			
		protected:
			::std::vector<Polynomial<T>> polynomials;
			
		private:
			
		};
	}
}

#endif // RL_MATH_SPLINE_H
