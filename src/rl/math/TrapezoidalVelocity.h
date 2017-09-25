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

#ifndef RL_MATH_TRAPEZOIDALVELOCITY_H
#define RL_MATH_TRAPEZOIDALVELOCITY_H

#include <cassert>
#include <cmath>
#include <limits>

#include "Real.h"

namespace rl
{
	namespace math
	{
		template<typename T>
		class TrapezoidalVelocity
		{
		public:
			TrapezoidalVelocity() :
				am(0),
				dm(0),
				x0(0),
				xe(0),
				v0(0),
				ve(0),
				vm(0),
				ah(0),
				dh(0),
				ta(::std::numeric_limits<Real>::max()),
				tc(::std::numeric_limits<Real>::max()),
				td(::std::numeric_limits<Real>::max())
			{
			}
			
			virtual ~TrapezoidalVelocity()
			{
			}
			
			T a(const Real& t) const
			{
				if (t < ta)
				{
					return ah;
				}
				else if (t < ta + tc)
				{
					return 0;
				}
				else if (t < ta + tc + td)
				{
					return -dh;
				}
				else
				{
					return 0;
				}
			}
			
			void interpolate()
			{
				T x = xe - x0;
				
				T ta1 = ( vm - v0) /  am;
				T ta2 = ( vm - v0) /  am;
				T ta3 = (-vm - v0) /  am;
				T ta4 = (-vm - v0) /  am;
				T ta5 = ( vm - v0) / -am;
				T ta6 = ( vm - v0) / -am;
				T ta7 = (-vm - v0) / -am;
				T ta8 = (-vm - v0) / -am;
				
				T tc1 =  ( dm * v0 * v0 - dm * vm * vm - am * vm * vm + am * ve * ve + 2 * x * am * dm) / vm / am / dm / 2;
				T tc2 =  ( dm * v0 * v0 - dm * vm * vm + am * vm * vm - am * ve * ve + 2 * x * am * dm) / vm / am / dm / 2;
				T tc3 = -( dm * v0 * v0 - dm * vm * vm - am * vm * vm + am * ve * ve + 2 * x * am * dm) / vm / dm / am / 2;
				T tc4 = -( dm * v0 * v0 - dm * vm * vm + am * vm * vm - am * ve * ve + 2 * x * am * dm) / vm / am / dm / 2;
				T tc5 =  (-dm * v0 * v0 + dm * vm * vm - am * vm * vm + am * ve * ve + 2 * x * am * dm) / vm / am / dm / 2;
				T tc6 =  (-dm * v0 * v0 + dm * vm * vm + am * vm * vm - am * ve * ve + 2 * x * am * dm) / vm / am / dm / 2;
				T tc7 = -(-dm * v0 * v0 + dm * vm * vm - am * vm * vm + am * ve * ve + 2 * x * am * dm) / vm / am / dm / 2;
				T tc8 = -(-dm * v0 * v0 + dm * vm * vm + am * vm * vm - am * ve * ve + 2 * x * am * dm) / vm / am / dm / 2;
				
				T td1 = (ve - vm) / -dm;
				T td2 = (ve - vm) /  dm;
				T td3 = (ve + vm) / -dm;
				T td4 = (ve + vm) /  dm;
				T td5 = (ve - vm) / -dm;
				T td6 = (ve - vm) /  dm;
				T td7 = (ve + vm) / -dm;
				T td8 = (ve + vm) /  dm;
				
				T t1 = ta1 + tc1 + td1;
				T t2 = ta2 + tc2 + td2;
				T t3 = ta3 + tc3 + td3;
				T t4 = ta4 + tc4 + td4;
				T t5 = ta5 + tc5 + td5;
				T t6 = ta6 + tc6 + td6;
				T t7 = ta7 + tc7 + td7;
				T t8 = ta8 + tc8 + td8;
				
				T t = ::std::numeric_limits<T>::max();
				
				T vh = vm;
				
				//  am,  vm, -dm
				if (ta1 >= 0 && tc1 >= 0 && td1 >= 0 && ::std::abs(t1) < ::std::abs(t))
				{
					t = t1;
					ta = ta1;
					tc = tc1;
					td = td1;
					ah =  am;
					vh =  vm;
					dh =  dm;
				}
				//  am,  vm,  dm
				if (ta2 >= 0 && tc2 >= 0 && td2 >= 0 && ::std::abs(t2) < ::std::abs(t))
				{
					t = t2;
					ta = ta2;
					tc = tc2;
					td = td2;
					ah =  am;
					vh =  vm;
					dh = -dm;
				}
				//  am, -vm, -dm
				if (ta3 >= 0 && tc3 >= 0 && td3 >= 0 && ::std::abs(t3) < ::std::abs(t))
				{
					t = t3;
					ta = ta3;
					tc = tc3;
					td = td3;
					ah =  am;
					vh = -vm;
					dh =  dm;
				}
				//  am, -vm,  dm
				if (ta4 >= 0 && tc4 >= 0 && td4 >= 0 && ::std::abs(t4) < ::std::abs(t))
				{
					t = t4;
					ta = ta4;
					tc = tc4;
					td = td4;
					ah =  am;
					vh = -vm;
					dh = -dm;
				}
				// -am,  vm, -dm
				if (ta5 >= 0 && tc5 >= 0 && td5 >= 0 && ::std::abs(t5) < ::std::abs(t))
				{
					t = t5;
					ta = ta5;
					tc = tc5;
					td = td5;
					ah = -am;
					vh =  vm;
					dh =  dm;
				}
				// -am,  vm,  dm
				if (ta6 >= 0 && tc6 >= 0 && td6 >= 0 && ::std::abs(t6) < ::std::abs(t))
				{
					t = t6;
					ta = ta6;
					tc = tc6;
					td = td6;
					ah = -am;
					vh =  vm;
					dh = -dm;
				}
				// -am, -vm, -dm
				if (ta7 >= 0 && tc7 >= 0 && td7 >= 0 && ::std::abs(t7) < ::std::abs(t))
				{
					t = t7;
					ta = ta7;
					tc = tc7;
					td = td7;
					ah = -am;
					vh = -vm;
					dh =  dm;
				}
				// -am, -vm,  dm
				if (ta8 >= 0 && tc8 >= 0 && td8 >= 0 && ::std::abs(t8) < ::std::abs(t))
				{
					t = t8;
					ta = ta8;
					tc = tc8;
					td = td8;
					ah = -am;
					vh = -vm;
					dh = -dm;
				}
				
				T v1 = 1 / ( dm + am) * ::std::sqrt( ( dm + am) * ( dm * v0 * v0 + am * ve * ve + 2 * x * am * dm));
				T v2 = 1 / (-dm + am) * ::std::sqrt(-(-dm + am) * ( dm * v0 * v0 - am * ve * ve + 2 * x * am * dm));
				T v3 = 1 / ( dm + am) * ::std::sqrt( ( dm + am) * ( dm * v0 * v0 + am * ve * ve + 2 * x * am * dm));
				T v4 = 1 / (-dm + am) * ::std::sqrt(-(-dm + am) * ( dm * v0 * v0 - am * ve * ve + 2 * x * am * dm));
				T v5 = 1 / (-dm + am) * ::std::sqrt( (-dm + am) * (-dm * v0 * v0 + am * ve * ve + 2 * x * am * dm));
				T v6 = 1 / ( dm + am) * ::std::sqrt(-( dm + am) * (-dm * v0 * v0 - am * ve * ve + 2 * x * am * dm));
				T v7 = 1 / (-dm + am) * ::std::sqrt( (-dm + am) * (-dm * v0 * v0 + am * ve * ve + 2 * x * am * dm));
				T v8 = 1 / ( dm + am) * ::std::sqrt(-( dm + am) * (-dm * v0 * v0 - am * ve * ve + 2 * x * am * dm));
				
				T ta1a = ( v1 - v0) /  am;
				T ta1b = (-v1 - v0) /  am;
				T ta2a = ( v2 - v0) /  am;
				T ta2b = (-v2 - v0) /  am;
				T ta3a = (-v3 - v0) /  am;
				T ta3b = ( v3 - v0) /  am;
				T ta4a = (-v4 - v0) /  am;
				T ta4b = ( v4 - v0) /  am;
				T ta5a = ( v5 - v0) / -am;
				T ta5b = (-v5 - v0) / -am;
				T ta6a = ( v6 - v0) / -am;
				T ta6b = (-v6 - v0) / -am;
				T ta7a = (-v7 - v0) / -am;
				T ta7b = ( v7 - v0) / -am;
				T ta8a = (-v8 - v0) / -am;
				T ta8b = ( v8 - v0) / -am;
				
				T td1a = (ve - v1) / -dm;
				T td1b = (ve + v1) / -dm;
				T td2a = (ve - v2) /  dm;
				T td2b = (ve + v2) /  dm;
				T td3a = (ve + v3) / -dm;
				T td3b = (ve - v3) / -dm;
				T td4a = (ve + v4) /  dm;
				T td4b = (ve - v4) /  dm;
				T td5a = (ve - v5) / -dm;
				T td5b = (ve + v5) / -dm;
				T td6a = (ve - v6) /  dm;
				T td6b = (ve + v6) /  dm;
				T td7a = (ve + v7) / -dm;
				T td7b = (ve - v7) / -dm;
				T td8a = (ve + v8) /  dm;
				T td8b = (ve - v8) /  dm;
				
				T t1a = ta1a + td1a;
				T t1b = ta1b + td1b;
				T t2a = ta2a + td2a;
				T t2b = ta2b + td2b;
				T t3a = ta3a + td3a;
				T t3b = ta3b + td3b;
				T t4a = ta4a + td4a;
				T t4b = ta4b + td4b;
				T t5a = ta5a + td5a;
				T t5b = ta5b + td5b;
				T t6a = ta6a + td6a;
				T t6b = ta6b + td6b;
				T t7a = ta7a + td7a;
				T t7b = ta7b + td7b;
				T t8a = ta8a + td8a;
				T t8b = ta8b + td8b;
				
				//  am,  vm, -dm
				if (ta1a >= 0 && td1a >= 0 && ::std::abs(t1a) < ::std::abs(t) && ::std::abs(v1) < ::std::abs(vh))
				{
					t =  t1a;
					ta = ta1a;
					tc = 0;
					td = td1a;
					ah =  am;
					vh =  v1;
					dh =  dm;
				}
				if (ta1b >= 0 && td1b >= 0 && ::std::abs(t1b) < ::std::abs(t) && ::std::abs(v1) < ::std::abs(vh))
				{
					t =  t1b;
					ta = ta1b;
					tc = 0;
					td = td1b;
					ah =  am;
					vh = -v1;
					dh =  dm;
				}
				//  am,  vm,  dm
				if (ta2a >= 0 && td2a >= 0 && ::std::abs(t2a) < ::std::abs(t) && ::std::abs(v2) < ::std::abs(vh))
				{
					t =  t2a;
					ta = ta2a;
					tc = 0;
					td = td2a;
					ah =  am;
					vh =  v2;
					dh = -dm;
				}
				if (ta2b >= 0 && td2 >= 0 && ::std::abs(t2b) < ::std::abs(t) && ::std::abs(v2) < ::std::abs(vh))
				{
					t =  t2b;
					ta = ta2b;
					tc = 0;
					td = td2b;
					ah =  am;
					vh = -v2;
					dh = -dm;
				}
				//  am, -vm, -dm
				if (ta3a >= 0 && td3a >= 0 && ::std::abs(t3a) < ::std::abs(t) && ::std::abs(v3) < ::std::abs(vh))
				{
					t =  t3a;
					ta = ta3a;
					tc = 0;
					td = td3a;
					ah =  am;
					vh = -v3;
					dh =  dm;
				}
				if (ta3b >= 0 && td3b >= 0 && ::std::abs(t3b) < ::std::abs(t) && ::std::abs(v3) < ::std::abs(vh))
				{
					t =  t3b;
					ta = ta3b;
					tc = 0;
					td = td3b;
					ah =  am;
					vh =  v3;
					dh =  dm;
				}
				//  am, -vm,  dm
				if (ta4a >= 0 && td4a >= 0 && ::std::abs(t4a) < ::std::abs(t) && ::std::abs(v4) < ::std::abs(vh))
				{
					t =  t4a;
					ta = ta4a;
					tc = 0;
					td = td4a;
					ah =  am;
					vh = -v4;
					dh = -dm;
				}
				if (ta4b >= 0 && td4b >= 0 && ::std::abs(t4b) < ::std::abs(t) && ::std::abs(v4) < ::std::abs(vh))
				{
					t =  t4b;
					ta = ta4b;
					tc = 0;
					td = td4b;
					ah =  am;
					vh =  v4;
					dh = -dm;
				}
				// -am,  vm, -dm
				if (ta5a >= 0 && td5a >= 0 && ::std::abs(t5a) < ::std::abs(t) && ::std::abs(v5) < ::std::abs(vh))
				{
					t =  t5a;
					ta = ta5a;
					tc = 0;
					td = td5a;
					ah = -am;
					vh =  v5;
					dh =  dm;
				}
				if (ta5b >= 0 && td5b >= 0 && ::std::abs(t5b) < ::std::abs(t) && ::std::abs(v5) < ::std::abs(vh))
				{
					t =  t5b;
					ta = ta5b;
					tc = 0;
					td = td5b;
					ah = -am;
					vh = -v5;
					dh =  dm;
				}
				// -am,  vm,  dm
				if (ta6a >= 0 && td6a >= 0 && ::std::abs(t6a) < ::std::abs(t) && ::std::abs(v6) < ::std::abs(vh))
				{
					t =  t6a;
					ta = ta6a;
					tc = 0;
					td = td6a;
					ah = -am;
					vh =  v6;
					dh = -dm;
				}
				if (ta6b >= 0 && td6b >= 0 && ::std::abs(t6b) < ::std::abs(t) && ::std::abs(v6) < ::std::abs(vh))
				{
					t =  t6b;
					ta = ta6b;
					tc = 0;
					td = td6b;
					ah = -am;
					vh = -v6;
					dh = -dm;
				}
				// -am, -vm, -dm
				if (ta7a >= 0 && td7a >= 0 && ::std::abs(t7a) < ::std::abs(t) && ::std::abs(v7) < ::std::abs(vh))
				{
					t =  t7a;
					ta = ta7a;
					tc = 0;
					td = td7a;
					ah = -am;
					vh = -v7;
					dh =  dm;
				}
				if (ta7b >= 0 && td7b >= 0 && ::std::abs(t7b) < ::std::abs(t) && ::std::abs(v7) < ::std::abs(vh))
				{
					t =  t7b;
					ta = ta7b;
					tc = 0;
					td = td7b;
					ah = -am;
					vh =  v7;
					dh =  dm;
				}
				// -am, -vm,  dm
				if (ta8a >= 0 && td8a >= 0 && ::std::abs(t8a) < ::std::abs(t) && ::std::abs(v8) < ::std::abs(vh))
				{
					t =  t8a;
					ta = ta8a;
					tc = 0;
					td = td8a;
					ah = -am;
					vh = -v8;
					dh = -dm;
				}
				if (ta8b >= 0 && td8b >= 0 && ::std::abs(t8b) < ::std::abs(t) && ::std::abs(v8) < ::std::abs(vh))
				{
					t =  t8b;
					ta = ta8b;
					tc = 0;
					td = td8b;
					ah = -am;
					vh =  v8;
					dh = -dm;
				}
			}
			
			void interpolate(const Real& t)
			{
				T x = xe - x0;
				
				T v1a =  (dm * v0 + am * ve + t * am * dm + ::std::sqrt( 2 * dm * v0 * am * ve + 2 * dm * dm * v0 * t * am + 2 * am * am * ve * t * dm + t * t * am * am * dm * dm - dm * am * ve * ve - 2 * x * am * dm * dm - am * dm * v0 * v0 - 2 * x * am * am * dm)) / ( dm + am);
				T v1b =  (dm * v0 + am * ve + t * am * dm - ::std::sqrt( 2 * dm * v0 * am * ve + 2 * dm * dm * v0 * t * am + 2 * am * am * ve * t * dm + t * t * am * am * dm * dm - dm * am * ve * ve - 2 * x * am * dm * dm - am * dm * v0 * v0 - 2 * x * am * am * dm)) / ( dm + am);
				T v2a;
				T v2b;
				T v3a = -(dm * v0 + am * ve + t * am * dm - ::std::sqrt( 2 * dm * v0 * am * ve + 2 * dm * dm * v0 * t * am + 2 * am * am * ve * t * dm + t * t * am * am * dm * dm - am * dm * v0 * v0 - 2 * x * am * am * dm - dm * am * ve * ve - 2 * x * am * dm * dm)) / ( am + dm);
				T v3b = -(dm * v0 + am * ve + t * am * dm + ::std::sqrt( 2 * dm * v0 * am * ve + 2 * dm * dm * v0 * t * am + 2 * am * am * ve * t * dm + t * t * am * am * dm * dm - am * dm * v0 * v0 - 2 * x * am * am * dm - dm * am * ve * ve - 2 * x * am * dm * dm)) / ( am + dm);
				T v4a;
				T v4b;
				T v5a;
				T v5b;
				T v6a =  (dm * v0 + am * ve - t * am * dm + ::std::sqrt( 2 * dm * v0 * am * ve - 2 * dm * dm * v0 * t * am - 2 * am * am * ve * t * dm + t * t * am * am * dm * dm - dm * am * ve * ve + 2 * x * am * dm * dm - am * dm * v0 * v0 + 2 * x * am * am * dm)) / ( dm + am);
				T v6b =  (dm * v0 + am * ve - t * am * dm - ::std::sqrt( 2 * dm * v0 * am * ve - 2 * dm * dm * v0 * t * am - 2 * am * am * ve * t * dm + t * t * am * am * dm * dm - dm * am * ve * ve + 2 * x * am * dm * dm - am * dm * v0 * v0 + 2 * x * am * am * dm)) / ( dm + am);
				T v7a;
				T v7b;
				T v8a = -(dm * v0 + am * ve - t * am * dm - ::std::sqrt( 2 * dm * v0 * am * ve - 2 * dm * dm * v0 * t * am - 2 * am * am * ve * t * dm + t * t * am * am * dm * dm - am * dm * v0 * v0 + 2 * x * am * am * dm - dm * am * ve * ve + 2 * x * am * dm * dm)) / ( am + dm);
				T v8b = -(dm * v0 + am * ve - t * am * dm + ::std::sqrt( 2 * dm * v0 * am * ve - 2 * dm * dm * v0 * t * am - 2 * am * am * ve * t * dm + t * t * am * am * dm * dm - am * dm * v0 * v0 + 2 * x * am * am * dm - dm * am * ve * ve + 2 * x * am * dm * dm)) / ( am + dm);
				
				if (::std::abs(am - dm) <= ::std::numeric_limits<T>::epsilon())
				{
					v2a =  ( v0 * v0 - ve * ve + 2 * x * am) / ( t * am + v0 - ve) / 2;
					v2b =  ( v0 * v0 - ve * ve + 2 * x * am) / ( t * am + v0 - ve) / 2;
					v4a = -( v0 * v0 - ve * ve + 2 * x * am) / ( t * am + v0 - ve) / 2;
					v4b = -( v0 * v0 - ve * ve + 2 * x * am) / ( t * am + v0 - ve) / 2;
					v5a = -(-v0 * v0 + ve * ve + 2 * x * am) / (-t * am + v0 - ve) / 2;
					v5b = -(-v0 * v0 + ve * ve + 2 * x * am) / (-t * am + v0 - ve) / 2;
					v7a =  (-v0 * v0 + ve * ve + 2 * x * am) / (-t * am + v0 - ve) / 2;
					v7b =  (-v0 * v0 + ve * ve + 2 * x * am) / (-t * am + v0 - ve) / 2;
				}
				else
				{
					v2a = -(dm * v0 - am * ve + t * am * dm - ::std::sqrt(-2 * dm * v0 * am * ve + 2 * dm * dm * v0 * t * am - 2 * am * am * ve * t * dm + t * t * am * am * dm * dm + dm * am * ve * ve - 2 * x * am * dm * dm + am * dm * v0 * v0 + 2 * x * am * am * dm)) / (-dm + am);
					v2b = -(dm * v0 - am * ve + t * am * dm + ::std::sqrt(-2 * dm * v0 * am * ve + 2 * dm * dm * v0 * t * am - 2 * am * am * ve * t * dm + t * t * am * am * dm * dm + dm * am * ve * ve - 2 * x * am * dm * dm + am * dm * v0 * v0 + 2 * x * am * am * dm)) / (-dm + am);
					v4a =  (dm * v0 - am * ve + t * am * dm + ::std::sqrt(-2 * dm * v0 * am * ve + 2 * dm * dm * v0 * t * am - 2 * am * am * ve * t * dm + t * t * am * am * dm * dm + am * dm * v0 * v0 + 2 * x * am * am * dm + dm * am * ve * ve - 2 * x * am * dm * dm)) / ( am - dm);
					v4b =  (dm * v0 - am * ve + t * am * dm - ::std::sqrt(-2 * dm * v0 * am * ve + 2 * dm * dm * v0 * t * am - 2 * am * am * ve * t * dm + t * t * am * am * dm * dm + am * dm * v0 * v0 + 2 * x * am * am * dm + dm * am * ve * ve - 2 * x * am * dm * dm)) / ( am - dm);
					v5a = -(dm * v0 - am * ve - t * am * dm - ::std::sqrt(-2 * dm * v0 * am * ve - 2 * dm * dm * v0 * t * am + 2 * am * am * ve * t * dm + t * t * am * am * dm * dm + dm * am * ve * ve + 2 * x * am * dm * dm + am * dm * v0 * v0 - 2 * x * am * am * dm)) / (-dm + am);
					v5b = -(dm * v0 - am * ve - t * am * dm + ::std::sqrt(-2 * dm * v0 * am * ve - 2 * dm * dm * v0 * t * am + 2 * am * am * ve * t * dm + t * t * am * am * dm * dm + dm * am * ve * ve + 2 * x * am * dm * dm + am * dm * v0 * v0 - 2 * x * am * am * dm)) / (-dm + am);
					v7a =  (dm * v0 - am * ve - t * am * dm + ::std::sqrt(-2 * dm * v0 * am * ve - 2 * dm * dm * v0 * t * am + 2 * am * am * ve * t * dm + t * t * am * am * dm * dm + am * dm * v0 * v0 - 2 * x * am * am * dm + dm * am * ve * ve + 2 * x * am * dm * dm)) / ( am - dm);
					v7b =  (dm * v0 - am * ve - t * am * dm - ::std::sqrt(-2 * dm * v0 * am * ve - 2 * dm * dm * v0 * t * am + 2 * am * am * ve * t * dm + t * t * am * am * dm * dm + am * dm * v0 * v0 - 2 * x * am * am * dm + dm * am * ve * ve + 2 * x * am * dm * dm)) / ( am - dm);
				}
				
				T ta1a = ( v1a - v0) /  am;
				T ta1b = ( v1b - v0) /  am;
				T ta2a = ( v2a - v0) /  am;
				T ta2b = ( v2b - v0) /  am;
				T ta3a = (-v3a - v0) /  am;
				T ta3b = (-v3b - v0) /  am;
				T ta4a = (-v4a - v0) /  am;
				T ta4b = (-v4b - v0) /  am;
				T ta5a = ( v5a - v0) / -am;
				T ta5b = ( v5b - v0) / -am;
				T ta6a = ( v6a - v0) / -am;
				T ta6b = ( v6b - v0) / -am;
				T ta7a = (-v7a - v0) / -am;
				T ta7b = (-v7b - v0) / -am;
				T ta8a = (-v8a - v0) / -am;
				T ta8b = (-v8b - v0) / -am;
				
				T td1a = (ve - v1a) / -dm;
				T td1b = (ve - v1b) / -dm;
				T td2a = (ve - v2a) /  dm;
				T td2b = (ve - v2b) /  dm;
				T td3a = (ve + v3a) / -dm;
				T td3b = (ve + v3b) / -dm;
				T td4a = (ve + v4a) /  dm;
				T td4b = (ve + v4b) /  dm;
				T td5a = (ve - v5a) / -dm;
				T td5b = (ve - v5b) / -dm;
				T td6a = (ve - v6a) /  dm;
				T td6b = (ve - v6b) /  dm;
				T td7a = (ve + v7a) / -dm;
				T td7b = (ve + v7b) / -dm;
				T td8a = (ve + v8a) /  dm;
				T td8b = (ve + v8b) /  dm;
				
				T tc1a = t - ta1a - td1a;
				T tc1b = t - ta1b - td1b;
				T tc2a = t - ta2a - td2a;
				T tc2b = t - ta2b - td2b;
				T tc3a = t - ta3a - td3a;
				T tc3b = t - ta3b - td3b;
				T tc4a = t - ta4a - td4a;
				T tc4b = t - ta4b - td4b;
				T tc5a = t - ta5a - td5a;
				T tc5b = t - ta5b - td5b;
				T tc6a = t - ta6a - td6a;
				T tc6b = t - ta6b - td6b;
				T tc7a = t - ta7a - td7a;
				T tc7b = t - ta7b - td7b;
				T tc8a = t - ta8a - td8a;
				T tc8b = t - ta8b - td8b;
				
				T vh = vm;
				
				//  am,  vm, -dm
				if (ta1a >= 0 && tc1a >= 0 && td1a >= 0 && ::std::abs(v1a) < ::std::abs(vh))
				{
					ta = ta1a;
					tc = tc1a;
					td = td1a;
					ah =  am;
					vh =  v1a;
					dh =  dm;
				}
				if (ta1b >= 0 && tc1b >= 0 && td1b >= 0 && ::std::abs(v1b) < ::std::abs(vh))
				{
					ta = ta1b;
					tc = tc1b;
					td = td1b;
					ah =  am;
					vh =  v1b;
					dh =  dm;
				}
				//  am,  vm,  dm
				if (ta2a >= 0 && tc2a >= 0 && td2a >= 0 && ::std::abs(v2a) < ::std::abs(vh))
				{
					ta = ta2a;
					tc = tc2a;
					td = td2a;
					ah =  am;
					vh =  v2a;
					dh = -dm;
				}
				if (ta2b >= 0 && tc2b >= 0 && td2b >= 0 && ::std::abs(v2b) < ::std::abs(vh))
				{
					ta = ta2b;
					tc = tc2b;
					td = td2b;
					ah =  am;
					vh =  v2b;
					dh = -dm;
				}
				//  am, -vm, -dm
				if (ta3a >= 0 && tc3a >= 0 && td3a >= 0 && ::std::abs(v3a) < ::std::abs(vh))
				{
					ta = ta3a;
					tc = tc3a;
					td = td3a;
					ah =  am;
					vh = -v3a;
					dh =  dm;
				}
				if (ta3b >= 0 && tc3b >= 0 && td3b >= 0 && ::std::abs(v3b) < ::std::abs(vh))
				{
					ta = ta3b;
					tc = tc3b;
					td = td3b;
					ah =  am;
					vh = -v3b;
					dh =  dm;
				}
				//  am, -vm,  dm
				if (ta4a >= 0 && tc4a >= 0 && td4a >= 0 && ::std::abs(v4a) < ::std::abs(vh))
				{
					ta = ta4a;
					tc = tc4a;
					td = td4a;
					ah =  am;
					vh = -v4a;
					dh = -dm;
				}
				if (ta4b >= 0 && tc4b >= 0 && td4b >= 0 && ::std::abs(v4b) < ::std::abs(vh))
				{
					ta = ta4b;
					tc = tc4b;
					td = td4b;
					ah =  am;
					vh = -v4b;
					dh = -dm;
				}
				// -am,  vm, -dm
				if (ta5a >= 0 && tc5a >= 0 && td5a >= 0 && ::std::abs(v5a) < ::std::abs(vh))
				{
					ta = ta5a;
					tc = tc5a;
					td = td5a;
					ah = -am;
					vh =  v5a;
					dh =  dm;
				}
				if (ta5b >= 0 && tc5b >= 0 && td5b >= 0 && ::std::abs(v5b) < ::std::abs(vh))
				{
					ta = ta5b;
					tc = tc5b;
					td = td5b;
					ah = -am;
					vh =  v5b;
					dh =  dm;
				}
				// -am,  vm,  dm
				if (ta6a >= 0 && tc6a >= 0 && td6a >= 0 && ::std::abs(v6a) < ::std::abs(vh))
				{
					ta = ta6a;
					tc = tc6a;
					td = td6a;
					ah = -am;
					vh =  v6a;
					dh = -dm;
				}
				if (ta6b >= 0 && tc6b >= 0 && td6b >= 0 && ::std::abs(v6b) < ::std::abs(vh))
				{
					ta = ta6b;
					tc = tc6b;
					td = td6b;
					ah = -am;
					vh =  v6b;
					dh = -dm;
				}
				// -am, -vm, -dm
				if (ta7a >= 0 && tc7a >= 0 && td7a >= 0 && ::std::abs(v7a) < ::std::abs(vh))
				{
					ta = ta7a;
					tc = tc7a;
					td = td7a;
					ah = -am;
					vh = -v7a;
					dh =  dm;
				}
				if (ta7b >= 0 && tc7b >= 0 && td7b >= 0 && ::std::abs(v7b) < ::std::abs(vh))
				{
					ta = ta7b;
					tc = tc7b;
					td = td7b;
					ah = -am;
					vh = -v7b;
					dh =  dm;
				}
				// -am, -vm,  dm
				if (ta8a >= 0 && tc8a >= 0 && td8a >= 0 && ::std::abs(v8a) < ::std::abs(vh))
				{
					ta = ta8a;
					tc = tc8a;
					td = td8a;
					ah = -am;
					vh = -v8a;
					dh = -dm;
				}
				if (ta8b >= 0 && tc8b >= 0 && td8b >= 0 && ::std::abs(v8b) < ::std::abs(vh))
				{
					ta = ta8b;
					tc = tc8b;
					td = td8b;
					ah = -am;
					vh = -v8b;
					dh = -dm;
				}
			}
			
			T t() const
			{
				return ta + tc + td;
			}
			
			T v(const Real& t) const
			{
				if (t < ta)
				{
					return v0 + ah * t;
				}
				else if (t < ta + tc)
				{
					return v0 + ah * ta;
				}
				else if (t < ta + tc + td)
				{
					return v0 + ah * ta - dh * t + dh * (ta + tc);
				}
				else
				{
					return ve;
				}
			}
			
			T x(const Real& t) const
			{
				if (t < ta)
				{
					return x0 + v0 * t + 0.5 * ah * ::std::pow(t, 2);
				}
				else if (t < ta + tc)
				{
					return x0 + v0 * t - 0.5 * ah * ::std::pow(ta, 2) + ah * ta * t;
				}
				else if (t < ta + tc + td)
				{
					return x0 + v0 * t - 0.5 * ah * ::std::pow(ta, 2) + ah * ta * t - 0.5 * dh * ::std::pow(ta + tc, 2) - 0.5 * dh * ::std::pow(t, 2) + dh * (ta + tc) * t;
				}
				else
				{
					return xe + ve * (t - ta - tc - td);
				}
			}
			
			T am;
			
			T dm;
			
			T x0;
			
			T xe;
			
			T v0;
			
			T ve;
			
			T vm;
			
		protected:
			
		private:
			T ah;
			
			T dh;
			
			Real ta;
			
			Real tc;
			
			Real td;
		};
	}
}

#endif // RL_MATH_TRAPEZOIDALVELOCITY_H
