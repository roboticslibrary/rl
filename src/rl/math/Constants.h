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

#ifndef RL_MATH_CONSTANTS_H
#define RL_MATH_CONSTANTS_H

#include <type_traits>

#include "Real.h"

namespace rl
{
	namespace math
	{
		template<typename T, typename Enable = typename ::std::enable_if<::std::is_floating_point<T>::value>::type>
		class Constants
		{
		public:
			static constexpr T e = static_cast<T>(2.718281828459045235360287471352662498L);
			
			static constexpr T log2e = static_cast<T>(1.442695040888963407359924681001892137L);
			
			static constexpr T log10e = static_cast<T>(0.434294481903251827651128918916605082L);
			
			static constexpr T pi = static_cast<T>(3.141592653589793238462643383279502884L);
			
			static constexpr T inv_pi = static_cast<T>(0.318309886183790671537767526745028724L);
			
			static constexpr T inv_sqrtpi = static_cast<T>(0.564189583547756286948079451560772586L);
			
			static constexpr T ln2 = static_cast<T>(0.693147180559945309417232121458176568L);
			
			static constexpr T ln10 = static_cast<T>(2.302585092994045684017991454684364208L);
			
			static constexpr T sqrt2 = static_cast<T>(1.414213562373095048801688724209698079L);
			
			static constexpr T sqrt3 = static_cast<T>(1.732050807568877293527446341505872367L);
			
			static constexpr T inv_sqrt3 = static_cast<T>(0.577350269189625764509148780501957456L);
			
			static constexpr T egamma = static_cast<T>(0.577215664901532860606512090082402431L);
			
			static constexpr T phi = static_cast<T>(1.618033988749894848204586834365638118L);
			
			/**
			 * Standard acceleration due to gravity.
			 *
			 * [Standard gravity](https://en.wikipedia.org/wiki/Standard_gravity) is the nominal
			 * [gravitational acceleration](https://en.wikipedia.org/wiki/Gravitational_acceleration)
			 * of an object in a vacuum near the surface of the earth. It is defined as 9.80665 m &middot; s<sup>-2</sup>
			 * by [ISO 80000-3](https://en.wikipedia.org/wiki/ISO_80000-3).
			 */
			static constexpr T gravity = static_cast<T>(9.80665);
			
			/**
			 * Constant for converting an angular value in
			 * [degree](https://en.wikipedia.org/wiki/Degree_(angle)) to
			 * [radian](https://en.wikipedia.org/wiki/Radian).
			 *
			 * This is equal to a multiplication by &pi; and a division by 180.
			 */
			static constexpr T deg2rad = pi / static_cast<T>(180);
			
			/**
			 * Constant for converting an angular value in
			 * [radian](https://en.wikipedia.org/wiki/Radian) to
			 * [degree](https://en.wikipedia.org/wiki/Degree_(angle)).
			 *
			 * This is equal to a multiplication by 180 and a division by &pi;.
			 */
			static constexpr T rad2deg = static_cast<T>(180) / pi;
			
			static constexpr T giga2unit = static_cast<T>(1.0e+9);
			
			static constexpr T kilo2unit = static_cast<T>(1.0e+3);
			
			static constexpr T mega2unit = static_cast<T>(1.0e+6);
			
			static constexpr T micro2unit = static_cast<T>(1.0e-6);
			
			static constexpr T milli2unit = static_cast<T>(1.0e-3);
			
			static constexpr T nano2unit = static_cast<T>(1.0e-9);
			
			static constexpr T unit2giga = static_cast<T>(1.0e-9);
			
			static constexpr T unit2kilo = static_cast<T>(1.0e-3);
			
			static constexpr T unit2mega = static_cast<T>(1.0e-6);
			
			static constexpr T unit2micro = static_cast<T>(1.0e+6);
			
			static constexpr T unit2milli = static_cast<T>(1.0e+3);
			
			static constexpr T unit2nano = static_cast<T>(1.0e+9);
			
		protected:
			
		private:
			
		};
		
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::e;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::log2e;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::log10e;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::pi;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::inv_pi;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::inv_sqrtpi;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::ln2;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::ln10;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::sqrt2;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::sqrt3;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::inv_sqrt3;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::egamma;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::phi;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::gravity;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::deg2rad;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::rad2deg;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::giga2unit;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::kilo2unit;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::mega2unit;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::micro2unit;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::milli2unit;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::nano2unit;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::unit2giga;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::unit2kilo;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::unit2mega;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::unit2micro;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::unit2milli;
		template<typename T, typename Enable> constexpr T Constants<T, Enable>::unit2nano;
		
		typedef Constants<Real> constants;
	}
}

#endif // RL_MATH_CONSTANTS_H
