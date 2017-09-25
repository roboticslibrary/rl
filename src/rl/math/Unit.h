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

#ifndef RL_MATH_UNIT_H
#define RL_MATH_UNIT_H

#include <cmath>

#include "Real.h"

namespace rl
{
	namespace math
	{
		/**
		 * <h3><a href="http://en.wikipedia.org/wiki/SI_base_unit">SI base units</a></h3>
		 * 
		 * <table>
		 *   <tr>
		 *     <td><strong>Name</strong></td>
		 *     <td><strong>Symbol</strong></td>
		 *     <td><strong>Measure</strong></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Meter">meter</a></td>
		 *     <td>m</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Length">length</a></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Kilogram">kilogram</a></td>
		 *     <td>kg</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Mass">mass</a></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Second">second</a></td>
		 *     <td>s</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Time">time</a></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Ampere">ampere</a></td>
		 *     <td>A</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Electric_current">electric current</a></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Kelvin">kelvin</a></td>
		 *     <td>K</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Temperature">thermodynamic temperature</a></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Mole_%28unit%29">mole</a></td>
		 *     <td>mol</td>
		 *     <td>quantity of <a href="http://en.wikipedia.org/wiki/Matter">matter</a> (mass/mass)</td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Candela">candela</a></td>
		 *     <td>cd</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Luminous_intensity">luminous intensity</a></td>
		 *   </tr>
		 * </table>
		 * 
		 * <h3><a href="http://en.wikipedia.org/wiki/SI_derived_unit">SI derived units</a></h3>
		 * 
		 * <table>
		 *   <tr>
		 *     <td><strong>Name</strong></td>
		 *     <td><strong>Symbol</strong></td>
		 *     <td><strong>Quantity</strong></td>
		 *     <td><strong>Expression in terms of <a href="http://en.wikipedia.org/wiki/SI_base_unit">SI base units</a></strong></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Radian">radian</a></td>
		 *     <td>rad</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Angle">Angle</a></td>
		 *     <td></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Hertz">hertz</a></td>
		 *     <td>Hz</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Frequency">frequency</a></td>
		 *     <td>s<sup>-1</sup></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Newton">newton</a></td>
		 *     <td>N</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Force">Force</a>, <a href="http://en.wikipedia.org/wiki/Weight">Weight</a></td>
		 *     <td>m &middot; kg &middot; s<sup>-2</sup></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Volt">volt</a></td>
		 *     <td>V</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Potential_difference">Electrical potential difference</a>, <a href="http://en.wikipedia.org/wiki/Electromotive_force">Electromotive force</a></td>
		 *     <td>m<sup>2</sup> &middot; kg &middot; s<sup>-3</sup> &middot; A<sup>-1</sup></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Celsius">degree Celsius</a></td>
		 *     <td>@htmlonly &#176C @endhtmlonly</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Thermodynamic_temperature">Thermodynamic temperature</a></td>
		 *     <td>T<sub>&ordm;C</sub> = T<sub>K</sub> - 273.16</td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Meter_per_second">meter per second</a></td>
		 *     <td>m &middot; s<sup>-1</sup></td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Speed">speed</a>, <a href="http://en.wikipedia.org/wiki/Velocity">velocity</a></td>
		 *     <td>m &middot; s<sup>-1</sup></td>
		 *   </tr>
		 *   <tr>
		 *     <td><a href="http://en.wikipedia.org/wiki/Meter_per_second_squared">meter per second squared</a></td>
		 *     <td>m &middot; s<sup>-2</sup></td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Acceleration">acceleration</a></td>
		 *     <td>m &middot; s<sup>-2</sup></td>
		 *   </tr>
		 *   <tr>
		 *     <td>radian per second</td>
		 *     <td>rad &middot; s<sup>-1</sup></td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Angular_velocity">angular velocity</a></td>
		 *     <td>s<sup>-1</sup></td>
		 *   </tr>
		 *   <tr>
		 *     <td>newton second</td>
		 *     <td>N &middot; s</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Momentum">momentum</a>, <a href="http://en.wikipedia.org/wiki/Impulse">impulse</a></td>
		 *     <td>kg &middot; m &middot; s<sup>-1</sup></td>
		 *   </tr>
		 *   <tr>
		 *     <td>newton meter second</td>
		 *     <td>N &middot; m &middot; s</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Angular_momentum">angular momentum</a></td>
		 *     <td>kg &middot; m<sup>2</sup> &middot; s<sup>-1</sup></td>
		 *   </tr>
		 *   <tr>
		 *     <td>newton meter</td>
		 *     <td>N &middot; m</td>
		 *     <td><a href="http://en.wikipedia.org/wiki/Torque">Torque, moment of force</a></td>
		 *     <td>kg &middot; m<sup>2</sup> &middot; s<sup>-2</sup></td>
		 *   </tr>
		 * </table>
		 */
		enum Unit
		{
			UNIT_NONE,
			UNIT_METER,
			UNIT_KILOGRAM,
			UNIT_SECOND,
			UNIT_AMPERE,
			UNIT_KELVIN,
			UNIT_MOLE,
			UNIT_CANDELA,
			UNIT_RADIAN,
			UNIT_HERTZ,
			UNIT_NEWTON,
			UNIT_VOLT,
			UNIT_CELSIUS,
			UNIT_METER_PER_SECOND,
			UNIT_METER_PER_SECOND_SQUARED,
			UNIT_RADIAN_PER_SECOND,
			UNIT_RADIAN_PER_SECOND_SQUARED,
			UNIT_NEWTON_SECOND,
			UNIT_NEWTON_METER_SECOND,
			UNIT_NEWTON_METER
		};
		
		static const Real DEG2RAD = static_cast<Real>(M_PI) / 180.0f;
		
		static const Real GIGA2UNIT = 1.0e+9f;
		
		/** [m &middot; s<sup>-2</sup>] */
		static const Real GRAVITY = 9.80665f;
		
		static const Real KILO2UNIT = 1.0e+3f;
		
		static const Real MEGA2UNIT = 1.0e+6f;
		
		static const Real MICRO2UNIT = 1.0e-6f;
		
		static const Real MILLI2UNIT = 1.0e-3f;
		
		static const Real NANO2UNIT = 1.0e-9f;
		
		static const Real RAD2DEG = 180.0f / static_cast<Real>(M_PI);
		
		static const Real UNIT2GIGA = 1.0e-9f;
		
		static const Real UNIT2KILO = 1.0e-3f;
		
		static const Real UNIT2MEGA = 1.0e-6f;
		
		static const Real UNIT2MICRO = 1.0e+6f;
		
		static const Real UNIT2MILLI = 1.0e+3f;
		
		static const Real UNIT2NANO = 1.0e+9f;
		
		inline void cartesianToPolar(const Real& x, const Real& y, Real& r, Real& theta)
		{
			r = ::std::sqrt(::std::pow(x, 2) + ::std::pow(y, 2));
			theta = ::std::atan2(y, x);
		}
		
		inline void cartesianToSpherical(const Real& x, const Real& y, const Real& z, Real& rho, Real& psi, Real& theta)
		{
			rho = ::std::sqrt(::std::pow(x, 2) + ::std::pow(y, 2) + ::std::pow(z, 2));
			psi = ::std::atan2(::std::sqrt(::std::pow(x, 2) + ::std::pow(y, 2)), z);
			theta = ::std::atan2(y, x);
		}
		
		inline void polarToCartesian(const Real& r, const Real& theta, Real& x, Real& y)
		{
			x = r * ::std::cos(theta);
			y = r * ::std::sin(theta);
		}
		
		inline void sphericalToCartesian(const Real& rho, const Real& psi, const Real& theta, Real& x, Real& y, Real& z)
		{
			x = rho * ::std::sin(psi) * ::std::cos(theta);
			y = rho * ::std::sin(psi) * ::std::sin(theta);
			z = rho * ::std::cos(psi);
		}
	}
}

#endif // RL_MATH_UNIT_H
