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

#include "Real.h"
#include "TypeTraits.h"

namespace rl
{
	namespace math
	{
		/**
		 * Values describing [base units](https://en.wikipedia.org/wiki/SI_base_unit) and
		 * [derived units](https://en.wikipedia.org/wiki/SI_derived_unit) of the
		 * [International System of Units](https://en.wikipedia.org/wiki/International_System_of_Units).
		 */
		enum Unit
		{
			/**
			 * Value used for describing dimensionless quantities.
			 */
			UNIT_NONE,
			/**
			 * The [second](https://en.wikipedia.org/wiki/Second) (symbol **s**) is the
			 * [SI base unit](https://en.wikipedia.org/wiki/SI_base_unit)
			 * of [time](https://en.wikipedia.org/wiki/Time).
			 */
			UNIT_SECOND,
			/**
			 * The [meter](https://en.wikipedia.org/wiki/Meter) (symbol **m**) is the
			 * [SI base unit](https://en.wikipedia.org/wiki/SI_base_unit)
			 * of [length](https://en.wikipedia.org/wiki/Length).
			 */
			UNIT_METER,
			/**
			 * The [kilogram](https://en.wikipedia.org/wiki/Kilogram) (symbol **kg**) is the
			 * [SI base unit](https://en.wikipedia.org/wiki/SI_base_unit)
			 * of [mass](https://en.wikipedia.org/wiki/Mass).
			 */
			UNIT_KILOGRAM,
			/**
			 * The [ampere](https://en.wikipedia.org/wiki/Ampere) (symbol **A**) is the
			 * [SI base unit](https://en.wikipedia.org/wiki/SI_base_unit)
			 * of [electric current](https://en.wikipedia.org/wiki/Electric_current).
			 */
			UNIT_AMPERE,
			/**
			 * The [kelvin](https://en.wikipedia.org/wiki/Kelvin) (symbol **K**) is the
			 * [SI base unit](https://en.wikipedia.org/wiki/SI_base_unit)
			 * of [thermodynamic temperature](https://en.wikipedia.org/wiki/Thermodynamic_temperature).
			 */
			UNIT_KELVIN,
			/**
			 * The [mole](https://en.wikipedia.org/wiki/Mole_%28unit%29) (symbol **mol**) is the
			 * [SI base unit](https://en.wikipedia.org/wiki/SI_base_unit)
			 * of [amount of substance](https://en.wikipedia.org/wiki/Amount_of_substance).
			 */
			UNIT_MOLE,
			/**
			 * The [candela](https://en.wikipedia.org/wiki/Candela) (symbol **cd**) is the
			 * [SI base unit](https://en.wikipedia.org/wiki/SI_base_unit)
			 * of [luminous intensity](https://en.wikipedia.org/wiki/Luminous_intensity).
			 */
			UNIT_CANDELA,
			/**
			 * The [radian](https://en.wikipedia.org/wiki/Radian) (symbol **rad**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [plane angle](https://en.wikipedia.org/wiki/Angle).
			 */
			UNIT_RADIAN,
			/**
			 * The [steradian](https://en.wikipedia.org/wiki/Radian) (symbol **sr**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [solid angle](https://en.wikipedia.org/wiki/Solid_angle).
			 */
			UNIT_STERADIAN,
			/**
			 * The [hertz](https://en.wikipedia.org/wiki/Hertz) (symbol **Hz**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [frequency](https://en.wikipedia.org/wiki/Frequency)
			 * and is defined as s<sup>-1</sup>.
			 */
			UNIT_HERTZ,
			/**
			 * The [newton](https://en.wikipedia.org/wiki/Newton_(unit)) (symbol **N**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [force](https://en.wikipedia.org/wiki/Force)
			 * and is defined as kg &middot; m &middot; s<sup>-2</sup>.
			 */
			UNIT_NEWTON,
			/**
			 * The [pascal](https://en.wikipedia.org/wiki/Pascal_(unit)) (symbol **Pa**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [pressure](https://en.wikipedia.org/wiki/Pressure)
			 * and is defined as kg &middot; m<sup>-1</sup> &middot; s<sup>-2</sup>.
			 */
			UNIT_PASCAL,
			/**
			 * The [joule](https://en.wikipedia.org/wiki/Joule) (symbol **J**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [energy](https://en.wikipedia.org/wiki/Energy)
			 * and is defined as kg &middot; m<sup>-2</sup> &middot; s<sup>-2</sup>.
			 */
			UNIT_JOULE,
			/**
			 * The [watt](https://en.wikipedia.org/wiki/Watt) (symbol **W**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [power](https://en.wikipedia.org/wiki/Power_(physics))
			 * and is defined as kg &middot; m<sup>-2</sup> &middot; s<sup>-3</sup>.
			 */
			UNIT_WATT,
			/**
			 * The [coulomb](https://en.wikipedia.org/wiki/Coulomb) (symbol **C**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [electric charge](https://en.wikipedia.org/wiki/Electric_charge)
			 * and is defined as s &middot; A.
			 */
			UNIT_COULOMB,
			/**
			 * The [volt](https://en.wikipedia.org/wiki/Volt) (symbol **V**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [electric potential](https://en.wikipedia.org/wiki/Electric_potential),
			 * [electric potential difference](https://en.wikipedia.org/wiki/Electric_potential_difference), and
			 * [electromotive force](https://en.wikipedia.org/wiki/Electromotive_force)
			 * and is defined as kg &middot; m<sup>2</sup> &middot; s<sup>-3</sup> &middot; A<sup>-1</sup>.
			 */
			UNIT_VOLT,
			/**
			 * The [farad](https://en.wikipedia.org/wiki/Farad) (symbol **F**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [electrical capacitance](https://en.wikipedia.org/wiki/Capacitance)
			 * and is defined as kg<sup>-1</sup> &middot; m<sup>-2</sup> &middot; s<sup>4</sup> &middot; A<sup>2</sup>.
			 */
			UNIT_FARAD,
			/**
			 * The [ohm](https://en.wikipedia.org/wiki/Ohm) (symbol **&Omega;**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [electrical resistance](https://en.wikipedia.org/wiki/Electrical_resistance)
			 * and is defined as kg &middot; m<sup>2</sup> &middot; s<sup>-3</sup> &middot; A<sup>-2</sup>.
			 */
			UNIT_OHM,
			/**
			 * The [siemens](https://en.wikipedia.org/wiki/Siemens_(unit)) (symbol **S**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [electrical conductance](https://en.wikipedia.org/wiki/Electric_conductance)
			 * and is defined as kg<sup>-1</sup> &middot; m<sup>-2</sup> &middot; s<sup>3</sup> &middot; A<sup>2</sup>.
			 */
			UNIT_SIEMENS,
			/**
			 * The [weber](https://en.wikipedia.org/wiki/Weber_(unit)) (symbol **Wb**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [magnetic flux](https://en.wikipedia.org/wiki/Magnetic_flux)
			 * and is defined as kg &middot; m<sup>2</sup> &middot; s<sup>-2</sup> &middot; A<sup>-1</sup>.
			 */
			UNIT_WEBER,
			/**
			 * The [tesla](https://en.wikipedia.org/wiki/Tesla_(unit)) (symbol **T**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [magnetic flux density](https://en.wikipedia.org/wiki/Magnetic_flux_density)
			 * and is defined as kg &middot; s<sup>-2</sup> &middot; A<sup>-1</sup>.
			 */
			UNIT_TESLA,
			/**
			 * The [henry](https://en.wikipedia.org/wiki/Henry_(unit)) (symbol **H**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [electrical inductance](https://en.wikipedia.org/wiki/Inductance)
			 * and is defined as kg &middot; m<sup>2</sup> &middot; s<sup>-2</sup> &middot; A<sup>-2</sup>.
			 */
			UNIT_HENRY,
			/**
			 * The [degree Celsius](https://en.wikipedia.org/wiki/Degree_Celsius) (symbol **&ordm;C**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * referring to a temperature on the Celsius scale and is defined as temperature
			 * relative to 273.15 K.
			 */
			UNIT_CELSIUS,
			/**
			 * The [lumen](https://en.wikipedia.org/wiki/Lumen_(unit)) (symbol **lm**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [luminous flux](https://en.wikipedia.org/wiki/Luminous_flux)
			 * and is defined as cd &middot; sr.
			 */
			UNIT_LUMEN,
			/**
			 * The [lux](https://en.wikipedia.org/wiki/Lux) (symbol **lx**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [illuminance](https://en.wikipedia.org/wiki/Illuminance)
			 * and is defined as m<sup>-2</sup> &middot; cd.
			 */
			UNIT_LUX,
			/**
			 * The [becquerel](https://en.wikipedia.org/wiki/Becquerel) (symbol **Bq**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [radioactivity](https://en.wikipedia.org/wiki/Radioactivity)
			 * and is defined as s<sup>-1</sup>.
			 */
			UNIT_BECQUEREL,
			/**
			 * The [gray](https://en.wikipedia.org/wiki/Gray_(unit)) (symbol **Gy**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [absorbed dose](https://en.wikipedia.org/wiki/Absorbed_dose)
			 * of [ionising radiation](https://en.wikipedia.org/wiki/Ionizing_radiation)
			 * and is defined as m<sup>2</sup> &middot; s<sup>-2</sup>.
			 */
			UNIT_GRAY,
			/**
			 * The [sievert](https://en.wikipedia.org/wiki/Sievert) (symbol **Sv**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [equivalent dose](https://en.wikipedia.org/wiki/Equivalent_dose)
			 * of [ionising radiation](https://en.wikipedia.org/wiki/Ionizing_radiation)
			 * and is defined as m<sup>2</sup> &middot; s<sup>-2</sup>.
			 */
			UNIT_SIEVERT,
			/**
			 * The [katal](https://en.wikipedia.org/wiki/Katal) (symbol **kat**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [catalytic activity](https://en.wikipedia.org/wiki/Catalytic_activity)
			 * and is defined as mol &middot; s<sup>-1</sup>.
			 */
			UNIT_KATAL,
			/**
			 * The [square meter](https://en.wikipedia.org/wiki/Square_metre) (symbol **m<sup>2</sup>**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [area](https://en.wikipedia.org/wiki/Area)
			 * and is defined as m<sup>2</sup>.
			 */
			UNIT_SQUARE_METER,
			/**
			 * The [cubic meter](https://en.wikipedia.org/wiki/Cubic_metre) (symbol **m<sup>3</sup>**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [volume](https://en.wikipedia.org/wiki/Volume)
			 * and is defined as m<sup>3</sup>.
			 */
			UNIT_CUBIC_METER,
			/**
			 * The [meter per second](https://en.wikipedia.org/wiki/Metre_per_second) (symbol **m &middot; s<sup>-1</sup>**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of both [speed](https://en.wikipedia.org/wiki/Speed) and
			 * [velocity](https://en.wikipedia.org/wiki/Velocity)
			 * and is defined as m &middot; s<sup>-1</sup>.
			 */
			UNIT_METER_PER_SECOND,
			/**
			 * The [meter per second squared](https://en.wikipedia.org/wiki/Metre_per_second_squared) (symbol **m &middot; s<sup>-2</sup>**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [acceleration](https://en.wikipedia.org/wiki/Acceleration)
			 * and is defined as m &middot; s<sup>-2</sup>.
			 */
			UNIT_METER_PER_SECOND_SQUARED,
			/**
			 * The kilogram per square meter (symbol **kg &middot; m<sup>-2</sup>**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [surface density](https://en.wikipedia.org/wiki/Surface_density)
			 * and is defined as kg &middot; m<sup>-2</sup>.
			 */
			UNIT_KILOGRAM_PER_SQUARE_METER,
			/**
			 * The [kilogram per cubic meter](https://en.wikipedia.org/wiki/Kilogram_per_cubic_metre) (symbol **kg &middot; m<sup>-3</sup>**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [density](https://en.wikipedia.org/wiki/Density)
			 * and is defined as kg &middot; m<sup>-3</sup>.
			 */
			UNIT_KILOGRAM_PER_CUBIC_METER,
			/**
			 * The [newton meter](https://en.wikipedia.org/wiki/Newton_metre) (symbol **N &middot; m**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [torque](https://en.wikipedia.org/wiki/Torque)
			 * and is defined as kg &middot; m<sup>2</sup> &middot; s<sup>-2</sup>.
			 */
			UNIT_NEWTON_METER,
			/**
			 * The newton per meter (symbol **N &middot; m<sup>-1</sup>**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [surface tension](https://en.wikipedia.org/wiki/Surface_tension)
			 * and is defined as kg &middot; s<sup>-2</sup>.
			 */
			UNIT_NEWTON_PER_METER,
			/**
			 * The [radian per second](https://en.wikipedia.org/wiki/Radian_per_second) (symbol **rad &middot; s<sup>-1</sup>**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [angular velocity](https://en.wikipedia.org/wiki/Angular_velocity)
			 * and is defined as rad &middot; s<sup>-1</sup>.
			 */
			UNIT_RADIAN_PER_SECOND,
			/**
			 * The [radian per second squared](https://en.wikipedia.org/wiki/Radian_per_second_squared) (symbol **rad &middot; s<sup>-2</sup>**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [angular acceleration](https://en.wikipedia.org/wiki/Angular_acceleration)
			 * and is defined as rad &middot; s<sup>-2</sup>.
			 */
			UNIT_RADIAN_PER_SECOND_SQUARED,
			/**
			 * The [newton second](https://en.wikipedia.org/wiki/Newton_second) (symbol **N &middot; s**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [impulse](https://en.wikipedia.org/wiki/Impulse_(physics))
			 * and is defined as kg &middot; m &middot; s<sup>-1</sup>.
			 */
			UNIT_NEWTON_SECOND,
			/**
			 * The newton meter second (symbol **N &middot; m &middot; s**) is the
			 * [SI derived unit](https://en.wikipedia.org/wiki/SI_derived_unit)
			 * of [angular momentum](https://en.wikipedia.org/wiki/Angular_momentum)
			 * and is defined as kg &middot; m<sup>2</sup> &middot; s<sup>-1</sup>.
			 */
			UNIT_NEWTON_METER_SECOND
		};
		
		/**
		 * Constant for converting an angular value in
		 * [degree](https://en.wikipedia.org/wiki/Degree_(angle)) to
		 * [radian](https://en.wikipedia.org/wiki/Radian).
		 *
		 * This is equal to a multiplication by &pi; and a division by 180.
		 */
		static const Real DEG2RAD = TypeTraits<Real>::pi / static_cast<Real>(180);
		
		static const Real GIGA2UNIT = static_cast<Real>(1.0e+9);
		
		/**
		 * Standard acceleration due to gravity.
		 *
		 * [Standard gravity](https://en.wikipedia.org/wiki/Standard_gravity) is the nominal
		 * [gravitational acceleration](https://en.wikipedia.org/wiki/Gravitational_acceleration)
		 * of an object in a vacuum near the surface of the earth. It is defined as 9.80665 m &middot; s<sup>-2</sup>
		 * by [ISO 80000-3](https://en.wikipedia.org/wiki/ISO_80000-3).
		 */
		static const Real GRAVITY = static_cast<Real>(9.80665);
		
		static const Real KILO2UNIT = static_cast<Real>(1.0e+3);
		
		static const Real MEGA2UNIT = static_cast<Real>(1.0e+6);
		
		static const Real MICRO2UNIT = static_cast<Real>(1.0e-6);
		
		static const Real MILLI2UNIT = static_cast<Real>(1.0e-3);
		
		static const Real NANO2UNIT = static_cast<Real>(1.0e-9);
		
		/**
		 * Constant for converting an angular value in
		 * [radian](https://en.wikipedia.org/wiki/Radian) to
		 * [degree](https://en.wikipedia.org/wiki/Degree_(angle)).
		 *
		 * This is equal to a multiplication by 180 and a division by &pi;.
		 */
		static const Real RAD2DEG = static_cast<Real>(180) / TypeTraits<Real>::pi;
		
		static const Real UNIT2GIGA = static_cast<Real>(1.0e-9);
		
		static const Real UNIT2KILO = static_cast<Real>(1.0e-3);
		
		static const Real UNIT2MEGA = static_cast<Real>(1.0e-6);
		
		static const Real UNIT2MICRO = static_cast<Real>(1.0e+6);
		
		static const Real UNIT2MILLI = static_cast<Real>(1.0e+3);
		
		static const Real UNIT2NANO = static_cast<Real>(1.0e+9);
	}
}

#endif // RL_MATH_UNIT_H
