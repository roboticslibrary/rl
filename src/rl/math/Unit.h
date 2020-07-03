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

#include "Constants.h"
#include "Real.h"
#include "Units.h"

#if defined(__GNUC__) || defined(__clang__)
#define RL_MATH_DEPRECATED __attribute__ ((__deprecated__))
#elif defined(_MSC_VER)
#define RL_MATH_DEPRECATED __declspec(deprecated)
#endif

namespace rl
{
	namespace math
	{
		RL_MATH_DEPRECATED typedef Units Unit;
		
		RL_MATH_DEPRECATED constexpr Units UNIT_NONE = Units::none;
		RL_MATH_DEPRECATED constexpr Units UNIT_SECOND = Units::second;
		RL_MATH_DEPRECATED constexpr Units UNIT_METER = Units::meter;
		RL_MATH_DEPRECATED constexpr Units UNIT_KILOGRAM = Units::kilogram;
		RL_MATH_DEPRECATED constexpr Units UNIT_AMPERE = Units::ampere;
		RL_MATH_DEPRECATED constexpr Units UNIT_KELVIN = Units::kelvin;
		RL_MATH_DEPRECATED constexpr Units UNIT_MOLE = Units::mole;
		RL_MATH_DEPRECATED constexpr Units UNIT_CANDELA = Units::candela;
		RL_MATH_DEPRECATED constexpr Units UNIT_RADIAN = Units::radian;
		RL_MATH_DEPRECATED constexpr Units UNIT_STERADIAN = Units::steradian;
		RL_MATH_DEPRECATED constexpr Units UNIT_HERTZ = Units::hertz;
		RL_MATH_DEPRECATED constexpr Units UNIT_NEWTON1 = Units::newton;
		RL_MATH_DEPRECATED constexpr Units UNIT_PASCAL = Units::pascals;
		RL_MATH_DEPRECATED constexpr Units UNIT_JOULE = Units::joule;
		RL_MATH_DEPRECATED constexpr Units UNIT_WATT = Units::watt;
		RL_MATH_DEPRECATED constexpr Units UNIT_COULOMB = Units::coulomb;
		RL_MATH_DEPRECATED constexpr Units UNIT_VOLT = Units::volt;
		RL_MATH_DEPRECATED constexpr Units UNIT_FARAD = Units::farad;
		RL_MATH_DEPRECATED constexpr Units UNIT_OHM = Units::ohm;
		RL_MATH_DEPRECATED constexpr Units UNIT_SIEMENS = Units::siemens;
		RL_MATH_DEPRECATED constexpr Units UNIT_WEBER = Units::weber;
		RL_MATH_DEPRECATED constexpr Units UNIT_TESLA = Units::tesla;
		RL_MATH_DEPRECATED constexpr Units UNIT_HENRY = Units::henry;
		RL_MATH_DEPRECATED constexpr Units UNIT_CELSIUS = Units::celsius;
		RL_MATH_DEPRECATED constexpr Units UNIT_LUMEN = Units::lumen;
		RL_MATH_DEPRECATED constexpr Units UNIT_LUX = Units::lux;
		RL_MATH_DEPRECATED constexpr Units UNIT_BECQUEREL = Units::becquerel;
		RL_MATH_DEPRECATED constexpr Units UNIT_GRAY = Units::gray;
		RL_MATH_DEPRECATED constexpr Units UNIT_SIEVERT = Units::sievert;
		RL_MATH_DEPRECATED constexpr Units UNIT_KATAL = Units::katal;
		RL_MATH_DEPRECATED constexpr Units UNIT_SQUARE_METER = Units::squareMeter;
		RL_MATH_DEPRECATED constexpr Units UNIT_CUBIC_METER = Units::cubicMeter;
		RL_MATH_DEPRECATED constexpr Units UNIT_METER_PER_SECOND = Units::meterPerSecond;
		RL_MATH_DEPRECATED constexpr Units UNIT_METER_PER_SECOND_SQUARED = Units::meterPerSecondSquared;
		RL_MATH_DEPRECATED constexpr Units UNIT_KILOGRAM_PER_SQUARE_METER = Units::kilogramPerSquareMeter;
		RL_MATH_DEPRECATED constexpr Units UNIT_KILOGRAM_PER_CUBIC_METER = Units::kilogramPerCubicMeter;
		RL_MATH_DEPRECATED constexpr Units UNIT_NEWTON_METER = Units::newtonMeter;
		RL_MATH_DEPRECATED constexpr Units UNIT_NEWTON_PER_METER = Units::newtonPerMeter;
		RL_MATH_DEPRECATED constexpr Units UNIT_RADIAN_PER_SECOND = Units::radianPerSecond;
		RL_MATH_DEPRECATED constexpr Units UNIT_RADIAN_PER_SECOND_SQUARED = Units::radianPerSecondSquared;
		RL_MATH_DEPRECATED constexpr Units UNIT_NEWTON_SECOND = Units::newtonSecond;
		RL_MATH_DEPRECATED constexpr Units UNIT_NEWTON_METER_SECOND = Units::newtonMeterSecond;
		
		RL_MATH_DEPRECATED constexpr Real DEG2RAD = constants::deg2rad;
		
		RL_MATH_DEPRECATED constexpr Real GIGA2UNIT = constants::giga2unit;
		
		RL_MATH_DEPRECATED constexpr Real GRAVITY = constants::gravity;
		
		RL_MATH_DEPRECATED constexpr Real KILO2UNIT = constants::kilo2unit;
		
		RL_MATH_DEPRECATED constexpr Real MEGA2UNIT = constants::mega2unit;
		
		RL_MATH_DEPRECATED constexpr Real MICRO2UNIT = constants::micro2unit;
		
		RL_MATH_DEPRECATED constexpr Real MILLI2UNIT = constants::milli2unit;
		
		RL_MATH_DEPRECATED constexpr Real NANO2UNIT = constants::nano2unit;
		
		RL_MATH_DEPRECATED constexpr Real RAD2DEG = constants::rad2deg;
		
		RL_MATH_DEPRECATED constexpr Real UNIT2GIGA = constants::unit2giga;
		
		RL_MATH_DEPRECATED constexpr Real UNIT2KILO = constants::unit2kilo;
		
		RL_MATH_DEPRECATED constexpr Real UNIT2MEGA = constants::unit2mega;
		
		RL_MATH_DEPRECATED constexpr Real UNIT2MICRO = constants::unit2micro;
		
		RL_MATH_DEPRECATED constexpr Real UNIT2MILLI = constants::unit2milli;
		
		RL_MATH_DEPRECATED constexpr Real UNIT2NANO = constants::unit2nano;
	}
}

#endif // RL_MATH_UNIT_H
