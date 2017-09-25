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

#include <iostream>
#include <rl/math/TrapezoidalVelocity.h>

int
main(int argc, char** argv)
{
	rl::math::TrapezoidalVelocity<rl::math::Real> interpolator;
	
	interpolator.x0 = 50.0;
	interpolator.xe = -200.0;
	interpolator.v0 = 5.0;
	interpolator.ve = 0.0;
	interpolator.vm = 10.5;
	interpolator.am = 0.055;
	interpolator.dm = 0.055;
	
	interpolator.interpolate();
	
	double updateRate = 0.00711;
	double t = interpolator.t();
	
	for (std::size_t i = 0; i < t / updateRate * 1.1; ++i)
	{
		interpolator.interpolate();
		double x = interpolator.x(updateRate);
		double v = interpolator.v(updateRate);
		double a = interpolator.a(updateRate);
		std::cout << i * updateRate;
		std::cout << " ";
		std::cout << x;
		std::cout << " ";
		std::cout << v * 100.0f;
		std::cout << " ";
		std::cout << a * 1000.0f;
		std::cout << std::endl;
		interpolator.x0 = x;
		interpolator.v0 = v;
	}
	
	return EXIT_SUCCESS;
}
