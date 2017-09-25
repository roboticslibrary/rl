//
// Copyright (c) 2015, Andre Gaschler
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
#include <fstream>
#include <rl/math/Array.h>
#include <rl/math/Spline.h>

int
main(int argc, char** argv)
{
#if 0
	{
		for (std::size_t i = 0; i < 4; ++i)
		{
			rl::math::Real q0 = 1, q1 = 7;
			rl::math::Real vmax = (i % 2 == 0) ? 1 : 1000;
			rl::math::Real amax = 10;
			rl::math::Real jmax = (i / 2 == 0) ? 3 : 300;
			rl::math::Spline<rl::math::Real> f = rl::math::Spline<rl::math::Real>::TrapeziodalAccelerationAtRest(q0, q1, vmax, amax, jmax);
	
#if 0
			// plot for [i=2:5] "interpolation.dat" using 1:i with lines
			std::ofstream stream;
			stream.open("interpolation.dat", std::fstream::trunc);
			int steps = 2000;
			for (std::size_t i = 0; i < steps; ++i)
			{
				rl::math::Real t = f.duration() * i / static_cast< rl::math::Real >(steps - 1);
				stream << t << "\t" << f(t)(0) << "\t" << f(t, 1)(0) << "\t" << f(t, 2)(0) << "\t" << f(t, 3)(0) << std::endl;
			}
#endif
			
			rl::math::Real eps = 1e-8;
			
			if (std::abs(f(f.lower()) - q0) > eps ||
				std::abs(f(f.lower(), 1) - 0) > eps ||
				std::abs(f(f.lower(), 2) - 0) > eps ||
				std::abs(f(f.upper()) - q1) > eps ||
				std::abs(f(f.upper(), 1) - 0) > eps ||
				std::abs(f(f.upper(), 2) - 0) > eps)
			{
				std::cerr <<  "TrapeziodalAcceleration must fulfill boundary conditions." << std::endl;
				return EXIT_FAILURE;
			}
			
			if (!f.isContinuous(2))
			{
				std::cerr <<  "TrapeziodalAcceleration must be 2-continuous." << std::endl;
				return EXIT_FAILURE;
			}
		}
		
		std::cout << "TrapeziodalAcceleration(Real): Ok, done." << std::endl;
	}
#endif
	
	{
		for (std::size_t i = 0; i < 4; ++i)
		{
			rl::math::ArrayX q0(4);
			rl::math::ArrayX q1(4);
			rl::math::ArrayX vmax(4);
			rl::math::ArrayX amax(4);
			rl::math::ArrayX jmax(4);
			
			q0 << 0, 1, 3, 3.5;
			q1 << -1, 4, 2, -0.6;
			
			if (i & 0x1)
			{
				vmax << 1, 2, 3, 4;
			}
			else
			{
				vmax << 400, 400, 500, 200;
			}
			
			if (i & 0x2)
			{
				amax << 400, 400, 500, 200;
			}
			else
			{
				amax << 3, 10, 5, 10;
			}
			
			jmax << 100, 100, 100, 100;
			
			rl::math::Spline<rl::math::ArrayX> f = rl::math::Spline<rl::math::ArrayX>::TrapeziodalAccelerationAtRest(q0, q1, vmax, amax, jmax);
			
#if 0
			// plot for [i=2:5] "interpolation.dat" using 1:i with lines
			std::ofstream stream;
			stream.open("interpolation.dat", std::fstream::trunc);
			std::size_t steps = 2000;
			for (std::size_t i = 0; i < steps; ++i)
			{
				rl::math::Real t = f.duration() * i / static_cast<rl::math::Real>(steps - 1);
				stream << t << "\t" << f(t)(1) << "\t" << f(t, 1)(1) << "\t" << f(t, 2)(1) << "\t" << f(t, 3)(1) << std::endl;
			}
#endif
			
			rl::math::Real eps = 1e-6;
			
			if ((f(f.lower()) - q0).matrix().norm() > eps ||
				f(f.lower(), 1).matrix().norm() > eps ||
				f(f.lower(), 2).matrix().norm() > eps ||
				(f(f.upper()) - q1).matrix().norm() > eps ||
				f(f.upper(), 1).matrix().norm() > eps ||
				f(f.upper(), 2).matrix().norm() > eps)
			{
				std::cerr << "rlTrapeziodalAccelerationTest q1: " << q1.transpose() << " vmax: " << vmax.transpose() << " amax: " << amax.transpose() << std::endl;
				std::cerr << "rlTrapeziodalAccelerationTest must fulfill boundary conditions up to acceleration." << std::endl;
				return EXIT_FAILURE;
			}
			
			if (!f.isContinuous(2))
			{
				std::cerr << "rlTrapeziodalAccelerationTest q1: " << q1.transpose() << " vmax: " << vmax.transpose() << " amax: " << amax.transpose() << std::endl;
				std::cerr << "rlTrapeziodalAccelerationTest must be 2-continuous." << std::endl;
				return EXIT_FAILURE;
			}
			
			rl::math::ArrayX maxVel = f.derivative().getAbsoluteMaximum();
			rl::math::ArrayX maxAcc = f.derivative().derivative().getAbsoluteMaximum();
			rl::math::ArrayX maxJerk = f.derivative().derivative().derivative().getAbsoluteMaximum();
			
			if ((maxVel / vmax > 1.001f).any() ||
				(maxAcc / amax > 1.001f).any() ||
				(maxJerk / jmax > 1.001f).any())
			{
				std::cerr << "rlTrapeziodalAccelerationTest must not exceed given bounds (by more than 1e-3)." << std::endl;
				std::cout << "Time: " << f.duration() << std::endl;
				std::cout << " vmax: " << vmax.transpose() << std::endl;
				std::cout << " maxVel: " << maxVel.transpose() << std::endl;
				std::cout << " amax: " << amax.transpose() << std::endl;
				std::cout << " maxAcc: " << maxAcc.transpose() << std::endl;
				std::cout << " jmax: " << jmax.transpose() << std::endl;
				std::cout << " maxJerk: " << maxJerk.transpose() << std::endl;
				return EXIT_FAILURE;
			}
		}
		
		std::cout << "rlTrapeziodalAccelerationTest(ArrayX): Ok, done." << std::endl;
	}
	
	std::cout << "rlTrapeziodalAccelerationTest: Done." << std::endl;
	
	return EXIT_SUCCESS;
}
