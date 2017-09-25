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
#if !(defined(_MSC_VER) && _MSC_VER < 1800)
	{
		for (std::size_t i = 0; i < 8; ++i)
		{
			rl::math::Real q0 = 1.1;
			rl::math::Real q1 = (i & 0x1) ? 42 : -7;
			rl::math::Real vmax = (i & 0x2) ? 100 : 1;
			rl::math::Real amax = (i & 0x4) ? 1 : 100;
			rl::math::Spline<rl::math::Real> f = rl::math::Spline<rl::math::Real>::QuarticLinearQuarticAtRest(q0, q1, vmax, amax);
		
#if 0
			// plot for [i=2:5] "interpolation.dat" using 1:i with lines
			std::ofstream stream;
			stream.open("interpolation.dat", std::fstream::trunc);
			int steps = 2000;
			for (std::size_t i = 0; i < steps; ++i)
			{
				rl::math::Real t = f.duration() * i / static_cast<rl::math::Real>(steps - 1);
				stream << t << "\t" << f(t)(0) << "\t" << f(t, 1)(0) << "\t" << f(t, 2)(0) << "\t" << f(t, 3)(0) << std::endl;
			}
#endif
			
			rl::math::Real eps = 1e-6;
			
			if (std::abs(f(f.lower()) - q0) > eps ||
				std::abs(f(f.lower(), 1) - 0) > eps ||
				std::abs(f(f.lower(), 2) - 0) > eps ||
				std::abs(f(f.upper()) - q1) > eps ||
				std::abs(f(f.upper(), 1) - 0) > eps ||
				std::abs(f(f.upper(), 2) - 0) > eps)
			{
				std::cerr << "QuarticLinearQuartic q1: " << q1 << " vmax: " << vmax << " amax: " << amax << std::endl;
				std::cerr << "QuarticLinearQuartic must fulfill boundary conditions up to the acceleration." << std::endl;
				return EXIT_FAILURE;
			}
			
			if (!f.isContinuous(2))
			{
				std::cerr << "QuarticLinearQuartic q1: " << q1 << " vmax: " << vmax << " amax: " << amax << std::endl;
				std::cerr << "QuarticLinearQuartic must be 2-continuous." << std::endl;
				return EXIT_FAILURE;
			}
		}
		
		std::cout << "rlQuarticLinearQuarticTest(Real): Ok, done." << std::endl;
	}
#endif
	
	{
		for (std::size_t i = 0; i < 4; ++i)
		{
			rl::math::ArrayX q0(4);
			rl::math::ArrayX q1(4);
			rl::math::ArrayX vmax(4);
			rl::math::ArrayX amax(4);
			
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
			
			rl::math::Spline<rl::math::ArrayX> f = rl::math::Spline<rl::math::ArrayX>::QuarticLinearQuarticAtRest(q0, q1, vmax, amax);
			
#if 1
			// plot for [i=2:5] "interpolation.dat" using 1:i with lines
			std::ofstream stream;
			stream.open("interpolation.dat", std::fstream::trunc);
			int steps = 2000;
			for (std::size_t i = 0; i < steps; ++i)
			{
				rl::math::Real t = f.duration() * i / static_cast<rl::math::Real>(steps - 1);
				stream << t << "\t" << f(t)(0) << "\t" << f(t, 1)(0) << "\t" << f(t, 2)(0) << "\t" << f(t, 3)(0) << std::endl;
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
				std::cerr << "QuarticLinearQuartic q1: " << q1.transpose() << " vmax: " << vmax.transpose() << " amax: " << amax.transpose() << std::endl;
				std::cerr << "QuarticLinearQuartic must fulfill boundary conditions up to acceleration." << std::endl;
				return EXIT_FAILURE;
			}
			
			if (!f.isContinuous(2))
			{
				std::cerr << "QuarticLinearQuartic q1: " << q1.transpose() << " vmax: " << vmax.transpose() << " amax: " << amax.transpose() << std::endl;
				std::cerr << "QuarticLinearQuartic must be 2-continuous." << std::endl;
				return EXIT_FAILURE;
			}
		}
		
		std::cout << "rlQuarticLinearQuarticTest(ArrayX): Ok, done." << std::endl;
	}
	
	std::cout << "rlQuarticLinearQuarticTest: Done." << std::endl;
	
	return EXIT_SUCCESS;
}
