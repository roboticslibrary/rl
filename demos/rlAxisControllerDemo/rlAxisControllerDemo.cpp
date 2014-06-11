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
#include <stdexcept>
#include <rl/hal/Coach.h>
#include <rl/hal/Gnuplot.h>
#include <rl/hal/MitsubishiH7.h>
#include <rl/math/Cubic.h>
#include <rl/math/Quintic.h>
#include <rl/math/Unit.h>

#define COACH
//#define GNUPLOT
//#define MITSUBISHI

#define CUBIC
//#define QUINTIC

int
main(int argc, char** argv)
{
	try
	{
#ifdef COACH
		rl::hal::Coach controller(6, 0.00711f, 0, "localhost");
#endif // COACH
#ifdef GNUPLOT
		rl::hal::Gnuplot controller(6, 0.00711f, -10.0f * rl::math::DEG2RAD, 10.0f * rl::math::DEG2RAD);
#endif // GNUPLOT
#ifdef MITSUBISHI
		rl::hal::MitsubishiH7 controller(6, "left", "lefthost");
#endif // MITSUBISHI
		
		controller.open();
		controller.start();
		
		controller.step();
		
		rl::math::Vector x0(controller.getDof());
		x0.setZero();
#ifdef MITSUBISHI
		controller.getJointPosition(x0);
#endif // MITSUBISHI
		
		rl::math::Vector xe(controller.getDof());
		xe = x0 + ::rl::math::Vector::Constant(controller.getDof(), 5.0f * rl::math::DEG2RAD);
		
#ifdef CUBIC
		std::vector< rl::math::Cubic< rl::math::Real > > interpolator(controller.getDof());
#endif // CUBIC
#ifdef QUINTIC
		std::vector< rl::math::Quintic< rl::math::Real > > interpolator(controller.getDof());
#endif // QUINTIC
		
		rl::math::Real te = controller.getUpdateRate() * 300.0f;
		
		for (std::size_t i = 0; i < interpolator.size(); ++i)
		{
			interpolator[i].te = te;
			interpolator[i].v0 = 0;
			interpolator[i].ve = 0;
#ifdef QUINTIC
			interpolator[i].a0 = 0;
			interpolator[i].ae = 0;
#endif // QUINTIC
		}
		
		rl::math::Vector x(controller.getDof());
		
		// start -> goal
		
		for (std::size_t i = 0; i < interpolator.size(); ++i)
		{
			interpolator[i].x0 = x0[i];
			interpolator[i].xe = xe[i];
			interpolator[i].interpolate();
		}
		
		for (std::size_t i = 0; i <= std::ceil(te / controller.getUpdateRate()); ++i)
		{
			for (std::size_t j = 0; j < controller.getDof(); ++j)
			{
				x(j) = interpolator[j].x(i * controller.getUpdateRate());
			}
			
			controller.setJointPosition(x);
			
			controller.step();
		}
		
		// goal -> start
		
		for (std::size_t i = 0; i < interpolator.size(); ++i)
		{
			interpolator[i].x0 = xe[i];
			interpolator[i].xe = x0[i];
			interpolator[i].interpolate();
		}
		
		for (std::size_t i = 0; i <= std::ceil(te / controller.getUpdateRate()); ++i)
		{
			for (std::size_t j = 0; j < controller.getDof(); ++j)
			{
				x(j) = interpolator[j].x(i * controller.getUpdateRate());
			}
			
			controller.setJointPosition(x);
			
			controller.step();
		}
		
		controller.stop();
		controller.close();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	
	return 0;
}
