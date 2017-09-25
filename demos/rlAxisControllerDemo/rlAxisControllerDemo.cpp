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
#include <rl/hal/UniversalRobotsRtde.h>
#include <rl/math/Polynomial.h>
#include <rl/math/Unit.h>

#define COACH
//#define GNUPLOT
//#define MITSUBISHI
//#define UNIVERSAL_ROBOTS_RTDE

#define CUBIC
//#define QUINTIC

int
main(int argc, char** argv)
{
	try
	{
#ifdef COACH
		rl::hal::Coach controller(6, std::chrono::microseconds(7110), 0, "localhost");
#endif // COACH
#ifdef GNUPLOT
		rl::hal::Gnuplot controller(6, std::chrono::microseconds(7110), -10.0f * rl::math::DEG2RAD, 10.0f * rl::math::DEG2RAD);
#endif // GNUPLOT
#ifdef MITSUBISHI
		rl::hal::MitsubishiH7 controller(6, "left", "lefthost");
#endif // MITSUBISHI
#ifdef UNIVERSAL_ROBOTS_RTDE
		rl::hal::UniversalRobotsRtde controller("localhost");
#endif // UNIVERSAL_ROBOTS_RTDE
		
		rl::math::Real updateRate = std::chrono::duration_cast<std::chrono::duration<rl::math::Real>>(controller.getUpdateRate()).count();
		
		controller.open();
		controller.start();
		
		controller.step();
		
		rl::math::Vector q0 = controller.getJointPosition();
		rl::math::Vector q1 = q0 + rl::math::Vector::Constant(controller.getDof(), 5.0f * rl::math::DEG2RAD);
		
		rl::math::Real te = updateRate * 300.0f;
		
		rl::math::Vector q(controller.getDof());
		
#ifdef CUBIC
		rl::math::Polynomial<rl::math::Vector> interpolator = rl::math::Polynomial<rl::math::Vector>::CubicFirst(
			q0,
			q1,
			rl::math::Vector::Zero(controller.getDof()),
			rl::math::Vector::Zero(controller.getDof()),
			te
		);
#endif // CUBIC
#ifdef QUINTIC
		rl::math::Polynomial<rl::math::Vector> interpolator = rl::math::Polynomial<rl::math::Vector>::QuinticFirstSecond(
			q0,
			q1,
			rl::math::Vector::Zero(controller.getDof()),
			rl::math::Vector::Zero(controller.getDof()),
			rl::math::Vector::Zero(controller.getDof()),
			rl::math::Vector::Zero(controller.getDof()),
			te
		);
#endif // QUINTIC
		
		for (std::size_t i = 0; i <= std::ceil(te / updateRate); ++i)
		{
			q = interpolator(i * updateRate);
			controller.setJointPosition(q);
			controller.step();
		}
		
#ifdef CUBIC
		interpolator = rl::math::Polynomial<rl::math::Vector>::CubicFirst(
			q1,
			q0,
			rl::math::Vector::Zero(controller.getDof()),
			rl::math::Vector::Zero(controller.getDof()),
			te
		);
#endif // CUBIC
#ifdef QUINTIC
		interpolator = rl::math::Polynomial<rl::math::Vector>::QuinticFirstSecond(
			q1,
			q0,
			rl::math::Vector::Zero(controller.getDof()),
			rl::math::Vector::Zero(controller.getDof()),
			rl::math::Vector::Zero(controller.getDof()),
			rl::math::Vector::Zero(controller.getDof()),
			te
		);
#endif // QUINTIC
		
		for (std::size_t i = 0; i <= std::ceil(te / updateRate); ++i)
		{
			q = interpolator(i * updateRate);
			controller.setJointPosition(q);
			controller.step();
		}
		
		controller.stop();
		controller.close();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
