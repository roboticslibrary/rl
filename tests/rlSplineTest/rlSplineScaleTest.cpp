//
// Copyright (c) 2013, Andre Gaschler
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
#include <rl/math/Array.h>
#include <rl/math/Spline.h>

int
main(int argc, char** argv)
{
	rl::math::Real factor = 1.23;
	rl::math::Real eps = 1e-8;
	
	::std::vector<rl::math::Real> x;
	x.push_back(0);
	x.push_back(1);
	x.push_back(3);
	x.push_back(3.5);
	x.push_back(6);
	x.push_back(7);
	::std::vector<rl::math::ArrayX> y;
	y.push_back(rl::math::ArrayX::Constant(2, 0));
	y.push_back(rl::math::ArrayX::Constant(2, 1));
	y.push_back(rl::math::ArrayX::Constant(2, -1));
	y.push_back(rl::math::ArrayX::Constant(2, 3));
	y.push_back(rl::math::ArrayX::Constant(2, 3));
	y.push_back(rl::math::ArrayX::Constant(2, -2));
	rl::math::ArrayX yd0(2);
	yd0 << 0, 2;
	rl::math::ArrayX yd1(2);
	yd1 << 0, 3;
	rl::math::Spline<rl::math::ArrayX> original = rl::math::Spline<rl::math::ArrayX>::LinearQuartic(x, y, 0.25);
	
	rl::math::Spline<rl::math::ArrayX> scaled = original.scaledX(factor);
	
	if (std::abs(original.duration() * factor - scaled.duration()) > eps)
	{
		std::cerr << "::std::abs(original.duration() * factor - scaled.duration()) > eps" << std::endl;
		return EXIT_FAILURE;
	}
	
	for (rl::math::Real t = 0; t < original.duration() - eps; t += 0.01)
	{
		if ((original(t) - scaled(factor * t)).matrix().norm() > eps)
		{
			std::cerr << "(original(t) - scaled(factor * t)).norm() > eps" << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << "rlSplineTest2: Verify Spline::scaledX done." << std::endl;
	return EXIT_SUCCESS;
}
