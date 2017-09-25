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
	/* We mostly test against (Eigen) asserts in the Spline methods,
	 * the tests for continuity are just in addition.
	 */ 
	
	{
		// One-dimensional
		::std::vector<rl::math::Real> x;
		x.push_back(0);
		x.push_back(1);
		x.push_back(3);
		x.push_back(3.5);
		x.push_back(6);
		x.push_back(7);
		::std::vector<rl::math::Real> y;
		y.push_back(0);
		y.push_back(1);
		y.push_back(-1);
		y.push_back(3);
		y.push_back(3);
		y.push_back(-2);
		rl::math::Real yd0 = 0;
		rl::math::Real yd1 = 0;
		rl::math::Spline<rl::math::Real> s3 = rl::math::Spline<rl::math::Real>::LinearParabolic(x, y, 0.25);
		rl::math::Spline<rl::math::Real> s4 = rl::math::Spline<rl::math::Real>::LinearQuartic(x, y, 0.25);
		rl::math::Spline<rl::math::Real> s5 = rl::math::Spline<rl::math::Real>::LinearSextic(x, y, 0.25);
		
		if (!s3.isContinuous(1))
		{
			std::cerr <<  "LinearParabolic should be 1-continuous." << std::endl;
			return EXIT_FAILURE;
		}
		
		if (!s4.isContinuous(2))
		{
			std::cerr <<  "LinearQuartic should be 2-continuous." << std::endl;
			return EXIT_FAILURE;
		}
		
		if (!s5.isContinuous(3))
		{
			std::cerr <<  "LinearSextic should be 3-continuous." << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	{
		// Two-dimensional
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
		rl::math::Spline<rl::math::ArrayX> s3 = rl::math::Spline<rl::math::ArrayX>::LinearParabolic(x, y, 0.25);
		rl::math::Spline<rl::math::ArrayX> s4 = rl::math::Spline<rl::math::ArrayX>::LinearQuartic(x, y, 0.25);
		rl::math::Spline<rl::math::ArrayX> s5 = rl::math::Spline<rl::math::ArrayX>::LinearSextic(x, y, 0.25);
		
		if (!s3.isContinuous(1))
		{
			std::cerr <<  "LinearParabolic should be 1-continuous." << std::endl;
			return EXIT_FAILURE;
		}
		
		if (!s4.isContinuous(2))
		{
			std::cerr <<  "LinearQuartic should be 2-continuous." << std::endl;
			return EXIT_FAILURE;
		}
		
		if (!s5.isContinuous(3))
		{
			std::cerr <<  "LinearSextic should be 3-continuous." << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << "rlSplineTest: Done." << std::endl;
	return EXIT_SUCCESS;
}
