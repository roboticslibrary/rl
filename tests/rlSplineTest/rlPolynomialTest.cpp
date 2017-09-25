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
#include <rl/math/Polynomial.h>

int
main(int argc, char** argv)
{
	rl::math::Real eps = 1e-8;
	rl::math::Real y0 = 2, y1 = 3, yd0 = 4, yd1 = 5, ydd0 = 6, ydd1 = 7, yddd0 = 8, yddd1 = 9, x1 = 10;
	
	{
		rl::math::Polynomial<rl::math::Real> p = rl::math::Polynomial<rl::math::Real>::Quadratic(y0, y1, yd0, x1);
		
		if (std::abs(p(0) - y0) > eps ||
			std::abs(p(x1) - y1) > eps ||
			std::abs(p(0, 1) - yd0) > eps)
		{
			std::cerr << "rlPolynomialTest: Quadratic" << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	{
		rl::math::Polynomial<rl::math::Real> p = rl::math::Polynomial<rl::math::Real>::CubicFirst(y0, y1, yd0, yd1, x1);
		
		if (std::abs(p(0) - y0) > eps ||
			std::abs(p(x1) - y1) > eps ||
			std::abs(p(0, 1) - yd0) > eps ||
			std::abs(p(x1, 1) - yd1) > eps)
		{
			std::cerr << "rlPolynomialTest: CubicFirst" << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	{
		rl::math::Polynomial<rl::math::Real> p = rl::math::Polynomial<rl::math::Real>::QuarticFirstSecond(y0, y1, yd0, yd1, ydd0, x1);
		
		if (std::abs(p(0) - y0) > eps ||
			std::abs(p(x1) - y1) > eps ||
			std::abs(p(0, 1) - yd0) > eps ||
			std::abs(p(x1, 1) - yd1) > eps ||
			std::abs(p(0, 2) - ydd0) > eps)
		{
			std::cerr << "rlPolynomialTest: QuarticFirstSecond" << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	{
		rl::math::Polynomial<rl::math::Real> p = rl::math::Polynomial<rl::math::Real>::QuinticFirstSecond(y0, y1, yd0, yd1, ydd0, ydd1, x1);
		
		if (std::abs(p(0) - y0) > eps ||
			std::abs(p(x1) - y1) > eps ||
			std::abs(p(0, 1) - yd0) > eps ||
			std::abs(p(x1, 1) - yd1) > eps ||
			std::abs(p(0, 2) - ydd0) > eps ||
			std::abs(p(x1, 2) - ydd1) > eps)
		{
			std::cerr << "rlPolynomialTest: QuinticFirstSecond" << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	{
		rl::math::Polynomial<rl::math::Real> p = rl::math::Polynomial<rl::math::Real>::SexticFirstSecondThird(y0, y1, yd0, yd1, ydd0, ydd1, yddd0, x1);
		
		if (std::abs(p(0) - y0) > eps ||
			std::abs(p(x1) - y1) > eps ||
			std::abs(p(0, 1) - yd0) > eps ||
			std::abs(p(x1, 1) - yd1) > eps ||
			std::abs(p(0, 2) - ydd0) > eps ||
			std::abs(p(x1, 2) - ydd1) > eps ||
			std::abs(p(0, 3) - yddd0) > eps)
		{
			std::cerr << "rlPolynomialTest: SexticFirstSecondThird" << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	{
		rl::math::Polynomial<rl::math::Real> p = rl::math::Polynomial<rl::math::Real>::SepticFirstSecondThird(y0, y1, yd0, yd1, ydd0, ydd1, yddd0, yddd1, x1);
		
		if (std::abs(p(0) - y0) > eps ||
			std::abs(p(x1) - y1) > eps ||
			std::abs(p(0, 1) - yd0) > eps ||
			std::abs(p(x1, 1) - yd1) > eps ||
			std::abs(p(0, 2) - ydd0) > eps ||
			std::abs(p(x1, 2) - ydd1) > eps ||
			std::abs(p(0, 3) - yddd0) > eps ||
			std::abs(p(x1, 3) - yddd1) > eps)
		{
			std::cerr << "rlPolynomialTest: SepticFirstSecondThird" << std::endl;
			return EXIT_FAILURE;
		}
	}

	std::cout << "rlPolynomialTest is ok." << std::endl;
	return EXIT_SUCCESS;
}
