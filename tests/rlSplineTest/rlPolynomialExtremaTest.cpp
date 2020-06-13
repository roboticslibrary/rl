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
	{
		rl::math::Polynomial<rl::math::ArrayX> p = rl::math::Polynomial<rl::math::ArrayX>::Quadratic(
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 0),
			1
		);
		
		if (p.getAbsoluteMaximum()(0) != 1)
		{
			std::cerr << "rlPolynomialExtremaTest: p.getAbsoluteMaximum()[0] != 1" << std::endl;
			return EXIT_FAILURE;
		}
		
		if (p.derivative().getAbsoluteMaximum()(0) != 2)
		{
			std::cerr << "rlPolynomialExtremaTest: p.derivative().getAbsoluteMaximum()[0] != 2" << std::endl;
			return EXIT_FAILURE;
		}
		
		if (p.derivative().derivative().getAbsoluteMaximum()(0) != 2)
		{
			std::cerr << "rlPolynomialExtremaTest: p.derivative().derivative().getAbsoluteMaximum()[0] != 2" << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	{
		rl::math::Real factor = 1.23;
		rl::math::Real eps = 1e-8;
		
		rl::math::ArrayX x(6);
		x << 0, 1, 2, 3.5, 6, 7;
		std::vector<rl::math::ArrayX> y(6, rl::math::ArrayX(2));
		y[0] << 0, 3;
		y[1] << 1, 2;
		y[2] << 2, -1;
		y[3] << 3, 3;
		y[4] << 3, 7;
		y[5] << -2, -2;
	
		rl::math::Spline<rl::math::ArrayX> original = rl::math::Spline<rl::math::ArrayX>::CubicNatural(x, y);
		rl::math::Spline<rl::math::ArrayX> scaled = original.scaledX(factor);
		
		// original and scaled spline segments should have the same maxima
		
		for (std::size_t i = 0; i < original.size(); ++i)
		{
			rl::math::ArrayX maxSpeedOriginal = original.at(i).derivative().getAbsoluteMaximum();
			rl::math::ArrayX maxSpeedScaled = scaled.at(i).derivative().getAbsoluteMaximum();
			
			if ((maxSpeedOriginal - factor * maxSpeedScaled).matrix().norm() > eps)
			{
				std::cerr << "rlPolynomialExtremaTest: (maxSpeedOriginal - factor * maxSpeedScaled).norm() > eps" << std::endl;
				return EXIT_FAILURE;
			}
			
			rl::math::ArrayX maxAccOriginal = original.at(i).derivative().derivative().getAbsoluteMaximum();
			rl::math::ArrayX maxAccScaled = scaled.at(i).derivative().derivative().getAbsoluteMaximum();
			
			if ((maxAccOriginal - factor * factor * maxAccScaled).matrix().norm() > eps)
			{
				std::cerr << "rlPolynomialExtremaTest: (maxAccOriginal - factor * factor * maxAccScaled).matrix().norm() > eps" << std::endl;
				return EXIT_FAILURE;
			}
			
#if 0
			std::cout << "maxSpeedOriginal: " << maxSpeedOriginal.transpose() << std::endl;
			std::cout << "maxSpeedScaled: " << maxSpeedScaled.transpose() << std::endl;
			std::cout << "maxAccOriginal: " << maxAccOriginal.transpose() << std::endl;
			std::cout << "maxAccScaled: " << maxAccScaled.transpose() << std::endl;
#endif
		}
		
	}
	
	std::cout << "rlPolynomialExtremaTest: Polynomial::getAbsoluteMaximum() is ok." << std::endl;
	
	return EXIT_SUCCESS;
}
