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
#include <rl/math/Polynomial.h>

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
	
	std::cout << "rlPolynomialExtremaTest: Polynomial::getAbsoluteMaximum() is ok." << std::endl;
	
	return EXIT_SUCCESS;
}
