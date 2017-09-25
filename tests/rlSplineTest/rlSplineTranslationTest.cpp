//
// Copyright (c) 2016, Andre Gaschler
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

void printPoly(const rl::math::Polynomial<rl::math::ArrayX>& p)
{
	for (int n = p.degree(); n >= 0; --n)
	{
		if (p.coefficient(n).matrix().squaredNorm() != 0)
		{
			std::cout << p.coefficient(n) << "*x^" << n << " ";
		}
	}
	
	std::cout << std::endl;
}

bool testPolynomialTranslation(const rl::math::Polynomial<rl::math::ArrayX>& p)
{
	rl::math::Real eps = 1e-6;
	rl::math::Real translation = 0.2 + (p.degree() / 3);
	rl::math::Polynomial<rl::math::ArrayX> q = p.translatedX(translation);
	printPoly(p);
	printPoly(q);
	bool ok = true;
	
	for (rl::math::Real x = 0; x < 1; x += 0.1)
	{
		if ((p(x) - q(x - translation)).matrix().squaredNorm() > eps)
		{
			ok = false;
		}
	}
	
	if (!ok)
	{
		std::cerr << "testPolynomialTranslation: at degree " << p.degree() << " a difference was detected." << std::endl;
	}
	
	return ok;
}

int
main(int argc, char** argv)
{
	bool ok = true;
	
	for (std::size_t degree = 5; degree < 15; ++degree)
	{
		rl::math::Polynomial<rl::math::ArrayX> p(degree);
		
		for (std::size_t n = 0; n < p.degree() + 1; ++n)
		{
			p.coefficient(n) = rl::math::ArrayX::Zero(1);
		}
		
		p.coefficient(degree) = rl::math::ArrayX::Constant(1, 1.7);
		p.coefficient(degree / 2) = rl::math::ArrayX::Constant(1, -2.5);
		p.coefficient(degree / 3) = rl::math::ArrayX::Constant(1, 1.5);
		ok &= testPolynomialTranslation(p);
	
		p.coefficient(degree - 1) = rl::math::ArrayX::Constant(1, -0.5);
		ok &= testPolynomialTranslation(p);
	}
	
	if (ok)
	{
		std::cout << "rlSplineTranslationTest: Verify Polynomial::translatedX done." << std::endl;
		return EXIT_SUCCESS;
	}
	else
	{
		std::cerr << "rlSplineTranslationTest: Polynomial::translatedX is wrong." << std::endl;
		return EXIT_FAILURE;
	}
}
