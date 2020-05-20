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
#include <rl/math/algorithm.h>
#include <rl/math/Polynomial.h>

std::ostream& operator<<(std::ostream& os, const rl::math::Polynomial<rl::math::Real>& p)
{
	bool first = true;
	
	for (int n = p.degree(); n >= 0; --n)
	{
		if (std::abs(p.coefficient(n)) > ::std::numeric_limits<rl::math::Real>::epsilon())
		{
			if (first)
			{
				os << p.coefficient(n);
			}
			else
			{
				os << (rl::math::sign(p.coefficient(n)) < 0 ? " - " : " + ") << std::abs(p.coefficient(n));
			}
			
			if (n > 0)
			{
				os << " x^" << n;
			}
			
			first = false;
		}
	}
	
	return os;
}

bool testPolynomialTranslation(const rl::math::Polynomial<rl::math::Real>& p)
{
	rl::math::Real translation = Eigen::internal::random(-10.0, 10.0);
	rl::math::Polynomial<rl::math::Real> q = p.translatedX(translation);
	
	std::cout << p.degree() << ": p(x) = " << p << std::endl;
	std::cout << q.degree() << ": q(x) = " << q << std::endl;
	std::cout << p.degree() << ": p(x) = q(x " << (rl::math::sign(translation) < 0 ? "-" : "+") << " " << std::abs(translation) << ")" << std::endl;
	
	for (rl::math::Real x = 0; x < 1; x += 0.1)
	{
		rl::math::Real squaredNorm = std::pow(p(x) - q(x - translation), 2);
		
		if (squaredNorm > 1.0e-6)
		{
			std::cerr << "testPolynomialTranslation: Error at degree " << p.degree() << std::endl;
			std::cerr << "p(" << x << ") = " << p(x) << std::endl;
			std::cerr << "q(" << x - translation << ") = " << q(x - translation) << std::endl;
			std::cerr << "squaredNorm: " << squaredNorm << std::endl;
			return false;
		}
	}
	
	return true;
}

int
main(int argc, char** argv)
{
	for (std::size_t degree = 0; degree < 10; ++degree)
	{
		if (degree > 0)
		{
			std::cout << std::endl;
		}
		
		rl::math::Polynomial<rl::math::Real> p(degree);
		p.upper() = 1;
		
		for (std::size_t n = 0; n < p.degree() + 1; ++n)
		{
			p.coefficient(n) = Eigen::internal::random(-10.0, 10.0);
		}
		
		if (!testPolynomialTranslation(p))
		{
			return EXIT_FAILURE;
		}
	}
	
	return EXIT_SUCCESS;
}
