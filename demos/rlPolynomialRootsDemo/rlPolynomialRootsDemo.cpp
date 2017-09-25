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
#include <boost/lexical_cast.hpp>
#include <rl/math/Polynomial.h>

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlPolynomialRootsDemo C0 ... CN" << std::endl;
		return EXIT_FAILURE;
	}
	
	std::vector<rl::math::Real> c(argc - 1);
	
	for (std::size_t i = 0; i < c.size(); ++i)
	{
		c[i] = boost::lexical_cast<rl::math::Real>(argv[i + 1]);
		std::cout << (i > 0 ? " + " : "") << c[i] << " * x^" << i;
	}
	
	std::cout << " = 0" << std::endl;
	
	std::vector<rl::math::Real> roots = rl::math::Polynomial<rl::math::Real>::realRoots(c);
	
	std::cout << roots.size() << " solution" << (roots.size() != 1 ? "(s)" : "") << std::endl;
	
	for (std::size_t i = 0; i < roots.size(); ++i)
	{
		std::cout << "x[" << i << "] = " << roots[i] << std::endl;
	}
	
	return EXIT_SUCCESS;
}
