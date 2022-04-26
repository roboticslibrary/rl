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

#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ";", "", "", "[", "]")

#include <iostream>
#include <rl/math/CompositeFunction.h>
#include <rl/math/Polynomial.h>
#include <rl/math/Vector.h>

template<typename T>
std::ostream&
operator<<(std::ostream& o, const rl::math::Polynomial<T>& f)
{
	for (std::size_t i = 0; i < f.degree() + 1; ++i)
	{
		o << (i > 0 ? " + " : "") << f.coefficient(i) << " * x^" << i;
	}
	
	return o;
}

template<typename T, typename T2>
void
check(const rl::math::Polynomial<T>& f, const rl::math::Polynomial<T2>& g, const rl::math::Polynomial<T>& h1, const rl::math::CompositeFunction<T, T2>& h2)
{
	for (rl::math::Real i = h1.lower(); i <= h1.upper(); ++i)
	{
		for (std::size_t j = 0; j < 3; ++j)
		{
			T tmp1 = h1(i, j);
			T tmp2 = h2(i, j);
			
			if (!rl::math::TypeTraits<T>::equal(tmp1, tmp2))
			{
				std::cerr << "f(x) = " << f << std::endl;
				std::cerr << "g(x) = " << g << std::endl;
				std::cerr << "h(x) = " << h1 << std::endl;
				std::cerr << "h1(" << i << ", " << j << ") != h2(" << i << ", " << j << ")" << std::endl;
				std::cerr << "h1(" << i << ", " << j << ") = " << tmp1 << std::endl;
				std::cerr << "h2(" << i << ", " << j << ") = " << tmp2 << std::endl;
				std::cerr << "g(" << i << ", 0) = " << g(i, 0) << std::endl;
				std::cerr << "g(" << i << ", 1) = " << g(i, 1) << std::endl;
				std::cerr << "g(" << i << ", 2) = " << g(i, 2) << std::endl;
				std::cerr << "g(" << i << ", 3) = " << g(i, 3) << std::endl;
				std::cerr << "f(g(" << i << "), 0) = " << f(g(i), 0) << std::endl;
				std::cerr << "f(g(" << i << "), 1) = " << f(g(i), 1) << std::endl;
				std::cerr << "f(g(" << i << "), 2) = " << f(g(i), 2) << std::endl;
				std::cerr << "f(g(" << i << "), 3) = " << f(g(i), 3) << std::endl;
				exit(EXIT_FAILURE);
			}
		}
	}
}

int
main(int argc, char** argv)
{
	{
		rl::math::Polynomial<rl::math::Real> f(2);
		f.coefficient(0) = 0;
		f.coefficient(1) = 0;
		f.coefficient(2) = 1;
		f.upper() = 10;
		
		rl::math::Polynomial<rl::math::Real> g(2);
		g.coefficient(0) = 0;
		g.coefficient(1) = 0;
		g.coefficient(2) = 1;
		g.upper() = 10;
		
		rl::math::Polynomial<rl::math::Real> h1(4);
		h1.coefficient(0) = 0;
		h1.coefficient(1) = 0;
		h1.coefficient(2) = 0;
		h1.coefficient(3) = 0;
		h1.coefficient(4) = 1;
		h1.upper() = g.upper();
		
		rl::math::CompositeFunction<rl::math::Real, rl::math::Real> h2(f, g);
		
		check(f, g, h1, h2);
	}
	
	{
		rl::math::Polynomial<rl::math::Real> f(1);
		f.coefficient(0) = -5;
		f.coefficient(1) = -1;
		f.upper() = 1;
		
		rl::math::Polynomial<rl::math::Real> g(3);
		g.coefficient(0) = 0;
		g.coefficient(1) = 0;
		g.coefficient(2) = -3;
		g.coefficient(3) = 4;
		g.upper() = 10;
		
		rl::math::Polynomial<rl::math::Real> h1(3);
		h1.coefficient(0) = -5;
		h1.coefficient(1) = 0;
		h1.coefficient(2) = 3;
		h1.coefficient(3) = -4;
		h1.upper() = g.upper();
		
		rl::math::CompositeFunction<rl::math::Real, rl::math::Real> h2(f, g);
		
		check(f, g, h1, h2);
	}
	
	{
		rl::math::Polynomial<rl::math::Vector3> f(3);
		f.coefficient(0) = rl::math::Vector3(1, 1, -2);
		f.coefficient(1) = rl::math::Vector3(0, 0, 1);
		f.coefficient(2) = rl::math::Vector3::Constant(0);
		f.coefficient(3) = rl::math::Vector3(2, 2, 4);
		f.upper() = 5;
		
		rl::math::Polynomial<rl::math::Real> g(1);
		g.coefficient(0) = 3;
		g.coefficient(1) = -2;
		g.upper() = 10;
		
		rl::math::Polynomial<rl::math::Vector3> h1(3);
		h1.coefficient(0) = rl::math::Vector3(55, 55, 109);
		h1.coefficient(1) = rl::math::Vector3(-108, -108, -218);
		h1.coefficient(2) = rl::math::Vector3(72, 72, 144);
		h1.coefficient(3) = rl::math::Vector3(-16, -16, -32);
		h1.upper() = g.upper();
		
		rl::math::CompositeFunction<rl::math::Vector3, rl::math::Real> h2(f, g);
		
		check(f, g, h1, h2);
	}
	
	return EXIT_SUCCESS;
}
