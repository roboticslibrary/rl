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
#include <rl/math/Polynomial.h>

template<typename T>
void
check(const rl::math::Polynomial<T>& f, const T& y0, const T& y1, const T& ydmax, const T& yddmax, const T& ydddmax)
{
	rl::math::Real epsilon = Eigen::NumTraits<rl::math::Real>::dummy_precision();
	
	T f0 = f(f.lower());
	T f1 = f(f.upper());
	
	rl::math::Polynomial<T> fd = f.derivative();
	rl::math::Polynomial<T> fdd = fd.derivative();
	rl::math::Polynomial<T> fddd = fdd.derivative();
	
	T fdmax = fd.getAbsoluteMaximum();
	T fddmax = fdd.getAbsoluteMaximum();
	T fdddmax = fddd.getAbsoluteMaximum();
	
	bool result = true;
	
	if (!rl::math::TypeTraits<T>::equal(f0, y0))
	{
		std::cerr << "f(" << f.lower() << ") != y0" << std::endl;
		result = false;;
	}
	
	if (!rl::math::TypeTraits<T>::equal(f1, y1))
	{
		std::cerr << "f(" << f.upper() << ") != y1" << std::endl;
		result = false;;
	}
	
	if ((fdmax.array().abs() > ydmax.array() + epsilon).any())
	{
		std::cerr << "abs(fdmax) > ydmax" << std::endl;
		result = false;;
	}
	
	if ((fddmax.array().abs() > yddmax.array() + epsilon).any())
	{
		std::cerr << "abs(fddmax) > yddmax" << std::endl;
		result = false;;
	}
	
	if ((fdddmax.array().abs() > ydddmax.array() + epsilon).any())
	{
		std::cerr << "abs(fdddmax) > ydddmax" << std::endl;
		result = false;;
	}
	
	if (!result)
	{
		std::cerr << "--------------------------------------------------------------------------------" << std::endl;
		std::cerr << "degree(f) = " << f.degree() << std::endl;
		std::cerr << "y0 = " << f0 << std::endl;
		std::cerr << "y1 = " << y1 << std::endl;
		std::cerr << "ydmax = " << ydmax << std::endl;
		std::cerr << "yddmax = " << yddmax << std::endl;
		std::cerr << "ydddmax = " << ydddmax << std::endl;
		std::cerr << "--------------------------------------------------------------------------------" << std::endl;
		std::cerr << "x0 = " << f.lower() << std::endl;
		std::cerr << "x1 = " << f.upper() << std::endl;
		std::cerr << "f0 = " << f0 << std::endl;
		std::cerr << "f1 = " << f1 << std::endl;
		std::cerr << "fdmax = " << fdmax << std::endl;
		std::cerr << "fddmax = " << fddmax << std::endl;
		std::cerr << "fdddmax = " << fdddmax << std::endl;
		exit(EXIT_FAILURE);
	}
}

int
main(int argc, char** argv)
{
	std::size_t n = 6;
	
	for (std::size_t i = 0; i < 100; ++i)
	{
		rl::math::Vector y0 = rl::math::Vector::Random(n) * 10;
		rl::math::Vector y1 = rl::math::Vector::Random(n) * 10;
		rl::math::Vector ydmax = rl::math::Vector::Random(n).cwiseAbs() * 10;
		rl::math::Vector yddmax = rl::math::Vector::Random(n).cwiseAbs() * 20;
		rl::math::Vector ydddmax = rl::math::Vector::Random(n).cwiseAbs() * 40;
		
		rl::math::Polynomial<rl::math::Vector> f3 = rl::math::Polynomial<rl::math::Vector>::CubicAtRest(y0, y1, ydmax, yddmax, ydddmax);
		check(f3, y0, y1, ydmax, yddmax, ydddmax);
		
		rl::math::Polynomial<rl::math::Vector> f5 = rl::math::Polynomial<rl::math::Vector>::QuinticAtRest(y0, y1, ydmax, yddmax, ydddmax);
		check(f5, y0, y1, ydmax, yddmax, ydddmax);
		
		rl::math::Polynomial<rl::math::Vector> f7 = rl::math::Polynomial<rl::math::Vector>::SepticAtRest(y0, y1, ydmax, yddmax, ydddmax);
		check(f7, y0, y1, ydmax, yddmax, ydddmax);
	}
	
	return EXIT_SUCCESS;
}
