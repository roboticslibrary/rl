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

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <rl/math/CircularVector2.h>
#include <rl/math/CircularVector3.h>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

template<typename T>
T
firstCentralDifference(const rl::math::Function<T>& f, const rl::math::Real& x)
{
	rl::math::Real eps = 1e-5;
	return (f(x + eps) - f(x - eps)) / (2 * eps);
}

template<typename T>
T
secondCentralDifference(const rl::math::Function<T>& f, const rl::math::Real& x)
{
	rl::math::Real eps = 1e-5;
	return (f(x + eps) - 2 * f(x) + f(x - eps)) / ::std::pow(eps, 2);
}

template<typename T>
void
runTest(const rl::math::Circular<T>& c, const T& y0, const T& yi, const T& y1)
{
	std::cout << "Circular: testing with y0: " << y0.transpose() << " yi: " << yi.transpose() << " y1: " << y1.transpose() << std::endl;
	
	if (::std::abs((y0 - c.getCenter()).norm() - (yi - c.getCenter()).norm()) > 1e-12 ||
		::std::abs((yi - c.getCenter()).norm() - (y1 - c.getCenter()).norm()) > 1e-12)
	{
		std::cerr << "Circular gives inconsistent radius." << std::endl;
		exit(EXIT_FAILURE);
	}
	
	if ((c(c.lower()) - y0).norm() > 1e-12 || (c(c.upper()) - y1).norm() > 1e-12)
	{
		std::cerr << "Circular gives wrong start and/or end points." << std::endl;
		exit(EXIT_FAILURE);
	}
	
	if ((c(0.5, 1) - firstCentralDifference(c, 0.5)).norm() > 1e-4)
	{
		std::cerr << "Circular first derivative seems wrong." << std::endl;
		std::cerr << "c(0.5, 1): " << c(0.5, 1).transpose() << std::endl;
		std::cerr << "firstCentralDifference(c, 0.5): " << firstCentralDifference(c, 0.5).transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	if ((c(0.5, 2) - secondCentralDifference(c, 0.5)).norm() > 1e-4)
	{
		std::cerr << "Circular second derivative seems wrong." << std::endl;
		std::cerr << "c(0.5, 2): " << c(0.5, 2).transpose() << std::endl;
		std::cerr << "secondCentralDifference(c, 0.5): " << secondCentralDifference(c, 0.5).transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	if ((c(0.2, 1) - firstCentralDifference(c, 0.2)).norm() > 1e-4)
	{
		std::cerr << "Circular first derivative seems wrong." << std::endl;
		std::cerr << "c(0.2, 1): " << c(0.2, 1).transpose() << std::endl;
		std::cerr << "firstCentralDifference(c, 0.2): " << firstCentralDifference(c, 0.2).transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	if ((c(0.2, 2) - secondCentralDifference(c, 0.2)).norm() > 1e-4)
	{
		std::cerr << "Circular second derivative seems wrong." << std::endl;
		std::cerr << "c(0.2, 2): " << c(0.2, 2).transpose() << std::endl;
		std::cerr << "secondCentralDifference(c, 0.2): " << secondCentralDifference(c, 0.2).transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
}

int
main(int argc, char** argv)
{
	std::cout.setf(std::ios::fixed, std::ios::floatfield);
	std::cout.precision(10);
	
	{
		rl::math::Vector2 y0(0, 0), yi(0, 1), y1(-1, 0);
		rl::math::Circular<rl::math::Vector2> c = rl::math::Circular<rl::math::Vector2>::ThreePoints(y0, yi, y1);
		runTest(c, y0, yi, y1);
	}
	{
		rl::math::Vector2 y0(2 + 0, 0), yi(2 + 1, -1), y1(2 + 0, -1);
		rl::math::Circular<rl::math::Vector2> c = rl::math::Circular<rl::math::Vector2>::ThreePoints(y0, yi, y1);
		runTest(c, y0, yi, y1);
	}
	{
		rl::math::Vector2 y0(0, 5 + 0), yi(0, 5 + 3), y1(-3, 5 + 0);
		rl::math::Circular<rl::math::Vector2> c = rl::math::Circular<rl::math::Vector2>::ThreePoints(y0, yi, y1);
		runTest(c, y0, yi, y1);
	}
	{
		rl::math::Vector3 y0(0, 5 + 0, 2), yi(0, 5 + 3, 2), y1(-3, 5 + 0, 2);
		rl::math::Circular<rl::math::Vector3> c = rl::math::Circular<rl::math::Vector3>::ThreePoints(y0, yi, y1);
		runTest(c, y0, yi, y1);
	}
	{
		rl::math::Vector3 y0(0, 5 + 0, 2), yi(0, 5 + 3, 2), y1(0.1, 5 + 0, 3);
		rl::math::Circular<rl::math::Vector3> c = rl::math::Circular<rl::math::Vector3>::ThreePoints(y0, yi, y1);
		runTest(c, y0, yi, y1);
	}
	
	return EXIT_SUCCESS;
}
