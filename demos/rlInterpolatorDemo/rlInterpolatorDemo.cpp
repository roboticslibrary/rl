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

#include <fstream>
#include <iostream>
#include <vector>
#include <rl/math/Array.h>
#include <rl/math/Polynomial.h>
#include <rl/math/Spline.h>

#define FILE

template<typename T>
void eval(const T& f)
{
	T fd = f.derivative();
	T fdd = fd.derivative();
	T fddd = fdd.derivative();
	
	std::size_t steps = 999;
	
#ifdef FILE
	// plot for [i=2:5] "interpolation.dat" using 1:i with lines
	std::ofstream stream;
	stream.open("interpolation.dat", std::fstream::trunc);
#else
	std::ostream& stream = std::cout;
#endif
	
	for (std::size_t i = 0; i < steps; ++i)
	{
		rl::math::Real t = f.duration() * i / static_cast<rl::math::Real>(steps - 1);
		stream << t << "\t" << f(t) << "\t" << fd(t) << "\t" << fdd(t) << "\t" << fddd(t) << std::endl;
	}
	
#ifdef FILE
	stream.close();
#endif
}

int
main(int argc, char** argv)
{
	{
		rl::math::Polynomial<rl::math::Real> f0 = rl::math::Polynomial<rl::math::Real>::Linear(0, 1, 2);
		rl::math::Polynomial<rl::math::Real> f1 = rl::math::Polynomial<rl::math::Real>::CubicFirst(1, 0, 0, 0, 2);
		rl::math::Polynomial<rl::math::Real> f2 = rl::math::Polynomial<rl::math::Real>::QuinticFirstSecond(0, 1, 0, 0, 0, 0, 2);
		eval(f0);
		eval(f1);
		eval(f2);
		rl::math::Spline<rl::math::Real> f3;
		f3.push_back(f0);
		f3.push_back(f1);
		f3.push_back(f2);
		eval(f3);
	}
	
	{
		rl::math::Polynomial<rl::math::Vector> f0 = rl::math::Polynomial<rl::math::Vector>::Linear(
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 1),
			2
		);
		rl::math::Polynomial<rl::math::Vector> f1 = rl::math::Polynomial<rl::math::Vector>::CubicFirst(
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 0),
			2
		);
		rl::math::Polynomial<rl::math::Vector> f2 = rl::math::Polynomial<rl::math::Vector>::QuinticFirstSecond(
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 0),
			2
		);
		eval(f0);
		eval(f1);
		eval(f2);
	}
	
	{
		rl::math::Polynomial<rl::math::ArrayX> f0 = rl::math::Polynomial<rl::math::ArrayX>::Linear(
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 1),
			2
		);
		rl::math::Polynomial<rl::math::ArrayX> f1 = rl::math::Polynomial<rl::math::ArrayX>::CubicFirst(
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 0),
			2
		);
		rl::math::Polynomial<rl::math::ArrayX> f2 = rl::math::Polynomial<rl::math::ArrayX>::QuinticFirstSecond(
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 0),
			2
		);
		eval(f0);
		eval(f1);
		eval(f2);
	}
	
	{
		rl::math::Polynomial<rl::math::Real> f0 = rl::math::Polynomial<rl::math::Real>::CubicFirst(0, 1, 0, 0, 1);
		rl::math::Polynomial<rl::math::Real> f1 = rl::math::Polynomial<rl::math::Real>::CubicFirst(1, -1, 0, 0, 2);
		rl::math::Polynomial<rl::math::Real> f2 = rl::math::Polynomial<rl::math::Real>::CubicFirst(-1, 0, 0, 0, 1.5);
		rl::math::Spline<rl::math::Real> f;
		f.push_back(f0);
		f.push_back(f1);
		f.push_back(f2);
		eval(f);
	}
	
	{
		rl::math::Polynomial<rl::math::Vector> f0 = rl::math::Polynomial<rl::math::Vector>::Linear(
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 1),
			2
		);
		rl::math::Polynomial<rl::math::Vector> f1 = rl::math::Polynomial<rl::math::Vector>::CubicFirst(
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 0),
			2
		);
		rl::math::Polynomial<rl::math::Vector> f2 = rl::math::Polynomial<rl::math::Vector>::QuinticFirstSecond(
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 0),
			2
		);
		rl::math::Spline<rl::math::Vector> f;
		f.push_back(f0);
		f.push_back(f1);
		f.push_back(f2);
		eval(f);
	}
	
	{
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
		rl::math::Spline<rl::math::Real> f2 = rl::math::Spline<rl::math::Real>::LinearParabolic(x, y, 0.25f);
		eval(f2);
		rl::math::Spline<rl::math::Real> f3 = rl::math::Spline<rl::math::Real>::LinearQuartic(x, y, 0.25f);
		eval(f3);
		rl::math::Spline<rl::math::Real> f4 = rl::math::Spline<rl::math::Real>::LinearSextic(x, y, 0.25f);
		eval(f4);
	}
	
	{
		::std::vector<rl::math::Real> x;
		x.push_back(0);
		x.push_back(1);
		x.push_back(3);
		x.push_back(3.5);
		x.push_back(6);
		x.push_back(7);
		::std::vector<rl::math::ArrayX> y;
		y.push_back(rl::math::ArrayX::Constant(1, 0));
		y.push_back(rl::math::ArrayX::Constant(1, 1));
		y.push_back(rl::math::ArrayX::Constant(1, -1));
		y.push_back(rl::math::ArrayX::Constant(1, 3));
		y.push_back(rl::math::ArrayX::Constant(1, 3));
		y.push_back(rl::math::ArrayX::Constant(1, -2));
		rl::math::Spline<rl::math::ArrayX> f3 = rl::math::Spline<rl::math::ArrayX>::LinearQuartic(x, y, 0.25f);
		eval(f3);
		rl::math::Spline<rl::math::ArrayX> f4 = rl::math::Spline<rl::math::ArrayX>::LinearSextic(x, y, 0.25f);
		eval(f4);
	}
	
	{
		rl::math::Spline<rl::math::Vector> f0 = rl::math::Spline<rl::math::Vector>::QuarticLinearQuarticAtRest(
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 1)
		);
		eval(f0);
		rl::math::Spline<rl::math::ArrayX> f1 = rl::math::Spline<rl::math::ArrayX>::QuarticLinearQuarticAtRest(
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 1)
		);
		eval(f1);
	}
	
	{
		rl::math::Spline<rl::math::Vector> f0 = rl::math::Spline<rl::math::Vector>::SexticLinearSexticAtRest(
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 1)
		);
		eval(f0);
		rl::math::Spline<rl::math::ArrayX> f1 = rl::math::Spline<rl::math::ArrayX>::SexticLinearSexticAtRest(
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 1)
		);
		eval(f1);
	}
	
	{
		rl::math::Spline<rl::math::Vector> f0 = rl::math::Spline<rl::math::Vector>::TrapeziodalAccelerationAtRest(
			rl::math::Vector::Constant(1, 0),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 1),
			rl::math::Vector::Constant(1, 1)
		);
		eval(f0);
		rl::math::Spline<rl::math::ArrayX> f1 = rl::math::Spline<rl::math::ArrayX>::TrapeziodalAccelerationAtRest(
			rl::math::ArrayX::Constant(1, 0),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 1),
			rl::math::ArrayX::Constant(1, 1)
		);
		eval(f1);
	}
	
	return EXIT_SUCCESS;
}
