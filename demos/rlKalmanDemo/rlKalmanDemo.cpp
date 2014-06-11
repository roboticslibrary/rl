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
#include <string>
#include <vector>
#include <rl/math/Kalman.h>

int
main(int argc, char** argv)
{
	std::size_t t = 20;
	
	std::size_t ss = 4;
	std::size_t os = 2;
	
	rl::math::Matrix a(ss, ss);
	a.setIdentity();
//	rl::math::Matrix b(ss, ss);
//	b.setZero();
	std::vector< rl::math::Matrix > p(t + 1, rl::math::Matrix(ss, ss));
	p[0].setIdentity();
	rl::math::Matrix q(ss, ss);
	q.setIdentity();
//	rl::math::Vector u(ss);
//	u.setZero();
	
	rl::math::Matrix h(os, ss);
	h.setIdentity();
	rl::math::Matrix r(os, os);
	r.setIdentity();
	
	std::vector< rl::math::Vector > x(t + 1, rl::math::Vector(ss));
	x[0].setZero();
	std::vector< rl::math::Vector > z(t, rl::math::Vector(os));
	z[0].setZero();
	
	a(0, 2) = 1;
	a(1, 3) = 1;
	p[0] *= 1.0e-12f;
	q *= 0.01f;
	r *= 0.01f;
	
	for (std::size_t i = 0; i < t; ++i)
	{
		x[i](0) = static_cast< rl::math::Real >(i) + 1;
		x[i](1) = static_cast< rl::math::Real >(i) + 1;
		
		if (i > 0)
		{
			x[i](2) = x[i](0) - x[i - 1](0);
			x[i](3) = x[i](1) - x[i - 1](1);
		}
		else
		{
			x[i](2) = 0;
			x[i](3) = 0;
		}
		
		z[i](0) = x[i](0);
		z[i](1) = x[i](1);
	}
	
	for (std::size_t i = 0; i < t; ++i) std::cout << "x[" << i << "] = " << x[i].transpose() << std::endl;
	for (std::size_t i = 0; i < t; ++i) std::cout << "z[" << i << "] = " << z[i].transpose() << std::endl;
	
	for (std::size_t i = 0; i < t; ++i)
	{
		if (i > 0)
		{
			rl::math::Kalman::predict(x[i - 1], p[i - 1], a, q, x[i], p[i]);
		}
		
		rl::math::Kalman::correct(x[i], p[i], h, r, z[i], x[i], p[i]);
	}
	
	rl::math::Kalman::predict(x[t - 1], p[t - 1], a, q, x[t], p[t]);
	
	for (std::size_t i = 0; i < t + 1; ++i) std::cout << "x[" << i << "] = " << x[i].transpose() << std::endl;
	for (std::size_t i = 0; i < t + 1; ++i) std::cout << "p[" << i << "] = " << std::endl << p[i] << std::endl;
	
	return 0;
}
