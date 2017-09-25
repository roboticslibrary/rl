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
#include <rl/math/Matrix.h>
#include <rl/math/Rotation.h>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	rl::math::Real a = 31.0f;
	rl::math::Real alpha = 271.0f * rl::math::DEG2RAD;
	rl::math::Real d = 101.0f;
	rl::math::Real theta = 181.0f * rl::math::DEG2RAD;
	
	rl::math::Transform t_d(rl::math::Translation(0, 0, d));
	
	std::cout << "T_d = " << std::endl << t_d.matrix() << std::endl;
	
	rl::math::Transform t_theta(
		rl::math::AngleAxis(theta, rl::math::Vector3::UnitZ()) *
		rl::math::AngleAxis(0, rl::math::Vector3::UnitY()) *
		rl::math::AngleAxis(0, rl::math::Vector3::UnitX())
	);
	
	std::cout << "T_theta = " << std::endl << t_theta.matrix() << std::endl;
	
	rl::math::Transform t_a(rl::math::Translation(a, 0, 0));
	
	std::cout << "T_a = " << std::endl << t_a.matrix() << std::endl;
	
	rl::math::Transform t_alpha(
		rl::math::AngleAxis(0, rl::math::Vector3::UnitZ()) *
		rl::math::AngleAxis(0, rl::math::Vector3::UnitY()) *
		rl::math::AngleAxis(alpha, rl::math::Vector3::UnitX())
	);
	
	std::cout << "T_alpha = " << std::endl << t_alpha.matrix() << std::endl;
	
	rl::math::Transform t = t_d * t_theta * t_a * t_alpha;
	
	std::cout << "A = " << std::endl << t.matrix() << std::endl;
	
	rl::math::Transform dh;
	dh.fromDenavitHartenbergPaul(d, theta, a, alpha);
	
	std::cout << "A = " << std::endl << dh.matrix() << std::endl;
	
	dh.toDenavitHartenbergPaul(d, theta, a, alpha);
	
	std::cout << "a = " << a << std::endl;
	std::cout << "alpha = " << alpha * rl::math::RAD2DEG << std::endl;
	std::cout << "d = " << d << std::endl;
	std::cout << "theta = " << theta * rl::math::RAD2DEG << std::endl;
	
	return EXIT_SUCCESS;
}
