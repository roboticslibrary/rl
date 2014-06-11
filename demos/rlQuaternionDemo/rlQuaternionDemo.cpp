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
#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	rl::math::Matrix33 r1(
		::rl::math::AngleAxis(120.0f * rl::math::DEG2RAD, ::rl::math::Vector3::UnitZ()) *
		::rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, ::rl::math::Vector3::UnitY()) *
		::rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, ::rl::math::Vector3::UnitX())
	);
	std::cout << "r1 = " << std::endl << r1 << std::endl;
	
	rl::math::Quaternion q1(r1);
	std::cout << "q1 = " << q1.coeffs().transpose() << std::endl;
	
	r1 = q1.toRotationMatrix();
	std::cout << "r1 = " << std::endl << r1 << std::endl;
	
	rl::math::Matrix33 r2(
		::rl::math::AngleAxis(-120.0f * rl::math::DEG2RAD, ::rl::math::Vector3::UnitZ()) *
		::rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, ::rl::math::Vector3::UnitY()) *
		::rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, ::rl::math::Vector3::UnitX())
	);
	std::cout << "r2 = " << std::endl << r2 << std::endl;
	
	rl::math::Quaternion q2(r2);
	std::cout << "q2 = " << q2.coeffs().transpose() << std::endl;
	
	rl::math::Quaternion q = q1.slerp(0.5f, q2);
	std::cout << "q = " << q.coeffs().transpose() << std::endl;
	
	rl::math::Matrix33 r(q);
	std::cout << "r = " << std::endl << r << std::endl;
	
	std::cout << q1.angularDistance(q2) << std::endl;
	std::cout << q1.angularDistance(q) << std::endl;
	std::cout << q.angularDistance(q2) << std::endl;
	
//	rl::math::Transform t1;
//	boost::numeric::bindings::ippm::loadIdentity(t1);
//	rl::math::rotation::fromXyz(0.0f * rl::math::DEG2RAD, 0.0f * rl::math::DEG2RAD, 120.0f * rl::math::DEG2RAD, t1);
//	std::cout << t1 << std::endl;
//	
//	rl::math::Transform t2;
//	boost::numeric::bindings::ippm::loadIdentity(t2);
//	rl::math::rotation::fromXyz(0.0f * rl::math::DEG2RAD, 0.0f * rl::math::DEG2RAD, -120.0f * rl::math::DEG2RAD, t2);
//	std::cout << t2 << std::endl;
//	
//	rl::math::Transform t3;
//	rl::math::transform::slerp(t1, t2, 0.5f, t3);
//	std::cout << t3 << std::endl;
	
	return 0;
}
