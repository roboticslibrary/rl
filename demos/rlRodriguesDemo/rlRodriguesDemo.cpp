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

#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	rl::math::Rotation r;
	rl::math::rotation::fromXyz(0.0f * rl::math::DEG2RAD, 0.0f * rl::math::DEG2RAD, 90.0f * rl::math::DEG2RAD, r);
	std::cout << r << std::endl;
	
	rl::math::Vector3 rodrigues;
	rl::math::rotation::toRodrigues(r, rodrigues);
	std::cout << rodrigues(0) * rl::math::RAD2DEG << " " << rodrigues(1) * rl::math::RAD2DEG << " " << rodrigues(2) * rl::math::RAD2DEG << std::endl;
	
	rl::math::rotation::fromRodrigues(rodrigues(0), rodrigues(1), rodrigues(2), r);
	std::cout << r << std::endl;
	
	rl::math::Rotation t0;
	boost::numeric::bindings::ippm::loadIdentity(t0);
	rl::math::Rotation t1;
	boost::numeric::bindings::ippm::loadIdentity(t1);
	rl::math::rotation::fromXyz(10.0f * rl::math::DEG2RAD, 20.0f * rl::math::DEG2RAD, 30.0f * rl::math::DEG2RAD, t1);
	
	rl::math::Vector3 e;
	
	e(0) = t0(1, 0) * t1(2, 0) / 0.2e1 - t0(2, 0) * t1(1, 0) / 0.2e1 + t0(1, 1) * t1(2, 1) / 0.2e1 - t0(2, 1) * t1(1, 1) / 0.2e1 + t0(1, 2) * t1(2, 2) / 0.2e1 - t0(2, 2) * t1(1, 2) / 0.2e1;
	e(1) = t0(2, 0) * t1(0, 0) / 0.2e1 - t0(0, 0) * t1(2, 0) / 0.2e1 + t0(2, 1) * t1(0, 1) / 0.2e1 - t0(0, 1) * t1(2, 1) / 0.2e1 + t0(2, 2) * t1(0, 2) / 0.2e1 - t0(0, 2) * t1(2, 2) / 0.2e1;
	e(2) = t0(0, 0) * t1(1, 0) / 0.2e1 - t0(1, 0) * t1(0, 0) / 0.2e1 + t0(0, 1) * t1(1, 1) / 0.2e1 - t0(1, 1) * t1(0, 1) / 0.2e1 + t0(0, 2) * t1(1, 2) / 0.2e1 - t0(1, 2) * t1(0, 2) / 0.2e1;
	
	std::cout << e(0) * rl::math::RAD2DEG << " " << e(1) * rl::math::RAD2DEG << " " << e(2) * rl::math::RAD2DEG << std::endl;
	
	return 0;
}
