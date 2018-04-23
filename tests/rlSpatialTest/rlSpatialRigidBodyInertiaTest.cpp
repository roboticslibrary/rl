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
#include <rl/math/Spatial.h>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	rl::math::RigidBodyInertia rbi1;
	rbi1.cog().setRandom();
	rbi1.inertia().setRandom();
	rbi1.mass() = 10;
	
	rl::math::RigidBodyInertia rbi2;
	rbi2.cog().setRandom();
	rbi2.inertia().setRandom();
	rbi2.mass() = 20;
	
	rl::math::RigidBodyInertia rbi3 = rbi1 + rbi2;
	rl::math::RigidBodyInertia rbi4 = rbi3 - rbi2;
	
	if (!rbi4.matrix().isApprox(rbi1.matrix()))
	{
		std::cerr << "rbi1 + rbi2 - rbi2 != rbi1" << std::endl;
		std::cerr << "rbi1 + rbi2 - rbi2 = " << std::endl << rbi4.matrix() << std::endl;
		std::cerr << "rbi1 = " << std::endl << rbi1.matrix() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::RigidBodyInertia rbi1_2 = rbi1 * 2;
	rl::math::RigidBodyInertia rbi1_rbi1 = rbi1 + rbi1;
	rl::math::RigidBodyInertia rbi1_2_05 = rbi1_2 * 0.5;
	
	if (!rbi1_2.matrix().isApprox(rbi1_rbi1.matrix()))
	{
		std::cerr << "rbi1 * 2 != rbi1 + rbi1" << std::endl;
		std::cerr << "rbi1 * 2 = " << std::endl << rbi1_2.matrix() << std::endl;
		std::cerr << "rbi1 + rbi1 = " << std::endl << rbi1_rbi1.matrix() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	if (!rbi1_2_05.matrix().isApprox(rbi1.matrix()))
	{
		std::cerr << "rbi1 * 2 * 0.5 != rbi1" << std::endl;
		std::cerr << "rbi1 * 2 * 0.5 = " << std::endl << rbi1_2_05.matrix() << std::endl;
		std::cerr << "rbi1 = " << std::endl << rbi1.matrix() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::PlueckerTransform pt1;
	pt1.rotation() = rl::math::Matrix33(rl::math::Quaternion::Random());
	pt1.translation().setRandom();
	
	rl::math::RigidBodyInertia rbi5 = pt1 * rbi1;
	rl::math::RigidBodyInertia rbi6 = pt1 / rbi5;
	
	if (!rbi6.matrix().isApprox(rbi1.matrix()))
	{
		std::cerr << "inv(pt1) * pt1 * rbi1 != rbi1" << std::endl;
		std::cerr << "inv(pt1) * pt1 * rbi1 = " << std::endl << rbi6.matrix() << std::endl;
		std::cerr << "rbi1 = " << std::endl << rbi1.matrix() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 m1 = pt1.matrixForce() * rbi1.matrix() * pt1.inverseMotion();
	
	if (!rbi5.matrix().isApprox(m1))
	{
		std::cerr << "pt1 * rbi1 != matrixForce(pt1) * rbi1 * inverseMotion(pt1)" << std::endl;
		std::cerr << "pt1 * rbi1 = " << std::endl << rbi5.matrix() << std::endl;
		std::cerr << "matrixForce(pt1) * rbi1 * inverseMotion(pt1) = " << std::endl << m1 << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 m2 = pt1.matrixMotion().transpose() * m1 * pt1.matrixMotion();
	
	if (!rbi1.matrix().isApprox(m2))
	{
		std::cerr << "rbi1 != matrixMotion(pt1)^T * rbi5 * matrixMotion(pt1)" << std::endl;
		std::cerr << "rbi1 = " << std::endl << rbi1.matrix() << std::endl;
		std::cerr << "matrixMotion(pt1)^T * rbi5 * matrixMotion(pt1) = " << std::endl << m2 << std::endl;
		exit(EXIT_FAILURE);
	}
	
	return EXIT_SUCCESS;
}
