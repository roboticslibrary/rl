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
	rl::math::ArticulatedBodyInertia abi1;
	abi1.cog().setRandom();
	abi1.inertia().setRandom();
	abi1.mass() = rl::math::DiagonalMatrix33(rl::math::Vector3::Random());
	
	rl::math::ArticulatedBodyInertia abi2;
	abi2.cog().setRandom();
	abi2.inertia().setRandom();
	abi2.mass() = rl::math::DiagonalMatrix33(rl::math::Vector3::Random());
	
	rl::math::ArticulatedBodyInertia abi3 = abi1 + abi2;
	rl::math::ArticulatedBodyInertia abi4 = abi3 - abi2;
	
	if (!abi4.matrix().isApprox(abi1.matrix()))
	{
		std::cerr << "abi1 + abi2 - abi2 != abi1" << std::endl;
		std::cerr << "abi1 + abi2 - abi2 = " << std::endl << abi4.matrix() << std::endl;
		std::cerr << "abi1 = " << std::endl << abi1.matrix() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::ArticulatedBodyInertia abi1_2 = abi1 * 2;
	rl::math::ArticulatedBodyInertia abi1_abi1 = abi1 + abi1;
	rl::math::ArticulatedBodyInertia abi1_2_05 = abi1_2 * 0.5;
	
	if (!abi1_2.matrix().isApprox(abi1_abi1.matrix()))
	{
		std::cerr << "abi1 * 2 != abi1 + abi1" << std::endl;
		std::cerr << "abi1 * 2 = " << std::endl << abi1_2.matrix() << std::endl;
		std::cerr << "abi1 + abi1 = " << std::endl << abi1_abi1.matrix() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	if (!abi1_2_05.matrix().isApprox(abi1.matrix()))
	{
		std::cerr << "abi1 * 2 * 0.5 != abi1" << std::endl;
		std::cerr << "abi1 * 2 * 0.5 = " << std::endl << abi1_2.matrix() << std::endl;
		std::cerr << "abi1 = " << std::endl << abi1.matrix() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::PlueckerTransform pt1;
	pt1.rotation() = rl::math::Matrix33(rl::math::Quaternion::Random());
	pt1.translation().setRandom();
	
	rl::math::ArticulatedBodyInertia abi5 = pt1 * abi1;
	rl::math::ArticulatedBodyInertia abi6 = pt1 / abi5;
	
	if (!abi6.matrix().isApprox(abi1.matrix()))
	{
		std::cerr << "inv(pt1) * pt1 * abi1 != abi1" << std::endl;
		std::cerr << "inv(pt1) * pt1 * abi1 = " << std::endl << abi6.matrix() << std::endl;
		std::cerr << "abi1 = " << std::endl << abi1.matrix() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 m1 = pt1.matrixForce() * abi1.matrix() * pt1.inverseMotion();
	
	if (!abi5.matrix().isApprox(m1))
	{
		std::cerr << "pt1 * abi1 != matrixForce(pt1) * abi1 * inverseMotion(pt1)" << std::endl;
		std::cerr << "pt1 * abi1 = " << std::endl << abi5.matrix() << std::endl;
		std::cerr << "matrixForce(pt1) * abi1 * inverseMotion(pt1) = " << std::endl << m1 << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 m2 = pt1.matrixMotion().transpose() * m1 * pt1.matrixMotion();
	
	if (!abi1.matrix().isApprox(m2))
	{
		std::cerr << "abi1 != matrixMotion(pt1)^T * abi5 * matrixMotion(pt1)" << std::endl;
		std::cerr << "abi1 = " << std::endl << abi1.matrix() << std::endl;
		std::cerr << "matrixMotion(pt1)^T * abi5 * matrixMotion(pt1) = " << std::endl << m2 << std::endl;
		exit(EXIT_FAILURE);
	}
	
	return EXIT_SUCCESS;
}
