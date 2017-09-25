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
#include <stdexcept>
#include <rl/math/Rotation.h>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	for(std::size_t i = 0; i < 5; ++i)
	{
		rl::math::Real translation = 0.1;
		rl::math::Real rotation;
		
		switch (i)
		{
		case 0:
			rotation = 0;
			break;
		case 1:
			rotation = 0.1;
			break;
		case 2:
			rotation = M_PI / 2;
			break;
		case 3:
			rotation = M_PI;
			break;
		case 4:
			rotation = 2 * M_PI;
			break;
		default:
			rotation = -M_PI / 2;
			break;
		}
		
		rl::math::Transform T1 = rl::math::Transform::Identity();
		rl::math::Transform T2 = rl::math::Transform::Identity();
		T1.translation().x() = translation;
		T1.linear() = rl::math::AngleAxis(rotation, rl::math::Vector3::UnitY()).toRotationMatrix();
		
		rl::math::Vector6 delta = T1.toDelta(T2);
		
		std::cout << "T1: " << std::endl << T1.matrix() << std::endl;
		std::cout << "T2: " << std::endl << T2.matrix() << std::endl;
		std::cout << "delta: " << delta.transpose() << std::endl;
		
		rl::math::Transform my_T2;
		my_T2.fromDelta(T1, delta);
		
		if ((T2.matrix() - my_T2.matrix()).norm() > 1e-8)
		{
			std::cerr << "toDelta and fromDelta are inconsistent." << std::endl;
			std::cerr << T1.matrix() << std::endl;
			std::cerr << T2.matrix() << std::endl;
			std::cerr << my_T2.matrix() << std::endl;
			return EXIT_FAILURE;
		}
		
		if (delta.topRows(3).norm() != translation)
		{
			std::cerr << "toDelta gives wrong translation." << std::endl;
			return EXIT_FAILURE;
		}
		
		if (::std::abs(rotation) < 1 && ::std::abs(delta.bottomRows(3).norm() - rotation) > 1e-8)
		{
			std::cerr << "toDelta gives Wrong rotation." << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	return EXIT_SUCCESS;
}
