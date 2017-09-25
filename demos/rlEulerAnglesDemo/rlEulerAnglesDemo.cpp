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
#include <boost/lexical_cast.hpp>
#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	if (argc < 7)
	{
		std::cout << "Usage: rlEulerAnglesDemo AXIS0 AXIS1 AXIS2 DEG0 DEG1 DEG2" << std::endl;
		return EXIT_FAILURE;
	}
	
	rl::math::Matrix33 rotation = rl::math::Matrix33::Identity();
	
	for (std::size_t i = 0; i < 3; ++i)
	{
		rl::math::Real angle = boost::lexical_cast<rl::math::Real>(argv[i + 4]) * rl::math::DEG2RAD;
		
		rl::math::Vector3 axis(
			0 == boost::lexical_cast<int>(argv[i + 1]) ? 1 : 0,
			1 == boost::lexical_cast<int>(argv[i + 1]) ? 1 : 0,
			2 == boost::lexical_cast<int>(argv[i + 1]) ? 1 : 0
		);
		std::cout << "angle" << i << ": " << angle << " rad - axis" << i << ": " << axis.transpose() << std::endl;
		
		rotation = rotation * rl::math::AngleAxis(angle, axis);
	}
	
	std::cout << std::endl;
	
	rl::math::Quaternion quaternion(rotation);
	std::cout << "quaternion.w: " << quaternion.w() << " - quaternion.vec: " << quaternion.vec().transpose() << std::endl;
	
	rl::math::AngleAxis angleAxis(rotation);
	std::cout << "angle: " << angleAxis.angle() << " rad - axis: " << angleAxis.axis().transpose() << std::endl;
	
	rl::math::Vector3 orientation = rotation.eulerAngles(2, 1, 0).reverse();
	std::cout << "x: " << orientation.x() * rl::math::RAD2DEG << " deg - y: " << orientation.y() * rl::math::RAD2DEG << " deg - z: " << orientation.z() * rl::math::RAD2DEG << " deg" << std::endl;
	
	return EXIT_SUCCESS;
}
