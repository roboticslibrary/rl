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
#include <rl/math/Constants.h>
#include <rl/math/Vector.h>

void
test(const rl::math::Vector2& cartesian, std::size_t& success, std::size_t& failure)
{
	rl::math::Vector2 polar = rl::math::Vector2::PolarFromCartesian(cartesian);
	rl::math::Vector2 cartesian2 = rl::math::Vector2::CartesianFromPolar(polar);
	
	if (cartesian.isApprox(cartesian2))
	{
		++success;
	}
	else
	{
		std::cout << "Cartesian:          x = " << cartesian.x() << ", y = " << cartesian.y() << std::endl;
		std::cout << "Cartesian -> Polar: r = " << polar(0) << ", theta = " <<  polar(1) << " rad = " << polar(1) * rl::math::constants::rad2deg << " deg" << std::endl;
		std::cout << "Polar -> Cartesian: x = " << cartesian2.x() << ", y = " << cartesian2.y() << std::endl;
		std::cout << "--------------------------------------------------------------------------------" << std::endl;
		++failure;
	}
}

int
main(int argc, char** argv)
{
	std::size_t success = 0;
	std::size_t failure = 0;
	
	test(rl::math::Vector2::Zero(), success, failure);
	test(rl::math::Vector2::UnitX(), success, failure);
	test(rl::math::Vector2::UnitY(), success, failure);
	test(-rl::math::Vector2::UnitX(), success, failure);
	test(-rl::math::Vector2::UnitY(), success, failure);
	
	for (std::size_t i = 0; i < 100000; ++i)
	{
		test(rl::math::Vector2::Random(), success, failure);
	}
	
	std::cout << "success: " << success << " failure: " << failure << std::endl;
	
	if (failure > 0)
	{
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
