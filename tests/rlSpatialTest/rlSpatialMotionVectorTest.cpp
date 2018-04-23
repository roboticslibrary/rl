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
#include <rl/math/Spatial.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	rl::math::Vector6 v0;
	v0.setRandom();
	
	rl::math::MotionVector mv0;
	mv0.angular() << v0.head<3>();
	mv0.linear() << v0.tail<3>();
	
	if (!mv0.matrix().isApprox(v0))
	{
		std::cerr << "mv0.matrix() != v0" << std::endl;
		std::cerr << "mv0 = " << mv0.matrix().transpose() << std::endl;
		std::cerr << "v0 = " << v0.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v1;
	v1.setRandom();
	
	rl::math::MotionVector mv1(v1);
	
	if (!mv1.matrix().isApprox(v1))
	{
		std::cerr << "mv1(v1).matrix() != v1" << std::endl;
		std::cerr << "mv1(v1) = " << mv1.matrix().transpose() << std::endl;
		std::cerr << "v1 = " << v1.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v2;
	v2.setRandom();
	
	rl::math::MotionVector mv2 = v2;
	
	if (!mv2.matrix().isApprox(v2))
	{
		std::cerr << "(mv2 = v2).matrix() != v2" << std::endl;
		std::cerr << "(mv2 = v2) = " << mv2.matrix().transpose() << std::endl;
		std::cerr << "v2 = " << v2.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v3 = v1 + v2;
	rl::math::MotionVector mv3 = mv1 + mv2;
	
	if (!mv3.matrix().isApprox(v3))
	{
		std::cerr << "mv1 + mv2 != v1 + v2" << std::endl;
		std::cerr << "mv1 + mv2 = " << mv3.matrix().transpose() << std::endl;
		std::cerr << "v1 + v2 = " << v3.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v4 = v1 - v2;
	rl::math::MotionVector mv4 = mv1 - mv2;
	
	if (!mv4.matrix().isApprox(v4))
	{
		std::cerr << "mv1 - mv2 != v1 - v2" << std::endl;
		std::cerr << "mv1 - mv2 = " << mv4.matrix().transpose() << std::endl;
		std::cerr << "v1 - v2 = " << v4.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v5 = v1 * 1.23f;
	rl::math::MotionVector mv5 = mv1 * 1.23f;
	
	if (!mv5.matrix().isApprox(v5))
	{
		std::cerr << "mv1 * 1.23 != v1 * 1.23" << std::endl;
		std::cerr << "mv1 * 1.23 = " << mv5.matrix().transpose() << std::endl;
		std::cerr << "v1 * 1.23 = " << v5.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v6 = v1 / 1.23f;
	rl::math::MotionVector mv6 = mv1 / 1.23f;
	
	if (!mv6.matrix().isApprox(v6))
	{
		std::cerr << "mv1 / 1.23 != v1 / 1.23" << std::endl;
		std::cerr << "v1 * 1.23 = " << mv6.matrix().transpose() << std::endl;
		std::cerr << "v1 / 1.23 = " << v6.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::MotionVector mv7 = v1;
	
	if (!mv7.matrix().isApprox(v1))
	{
		std::cerr << "mv7 != v1" << std::endl;
		std::cerr << "mv7 = " << mv7.matrix().transpose() << std::endl;
		std::cerr << "v1 = " << v1.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	return EXIT_SUCCESS;
}
