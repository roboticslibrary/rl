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
	
	rl::math::ForceVector fv0;
	fv0.moment() << v0.head<3>();
	fv0.force() << v0.tail<3>();
	
	if (!fv0.matrix().isApprox(v0))
	{
		std::cerr << "fv0.matrix() != v0" << std::endl;
		std::cerr << "fv0 = " << fv0.matrix().transpose() << std::endl;
		std::cerr << "v0 = " << v0.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v1;
	v1.setRandom();
	
	rl::math::ForceVector fv1(v1);
	
	if (!fv1.matrix().isApprox(v1))
	{
		std::cerr << "fv1(v1).matrix() != v1" << std::endl;
		std::cerr << "fv1(v1) = " << fv1.matrix().transpose() << std::endl;
		std::cerr << "v1 = " << v1.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v2;
	v2.setRandom();
	
	rl::math::ForceVector fv2 = v2;
	
	if (!fv2.matrix().isApprox(v2))
	{
		std::cerr << "(fv2 = v2).matrix() != v2" << std::endl;
		std::cerr << "(fv2 = v2) = " << fv2.matrix().transpose() << std::endl;
		std::cerr << "v2 = " << v2.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v3 = v1 + v2;
	rl::math::ForceVector fv3 = fv1 + fv2;
	
	if (!fv3.matrix().isApprox(v3))
	{
		std::cerr << "fv1 + fv2 != v1 + v2" << std::endl;
		std::cerr << "fv1 + fv2 = " << fv3.matrix().transpose() << std::endl;
		std::cerr << "v1 + v2 = " << v3.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v4 = v1 - v2;
	rl::math::ForceVector fv4 = fv1 - fv2;
	
	if (!fv4.matrix().isApprox(v4))
	{
		std::cerr << "fv1 - fv2 != v1 - v2" << std::endl;
		std::cerr << "fv1 - fv2 = " << fv4.matrix().transpose() << std::endl;
		std::cerr << "v1 - v2 = " << v4.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v5 = v1 * 1.23f;
	rl::math::ForceVector fv5 = fv1 * 1.23f;
	
	if (!fv5.matrix().isApprox(v5))
	{
		std::cerr << "fv1 * 1.23 != v1 * 1.23" << std::endl;
		std::cerr << "fv1 * 1.23 = " << fv5.matrix().transpose() << std::endl;
		std::cerr << "v1 * 1.23 = " << v5.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v6 = v1 / 1.23f;
	rl::math::ForceVector fv6 = fv1 / 1.23f;
	
	if (!fv6.matrix().isApprox(v6))
	{
		std::cerr << "fv1 / 1.23 != v1 / 1.23" << std::endl;
		std::cerr << "v1 * 1.23 = " << fv6.matrix().transpose() << std::endl;
		std::cerr << "v1 / 1.23 = " << v6.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::ForceVector fv7 = v1;
	
	if (!fv7.matrix().isApprox(v1))
	{
		std::cerr << "fv7 != v1" << std::endl;
		std::cerr << "fv7 = " << fv7.matrix().transpose() << std::endl;
		std::cerr << "v1 = " << v1.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	return EXIT_SUCCESS;
}
