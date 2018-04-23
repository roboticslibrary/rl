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

int
main(int argc, char** argv)
{
	rl::math::MotionVector mv1;
	mv1.angular().setRandom();
	mv1.linear().setRandom();
	
	rl::math::ForceVector fv2;
	fv2.moment().setRandom();
	fv2.force().setRandom();
	
	rl::math::Real dot1 = mv1.angular().dot(fv2.moment()) + mv1.linear().dot(fv2.force());
	rl::math::Real dot2 = mv1.dot(fv2);
	
	if (!Eigen::internal::isApprox(dot2, dot1))
	{
		std::cerr << "mv1 * fv2 != mv1.angular * fv2.moment + mv1.linear * fv2.force" << std::endl;
		std::cerr << "mv1 * fv2 = " << dot2 << std::endl;
		std::cerr << "mv1.angular * fv2.moment + mv1.linear * fv2.force = " << dot1 << std::endl;
		exit(EXIT_FAILURE);
	}
	
	return EXIT_SUCCESS;
}
