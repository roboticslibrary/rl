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
#include <rl/math/Cubic.h>
#include <rl/math/Quintic.h>

int
main(int argc, char** argv)
{
//	rl::math::Cubic< rl::math::Real > interpolator;
	rl::math::Quintic< rl::math::Real > interpolator;
	
	interpolator.te = 2.0f;
	
	interpolator.x0 = 0.0f;
	interpolator.v0 = 0.0f;
	interpolator.a0 = 0.0f;
	interpolator.xe = 1.0f;
	interpolator.ve = 0.0f;
	interpolator.ae = 0.0f;
	
	interpolator.interpolate();
	
	for (std::size_t i = 0; i <= 100; ++i)
	{
		std::cout << i * interpolator.te / 100.0f;
		std::cout << "\t";
		std::cout << interpolator.x(i * interpolator.te / 100.0f);
		std::cout << "\t";
		std::cout << interpolator.v(i * interpolator.te / 100.0f);
		std::cout << "\t";
		std::cout << interpolator.a(i * interpolator.te / 100.0f);
		std::cout << std::endl;
	}
	
	return 0;
}
