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
#include <vector>
#include <rl/math/Kalman.h>
#include <rl/math/Matrix.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	std::size_t ss = 4;
	std::size_t os = 2;
	
	rl::math::Kalman<rl::math::Real> kalman(ss, os);
	
	kalman.stateTransitionModel().setIdentity();
	kalman.stateTransitionModel().topRightCorner(os, os).setIdentity();
	
	kalman.measurementModel().setIdentity();
	kalman.processNoiseCovariance() = 0.1 * rl::math::Matrix::Identity(ss, ss);
	kalman.measurementNoiseCovariance().setIdentity();
	
	rl::math::Vector initx(ss);
	initx << 10, 10, 1, 0;
	
	rl::math::Matrix initV = 10 * rl::math::Matrix::Identity(ss, ss);
	
	std::vector<rl::math::Vector> y(15, rl::math::Vector(os));
	y[0] << 11.4975, 10.3752;
	y[1] << 8.3492, 11.7446;
	y[2] << 12.2346, 10.1007;
	y[3] << 13.0197, 9.0750;
	y[4] << 12.9370, 9.4789;
	y[5] << 14.9108, 9.6574;
	y[6] << 17.0387, 9.1131;
	y[7] << 14.6307, 7.5850;
	y[8] << 18.1220, 6.4678;
	y[9] << 16.8157, 5.0222;
	y[10] << 18.0819, 5.7949;
	y[11] << 17.9199, 4.4554;
	y[12] << 20.4956, 6.4279;
	y[13] << 18.9063, 6.4588;
	y[14] << 19.4037, 2.7445;
	
	for (std::size_t i = 0; i < y.size(); ++i)
	{
		if (i > 0)
		{
			kalman.predict();
		}
		else
		{
			kalman.statePriori() = initx;
			kalman.errorCovariancePriori() = initV;
		}
		
		kalman.correct(y[i]);
		
		std::cout << "xfilt[" << i << "] = " << kalman.statePosteriori().transpose() << std::endl;
		std::cout << "Vfilt[" << i << "] = " << std::endl << kalman.errorCovariancePosteriori().transpose() << std::endl;
	}
	
	return EXIT_SUCCESS;
}
