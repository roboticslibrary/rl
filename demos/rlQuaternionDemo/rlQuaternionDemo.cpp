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

#include <fstream>
#include <functional>
#include <iostream>
#include <random>
#include <rl/math/Matrix.h>
#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>
#include <rl/math/SplineQuaternion.h>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	{
		rl::math::Matrix33 r1(
			rl::math::AngleAxis(90.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *
			rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
		);
		std::cout << "r1 = " << std::endl << r1 << std::endl;
		
		rl::math::Quaternion q1(r1);
		std::cout << "q1 = " << q1.coeffs().transpose() << std::endl;
		
		r1 = q1.toRotationMatrix();
		std::cout << "r1 = " << std::endl << r1 << std::endl;
		
		rl::math::Matrix33 r2(
			rl::math::AngleAxis(-90.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *
			rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
		);
		std::cout << "r2 = " << std::endl << r2 << std::endl;
		
		rl::math::Quaternion q2(r2);
		std::cout << "q2 = " << q2.coeffs().transpose() << std::endl;
		
		rl::math::Quaternion q = q1.slerp(0.5f, q2);
		std::cout << "q = " << q.coeffs().transpose() << std::endl;
		
		rl::math::Matrix33 r(q);
		std::cout << "r = " << std::endl << r << std::endl;
		
		std::cout << "q1 -> q2 = " << q1.angularDistance(q2) << std::endl;
		std::cout << "q1 -> q  = " << q1.angularDistance(q) << std::endl;
		std::cout << "q  -> q2 = " << q.angularDistance(q2) << std::endl;
		
		std::cout << "log(q1) = " << q1.log().coeffs().transpose() << std::endl;
		std::cout << "exp(q1) = " << q1.exp().coeffs().transpose() << std::endl;
		std::cout << "exp(log(q1)) = " << q1.log().exp().coeffs().transpose() << std::endl;
		
		std::cout << "q1^1 = " << q1.pow(1).coeffs().transpose() << std::endl;
		std::cout << "q1^2 = " << q1.pow(2).coeffs().transpose() << std::endl;
		std::cout << "(q1^2)^0.5 = " << q1.pow(2).pow(0.5).coeffs().transpose() << std::endl;
		
		rl::math::Vector3 omega(
			45.0f * rl::math::DEG2RAD,
			90.0f * rl::math::DEG2RAD,
			135.0f * rl::math::DEG2RAD
		);
		std::cout << "omega = " << omega.transpose() * rl::math::RAD2DEG << std::endl;
		
		rl::math::Quaternion qd = q1.firstDerivative(omega);
		std::cout << "qd = " << qd.coeffs().transpose() << std::endl;
		
		omega = q1.angularVelocity(qd);
		std::cout << "omega = " << omega.transpose() * rl::math::RAD2DEG << std::endl;
		
		rl::math::Vector3 omegad(
			450.0f * rl::math::DEG2RAD,
			900.0f * rl::math::DEG2RAD,
			1350.0f * rl::math::DEG2RAD
		);
		std::cout << "omegad = " << omegad.transpose() * rl::math::RAD2DEG << std::endl;
		
		rl::math::Quaternion qdd = q1.secondDerivative(qd, omega, omegad);
		std::cout << "qdd = " << qdd.coeffs().transpose() << std::endl;
		
		omegad = q1.angularAcceleration(qd, qdd);
		std::cout << "omegad = " << omegad.transpose() * rl::math::RAD2DEG << std::endl;
	}
	
	{
		rl::math::Quaternion q0(
			rl::math::AngleAxis(90.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *
			rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
		);
		rl::math::Quaternion q1(
			rl::math::AngleAxis(45.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *
			rl::math::AngleAxis(60.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(45.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
		);
		rl::math::Quaternion q2(
			rl::math::AngleAxis(45.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *
			rl::math::AngleAxis(10.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(30.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
		);
		rl::math::Quaternion q3(
			rl::math::AngleAxis(90.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *
			rl::math::AngleAxis(10.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(0.0f * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
		);
		
		rl::math::Vector3 p(1, 0, 0);
		
		std::size_t steps = 100;
		
		// splot "slerp.dat" u 1:2:3 with lines
		std::ofstream slerp;
		slerp.open("slerp.dat", std::fstream::trunc);
		
		// plot for [i=2:4] "slerpFirstDerivative.dat" using 1:i with lines
		std::ofstream slerpFirstDerivative;
		slerpFirstDerivative.open("slerpFirstDerivative.dat", std::fstream::trunc);
		
		for (std::size_t i = 0; i < steps; ++i)
		{
			rl::math::Real t = static_cast<rl::math::Real>(i) / static_cast<rl::math::Real>(steps);
			
			rl::math::Quaternion q = q0.slerp(t, q1);
			slerp << q._transformVector(p).transpose() << std::endl;
			
			rl::math::Quaternion qd = q0.slerpFirstDerivative(t, q1);
			rl::math::Vector3 omega = q.angularVelocity(qd);
			slerpFirstDerivative << i << "\t" << omega.transpose() * rl::math::RAD2DEG << std::endl;
		}
		
		for (std::size_t i = 0; i < steps; ++i)
		{
			rl::math::Real t = static_cast<rl::math::Real>(i) / static_cast<rl::math::Real>(steps);
			
			rl::math::Quaternion q = q1.slerp(t, q2);
			slerp << q._transformVector(p).transpose() << std::endl;
			
			rl::math::Quaternion qd = q1.slerpFirstDerivative(t, q2);
			rl::math::Vector3 omega = q.angularVelocity(qd);
			slerpFirstDerivative << steps + i << "\t" << omega.transpose() * rl::math::RAD2DEG << std::endl;
		}
		
		for (std::size_t i = 0; i < steps; ++i)
		{
			rl::math::Real t = static_cast<rl::math::Real>(i) / static_cast<rl::math::Real>(steps);
			
			rl::math::Quaternion q = q2.slerp(static_cast<rl::math::Real>(i) / static_cast<rl::math::Real>(steps), q3);
			slerp << q._transformVector(p).transpose() << std::endl;
			
			rl::math::Quaternion qd = q2.slerpFirstDerivative(t, q3);
			rl::math::Vector3 omega = q.angularVelocity(qd);
			slerpFirstDerivative << 2 * steps + i << "\t" << omega.transpose() * rl::math::RAD2DEG << std::endl;
		}
		
		slerp.close();
		slerpFirstDerivative.close();
		
		// splot "squad.dat" u 1:2:3 with lines
		std::ofstream squad;
		squad.open("squad.dat", std::fstream::trunc);
		
		// plot for [i=2:4] "squadFirstDerivative.dat" using 1:i with lines
		std::ofstream squadFirstDerivative;
		squadFirstDerivative.open("squadFirstDerivative.dat", std::fstream::trunc);
		
		for (std::size_t i = 0; i < steps; ++i)
		{
			rl::math::Real t = static_cast<rl::math::Real>(i) / static_cast<rl::math::Real>(steps);
			
			rl::math::Quaternion a = q0.squadControlPoint(q0, q1);
			rl::math::Quaternion b = q1.squadControlPoint(q0, q2);
			rl::math::Quaternion q = q0.squad(t, a, b, q1);
			squad << q._transformVector(p).transpose() << std::endl;
			
			rl::math::Quaternion qd = q0.squadFirstDerivative(t, a, b, q1);
			rl::math::Vector3 omega = q.angularVelocity(qd);
			squadFirstDerivative << i << "\t" << omega.transpose() * rl::math::RAD2DEG << std::endl;
		}
		
		for (std::size_t i = 0; i < steps; ++i)
		{
			rl::math::Real t = static_cast<rl::math::Real>(i) / static_cast<rl::math::Real>(steps);
			
			rl::math::Quaternion a = q1.squadControlPoint(q0, q2);
			rl::math::Quaternion b = q2.squadControlPoint(q1, q3);
			rl::math::Quaternion q = q1.squad(t, a, b, q2);
			squad << q._transformVector(p).transpose() << std::endl;
			
			rl::math::Quaternion qd = q1.squadFirstDerivative(t, a, b, q2);
			rl::math::Vector3 omega = q.angularVelocity(qd);
			squadFirstDerivative << steps + i << "\t" << omega.transpose() * rl::math::RAD2DEG << std::endl;
		}
		
		for (std::size_t i = 0; i < steps; ++i)
		{
			rl::math::Real t = static_cast<rl::math::Real>(i) / static_cast<rl::math::Real>(steps);
			
			rl::math::Quaternion a = q2.squadControlPoint(q1, q3);
			rl::math::Quaternion b = q3.squadControlPoint(q2, q3);
			rl::math::Quaternion q = q2.squad(t, a, b, q3);
			squad << q._transformVector(p).transpose() << std::endl;
			
			rl::math::Quaternion qd = q2.squadFirstDerivative(t, a, b, q3);
			rl::math::Vector3 omega = q.angularVelocity(qd);
			squadFirstDerivative << 2 * steps + i << "\t" << omega.transpose() * rl::math::RAD2DEG << std::endl;
		}
		
		squad.close();
		squadFirstDerivative.close();
	}
	
	{
		std::vector<rl::math::Quaternion> y;
		y.push_back(rl::math::Quaternion(1, 0, 0, 0));
		y.push_back(rl::math::Quaternion(0.5f * std::sqrt(2.0f), 0, -0.5f * std::sqrt(2.0f), 0));
		y.push_back(rl::math::Quaternion(0.5, -0.5, -0.5, 0.5));
		y.push_back(rl::math::Quaternion(0.5f * std::sqrt(2.0f), 0, 0, 0.5f * std::sqrt(2.0f)));
		y.push_back(rl::math::Quaternion(1, 0, 0, 0));
		
		std::vector<rl::math::Real> x;
		x.push_back(0);
		x.push_back(90);
		x.push_back(180);
		x.push_back(270);
		x.push_back(360);
		
		rl::math::Vector3 yd0;
		yd0 << 0, 1 * rl::math::DEG2RAD, 0;
		rl::math::Vector3 yd1;
		yd1 << 0, 1 * rl::math::DEG2RAD, 0;
		
		rl::math::Spline<rl::math::Quaternion> f = rl::math::Spline<rl::math::Quaternion>::CubicFirst(x, y, yd0, yd1);
		
		std::size_t steps = 999;
		
		// plot for [i=2:5] "cubic.dat" using 1:i with lines
		std::ofstream cubic;
		cubic.open("cubic.dat", std::fstream::trunc);
		
		// plot for [i=2:4] "cubicFirstDerivative.dat" using 1:i with lines
		std::ofstream cubicFirstDerivative;
		cubicFirstDerivative.open("cubicFirstDerivative.dat", std::fstream::trunc);
		
		// plot for [i=2:4] "cubicSecondDerivative.dat" using 1:i with lines
		std::ofstream cubicSecondDerivative;
		cubicSecondDerivative.open("cubicSecondDerivative.dat", std::fstream::trunc);
		
		for (std::size_t i = 0; i < steps; ++i)
		{
			rl::math::Real t = f.duration() * i / static_cast<rl::math::Real>(steps - 1);
			
			rl::math::Quaternion q = f(t);
			cubic << t << "\t" << q.coeffs().transpose() << std::endl;
			
			rl::math::Quaternion qd = f(t, 1);
			rl::math::Vector3 omega = q.angularVelocity(qd);
			cubicFirstDerivative << t << "\t" << omega.transpose() * rl::math::RAD2DEG << std::endl;
			
			rl::math::Quaternion qdd = f(t, 2);
			rl::math::Vector3 omegad = q.angularAcceleration(qd, qdd);
			cubicSecondDerivative << t << "\t" << omegad.transpose() * rl::math::RAD2DEG << std::endl;
		}
		
		cubic.close();
		cubicFirstDerivative.close();
		cubicSecondDerivative.close();
	}
	
	{
		rl::math::Vector3 point(0, 0, 1);
		
		std::function<rl::math::Real()> rand = std::bind(std::uniform_real_distribution<rl::math::Real>(0.0, 1.0), std::mt19937(0));
		rl::math::Quaternion uniform;
		
		// splot "uniformQuaternions.dat" using 2:3:4
		std::ofstream uniformQuaternions;
		uniformQuaternions.open("uniformQuaternions.dat", std::fstream::trunc);
		
		for (std::size_t i = 0; i < 10000; ++i)
		{
			uniform.setFromUniform(rl::math::Vector3(rand(), rand(), rand()));
			rl::math::Vector3 rotated = uniform._transformVector(point);
			uniformQuaternions << i << "\t" << rotated.transpose() << std::endl;
		}
		
		uniformQuaternions.close();
		
		std::function<rl::math::Real()> gauss = std::bind(std::normal_distribution<rl::math::Real>(0.0, 1.0), std::mt19937(0));
		rl::math::Quaternion normal;
		
		// splot "normalQuaternions.dat" using 2:3:4
		std::ofstream normalQuaternions;
		normalQuaternions.open("normalQuaternions.dat", std::fstream::trunc);
		
		for (std::size_t i = 0; i < 10000; ++i)
		{
			normal.setFromGaussian(rl::math::Vector3(gauss(), gauss(), gauss()), rl::math::Quaternion::Identity(), rl::math::Vector3(0.25, 0.125, 0.0625));
			rl::math::Vector3 rotated = normal._transformVector(point);
			normalQuaternions << i << "\t" << rotated.transpose() << std::endl;
		}
		
		normalQuaternions.close();
	}
	
	return EXIT_SUCCESS;
}
