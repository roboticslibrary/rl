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

#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>

#include <iostream>
#include <rl/math/Spatial.h>

int
main(int argc, char** argv)
{
	{
		rl::math::MotionVector v1;
		v1.angular() << 0, 1, 2;
		v1.linear() << 3, 4, 5;
		std::cout << "v1 = " << v1.matrix().transpose() << std::endl;
		v1 = v1.matrix();
		std::cout << "v1 = " << v1.matrix().transpose() << std::endl;
		
		rl::math::MotionVector v2;
		v2.angular() << 0, 1, 2;
		v2.linear() << 3, 4, 5;
		std::cout << "v2 = " << v2.matrix().transpose() << std::endl;
		
		rl::math::MotionVector v3;
		v3 = v1 + v2;
		std::cout << "v3 = " << v3.matrix().transpose() << std::endl;
		v3 = v1 + v2 + v1;
		std::cout << "v3 = " << v3.matrix().transpose() << std::endl;
		v3 = v1 - v2;
		std::cout << "v3 = " << v3.matrix().transpose() << std::endl;
		v3 = v1 - v2 - v1;
		std::cout << "v3 = " << v3.matrix().transpose() << std::endl;
		
		v3 = v1.cross(v2);
		std::cout << "v3 = " << v3.matrix().transpose() << std::endl;
	}
	
	std::cout << std::endl;
	
	{
		rl::math::ForceVector f1;
		f1.moment() << 0, 1, 2;
		f1.force() << 3, 4, 5;
		std::cout << "f1 = " << f1.matrix().transpose() << std::endl;
		f1 = f1.matrix();
		std::cout << "f1 = " << f1.matrix().transpose() << std::endl;
		
		rl::math::ForceVector f2;
		f2.moment() << 0, 1, 2;
		f2.force() << 3, 4, 5;
		std::cout << "f2 = " << f2.matrix().transpose() << std::endl;
		
		rl::math::ForceVector f3;
		f3 = f1 + f2;
		std::cout << "f3 = " << f3.matrix().transpose() << std::endl;
		f3 = f1 + f2 + f3;
		std::cout << "f3 = " << f3.matrix().transpose() << std::endl;
		f3 = f1 - f2;
		std::cout << "f3 = " << f3.matrix().transpose() << std::endl;
		f3 = f1 - f2 - f1;
		std::cout << "f3 = " << f3.matrix().transpose() << std::endl;
		
		rl::math::MotionVector v1;
		v1.angular() << 0, 1, 2;
		v1.linear() << 3, 4, 5;
		
		f3 = v1.cross(f2);
		std::cout << "f3 = " << f3.matrix().transpose() << std::endl;
		
		std::cout << v1.dot(f1) << std::endl;
		std::cout << f1.dot(v1) << std::endl;
	}
	
	std::cout << std::endl;
	
	{
		rl::math::PlueckerTransform X1;
		X1.rotation() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		X1.translation() << 0, 1, 2;
		std::cout << "X1.rotation = " << std::endl << X1.rotation() << std::endl;
		std::cout << "X1.translation = " << X1.translation().transpose() << std::endl;
		std::cout << "X1.matrixMotion = " << std::endl << X1.matrixMotion() << std::endl;
		std::cout << "X1.matrixForce = " << std::endl << X1.matrixForce() << std::endl;
		std::cout << "X1.inverseMotion = " << std::endl << X1.inverseMotion() << std::endl;
		std::cout << "X1.inverseForce = " << std::endl << X1.inverseForce() << std::endl;
		std::cout << "X1.transform = " << std::endl << X1.transform().matrix() << std::endl;
		
		rl::math::PlueckerTransform X2;
		X2.rotation() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		X2.translation() << 0, 1, 2;
		std::cout << "X2.rotation = " << std::endl << X2.rotation() << std::endl;
		std::cout << "X2.translation = " << X2.translation().transpose() << std::endl;
		
		rl::math::PlueckerTransform X3;
		X3 = X1 * X2;
		std::cout << "X3.rotation = " << std::endl << X3.rotation() << std::endl;
		std::cout << "X3.translation = " << X3.translation().transpose() << std::endl;
		
		rl::math::MotionVector v1;
		v1.angular() << 0, 1, 2;
		v1.linear() << 3, 4, 5;
		std::cout << "v1 = " << v1.matrix().transpose() << std::endl;
		
		rl::math::MotionVector v2;
		v2 = X1 * v1;
		std::cout << "v2 = " << v2.matrix().transpose() << std::endl;
		rl::math::MotionVector::MatrixType v2vector;
		v2vector = X1.matrixMotion() * v1.matrix();
		std::cout << "v2vector = " << v2vector.transpose() << std::endl;
		v2 = X1 / v1;
		std::cout << "v2 = " << v2.matrix().transpose() << std::endl;
		v2vector = X1.inverseMotion() * v1.matrix();
		std::cout << "v2vector = " << v2vector.transpose() << std::endl;
		
		rl::math::ForceVector f1;
		f1.moment() << 0, 1, 2;
		f1.force() << 3, 4, 5;
		std::cout << "f1 = " << f1.matrix().transpose() << std::endl;
		
		rl::math::ForceVector f2;
		f2 = X1 * f1;
		std::cout << "f2 = " << f2.matrix().transpose() << std::endl;
		rl::math::ForceVector::MatrixType f2vector;
		f2vector = X1.matrixForce() * f1.matrix();
		std::cout << "f2vector = " << f2vector.transpose() << std::endl;
		f2 = X1 / f1;
		std::cout << "f2 = " << f2.matrix().transpose() << std::endl;
		f2vector = X1.inverseForce() * f1.matrix();
		std::cout << "f2vector = " << f2vector.transpose() << std::endl;
	}
	
	std::cout << std::endl;
	
	{
		rl::math::PlueckerTransform X1;
		X1.rotation() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		X1.translation() << 0, 1, 2;
		std::cout << "X1.rotation = " << std::endl << X1.rotation() << std::endl;
		std::cout << "X1.translation = " << X1.translation().transpose() << std::endl;
		
		rl::math::RigidBodyInertia I1;
		I1.cog() << 0, 1, 2;
		I1.inertia() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		I1.mass() = 1;
		std::cout << "I1.cog = " << I1.cog().transpose() << std::endl;
		std::cout << "I1.inertia = " << std::endl << I1.inertia() << std::endl;
		std::cout << "I1.mass = " << I1.mass() << std::endl;
		std::cout << "I1.matrix = " << std::endl << I1.matrix() << std::endl;
		I1 = I1.matrix();
		std::cout << "I1.matrix = " << std::endl << I1.matrix() << std::endl;
		
		rl::math::RigidBodyInertia I2;
		I2.cog() << 0, 1, 2;
		I2.inertia() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		I2.mass() = 1;
		
		rl::math::RigidBodyInertia I3;
		I3 = I1 + I2;
		std::cout << "I3.cog = " << I3.cog().transpose() << std::endl;
		std::cout << "I3.inertia = " << std::endl << I3.inertia() << std::endl;
		std::cout << "I3.mass = " << I3.mass() << std::endl;
		I3 = I1 - I2;
		std::cout << "I3.cog = " << I3.cog().transpose() << std::endl;
		std::cout << "I3.inertia = " << std::endl << I3.inertia() << std::endl;
		std::cout << "I3.mass = " << I3.mass() << std::endl;
		I3 = I1 * 2;
		std::cout << "I3.cog = " << I3.cog().transpose() << std::endl;
		std::cout << "I3.inertia = " << std::endl << I3.inertia() << std::endl;
		std::cout << "I3.mass = " << I3.mass() << std::endl;
		I3 = I1 / 2;
		std::cout << "I3.cog = " << I3.cog().transpose() << std::endl;
		std::cout << "I3.inertia = " << std::endl << I3.inertia() << std::endl;
		std::cout << "I3.mass = " << I3.mass() << std::endl;
		
		I2 = X1 * I1;
		std::cout << "I2.cog = " << I2.cog().transpose() << std::endl;
		std::cout << "I2.inertia = " << std::endl << I2.inertia() << std::endl;
		std::cout << "I2.mass = " << I2.mass() << std::endl;
		std::cout << "I2.matrix = " << std::endl << I2.matrix() << std::endl;
		rl::math::RigidBodyInertia::MatrixType I2matrix;
		I2matrix = X1.matrixForce() * I1.matrix() * X1.inverseMotion();
		std::cout << "I2matrix = " << std::endl << I2matrix << std::endl;
		
		I2 = X1 / I1;
		std::cout << "I2.cog = " << I2.cog().transpose() << std::endl;
		std::cout << "I2.inertia = " << std::endl << I2.inertia() << std::endl;
		std::cout << "I2.mass = " << I2.mass() << std::endl;
		std::cout << "I2.matrix = " << std::endl << I2.matrix() << std::endl;
		I2matrix = X1.inverseForce() * I1.matrix() * X1.matrixMotion();
		std::cout << "I2matrix = " << std::endl << I2matrix << std::endl;
		
		rl::math::MotionVector v1;
		v1.angular() << 0, 1, 2;
		v1.linear() << 3, 4, 5;
		std::cout << "v1 = " << v1.matrix().transpose() << std::endl;
		
		rl::math::ForceVector f1;
		f1 = I1 * v1;
		std::cout << "f1 = " << f1.matrix().transpose() << std::endl;
	}
	
	std::cout << std::endl;
	
	{
		rl::math::PlueckerTransform X1;
		X1.rotation() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		X1.translation() << 0, 1, 2;
		std::cout << "X1.rotation = " << std::endl << X1.rotation() << std::endl;
		std::cout << "X1.translation = " << X1.translation().transpose() << std::endl;
		
		rl::math::ArticulatedBodyInertia IA1;
		IA1.cog() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		IA1.inertia() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		IA1.mass() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		std::cout << "IA1.cog = " << std::endl << IA1.cog() << std::endl;
		std::cout << "IA1.inertia = " << std::endl << IA1.inertia() << std::endl;
		std::cout << "IA1.mass = " << std::endl << IA1.mass() << std::endl;
		std::cout << "IA1.matrix = " << std::endl << IA1.matrix() << std::endl;
		IA1 = IA1.matrix();
		std::cout << "IA1.matrix = " << std::endl << IA1.matrix() << std::endl;
		
		rl::math::RigidBodyInertia I1;
		I1.cog() << 0, 1, 2;
		I1.inertia() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		I1.mass() = 1;
		IA1 = I1;
		std::cout << "IA1.cog = " << std::endl << IA1.cog() << std::endl;
		std::cout << "IA1.inertia = " << std::endl << IA1.inertia() << std::endl;
		std::cout << "IA1.mass = " << std::endl << IA1.mass() << std::endl;
		
		rl::math::ArticulatedBodyInertia IA2;
		IA2.cog() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		IA2.inertia() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		IA2.mass() << 0, 1, 2, 3, 4, 5, 6, 7, 8;
		
		rl::math::ArticulatedBodyInertia IA3;
		IA3 = IA1 + IA2;
		std::cout << "IA3.cog = " << std::endl << IA3.cog() << std::endl;
		std::cout << "IA3.inertia = " << std::endl << IA3.inertia() << std::endl;
		std::cout << "IA3.mass = " << std::endl << IA3.mass() << std::endl;
		IA3 = IA1 - IA2;
		std::cout << "IA3.cog = " << std::endl << IA3.cog() << std::endl;
		std::cout << "IA3.inertia = " << std::endl << IA3.inertia() << std::endl;
		std::cout << "IA3.mass = " << std::endl << IA3.mass() << std::endl;
		IA3 = IA1 * 2;
		std::cout << "IA3.cog = " << std::endl << IA3.cog() << std::endl;
		std::cout << "IA3.inertia = " << std::endl << IA3.inertia() << std::endl;
		std::cout << "IA3.mass = " << std::endl << IA3.mass() << std::endl;
		IA3 = IA1 / 2;
		std::cout << "IA3.cog = " << std::endl << IA3.cog() << std::endl;
		std::cout << "IA3.inertia = " << std::endl << IA3.inertia() << std::endl;
		std::cout << "IA3.mass = " << std::endl << IA3.mass() << std::endl;
		
		IA2 = X1 * IA1;
		std::cout << "IA2.cog = " << std::endl << IA2.cog() << std::endl;
		std::cout << "IA2.inertia = " << std::endl << IA2.inertia() << std::endl;
		std::cout << "IA2.mass = " << std::endl << IA2.mass() << std::endl;
		std::cout << "IA2.matrix = " << std::endl << IA2.matrix() << std::endl;
		rl::math::ArticulatedBodyInertia::MatrixType IA2matrix;
		IA2matrix = X1.matrixForce() * IA1.matrix() * X1.inverseMotion();
		std::cout << "IA2matrix = " << std::endl << IA2matrix << std::endl;
		
		IA2 = X1 / IA1;
		std::cout << "IA2.cog = " << std::endl << IA2.cog() << std::endl;
		std::cout << "IA2.inertia = " << std::endl << IA2.inertia() << std::endl;
		std::cout << "IA2.mass = " << std::endl << IA2.mass() << std::endl;
		std::cout << "IA2.matrix = " << std::endl << IA2.matrix() << std::endl;
		IA2matrix = X1.inverseForce() * IA1.matrix() * X1.matrixMotion();
		std::cout << "IA2matrix = " << std::endl << IA2matrix << std::endl;
		
		rl::math::MotionVector v1;
		v1.angular() << 0, 1, 2;
		v1.linear() << 3, 4, 5;
		std::cout << "v1 = " << v1.matrix().transpose() << std::endl;
		
		rl::math::ForceVector f1;
		f1 = IA1 * v1;
		std::cout << "f1 = " << f1.matrix().transpose() << std::endl;
	}
	
	return EXIT_SUCCESS;
}
