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
#include <rl/math/Matrix.h>
#include <rl/math/Quaternion.h>
#include <rl/math/Spatial.h>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>

int
main(int argc, char** argv)
{
	rl::math::Transform t0;
	t0 = rl::math::Quaternion::Random();
	t0.translation().setRandom();
	
	rl::math::PlueckerTransform pt0;
	pt0.rotation() = t0.linear();
	pt0.translation() = t0.translation();
	
	if (!pt0.rotation().isApprox(t0.linear()) || !pt0.translation().isApprox(t0.translation()))
	{
		std::cerr << "pt0 != t0" << std::endl;
		std::cerr << "pt0.rotation = " << std::endl << pt0.rotation() << std::endl;
		std::cerr << "pt0.translation = " << pt0.translation().transpose() << std::endl;
		std::cerr << "t0.linear = " << std::endl << t0.linear() << std::endl;
		std::cerr << "t0.translation = " << t0.translation().transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Transform t1;
	t1 = rl::math::Quaternion::Random();
	t1.translation().setRandom();
	
	rl::math::PlueckerTransform pt1(t1);
	
	if (!pt1.rotation().isApprox(t1.linear()) || !pt1.translation().isApprox(t1.translation()))
	{
		std::cerr << "pt1(t1) != t1" << std::endl;
		std::cerr << "pt1(t1).rotation = " << std::endl << pt1.rotation() << std::endl;
		std::cerr << "pt1(t1).translation = " << pt1.translation().transpose() << std::endl;
		std::cerr << "t1.linear = " << std::endl << t1.linear() << std::endl;
		std::cerr << "t1.translation = " << t1.translation().transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Transform t2;
	t2 = rl::math::Quaternion::Random();
	t2.translation().setRandom();
	
	rl::math::PlueckerTransform pt2 = t2;
	
	if (!pt2.rotation().isApprox(t2.linear()) || !pt2.translation().isApprox(t2.translation()))
	{
		std::cerr << "(pt2 = t2) != t2" << std::endl;
		std::cerr << "(pt2 = t2).rotation = " << std::endl << pt2.rotation() << std::endl;
		std::cerr << "(pt2 = t2).translation = " << pt2.translation().transpose() << std::endl;
		std::cerr << "t2.linear = " << std::endl << t2.linear() << std::endl;
		std::cerr << "t2.translation = " << t2.translation().transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Transform t3 = t1 * t2;
	rl::math::PlueckerTransform pt3 = pt1 * pt2;
	
	if (!pt3.rotation().isApprox(t3.linear()) || !pt3.translation().isApprox(t3.translation()))
	{
		std::cerr << "pt1 * pt2 != t1 * t2" << std::endl;
		std::cerr << "(pt1 * pt2).rotation = " << std::endl << pt3.rotation() << std::endl;
		std::cerr << "(pt1 * pt2).translation = " << pt3.translation().transpose() << std::endl;
		std::cerr << "(t1 * t2).linear = " << std::endl << t3.linear() << std::endl;
		std::cerr << "(t1 * t2).translation = " << t3.translation().transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Transform t4 = t1.inverse() * t3;
	rl::math::PlueckerTransform pt4 = pt1.inverse() * pt3;
	
	if (!pt4.rotation().isApprox(pt2.rotation()) || !pt4.translation().isApprox(pt2.translation()))
	{
		std::cerr << "inv(pt1) * pt1 * pt2 != pt2" << std::endl;
		std::cerr << "(inv(pt1) * pt1 * pt2).rotation = " << std::endl << pt4.rotation() << std::endl;
		std::cerr << "(inv(pt1) * pt1 * pt2).translation = " << pt4.translation().transpose() << std::endl;
		std::cerr << "pt2.rotation = " << std::endl << pt2.rotation().matrix() << std::endl;
		std::cerr << "pt2.translation = " << pt2.translation().transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	if (!pt4.rotation().isApprox(t4.linear()) || !pt4.translation().isApprox(t4.translation()))
	{
		std::cerr << "inv(pt1) * pt3 != inv(t1) * t3" << std::endl;
		std::cerr << "(inv(pt1) * pt3).rotation = " << std::endl << pt4.rotation() << std::endl;
		std::cerr << "(inv(pt1) * pt3).translation = " << pt4.translation().transpose() << std::endl;
		std::cerr << "(inv(t1) * t3).linear = " << std::endl << t4.linear() << std::endl;
		std::cerr << "(inv(t1) * t3).translation = " << t4.translation().transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::PlueckerTransform pt5;
	pt5.setIdentity();
	rl::math::PlueckerTransform pt6 = pt1.inverse() * pt1;
	
	if (!pt5.rotation().isApprox(pt6.rotation()) || !pt5.translation().isMuchSmallerThan(pt6.translation()))
	{
		std::cerr << "I != inv(pt1) * pt1" << std::endl;
		std::cerr << "I.rotation = " << std::endl << pt5.rotation().matrix() << std::endl;
		std::cerr << "I.translation = " << pt5.translation().transpose() << std::endl;
		std::cerr << "(inv(pt1) * pt1).rotation = " << std::endl << pt6.rotation() << std::endl;
		std::cerr << "(inv(pt1) * pt1).translation = " << pt6.translation().transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 m1a = pt1.matrixMotion().inverse();
	rl::math::Matrix66 m1b = pt1.inverseMotion();
	
	if (!m1a.matrix().isApprox(m1b))
	{
		std::cerr << "matrixMotion(pt1)^-1 != inverseMotion(pt1)" << std::endl;
		std::cerr << "matrixMotion(pt1)^-1 = " << std::endl << m1a << std::endl;
		std::cerr << "inverseMotion(pt1) = " << std::endl << m1b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 m2a = pt1.matrixForce().inverse();
	rl::math::Matrix66 m2b = pt1.inverseForce();
	
	if (!m2a.matrix().isApprox(m2b))
	{
		std::cerr << "matrixForce(pt1)^-1 != inverseForce(pt1)" << std::endl;
		std::cerr << "matrixForce(pt1)^-1 = " << std::endl << m2a << std::endl;
		std::cerr << "inverseForce(pt1) = " << std::endl << m2b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 m3a = pt1.matrixForce();
	rl::math::Matrix66 m3b = pt1.matrixMotion().inverse().transpose();
	
	if (!m3a.matrix().isApprox(m3b))
	{
		std::cerr << "matrixForce(pt1) != matrixMotion(pt1)^-T" << std::endl;
		std::cerr << "matrixForce(pt1) = " << std::endl << m3a << std::endl;
		std::cerr << "matrixMotion(pt1)^-T = " << std::endl << m3b << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Vector6 v1;
	v1.setRandom();
	
	{
		rl::math::MotionVector mv1 = v1;
		
		rl::math::Vector6 v2 = pt1.matrixMotion() * v1;
		rl::math::MotionVector mv2 = pt1 * mv1;
		
		if (!mv2.matrix().isApprox(v2))
		{
			std::cerr << "pt1 * mv1 != matrixMotion(pt1) * v1" << std::endl;
			std::cerr << "pt1 * mv1 = " << mv2.matrix().transpose() << std::endl;
			std::cerr << "matrixMotion(pt1) * v1 = " << v2.transpose() << std::endl;
			exit(EXIT_FAILURE);
		}
		
		rl::math::Vector6 v3 = pt1.inverseMotion() * v2;
		rl::math::MotionVector mv3 = pt1 / mv2;
		
		if (!mv3.matrix().isApprox(v3))
		{
			std::cerr << "inv(pt1) / mv2 != inverseMotion(pt1) * v2" << std::endl;
			std::cerr << "inv(pt1) / mv2 = " << mv3.matrix().transpose() << std::endl;
			std::cerr << "inverseMotion(pt1) * v2 = " << v3.transpose() << std::endl;
			exit(EXIT_FAILURE);
		}
		
		if (!mv3.matrix().isApprox(mv1.matrix()))
		{
			std::cerr << "pt1.inverse() * pt1 * mv1 != mv1" << std::endl;
			std::cerr << "pt1.inverse() * pt1 * mv1 = " << mv3.matrix().transpose() << std::endl;
			std::cerr << "mv1 = " << mv1.matrix().transpose() << std::endl;
			exit(EXIT_FAILURE);
		}
		
		rl::math::MotionVector mv4 = pt1.inverse() * mv2;
		
		if (!mv4.matrix().isApprox(mv1.matrix()))
		{
			std::cerr << "pt1.inverse() * pt1 * mv1 != mv1" << std::endl;
			std::cerr << "pt1.inverse() * pt1 * mv1 = " << mv4.matrix().transpose() << std::endl;
			std::cerr << "mv1 = " << mv1.matrix().transpose() << std::endl;
			exit(EXIT_FAILURE);
		}
	}
	
	{
		rl::math::ForceVector fv1 = v1;
		
		rl::math::Vector6 v2 = pt1.matrixForce() * v1;
		rl::math::ForceVector fv2 = pt1 * fv1;
		
		if (!fv2.matrix().isApprox(v2))
		{
			std::cerr << "pt1 * fv1 != matrixForce(pt1) * v1" << std::endl;
			std::cerr << "pt1 * fv1 = " << fv2.matrix().transpose() << std::endl;
			std::cerr << "matrixForce(pt1) * v1 = " << v2.transpose() << std::endl;
			exit(EXIT_FAILURE);
		}
		
		rl::math::Vector6 v3 = pt1.inverseForce() * v2;
		rl::math::ForceVector fv3 = pt1 / fv2;
		
		if (!fv3.matrix().isApprox(v3))
		{
			std::cerr << "inv(pt1) / fv2 != inverseForce(pt1) * v2" << std::endl;
			std::cerr << "inv(pt1) / fv2 = " << fv3.matrix().transpose() << std::endl;
			std::cerr << "inverseForce(pt1) * v2 = " << v3.transpose() << std::endl;
			exit(EXIT_FAILURE);
		}
		
		if (!fv3.matrix().isApprox(fv1.matrix()))
		{
			std::cerr << "pt1.inverse() * pt1 * fv1 != fv1" << std::endl;
			std::cerr << "pt1.inverse() * pt1 * fv1 = " << fv3.matrix().transpose() << std::endl;
			std::cerr << "fv1 = " << fv1.matrix().transpose() << std::endl;
			exit(EXIT_FAILURE);
		}
		
		rl::math::ForceVector fv4 = pt1.inverse() * fv2;
		
		if (!fv4.matrix().isApprox(fv1.matrix()))
		{
			std::cerr << "pt1.inverse() * pt1 * fv1 != fv1" << std::endl;
			std::cerr << "pt1.inverse() * pt1 * fv1 = " << fv4.matrix().transpose() << std::endl;
			std::cerr << "fv1 = " << fv1.matrix().transpose() << std::endl;
			exit(EXIT_FAILURE);
		}
	}
	
	return EXIT_SUCCESS;
}
