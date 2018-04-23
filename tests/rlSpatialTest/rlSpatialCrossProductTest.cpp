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
	rl::math::Vector6 v1;
	v1.setRandom();
	
	rl::math::Vector6 v2;
	v2.setRandom();
	
	rl::math::MotionVector mv1 = v1;
	
	rl::math::MotionVector mv2 = v2;
	
	rl::math::Vector6 v3 = mv1.cross66Motion() * v2;
	rl::math::MotionVector mv3 = mv1.cross(mv2);
	
	if (!mv3.matrix().isApprox(v3))
	{
		std::cerr << "mv1 x mv2 != crossMotion(mv1) x v2" << std::endl;
		std::cerr << "mv1 x mv2 = " << mv3.matrix().transpose() << std::endl;
		std::cerr << "crossMotion(mv1) x v2 = " << v3.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::MotionVector mv4 = -mv2.cross(mv1);
	
	if (!mv4.matrix().isApprox(mv3.matrix()))
	{
		std::cerr << "mv1 x mv2 != -mv2 x mv1" << std::endl;
		std::cerr << "mv1 x mv2 = " << mv3.matrix().transpose() << std::endl;
		std::cerr << "-mv2 x mv1 = " << mv4.matrix().transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::ForceVector fv2 = v2;
	
	rl::math::Vector6 v5 = mv1.cross66Force() * v2;
	rl::math::ForceVector fv5 = mv1.cross(fv2);
	
	if (!fv5.matrix().isApprox(v5))
	{
		std::cerr << "mv1 x fv2 != crossForce(mv1) x v2" << std::endl;
		std::cerr << "mv1 x fv2 = " << fv5.matrix().transpose() << std::endl;
		std::cerr << "crossForce(mv1) x v2 = " << v5.transpose() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 mv1CrossForce = -mv1.cross66Motion().transpose();
	
	if (!mv1CrossForce.matrix().isApprox(mv1.cross66Force()))
	{
		std::cerr << "crossForce(mv1) != -crossMotion(mv1)^T" << std::endl;
		std::cerr << "crossForce(mv1) = " << std::endl << mv1CrossForce << std::endl;
		std::cerr << "-crossMotion(mv1)^T = " << std::endl << mv1.cross66Force() << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 cross1 = (mv1 + mv2).cross66Motion();
	rl::math::Matrix66 cross2 = mv1.cross66Motion() + mv2.cross66Motion();
	
	if (!cross1.matrix().isApprox(cross2))
	{
		std::cerr << "crossMotion(mv1 + mv2) != crossMotion(mv1) + crossMotion(mv2)" << std::endl;
		std::cerr << "crossMotion(mv1 + mv2) = " << std::endl << cross1 << std::endl;
		std::cerr << "crossMotion(mv1) + crossMotion(mv2) = " << std::endl << cross2 << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 cross3 = (mv1 + mv2).cross66Force();
	rl::math::Matrix66 cross4 = mv1.cross66Force() + mv2.cross66Force();
	
	if (!cross3.matrix().isApprox(cross4))
	{
		std::cerr << "crossForce(mv1 + mv2) != crossForce(mv1) + crossForce(mv2)" << std::endl;
		std::cerr << "crossForce(mv1 + mv2) = " << std::endl << cross3 << std::endl;
		std::cerr << "crossForce(mv1) + crossForce(mv2) = " << std::endl << cross4 << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::PlueckerTransform pt;
	pt.linear() = rl::math::Quaternion::Random().toRotationMatrix();
	pt.translation().setRandom();
	
	rl::math::Matrix66 m1 = (pt * mv1).cross66Motion();
	rl::math::Matrix66 m2 = pt.matrixMotion() * mv1.cross66Motion() * pt.inverse().matrixMotion();
	
	if (!m1.matrix().isApprox(m2))
	{
		std::cerr << "crossMotion(pt * mv1) != pt * crossMotion(mv1) * pt^-1" << std::endl;
		std::cerr << "crossMotion(pt * mv1) = " << std::endl << m1 << std::endl;
		std::cerr << "pt * crossMotion(mv1) * pt^-1 = " << std::endl << m2 << std::endl;
		exit(EXIT_FAILURE);
	}
	
	rl::math::Matrix66 m3 = (pt * mv1).cross66Force();
	rl::math::Matrix66 m4 = pt.matrixForce() * mv1.cross66Force() * pt.inverse().matrixForce();
	
	if (!m3.matrix().isApprox(m4))
	{
		std::cerr << "crossForce(pt * mv1) != pt * crossForce(mv1) * pt^-1" << std::endl;
		std::cerr << "crossForce(pt * mv1) = " << std::endl << m3 << std::endl;
		std::cerr << "pt * crossForce(mv1) * pt^-1 = " << std::endl << m4 << std::endl;
		exit(EXIT_FAILURE);
	}
	
	return EXIT_SUCCESS;
}
