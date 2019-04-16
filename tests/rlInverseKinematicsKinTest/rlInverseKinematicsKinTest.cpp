//
// Copyright (c) 2012, Andre Gaschler
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
#include <memory>
#include <stdexcept>
#include <rl/kin/Kinematics.h>
#include <rl/kin/Puma.h>

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlInverseKinematicsTest KINEMATICSFILE" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		std::string filename = argv[1];
		
		std::shared_ptr<rl::kin::Kinematics> kinematics(rl::kin::Kinematics::create(filename));
		kinematics->seed(0);
		
		for (std::size_t n = 0; n < 100; ++n)
		{
			rl::math::Vector q1 = kinematics->generatePositionUniform();
			kinematics->setPosition(q1);
			kinematics->updateFrames();
			rl::math::Transform t1 = kinematics->forwardPosition();
			
			rl::math::Vector q2 = kinematics->generatePositionUniform();
			kinematics->setPosition(q2);
			
			rl::math::Vector q3(kinematics->getDof());
			
			bool solved = kinematics->inversePosition(
				t1,
				q3,
				0,
				std::numeric_limits< ::rl::math::Real>::infinity(),
				static_cast<rl::math::Real>(1.0e-3),
				1000,
				std::chrono::seconds(10)
			);
			
			if (!solved)
			{
				std::cerr << "rl::kin::Kinematics::inversePosition on file " << filename << " with no solution." << std::endl;
				std::cerr << "t1 = " << std::endl << t1.matrix() << std::endl;
				std::cerr << "q1 = " << q1.transpose() << std::endl;
				std::cerr << "q2 = " << q2.transpose() << std::endl;
				return EXIT_FAILURE;
			}
			
			kinematics->setPosition(q3);
			kinematics->updateFrames();
			rl::math::Transform t3 = kinematics->forwardPosition();
			
			if (!t3.isApprox(t1, static_cast<rl::math::Real>(1.03-6)))
			{
				std::cerr << "rl::kin::Kinematics::inversePosition on file " << filename << " with incorrect operational position." << std::endl;
				std::cerr << "norm(t1 - t3) = " << (t1.matrix() - t3.matrix()).norm() << std::endl;
				std::cerr << "t1 = " << std::endl << t1.matrix() << std::endl;
				std::cerr << "t3 = " << std::endl << t3.matrix() << std::endl;
				std::cerr << "q1 = " << q1.transpose() << std::endl;
				std::cerr << "q2 = " << q2.transpose() << std::endl;
				std::cerr << "q3 = " << q3.transpose() << std::endl;
				return EXIT_FAILURE;
			}
			
			if (rl::kin::Puma* puma = dynamic_cast<rl::kin::Puma*>(kinematics.get()))
			{
				rl::kin::Puma::Arm arm;
				rl::kin::Puma::Elbow elbow;
				rl::kin::Puma::Wrist wrist;
				puma->parameters(q1, arm, elbow, wrist);
				puma->setArm(arm);
				puma->setElbow(elbow);
				puma->setWrist(wrist);
				
				rl::math::Vector q4(puma->getDof());
				
				if (!puma->inversePosition(t1, q4, true))
				{
					std::cerr << "rl::kin::Puma::inversePosition on file " << filename << " with no solution." << std::endl;
					std::cerr << "t1 = " << std::endl << t1.matrix() << std::endl;
					std::cerr << "q1 = " << q1.transpose() << std::endl;
					return EXIT_FAILURE;
				}
				
				puma->setPosition(q4);
				puma->updateFrames();
				rl::math::Transform t4 = puma->forwardPosition();
				
				if (!t4.isApprox(t1, static_cast<rl::math::Real>(1.03-6)))
				{
					std::cerr << "rl::kin::Puma::inversePosition on file " << filename << " with incorrect operational position." << std::endl;
					std::cerr << "norm(t1 - t4) = " << (t1.matrix() - t4.matrix()).norm() << std::endl;
					std::cerr << "t1 = " << std::endl << t1.matrix() << std::endl;
					std::cerr << "t4 = " << std::endl << t4.matrix() << std::endl;
					std::cerr << "q1 = " << q1.transpose() << std::endl;
					std::cerr << "q4 = " << q4.transpose() << std::endl;
					return EXIT_FAILURE;
				}
			}
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
