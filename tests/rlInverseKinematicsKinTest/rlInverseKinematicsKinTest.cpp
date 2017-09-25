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
#include <random>
#include <rl/kin/Kinematics.h>
#include <rl/kin/Puma.h>
#include <rl/math/Unit.h>

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
		std::mt19937 randomGenerator(0);
		std::uniform_real_distribution<rl::math::Real> randomDistribution(-180 * rl::math::DEG2RAD, 180 * rl::math::DEG2RAD);
		
		std::shared_ptr<rl::kin::Kinematics> kinematics(rl::kin::Kinematics::create(argv[1]));		
		
		std::size_t nTests;
		
		rl::math::Vector q(6);
		rl::math::Vector qinv(6);
		rl::math::Vector qzero(6);
		std::size_t n;
		std::size_t wrongs;
		std::size_t wrongT;
		std::size_t ngotinverse;
		
		for (std::size_t ispuma = 0; ispuma < 2; ++ispuma)
		{
			nTests = 0 == ispuma ? 1000 : 100;
			
			for (n = 0, wrongs = 0, wrongT = 0, ngotinverse = 0; n < nTests && wrongT < 100 && wrongs < 100; ++n)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					q(i) = randomDistribution(randomGenerator);
					qzero(i) = 0;
				}
				
				kinematics->setPosition(q);
				kinematics->updateFrames();
				
				rl::math::Transform t = kinematics->forwardPosition();
				
				// For iterative inverse, set starting point far away
				kinematics->setPosition(qzero);
				kinematics->updateFrames();
				
				if (0 == ispuma)
				{
					rl::kin::Puma::Arm arm;
					rl::kin::Puma::Elbow elbow;
					rl::kin::Puma::Wrist wrist;
					dynamic_cast<rl::kin::Puma*>(kinematics.get())->parameters(q, arm, elbow, wrist);
					dynamic_cast<rl::kin::Puma*>(kinematics.get())->setArm(arm);
					dynamic_cast<rl::kin::Puma*>(kinematics.get())->setElbow(elbow);
					dynamic_cast<rl::kin::Puma*>(kinematics.get())->setWrist(wrist);
				}
				
				if (
					0 == ispuma
					? dynamic_cast< ::rl::kin::Puma*>(kinematics.get())->inversePosition(t, qinv)
					: dynamic_cast< ::rl::kin::Kinematics*>(kinematics.get())->inversePosition(t, qinv, 0, 100)
				)
				{
					kinematics->setPosition(qinv);
					kinematics->updateFrames();
					rl::math::Transform tinv = kinematics->forwardPosition();
					
					if ((t.matrix() - tinv.matrix()).norm() > 1e-6)
					{
						++wrongT;
					}
					
					if ((q - qinv).norm() > 1e-4)
					{
						++wrongs;
					}
					
					if (true) //wrongT < 3 && (t.matrix() - tinv.matrix()).norm() > 1e-6)
					{
						std::cout << "      q    = " << q.transpose() << std::endl;
						std::cout << "      T    = " << t.matrix() << std::endl;
						std::cout << "      qinv = " << qinv.transpose() << std::endl;
						std::cout << "      Tinv = " << tinv.matrix() << std::endl;
						std::cout << std::endl;
					}
					
					++ngotinverse;
				}
			}
			
			std::cout << "Notice: "
				<< (0 == ispuma ? "Puma direct " : "Iterative ") << "inverse kinematics "
				<< "on file " << argv[1] << " "
				<< "tested with " << n << " cases, "
				<< ngotinverse << " returned a solution, "
				<< "thereof " << wrongs << " in wrong configuration, and "
				<< wrongT << " with completely wrong pose."
				<< std::endl;
			
			if (wrongT > 0)
			{
				std::cerr << "Error: "
					<< (0 == ispuma ? "Puma direct " : "Iterative ") 
					<< "inverse kinematics " << "on file " << argv[1] 
					<< " gave incorrect poses." << std::endl;
				return EXIT_FAILURE;
			}
			
			if (0 == ngotinverse)
			{
				std::cerr << "Error: "
					<< (0 == ispuma ? "Puma direct " : "Iterative ") 
					<< "inverse kinematics "<< "on file " << argv[1]
					<< " gave no solutions."
					<< std::endl;
				return EXIT_FAILURE;
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
