//
// Copyright (c) 2014, Andre Gaschler
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
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/XmlFactory.h>

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlInverseKinematicsMdlTest KINEMATICSFILE" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		std::mt19937 randomGenerator(0);
		std::uniform_real_distribution<rl::math::Real> randomDistribution(-180 * rl::math::DEG2RAD, 180 * rl::math::DEG2RAD);
		
		rl::mdl::XmlFactory factory;
		std::shared_ptr<rl::mdl::Model> model(factory.create(argv[1]));
		rl::mdl::Kinematic* kinematics = dynamic_cast<rl::mdl::Kinematic*>(model.get());	
		
		std::size_t nTests;
		
		rl::math::Vector q(6);
		rl::math::Vector qinv(6);
		rl::math::Vector qzero(6);
		std::size_t n;
		std::size_t wrongs;
		std::size_t wrongT;
		std::size_t ngotinverse;
		
		nTests = 100;
		
		for (n = 0, wrongs = 0, wrongT = 0, ngotinverse = 0; n < nTests && wrongT < 100 && wrongs < 100; ++n)
		{
			for (std::size_t i = 0; i < 6; ++i)
			{
				q(i) = randomDistribution(randomGenerator);
				qzero(i) = 0;
			}
			
			kinematics->setPosition(q);
			kinematics->forwardPosition();
			rl::math::Transform t = kinematics->getOperationalPosition(0);
			
			// For iterative inverse, set starting point far away
			kinematics->setPosition(qzero);
			kinematics->forwardPosition();
			
			if (!kinematics->calculateInversePosition(t, 0, 0.5))
			{
				continue;
			}
			
			qinv = kinematics->getPosition();
			kinematics->forwardPosition();
			rl::math::Transform tinv = kinematics->getOperationalPosition(0);
			
			if ((t.matrix() - tinv.matrix()).norm() > 1e-5)
			{
				++wrongT;
			}
			
			if ((q - qinv).norm() > 1e-4)
			{
				++wrongs;
			}
			
			if (wrongT < 3 && (t.matrix() - tinv.matrix()).norm() > 1e-5)
			{
				std::cout << "      q    = " << q.transpose() << std::endl;
				std::cout << "      T    = " << t.matrix() << std::endl;
				std::cout << "      qinv = " << qinv.transpose() << std::endl;
				std::cout << "      Tinv = " << tinv.matrix() << std::endl;
				std::cout << std::endl;
			}
			
			++ngotinverse;
			
			
			if (wrongT > 0)
			{
				std::cerr << "Error: "
					<< "Iterative inverse kinematics " << "on file " << argv[1] 
					<< " gave incorrect poses." << std::endl;
				return EXIT_FAILURE;
			}
			
			if (0 == ngotinverse)
			{
				std::cerr << "Error: "
					<< "Iterative inverse kinematics "<< "on file " << argv[1]
					<< " gave no solutions."
					<< std::endl;
				return EXIT_FAILURE;
			}
		}
		std::cout << "Notice: "
			<< "Iterative inverse kinematics "
			<< "on file " << argv[1] << " "
			<< "tested with " << n << " cases, "
			<< ngotinverse << " returned a solution, "
			<< "thereof " << wrongs << " in wrong configuration, and "
			<< wrongT << " with completely wrong pose."
			<< std::endl;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
