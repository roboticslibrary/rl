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
#include <memory>
#include <random>
#include <stdexcept>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/XmlFactory.h>

int
main(int argc, char** argv)
{
	if (argc < 3)
	{
		std::cout << "Usage: rlDynamicsTest MODELFILE LOOP" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		srand(std::random_device()());
		
		rl::mdl::XmlFactory factory;
		std::shared_ptr<rl::mdl::Dynamic> dynamic = std::dynamic_pointer_cast<rl::mdl::Dynamic>(factory.create(argv[1]));
		
		rl::math::Vector q(dynamic->getDofPosition());
		rl::math::Vector qd(dynamic->getDof());
		rl::math::Vector qdd(dynamic->getDof());
		
		for (::std::size_t i = 0; i < atoi(argv[2]); ++i)
		{
			if (0 == i)
			{
				q.setZero();
				qd.setOnes();
				qdd.setOnes();
			}
			else
			{
				q.setRandom();
				qd.setRandom();
				qdd.setRandom();
			}
			
			// forward velocity (recursive)
			
			dynamic->setPosition(q);
			dynamic->setVelocity(qd);
			dynamic->forwardVelocity();
			
			rl::math::Vector xdRecursive(6 * dynamic->getOperationalDof());
			
			for (::std::size_t j = 0; j < dynamic->getOperationalDof(); ++j)
			{
				xdRecursive.segment(6 * j, 3) = dynamic->getOperationalVelocity(j).linear();
				xdRecursive.segment(6 * j + 3, 3) = dynamic->getOperationalVelocity(j).angular();
			}
			
			// forward velocity (matrices)
			
			dynamic->setPosition(q);
			dynamic->calculateJacobian(false);
			
			rl::math::Vector xdMatrices = dynamic->getJacobian() * qd;
			
			if (!xdMatrices.isApprox(xdRecursive))
			{
				std::cerr << "q = " << q.transpose() << std::endl;
				std::cerr << "qd = " << qd.transpose() << std::endl;
				std::cerr << "xd (recursive) = " << xdRecursive.transpose() << std::endl;
				std::cerr << "xd (matrices) = " << xdMatrices.transpose() << std::endl;
				return EXIT_FAILURE;
			}
			
			// forward acceleration (recursive)
			
			dynamic->setPosition(q);
			dynamic->setVelocity(qd);
			dynamic->setAcceleration(qdd);
			dynamic->forwardVelocity();
			dynamic->forwardAcceleration();
			
			rl::math::Vector xddRecursive(6 * dynamic->getOperationalDof());
			
			for (::std::size_t j = 0; j < dynamic->getOperationalDof(); ++j)
			{
				xddRecursive.segment(6 * j, 3) = dynamic->getOperationalAcceleration(j).linear();
				xddRecursive.segment(6 * j + 3, 3) = dynamic->getOperationalAcceleration(j).angular();
			}
			
			// forward acceleration (matrices)
			
			dynamic->setPosition(q);
			dynamic->calculateJacobian(false);
			dynamic->setVelocity(qd);
			dynamic->calculateJacobianDerivative(false);
			
			rl::math::Vector xddMatrices = dynamic->getJacobian() * qdd + dynamic->getJacobianDerivative();
			
			if (!xddMatrices.isApprox(xddRecursive))
			{
				std::cerr << "q = " << q.transpose() << std::endl;
				std::cerr << "qd = " << qd.transpose() << std::endl;
				std::cerr << "qdd = " << qdd.transpose() << std::endl;
				std::cerr << "xdd (recursive) = " << xddRecursive.transpose() << std::endl;
				std::cerr << "xdd (matrices) = " << xddMatrices.transpose() << std::endl;
				return EXIT_FAILURE;
			}
			
			// inverse dynamics (recursive)
			
			dynamic->setPosition(q);
			dynamic->setVelocity(qd);
			dynamic->setAcceleration(qdd);
			dynamic->inverseDynamics();
			
			rl::math::Vector tauRecursive = dynamic->getTorque();
			
			// inverse dynamics (matrices)
			
			dynamic->setPosition(q);
			dynamic->calculateMassMatrix();
			dynamic->setPosition(q);
			dynamic->setVelocity(qd);
			dynamic->calculateCentrifugalCoriolis();
			dynamic->setPosition(q);
			dynamic->calculateGravity();
			
			rl::math::Vector tauMatrices = dynamic->getMassMatrix() * qdd + dynamic->getCentrifugalCoriolis() + dynamic->getGravity();
			
			if (!tauMatrices.isApprox(tauRecursive))
			{
				std::cerr << "q = " << q.transpose() << std::endl;
				std::cerr << "qd = " << qd.transpose() << std::endl;
				std::cerr << "qdd = " << qdd.transpose() << std::endl;
				std::cerr << "tau (recursive) = " << tauRecursive.transpose() << std::endl;
				std::cerr << "tau (matrices) = " << tauMatrices.transpose() << std::endl;
				return EXIT_FAILURE;
			}
			
			// forward dynamics (recursive)
			
			dynamic->setPosition(q);
			dynamic->setVelocity(qd);
			dynamic->setTorque(tauRecursive);
			dynamic->forwardDynamics();
			
			rl::math::Vector qddRecursive = dynamic->getAcceleration();
			
			if (!qddRecursive.isApprox(qdd))
			{
				std::cerr << "q = " << q.transpose() << std::endl;
				std::cerr << "qd = " << qd.transpose() << std::endl;
				std::cerr << "qdd = " << qdd.transpose() << std::endl;
				std::cerr << "tau (recursive) = " << tauRecursive.transpose() << std::endl;
				std::cerr << "tau (matrices) = " << tauMatrices.transpose() << std::endl;
				std::cerr << "qdd (recursive) = " << qddRecursive.transpose() << std::endl;
				return EXIT_FAILURE;
			}
			
			// forward dynamics (matrices)
			
			dynamic->setPosition(q);
			dynamic->calculateMassMatrixInverse();
			dynamic->setPosition(q);
			dynamic->setVelocity(qd);
			dynamic->calculateCentrifugalCoriolis();
			dynamic->setPosition(q);
			dynamic->calculateGravity();
			
			rl::math::Vector qddMatrices = dynamic->getMassMatrixInverse() * (tauMatrices - dynamic->getCentrifugalCoriolis() - dynamic->getGravity());
			
			if (!qddMatrices.isApprox(qddRecursive))
			{
				std::cerr << "q = " << q.transpose() << std::endl;
				std::cerr << "qd = " << qd.transpose() << std::endl;
				std::cerr << "qdd = " << qdd.transpose() << std::endl;
				std::cerr << "tau (recursive) = " << tauRecursive.transpose() << std::endl;
				std::cerr << "tau (matrices) = " << tauMatrices.transpose() << std::endl;
				std::cerr << "qdd (recursive) = " << qddRecursive.transpose() << std::endl;
				std::cerr << "qdd (matrices) = " << qddMatrices.transpose() << std::endl;
				return EXIT_FAILURE;
			}
		}
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
