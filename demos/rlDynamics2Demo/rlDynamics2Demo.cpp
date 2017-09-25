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
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <rl/math/Unit.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlDynamics2Demo MODELFILE Q1 ... Qn QD1 ... QDn QDD1 ... QDDn" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		rl::mdl::XmlFactory factory;
		std::shared_ptr<rl::mdl::Model> model(factory.create(argv[1]));
		
		rl::mdl::Dynamic* dynamic = dynamic_cast<rl::mdl::Dynamic*>(model.get());
		
		rl::math::Vector q(dynamic->getDof());
		rl::math::Vector qd(dynamic->getDof());
		rl::math::Vector qdd(dynamic->getDof());
		
		for (std::size_t i = 0; i < dynamic->getDof(); ++i)
		{
			q(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2]);
			qd(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2 + dynamic->getDof()]);
			qdd(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2 + 2 * dynamic->getDof()]);
		}
		
		rl::math::Vector g(3);
		dynamic->getWorldGravity(g);
		
		rl::math::Vector tmp(dynamic->getDof());
		rl::math::Vector tmp2(6 * dynamic->getOperationalDof());
		
		std::cout << "===============================================================================" << std::endl;
		
		// FP
		
		dynamic->setPosition(q);
		dynamic->forwardPosition();
		const rl::math::Transform::ConstTranslationPart& position = dynamic->getOperationalPosition(0).translation();
		rl::math::Vector3 orientation = dynamic->getOperationalPosition(0).rotation().eulerAngles(2, 1, 0).reverse();
		std::cout << "x = " << position.x() << " m, y = " << position.y() << " m, z = " << position.z() << " m, a = " << orientation.x() * rl::math::RAD2DEG << " deg, b = " << orientation.y() * rl::math::RAD2DEG << " deg, c = " << orientation.z() * rl::math::RAD2DEG << " deg" << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
		
		// FV
		
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->forwardVelocity();
		std::cout << "xd = " << dynamic->getOperationalVelocity(0).linear().transpose() << " " << dynamic->getOperationalVelocity(0).angular().transpose() << std::endl;
		
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		
		// J
		
		rl::math::Matrix J(6 * dynamic->getOperationalDof(), dynamic->getDof());
		dynamic->calculateJacobian(J);
		std::cout << "J = " << std::endl << J << std::endl;
		
		// J * qd
		
		tmp2 = J * qd;
		std::cout << "xd = " << tmp2.transpose() << std::endl;
		
		// invJ
		
		rl::math::Matrix invJ(dynamic->getDof(), 6 * dynamic->getOperationalDof());
		dynamic->calculateJacobianInverse(J, invJ);
		std::cout << "J^{-1} = " << std::endl << invJ << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
		
		// FA
		
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->setAcceleration(qdd);
		dynamic->setWorldGravity(g);
		dynamic->forwardVelocity();
		dynamic->forwardAcceleration();
		std::cout << "xdd = " << dynamic->getOperationalAcceleration(0).linear().transpose() << " " << dynamic->getOperationalAcceleration(0).angular().transpose() << std::endl;
		
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		
		// Jd * qd
		
		rl::math::Vector Jdqd(6 * dynamic->getOperationalDof());
		dynamic->calculateJacobianDerivative(Jdqd);
		std::cout << "Jd*qd = " << Jdqd.transpose() << std::endl;
		
		// J * qdd + Jd * qd 
		
		tmp2 = J * qdd + Jdqd;
		std::cout << "xdd = " << tmp2.transpose() << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
		
		// RNE
		
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->setAcceleration(qdd);
		dynamic->inverseDynamics();
		std::cout << "tau = " << dynamic->getTorque().transpose() << std::endl;
		
		rl::math::Vector tau = dynamic->getTorque();
		
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		
		// M
		
		rl::math::Matrix M(dynamic->getDof(), dynamic->getDof());
		dynamic->setPosition(q);
		dynamic->calculateMassMatrix(M);
		std::cout << "M = " << std::endl << M << std::endl;
		
		// V
		
		rl::math::Vector V(dynamic->getDof());
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->calculateCentrifugalCoriolis(V);
		std::cout << "V = " << V.transpose() << std::endl;
		
		// G
		
		rl::math::Vector G(dynamic->getDof());
		dynamic->setPosition(q);
		dynamic->setWorldGravity(g);
		dynamic->calculateGravity(G);
		std::cout << "G = " << G.transpose() << std::endl;
		
		// M * qdd + V + G
		
		tmp = M * qdd + V + G;
		std::cout << "tau = " << tmp.transpose() << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
		
		// FD
		
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->setTorque(tau);
		dynamic->forwardDynamics();
		std::cout << "qdd = " << dynamic->getAcceleration().transpose() << std::endl;
		
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		
		// M^{-1}
		
		rl::math::Matrix invM(dynamic->getDof(), dynamic->getDof());
		dynamic->setPosition(q);
		dynamic->calculateMassMatrixInverse(invM);
		std::cout << "M^{-1} = " << std::endl << invM << std::endl;
		
		// V
		
		std::cout << "V = " << V.transpose() << std::endl;
		
		// G
		
		std::cout << "G = " << G.transpose() << std::endl;
		
		// M^{-1} * ( tau - V - G )
		
		tmp = invM * (tau - V - G);
		std::cout << "qdd = " << tmp.transpose() << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
