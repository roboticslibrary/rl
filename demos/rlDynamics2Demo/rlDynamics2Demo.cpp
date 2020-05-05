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
#include <rl/mdl/UrdfFactory.h>
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
		std::string filename(argv[1]);
		std::shared_ptr<rl::mdl::Dynamic> dynamic;
		
		if ("urdf" == filename.substr(filename.length() - 4, 4))
		{
			rl::mdl::UrdfFactory factory;
			dynamic = std::dynamic_pointer_cast<rl::mdl::Dynamic>(factory.create(filename));
		}
		else
		{
			rl::mdl::XmlFactory factory;
			dynamic = std::dynamic_pointer_cast<rl::mdl::Dynamic>(factory.create(filename));
		}
		
		rl::math::Vector q(dynamic->getDofPosition());
		rl::math::Vector qd(dynamic->getDof());
		rl::math::Vector qdd(dynamic->getDof());
		
		for (std::size_t i = 0; i < dynamic->getDofPosition(); ++i)
		{
			q(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2]);
		}
		
		for (std::size_t i = 0; i < dynamic->getDof(); ++i)
		{
			qd(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2 + dynamic->getDofPosition()]);
			qdd(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2 + dynamic->getDofPosition() + dynamic->getDof()]);
		}
		
		std::cout << "===============================================================================" << std::endl;
		
		// forward position
		
		dynamic->setPosition(q);
		dynamic->forwardPosition();
		const rl::math::Transform::ConstTranslationPart& position = dynamic->getOperationalPosition(0).translation();
		rl::math::Vector3 orientation = dynamic->getOperationalPosition(0).rotation().eulerAngles(2, 1, 0).reverse();
		std::cout << "x = " << position.x() << " m, y = " << position.y() << " m, z = " << position.z() << " m, a = " << orientation.x() * rl::math::RAD2DEG << " deg, b = " << orientation.y() * rl::math::RAD2DEG << " deg, c = " << orientation.z() * rl::math::RAD2DEG << " deg" << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
		
		// forward velocity
		
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->forwardVelocity();
		std::cout << "xd = " << dynamic->getOperationalVelocity(0).linear().transpose() << " " << dynamic->getOperationalVelocity(0).angular().transpose() << std::endl;
		
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		
		// J
		
		rl::math::Matrix J(6 * dynamic->getOperationalDof(), dynamic->getDof());
		dynamic->calculateJacobian(J, false);
		std::cout << "J = " << std::endl << J << std::endl;
		
		// J * qd
		
		rl::math::Vector Jqd = J * qd;
		std::cout << "xd = J * qd = " << Jqd.transpose() << std::endl;
		
		// J^{-1}
		
		rl::math::Matrix invJ(dynamic->getDof(), 6 * dynamic->getOperationalDof());
		dynamic->calculateJacobianInverse(J, invJ);
		std::cout << "J^{-1} = " << std::endl << invJ << std::endl;
		
		// J^{-1} * xd
		
		rl::math::Vector invJxd = invJ * Jqd;
		std::cout << "qd = J^{-1} * xd = " << invJxd.transpose() << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
		
		// forward acceleration
		
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->setAcceleration(qdd);
		dynamic->forwardVelocity();
		dynamic->forwardAcceleration();
		std::cout << "xdd = " << dynamic->getOperationalAcceleration(0).linear().transpose() << " " << dynamic->getOperationalAcceleration(0).angular().transpose() << std::endl;
		
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		
		// Jd * qd
		
		rl::math::Vector Jdqd(6 * dynamic->getOperationalDof());
		dynamic->calculateJacobianDerivative(Jdqd, false);
		std::cout << "Jd * qd = " << Jdqd.transpose() << std::endl;
		
		// J * qdd + Jd * qd
		
		rl::math::Vector JqddJdqd = J * qdd + Jdqd;
		std::cout << "xdd = J * qdd + Jd * qd = " << JqddJdqd.transpose() << std::endl;
		
		// J^{-1} * (xdd - Jd * qd)
		
		rl::math::Vector invJxddJdqd = invJ * (JqddJdqd - Jdqd);
		std::cout << "qdd = J^{-1} * (xdd - Jd * qd) = " << invJxddJdqd.transpose() << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
		
		// inverse dynamics
		
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
		dynamic->calculateGravity(G);
		std::cout << "G = " << G.transpose() << std::endl;
		
		// M * qdd + V + G
		
		rl::math::Vector MqddVG = M * qdd + V + G;
		std::cout << "tau = M * qdd + V + G = " << MqddVG.transpose() << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
		
		// forward dynamics
		
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
		
		// M^{-1} * (tau - V - G)
		
		rl::math::Vector invMtauVG = invM * (tau - V - G);
		std::cout << "qdd = M^{-1} * (tau - V - G) = " << invMtauVG.transpose() << std::endl;
		
		std::cout << "===============================================================================" << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
