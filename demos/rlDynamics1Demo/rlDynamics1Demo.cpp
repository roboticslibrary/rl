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
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlDynamics1Demo MODELFILE Q1 ... Qn QD1 ... QDn QDD1 ... QDDn" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		rl::mdl::XmlFactory factory;
		std::shared_ptr<rl::mdl::Model> model(factory.create(argv[1]));
		
		rl::mdl::Dynamic* dynamic = dynamic_cast<rl::mdl::Dynamic*>(model.get());
		
		rl::math::Vector q(model->getDof());
		rl::math::Vector qd(model->getDof());
		rl::math::Vector qdd(model->getDof());
		
		for (std::size_t i = 0; i < model->getDof(); ++i)
		{
			q(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2]);
			qd(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2 + model->getDof()]);
			qdd(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2 + 2 * model->getDof()]);
		}
		
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->setAcceleration(qdd);
		
		dynamic->inverseDynamics();
		std::cout << "tau = " << dynamic->getTorque().transpose() << std::endl;
		
		dynamic->forwardDynamics();
		std::cout << "qdd = " << dynamic->getAcceleration().transpose() << std::endl;
		
		dynamic->rungeKuttaNystrom(1);
		std::cout << "q = " << dynamic->getPosition().transpose() << std::endl;
		std::cout << "qd = " << dynamic->getVelocity().transpose() << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
