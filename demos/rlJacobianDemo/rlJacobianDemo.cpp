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
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/XmlFactory.h>

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlJacobianDemo KINEMATICSFILE Q1 ... Qn QD1 ... QDn" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		std::string filename(argv[1]);
		std::shared_ptr<rl::mdl::Kinematic> kinematic;
		
		if ("urdf" == filename.substr(filename.length() - 4, 4))
		{
			rl::mdl::UrdfFactory factory;
			kinematic = std::dynamic_pointer_cast<rl::mdl::Kinematic>(factory.create(filename));
		}
		else
		{
			rl::mdl::XmlFactory factory;
			kinematic = std::dynamic_pointer_cast<rl::mdl::Kinematic>(factory.create(filename));
		}
		
		rl::math::Vector q(kinematic->getDofPosition());
		
		for (std::ptrdiff_t i = 0; i < q.size(); ++i)
		{
			q(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2]);
		}
		
		rl::math::Vector qdot(kinematic->getDof());
		
		for (std::ptrdiff_t i = 0; i < qdot.size(); ++i)
		{
			qdot(i) = boost::lexical_cast<rl::math::Real>(argv[q.size() + i + 2]);
		}
		
		kinematic->setPosition(q);
		kinematic->forwardPosition();
		kinematic->calculateJacobian();
		
		std::cout << "J=" << std::endl << kinematic->getJacobian() << std::endl;
		
		rl::math::Vector xdot = kinematic->getJacobian() * qdot;
		
		std::cout << "xdot=" << std::endl;
		
		for (std::size_t i = 0; i < kinematic->getOperationalDof(); ++i)
		{
			std::cout << i << " " << xdot.transpose() << std::endl;
		}
		
		kinematic->calculateJacobianInverse();
		
		std::cout << "invJ=" << std::endl << kinematic->getJacobianInverse() << std::endl;
		
		rl::math::Vector qdot2 = kinematic->getJacobianInverse() * xdot;
		
		std::cout << "qdot=" << std::endl << qdot2.transpose() << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
