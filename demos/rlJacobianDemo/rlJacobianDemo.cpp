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
#include <rl/kin/Kinematics.h>
#include <rl/math/Unit.h>

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
		std::shared_ptr<rl::kin::Kinematics> kinematics(rl::kin::Kinematics::create(argv[1]));
		
		rl::math::Vector q(kinematics->getDof());
		
		for (std::ptrdiff_t i = 0; i < q.size(); ++i)
		{
			q(i) = boost::lexical_cast<rl::math::Real>(argv[i + 2]);
		}
		
		rl::math::Vector qdot(kinematics->getDof());
		
		for (std::ptrdiff_t i = 0; i < qdot.size(); ++i)
		{
			qdot(i) = boost::lexical_cast<rl::math::Real>(argv[q.size() + i + 2]);
		}
		
		kinematics->setPosition(q);
		kinematics->updateFrames();
		kinematics->updateJacobian();
		
		std::cout << "J=" << std::endl << kinematics->getJacobian() << std::endl;
		
		rl::math::Vector xdot(kinematics->getOperationalDof() * 6);
		
		kinematics->forwardVelocity(qdot, xdot);
		
		std::cout << "xdot=" << std::endl;
		
		for (std::size_t i = 0; i < kinematics->getOperationalDof(); ++i)
		{
			std::cout << i << " " << xdot.transpose() << std::endl;
		}
		
		kinematics->updateJacobianInverse(1.0e-9f);
		
		std::cout << "invJ=" << std::endl << kinematics->getJacobianInverse() << std::endl;
		
		kinematics->inverseVelocity(xdot, qdot);
		
		std::cout << "qdot=" << std::endl << qdot.transpose() << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
