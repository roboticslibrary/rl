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

#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/XmlFactory.h>

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlJacobianTest KINEMATICFILE" << std::endl;
		return EXIT_FAILURE;
	}
	
	rl::mdl::XmlFactory factory;
	std::shared_ptr<rl::mdl::Model> model(factory.create(argv[1]));
	rl::mdl::Kinematic* kinematics = dynamic_cast<rl::mdl::Kinematic*>(model.get());
	
	std::size_t dof = kinematics->getDof();
	std::srand(0); // get reproducible results
	
	for (std::size_t n = 0; n < 5; ++n)
	{					
		rl::math::Vector q(dof);
		
		for (std::size_t i = 0; i < dof; ++i)
		{
			rl::math::Real r = static_cast<rl::math::Real>(std::rand()) / static_cast<rl::math::Real>(RAND_MAX); 
			q(i) = (r - 0.5) * 360 * rl::math::DEG2RAD;
		}
		
		kinematics->setPosition(q);
		kinematics->forwardPosition();
		kinematics->calculateJacobian();
		rl::math::Matrix jacobianAlgebraic = kinematics->getJacobian();
		
		// Compute Jacobian with central differences purely using forwardPosition
		rl::math::Matrix jacobianNumeric(6, dof);
		rl::math::Real big_eps = 1e-4;
		rl::math::Matrix eps_matrix = rl::math::Matrix66::Identity(dof, dof) * big_eps;
		for (std::size_t i = 0; i < dof; ++i)
		{
			kinematics->setPosition(q + eps_matrix.col(i));
			kinematics->forwardPosition();
			rl::math::Transform t_plus = kinematics->getOperationalPosition(0);
			kinematics->setPosition(q - eps_matrix.col(i));
			kinematics->forwardPosition();
			rl::math::Transform t_minus = kinematics->getOperationalPosition(0);
			rl::math::Vector6 delta_i = t_minus.toDelta(t_plus);
			jacobianNumeric.col(i) = delta_i / 2 / big_eps;
		}
		
		if ((jacobianAlgebraic - jacobianNumeric).norm() > big_eps)
		{
			std::cerr << "jacobianNumeric differs from jacobianAlgebraic, one of them must be wrong." << std::endl;
			std::cerr << " q [rad]: " << q.transpose() << std::endl;
			std::cerr << " jacobianAlgebraic: " << std::endl << jacobianAlgebraic.matrix() << std::endl;
			std::cerr << " jacobianNumeric: " << std::endl << jacobianNumeric.matrix() << std::endl;
		
			return EXIT_FAILURE;
		}
		
	}
	
	return EXIT_SUCCESS;
}
