//
// Copyright (c) 2019, Markus Rickert
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
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/XmlFactory.h>

#ifdef RL_MDL_NLOPT
#include <rl/mdl/NloptInverseKinematics.h>
#endif

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
		std::string filename = argv[1];
		
		rl::mdl::XmlFactory factory;
		std::shared_ptr<rl::mdl::Kinematic> kinematics = std::dynamic_pointer_cast<rl::mdl::Kinematic>(factory.create(filename));
		kinematics->seed(0);
		
		std::vector<std::pair<std::shared_ptr<rl::mdl::IterativeInverseKinematics>, std::string>> ik;
		
		std::shared_ptr<rl::mdl::JacobianInverseKinematics> jacobianSvd = std::make_shared<rl::mdl::JacobianInverseKinematics>(kinematics.get());
		jacobianSvd->seed(0);
		jacobianSvd->setMethod(rl::mdl::JacobianInverseKinematics::Method::svd);
		ik.push_back(std::make_pair(jacobianSvd, "rl::mdl::JacobianInverseKinematics::Method::svd"));
		
		std::shared_ptr<rl::mdl::JacobianInverseKinematics> jacobianDls = std::make_shared<rl::mdl::JacobianInverseKinematics>(kinematics.get());
		jacobianDls->seed(0);
		jacobianDls->setMethod(rl::mdl::JacobianInverseKinematics::Method::dls);
		ik.push_back(std::make_pair(jacobianDls, "rl::mdl::JacobianInverseKinematics::Method::dls"));
		
		std::shared_ptr<rl::mdl::JacobianInverseKinematics> jacobianTranspose = std::make_shared<rl::mdl::JacobianInverseKinematics>(kinematics.get());
		jacobianTranspose->seed(0);
		jacobianTranspose->setMethod(rl::mdl::JacobianInverseKinematics::Method::transpose);
		ik.push_back(std::make_pair(jacobianTranspose, "rl::mdl::JacobianInverseKinematics::Method::transpose"));
		
#ifdef RL_MDL_NLOPT
		std::shared_ptr<rl::mdl::NloptInverseKinematics> nlopt = std::make_shared<rl::mdl::NloptInverseKinematics>(kinematics.get());
		nlopt->seed(0);
		ik.push_back(std::make_pair(nlopt, "rl::mdl::NloptInverseKinematics"));
#endif
		
		for (std::size_t i = 0; i < ik.size(); ++i)
		{
			ik[i].first->setDuration(std::chrono::nanoseconds::max());
			ik[i].first->setIterations(100000);
			
			for (std::size_t n = 0; n < 100; ++n)
			{
				rl::math::Vector q1 = kinematics->generatePositionUniform();
				kinematics->setPosition(q1);
				kinematics->forwardPosition();
				rl::math::Transform t1 = kinematics->getOperationalPosition(0);
				
				rl::math::Vector q2 = kinematics->generatePositionUniform();
				kinematics->setPosition(q2);
				ik[i].first->clearGoals();
				ik[i].first->addGoal(t1, 0);
				
				if (!ik[i].first->solve())
				{
					std::cerr << ik[i].second << " on file " << filename << " with no solution." << std::endl;
					std::cerr << "t1 = " << std::endl << t1.matrix() << std::endl;
					std::cerr << "q1 = " << q1.transpose() << std::endl;
					std::cerr << "q2 = " << q2.transpose() << std::endl;
					return EXIT_FAILURE;
				}
				
				rl::math::Vector q3 = kinematics->getPosition();
				kinematics->forwardPosition();
				rl::math::Transform t3 = kinematics->getOperationalPosition(0);
				
				if (t3.toDelta(t1).squaredNorm() > std::pow(ik[i].first->getEpsilon(), 2))
				{
					std::cerr << ik[i].second << " on file " << filename << " with incorrect operational position." << std::endl;
					std::cerr << "t3.toDelta(t1).squaredNorm() = " << t3.toDelta(t1).squaredNorm() << std::endl;
					std::cerr << "t3.toDelta(t1) = " << t3.toDelta(t1).transpose() << std::endl;
					std::cerr << "t1 = " << std::endl << t1.matrix() << std::endl;
					std::cerr << "t3 = " << std::endl << t3.matrix() << std::endl;
					std::cerr << "q1 = " << q1.transpose() << std::endl;
					std::cerr << "q2 = " << q2.transpose() << std::endl;
					std::cerr << "q3 = " << q3.transpose() << std::endl;
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
