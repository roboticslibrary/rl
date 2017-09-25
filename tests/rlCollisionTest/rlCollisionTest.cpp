//
// Copyright (c) 2009, Andre Gaschler
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
#include <boost/lexical_cast.hpp>

#include <rl/mdl/Kinematic.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/sg/Body.h>
#include <rl/sg/Model.h>
#include <rl/sg/SimpleScene.h>
#ifdef RL_SG_BULLET
#include <rl/sg/bullet/Scene.h>
#endif // RL_SG_BULLET
#ifdef RL_SG_FCL
#include <rl/sg/fcl/Scene.h>
#endif // RL_SG_FCL
#ifdef RL_SG_ODE
#include <rl/sg/ode/Scene.h>
#endif // RL_SG_ODE
#ifdef RL_SG_PQP
#include <rl/sg/pqp/Scene.h>
#endif // RL_SG_PQP
#ifdef RL_SG_SOLID
#include <rl/sg/solid/Scene.h>
#endif // RL_SG_SOLID

// mdl/sg-only version
bool
collides(rl::sg::SimpleScene* scene, const rl::mdl::Kinematic* kinematic)
{
	::rl::sg::Model* robotModel = scene->getModel(0);
	
	for (::std::size_t i = 0; i < robotModel->getNumBodies(); ++i)
	{
		if (kinematic->isColliding(i))
		{
			for (::rl::sg::Scene::Iterator j = scene->begin(); j != scene->end(); ++j)
			{
				if (robotModel != *j)
				{
					for (::rl::sg::Model::Iterator k = (*j)->begin(); k != (*j)->end(); ++k)
					{
						if (dynamic_cast< ::rl::sg::SimpleScene*>(scene)->areColliding(robotModel->getBody(i), *k))
						{
							return true;
						}
					}
				}
			}
		}
	
		for (::std::size_t j = 0; j < i; ++j)
		{
			if (kinematic->areColliding(i, j))
			{
				if (dynamic_cast< ::rl::sg::SimpleScene*>(scene)->areColliding(robotModel->getBody(i), robotModel->getBody(j)))
				{
					return true;
				}
			}
		}
	}
	
	return false;
}

int
main(int argc, char** argv)
{
	std::cout.precision(15);
	std::cerr.precision(15);
	
	if (argc < 4)
	{
		std::cout << "Usage: rlCollisionTest SCENEFILE KINEMATICSFILE JOINT1 ... JOINTn (in degrees)" << std::endl;
		return EXIT_FAILURE;
	}
	
	rl::mdl::XmlFactory factory;
	std::shared_ptr<rl::mdl::Model> model(factory.create(argv[2]));
	rl::mdl::Kinematic* kinematics = dynamic_cast<rl::mdl::Kinematic*>(model.get());
	
	std::size_t dof = kinematics->getDof();
	rl::math::Vector q(kinematics->getDof());

	for (std::size_t i = 0; i < kinematics->getDof(); ++i)
	{
		q(i) = boost::lexical_cast<rl::math::Real>(argv[i + 3]) * rl::math::DEG2RAD;
	}
	
	std::cout << "Set joint angle [rad] " << q.transpose() << std::endl;
	
	kinematics->setPosition(q);
	kinematics->forwardPosition();
	
	std::vector<rl::sg::SimpleScene*> scenes;
	std::vector<std::string> sceneNames;
	
#ifdef RL_SG_BULLET
	scenes.push_back(new rl::sg::bullet::Scene);
	sceneNames.push_back("bullet");
#endif // RL_SG_BULLET
#ifdef RL_SG_FCL
	scenes.push_back(new rl::sg::fcl::Scene);
	sceneNames.push_back("fcl");
#endif // RL_SG_FCL
#ifdef RL_SG_ODE
	scenes.push_back(new rl::sg::ode::Scene);
	sceneNames.push_back("ode");
#endif // RL_SG_ODE
#ifdef RL_SG_PQP
	scenes.push_back(new rl::sg::pqp::Scene);
	sceneNames.push_back("pqp");
#endif // RL_SG_PQP
#ifdef RL_SG_SOLID
	scenes.push_back(new rl::sg::solid::Scene);
	sceneNames.push_back("solid");
#endif // RL_SG_SOLID
	
	for (std::size_t i = 0; i < scenes.size(); ++i)
	{
		std::cout << "Loading " << sceneNames[i] << " scene" << std::endl;
		scenes[i]->load(argv[1]);
		assert(scenes[i]->getModel(0)->getNumBodies() == kinematics->getBodies());
		for (std::size_t b = 0; b < kinematics->getBodies(); ++b)
		{
			scenes[i]->getModel(0)->getBody(b)->setFrame(kinematics->getFrame(b));
		}
	}
	
	std::cout << "Loading done." << std::endl;
	
	for (std::size_t i = 0; i < scenes.size(); ++i)
	{
		std::cout << "Testing SimpleScene::isColliding() in " << sceneNames[i] << ": "; 
		std::cout << (collides(scenes[i], kinematics) ? "true" : "false") << std::endl;
	}
	
	std::mt19937 randomGenerator(0);
	std::uniform_real_distribution<rl::math::Real> randomDistribution(-180 * rl::math::DEG2RAD, 180 * rl::math::DEG2RAD);
		
	std::size_t j;
	for (j = 0; j < 10; ++j)
	{
		rl::math::Vector q(dof);
		
		for (std::size_t i = 0; i < dof; ++i)
		{
			q(i) = randomDistribution(randomGenerator);
		}
		
		kinematics->setPosition(q);
		kinematics->forwardPosition();
		
		rl::math::Vector results(scenes.size());
		
		for (std::size_t i = 0; i < scenes.size(); ++i)
		{
			for (std::size_t b = 0; b < kinematics->getBodies(); ++b)
			{
				scenes[i]->getModel(0)->getBody(b)->setFrame(kinematics->getFrame(b));
			}
			results[i] = collides(scenes[i], kinematics);
		}
		
		if ((results.array() == false).any() && (results.array() == true).any())
		{
			std::cerr << "Error: Counterexample " << j << ": SimpleScene::isColliding() Joint angle [rad] " << q.transpose();
			std::cerr << " [deg] " << q.transpose() * rl::math::RAD2DEG << " ";
			for (std::size_t i = 0; i < scenes.size(); ++i)
			{
				std::cerr << "  " << sceneNames[i] << "=" << results[i];
			}
			std::cerr << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << "Tested " << j << " random poses, no differences between the collision libraries." << std::endl;
	
	return EXIT_SUCCESS;
}
