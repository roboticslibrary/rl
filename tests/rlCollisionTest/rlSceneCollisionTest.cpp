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

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlSceneCollisionTest SCENEFILE [shouldCollide]" << std::endl;
		return EXIT_FAILURE;
	}
	
	bool shouldCollide = (argc > 2);
	
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
	}
	
	std::cout << "Loading done." << std::endl;
	
	int errorlevel = EXIT_SUCCESS;
	std::cout << "Now testing, expect isColliding()=" << shouldCollide << std::endl;
	
	for (std::size_t i = 0; i < scenes.size(); ++i)
	{
		if (scenes[i]->isColliding() ^ shouldCollide)
		{
			std::cerr << "Error: " << sceneNames[i] << " reports SimpleScene::isColliding()=" << scenes[i]->isColliding() << std::endl;
			errorlevel = EXIT_FAILURE;
		}
	}
	
	std::cout << "Testing done." << std::endl;
	
	return errorlevel;
}
