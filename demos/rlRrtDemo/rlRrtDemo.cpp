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
#include <rl/math/Constants.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/plan/KdtreeNearestNeighbors.h>
#include <rl/plan/RrtConCon.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/UniformSampler.h>
#include <rl/sg/UrdfFactory.h>
#include <rl/sg/XmlFactory.h>

#if defined(RL_SG_SOLID)
#include <rl/sg/solid/Model.h>
#include <rl/sg/solid/Scene.h>
#elif defined(RL_SG_BULLET)
#include <rl/sg/bullet/Model.h>
#include <rl/sg/bullet/Scene.h>
#elif defined(RL_SG_ODE)
#include <rl/sg/ode/Model.h>
#include <rl/sg/ode/Scene.h>
#endif

int
main(int argc, char** argv)
{
	if (argc < 3)
	{
		std::cout << "Usage: rlRrtDemo SCENEFILE KINEMATICSFILE START1 ... STARTn GOAL1 ... GOALn" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		std::string scenefile(argv[1]);
#if defined(RL_SG_SOLID)
		rl::sg::solid::Scene scene;
#elif defined(RL_SG_BULLET)
		rl::sg::bullet::Scene scene;
#elif defined(RL_SG_ODE)
		rl::sg::ode::Scene scene;
#endif
		
		if ("urdf" == scenefile.substr(scenefile.length() - 4, 4))
		{
			rl::sg::UrdfFactory factory;
			factory.load(scenefile, &scene);
		}
		else
		{
			rl::sg::XmlFactory factory;
			factory.load(scenefile, &scene);
		}
		
		std::string kinematicsfile(argv[2]);
		std::shared_ptr<rl::mdl::Kinematic> kinematic;
		
		if ("urdf" == kinematicsfile.substr(kinematicsfile.length() - 4, 4))
		{
			rl::mdl::UrdfFactory factory;
			kinematic = std::dynamic_pointer_cast<rl::mdl::Kinematic>(factory.create(kinematicsfile));
		}
		else
		{
			rl::mdl::XmlFactory factory;
			kinematic = std::dynamic_pointer_cast<rl::mdl::Kinematic>(factory.create(kinematicsfile));
		}
		
		rl::plan::SimpleModel model;
		model.mdl = kinematic.get();
		model.model = scene.getModel(0);
		model.scene = &scene;
		
		rl::plan::KdtreeNearestNeighbors nearestNeighbors0(&model);
		rl::plan::KdtreeNearestNeighbors nearestNeighbors1(&model);
		rl::plan::RrtConCon planner;
		rl::plan::UniformSampler sampler;
		
		planner.model = &model;
		planner.setNearestNeighbors(&nearestNeighbors0, 0);
		planner.setNearestNeighbors(&nearestNeighbors1, 1);
		planner.sampler = &sampler;
		
		sampler.model = &model;
		
		planner.delta = 1 * rl::math::constants::deg2rad;
		
		rl::math::Vector start(kinematic->getDofPosition());
		
		for (std::ptrdiff_t i = 0; i < start.size(); ++i)
		{
			start(i) = boost::lexical_cast<rl::math::Real>(argv[i + 3]) * rl::math::constants::deg2rad;
		}
		
		planner.start = &start;
		
		rl::math::Vector goal(kinematic->getDofPosition());
		
		for (std::ptrdiff_t i = 0; i < goal.size(); ++i)
		{
			goal(i) = boost::lexical_cast<rl::math::Real>(argv[start.size() + i + 3]) * rl::math::constants::deg2rad;
		}
		
		planner.goal = &goal;
		
		std::cout << "start: " << start.transpose() * rl::math::constants::rad2deg << std::endl;;
		std::cout << "goal: " << goal.transpose() * rl::math::constants::rad2deg << std::endl;;
		
		std::cout << "solve() ... " << std::endl;;
		std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
		bool solved = planner.solve();
		std::chrono::steady_clock::time_point stopTime = std::chrono::steady_clock::now();
		std::cout << "solve() " << (solved ? "true" : "false") << " " << std::chrono::duration_cast<std::chrono::duration<double>>(stopTime - startTime).count() * 1000 << " ms" << std::endl;
		
		return EXIT_SUCCESS;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}
