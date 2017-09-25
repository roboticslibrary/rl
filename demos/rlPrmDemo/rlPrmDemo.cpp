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
#include <rl/plan/KdtreeNearestNeighbors.h>
#include <rl/plan/Prm.h>
#include <rl/plan/RecursiveVerifier.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/SimpleOptimizer.h>
#include <rl/plan/UniformSampler.h>
#include <rl/sg/solid/Model.h>
#include <rl/sg/solid/Scene.h>

int
main(int argc, char** argv)
{
	if (argc < 3)
	{
		std::cout << "Usage: rlPrmDemo SCENEFILE KINEMATICSFILE START1 ... STARTn GOAL1 ... GOALn" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		rl::sg::solid::Scene scene;
		scene.load(argv[1]);
		
		std::shared_ptr<rl::kin::Kinematics> kinematics(rl::kin::Kinematics::create(argv[2]));
		
		rl::plan::SimpleModel model;
		model.kin = kinematics.get();
		model.model = scene.getModel(0);
		model.scene = &scene;
		
		rl::plan::KdtreeNearestNeighbors nearestNeighbors(&model);
		rl::plan::Prm planner;
		rl::plan::UniformSampler sampler;
		rl::plan::RecursiveVerifier verifier;
		
		planner.model = &model;
		planner.setNearestNeighbors(&nearestNeighbors);
		planner.sampler = &sampler;
		planner.verifier = &verifier;
		
		sampler.model = &model;
		
		verifier.delta = 1.0f * rl::math::DEG2RAD;
		verifier.model = &model;
		
		rl::math::Vector start(kinematics->getDof());
		
		for (std::size_t i = 0; i < kinematics->getDof(); ++i)
		{
			start(i) = boost::lexical_cast<rl::math::Real>(argv[i + 3]);
		}
		
		planner.start = &start;
		
		rl::math::Vector goal(kinematics->getDof());
		
		for (std::size_t i = 0; i < kinematics->getDof(); ++i)
		{
			goal(i) = boost::lexical_cast<rl::math::Real>(argv[kinematics->getDof() + i + 3]);
		}
		
		planner.goal = &goal;
		
		std::cout << "construct() ... " << std::endl;;
		std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
		planner.construct(15);
		std::chrono::steady_clock::time_point stopTime = std::chrono::steady_clock::now();
		std::cout << "construct() " << std::chrono::duration_cast<std::chrono::duration<double>>(stopTime - startTime).count() * 1000 << " ms" << std::endl;
		
		std::cout << "solve() ... " << std::endl;;
		startTime = std::chrono::steady_clock::now();
		bool solved = planner.solve();
		stopTime = std::chrono::steady_clock::now();
		std::cout << "solve() " << (solved ? "true" : "false") << " " << std::chrono::duration_cast<std::chrono::duration<double>>(stopTime - startTime).count() * 1000 << " ms" << std::endl;
		
		return EXIT_SUCCESS;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}
