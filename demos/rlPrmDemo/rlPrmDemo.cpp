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
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/math/Unit.h>
#include <rl/plan/Prm.h>
#include <rl/plan/RecursiveVerifier.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/SimpleOptimizer.h>
#include <rl/plan/UniformSampler.h>
#include <rl/sg/solid/Model.h>
#include <rl/sg/solid/Scene.h>
#include <rl/util/Timer.h>

int
main(int argc, char** argv)
{
	if (argc < 3)
	{
		std::cout << "Usage: rlPrmDemo SCENEFILE KINEMATICSFILE START1 ... STARTn GOAL1 ... GOALn" << std::endl;
		return 1;
	}
	
	try
	{
		rl::sg::solid::Scene scene;
		scene.load(argv[1]);
		
		::boost::shared_ptr< ::rl::kin::Kinematics > kinematics(::rl::kin::Kinematics::create(argv[2]));
		
		rl::plan::SimpleModel model;
		model.kin = kinematics.get();
		model.model = scene.getModel(0);
		model.scene = &scene;
		
		rl::plan::Prm planner;
		rl::plan::UniformSampler sampler;
		rl::plan::RecursiveVerifier verifier;
		
		planner.model = &model;
		planner.sampler = &sampler;
		planner.verifier = &verifier;
		
		sampler.model = &model;
		
		verifier.delta = 1.0f * rl::math::DEG2RAD;
		verifier.model = &model;
		
		rl::math::Vector start(kinematics->getDof());
		
		for (std::size_t i = 0; i < kinematics->getDof(); ++i)
		{
			start(i) = boost::lexical_cast< rl::math::Real >(argv[i + 3]);
		}
		
		planner.start = &start;
		
		rl::math::Vector goal(kinematics->getDof());
		
		for (std::size_t i = 0; i < kinematics->getDof(); ++i)
		{
			goal(i) = boost::lexical_cast< rl::math::Real >(argv[kinematics->getDof() + i + 3]);
		}
		
		planner.goal = &goal;
		
		rl::util::Timer timer;
		
		std::cout << "construct() ... " << std::endl;;
		timer.start();
		planner.construct(15);
		timer.stop();
		std::cout << "construct() " << timer.elapsed() * 1000.0f << " ms" << std::endl;
		
		std::cout << "solve() ... " << std::endl;;
		timer.start();
		bool solved = planner.solve();
		timer.stop();
		std::cout << "solve() " << (solved ? "true" : "false") << " " << timer.elapsed() * 1000.0f << " ms" << std::endl;
		
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return -1;
	}
}
