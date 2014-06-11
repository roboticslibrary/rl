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
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/math/Unit.h>
#include <rl/plan/Prm.h>
#include <rl/plan/RecursiveVerifier.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/SimpleOptimizer.h>
#include <rl/plan/UniformSampler.h>
#include <rl/sg/Model.h>
#include <rl/util/Timer.h>

#ifdef RL_SG_HAVE_BULLET
#include <rl/sg/bullet/Scene.h>
#endif // RL_SG_HAVE_BULLET
#ifdef RL_SG_HAVE_ODE
#include <rl/sg/ode/Scene.h>
#endif // RL_SG_HAVE_ODE
#ifdef RL_SG_HAVE_PQP
#include <rl/sg/pqp/Scene.h>
#endif // RL_SG_HAVE_PQP
#ifdef RL_SG_HAVE_SOLID
#include <rl/sg/solid/Scene.h>
#endif // RL_SG_HAVE_SOLID

int
main(int argc, char** argv)
{
	if (argc < 6)
	{
		std::cout << "Usage: rlPrmTest ENGINE SCENEFILE KINEMATICSFILE EXPECTED_NUM_VERTICES EXPECTED_NUM_EDGES START1 ... STARTn GOAL1 ... GOALn" << std::endl;
		return 1;
	}
	
	try
	{
		boost::shared_ptr< rl::sg::Scene > scene;
		
#ifdef RL_SG_HAVE_BULLET
		if ("bullet" == std::string(argv[1]))
		{
			scene = boost::make_shared< rl::sg::bullet::Scene >();
		}
#endif // RL_SG_HAVE_BULLET
#ifdef RL_SG_HAVE_ODE
		if ("ode" == std::string(argv[1]))
		{
			scene = boost::make_shared< rl::sg::ode::Scene >();
		}
#endif // RL_SG_HAVE_ODE
#ifdef RL_SG_HAVE_PQP
		if ("pqp" == std::string(argv[1]))
		{
			scene = boost::make_shared< rl::sg::pqp::Scene >();
		}
#endif // RL_SG_HAVE_PQP
#ifdef RL_SG_HAVE_SOLID
		if ("solid" == std::string(argv[1]))
		{
			scene = boost::make_shared< rl::sg::solid::Scene >();
		}
#endif // RL_SG_HAVE_SOLID
		
		scene->load(argv[2]);
		
		boost::shared_ptr< rl::kin::Kinematics > kinematics(rl::kin::Kinematics::create(argv[3]));
		
		rl::plan::SimpleModel model;
		model.kin = kinematics.get();
		model.model = scene->getModel(0);
		model.scene = scene.get();
		
		rl::plan::Prm planner;
		rl::plan::UniformSampler sampler;
		rl::plan::RecursiveVerifier verifier;
		
		sampler.seed(0);
		
		planner.model = &model;
		planner.sampler = &sampler;
		planner.verifier = &verifier;
		
		sampler.model = &model;
		
		verifier.delta = 1.0f * rl::math::DEG2RAD;
		verifier.model = &model;
		
		rl::math::Vector start(kinematics->getDof());
		
		for (std::size_t i = 0; i < kinematics->getDof(); ++i)
		{
			start(i) = boost::lexical_cast< rl::math::Real >(argv[i + 6]) * rl::math::DEG2RAD;
		}
		
		planner.start = &start;
		
		rl::math::Vector goal(kinematics->getDof());
		
		for (std::size_t i = 0; i < kinematics->getDof(); ++i)
		{
			goal(i) = boost::lexical_cast< rl::math::Real >(argv[kinematics->getDof() + i + 6]) * rl::math::DEG2RAD;
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
		
		std::cout << "NumVertices: " << planner.getNumVertices() << "  NumEdges: " << planner.getNumEdges() << std::endl;
		
		if (solved)
		{
			if (boost::lexical_cast< rl::math::Real >(argv[4]) == planner.getNumVertices() &&
				boost::lexical_cast< rl::math::Real >(argv[5]) == planner.getNumEdges())
			{
				return 0;
			}
			else
			{
				std::cerr << "NumVertices and NumEdges are not as expected for this test case.";
				return 1;
			}
		}
		
		return 1;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return 1;
	}
}
