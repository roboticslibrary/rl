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
#include <rl/mdl/XmlFactory.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/Eet.h>
#include <rl/plan/LinearNearestNeighbors.h>
#include <rl/plan/UniformSampler.h>
#include <rl/plan/WorkspaceSphereExplorer.h>
#include <rl/sg/Model.h>
#include <rl/sg/XmlFactory.h>

#ifdef RL_SG_BULLET
#include <rl/sg/bullet/Scene.h>
#endif // RL_SG_BULLET
#ifdef RL_SG_FCL
#include <rl/sg/fcl/Scene.h>
#endif // RL_SG_FCL
#ifdef RL_SG_PQP
#include <rl/sg/pqp/Scene.h>
#endif // RL_SG_PQP
#ifdef RL_SG_SOLID
#include <rl/sg/solid/Scene.h>
#endif // RL_SG_SOLID

int
main(int argc, char** argv)
{
	if (argc < 6)
	{
		std::cout << "Usage: rlEetTest ENGINE SCENEFILE KINEMATICSFILE EXPECTED_NUM_VERTICES_MAX EXPECTED_NUM_EDGES_MAX START1 ... STARTn GOAL1 ... GOALn" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		std::shared_ptr<rl::sg::Scene> scene;
		
#ifdef RL_SG_BULLET
		if ("bullet" == std::string(argv[1]))
		{
			scene = std::make_shared<rl::sg::bullet::Scene>();
		}
#endif // RL_SG_BULLET
#ifdef RL_SG_FCL
		if ("fcl" == std::string(argv[1]))
		{
			scene = std::make_shared<rl::sg::fcl::Scene>();
		}
#endif // RL_SG_FCL
#ifdef RL_SG_PQP
		if ("pqp" == std::string(argv[1]))
		{
			scene = std::make_shared<rl::sg::pqp::Scene>();
		}
#endif // RL_SG_PQP
#ifdef RL_SG_SOLID
		if ("solid" == std::string(argv[1]))
		{
			scene = std::make_shared<rl::sg::solid::Scene>();
		}
#endif // RL_SG_SOLID
		
		rl::sg::XmlFactory factory1;
		factory1.load(argv[2], scene.get());
		
		rl::mdl::XmlFactory factory2;
		std::shared_ptr<rl::mdl::Kinematic> kinematic = std::dynamic_pointer_cast<rl::mdl::Kinematic>(factory2.create(argv[3]));
		
		Eigen::Matrix<rl::math::Units, Eigen::Dynamic, 1> qUnits = kinematic->getPositionUnits();
		
		rl::math::Vector start(kinematic->getDofPosition());
		
		for (std::ptrdiff_t i = 0; i < start.size(); ++i)
		{
			start(i) = boost::lexical_cast<rl::math::Real>(argv[i + 6]);
			
			if (rl::math::Units::radian == qUnits(i))
			{
				start(i) *= rl::math::constants::deg2rad;
			}
		}
		
		rl::math::Vector goal(kinematic->getDofPosition());
		
		for (std::ptrdiff_t i = 0; i < goal.size(); ++i)
		{
			goal(i) = boost::lexical_cast<rl::math::Real>(argv[start.size() + i + 6]);
			
			if (rl::math::Units::radian == qUnits(i))
			{
				goal(i) *= rl::math::constants::deg2rad;
			}
		}
		
		rl::plan::DistanceModel model;
		model.mdl = kinematic.get();
		model.model = scene->getModel(0);
		model.scene = scene.get();
		
		rl::plan::WorkspaceSphereExplorer explorer;
		rl::plan::Eet::ExplorerSetup explorerSetup;
		rl::plan::LinearNearestNeighbors nearestNeighbors(&model);
		rl::plan::Eet planner;
		rl::plan::UniformSampler sampler;
		
		rl::math::Vector3 explorerGoal;
		rl::math::Vector3 explorerStart;
		
		explorer.seed(0);
		planner.seed(0);
		sampler.seed(0);
		
		explorer.goal = &explorerGoal;
		explorer.greedy = rl::plan::WorkspaceSphereExplorer::Greedy::space;
		explorer.model = &model;
		explorer.radius = 0.025;
		explorer.range = 45;
		explorer.samples = 100;
		explorer.start = &explorerStart;
		
		explorerSetup.goalConfiguration = &goal;
		explorerSetup.goalFrame = -1;
		explorerSetup.startConfiguration = &start;
		explorerSetup.startFrame = -1;
		
		planner.alpha = static_cast<rl::math::Real>(0.01);
		planner.alternativeDistanceComputation = false;
		planner.beta = 0;
		planner.delta = 1 * rl::math::constants::deg2rad;
		planner.distanceWeight = static_cast<rl::math::Real>(0.1);
		planner.epsilon = static_cast<rl::math::Real>(1.0e-9);
		planner.explorers.push_back(&explorer);
		planner.explorersSetup.push_back(explorerSetup);
		planner.gamma = static_cast<rl::math::Real>(1) / static_cast<rl::math::Real>(3);
		planner.goal = &goal;
		planner.goalEpsilon = static_cast<rl::math::Real>(0.1);
		planner.goalEpsilonUseOrientation = false;
		planner.max.x() = 30;
		planner.max.y() = 30;
		planner.max.z() = 2;
		planner.model = &model;
		planner.min.x() = 0;
		planner.min.y() = 0;
		planner.min.z() = 0;
		planner.setNearestNeighbors(&nearestNeighbors, 0);
		planner.sampler = &sampler;
		planner.start = &start;
		
		sampler.model = &model;
		
		std::cout << "verify() ... " << std::endl;;
		bool verified = planner.verify();
		std::cout << "verify() " << (verified ? "true" : "false") << std::endl;
		
		if (!verified)
		{
			return EXIT_FAILURE;
		}
		
		std::cout << "solve() ... " << std::endl;;
		std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
		bool solved = planner.solve();
		std::chrono::steady_clock::time_point stopTime = std::chrono::steady_clock::now();
		std::cout << "solve() " << (solved ? "true" : "false") << " " << std::chrono::duration_cast<std::chrono::duration<double>>(stopTime - startTime).count() * 1000 << " ms" << std::endl;
		
		std::cout << "NumVertices: " << planner.getNumVertices() << "  NumEdges: " << planner.getNumEdges() << std::endl;
		
		if (solved)
		{
			if (boost::lexical_cast<std::size_t>(argv[4]) >= planner.getNumVertices() &&
				boost::lexical_cast<std::size_t>(argv[5]) >= planner.getNumEdges())
			{
				return EXIT_SUCCESS;
			}
			else
			{
				std::cerr << "NumVertices and NumEdges are more than expected for this test case.";
				return EXIT_FAILURE;
			}
		}
		
		return EXIT_FAILURE;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}
