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
#include <rl/math/Unit.h>
#include <rl/mdl/Body.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Fixed.h>
#include <rl/mdl/Frame.h>
#include <rl/mdl/Revolute.h>
#include <rl/mdl/RungeKuttaNystromIntegrator.h>
#include <rl/mdl/World.h>

std::shared_ptr<rl::mdl::Dynamic>
createPlanar2()
{
	std::shared_ptr<rl::mdl::Dynamic> dynamic = std::make_shared<rl::mdl::Dynamic>();
	dynamic->setName("planar2");
	
	// frames
	
	std::shared_ptr<rl::mdl::World> world = std::make_shared<rl::mdl::World>();
	world->setName("world");
	world->setGravity(rl::math::Vector3(0, 0, rl::math::constants::gravity));
	dynamic->add(world);
	
	std::shared_ptr<rl::mdl::Body> link0 = std::make_shared<rl::mdl::Body>();
	link0->setName("link0");
	dynamic->add(link0);
	
	std::shared_ptr<rl::mdl::Body> link1 = std::make_shared<rl::mdl::Body>();
	link1->setCenterOfMass(static_cast<rl::math::Real>(0.5), 0, 0);
	link1->setInertia(static_cast<rl::math::Real>(0.01), static_cast<rl::math::Real>(0.083333333), static_cast<rl::math::Real>(0.083333333), 0, 0, 0);
	link1->setMass(1);
	link1->setName("link1");
	dynamic->add(link1);
	
	std::shared_ptr<rl::mdl::Frame> frame0 = std::make_shared<rl::mdl::Frame>();
	frame0->setName("frame0");
	dynamic->add(frame0);
	
	std::shared_ptr<rl::mdl::Body> link2 = std::make_shared<rl::mdl::Body>();
	link2->setCenterOfMass(static_cast<rl::math::Real>(0.5), 0, 0);
	link2->setInertia(static_cast<rl::math::Real>(0.01), static_cast<rl::math::Real>(0.083333333), static_cast<rl::math::Real>(0.083333333), 0, 0, 0);
	link2->setMass(1);
	link2->setName("link2");
	dynamic->add(link2);
	
	std::shared_ptr<rl::mdl::Frame> frame1 = std::make_shared<rl::mdl::Frame>();
	frame1->setName("frame1");
	dynamic->add(frame1);
	
	// selfcollision
	
	link0->setCollision(false);
	link1->setCollision(link0.get(), false);
	link1->setCollision(link2.get(), false);
	link2->setCollision(link1.get(), false);
	
	// transforms
	
	std::shared_ptr<rl::mdl::Fixed> fixed0 = std::make_shared<rl::mdl::Fixed>();
	fixed0->setName("fixed0");
	dynamic->add(fixed0, world.get(), link0.get());
	
	std::shared_ptr<rl::mdl::Revolute> joint0 = std::make_shared<rl::mdl::Revolute>();
	joint0->setMaximum(rl::math::Vector::Constant(1, 360 * rl::math::constants::deg2rad));
	joint0->setMinimum(rl::math::Vector::Constant(1, -360 * rl::math::constants::deg2rad));
	joint0->setName("joint0");
	dynamic->add(joint0, link0.get(), link1.get());
	
	std::shared_ptr<rl::mdl::Fixed> fixed1 = std::make_shared<rl::mdl::Fixed>();
	fixed1->setName("fixed1");
	fixed1->setTransform(rl::math::Transform(rl::math::Translation(rl::math::Vector3(1, 0, 0))));
	dynamic->add(fixed1, link1.get(), frame0.get());
	
	std::shared_ptr<rl::mdl::Revolute> joint1 = std::make_shared<rl::mdl::Revolute>();
	joint1->setMaximum(rl::math::Vector::Constant(1, 360 * rl::math::constants::deg2rad));
	joint1->setMinimum(rl::math::Vector::Constant(1, -360 * rl::math::constants::deg2rad));
	joint1->setName("joint1");
	dynamic->add(joint1, frame0.get(), link2.get());
	
	std::shared_ptr<rl::mdl::Fixed> fixed2 = std::make_shared<rl::mdl::Fixed>();
	fixed2->setName("fixed2");
	fixed2->setTransform(rl::math::Transform(rl::math::Translation(rl::math::Vector3(1, 0, 0))));
	dynamic->add(fixed2, link2.get(), frame1.get());
	
	// initialize
	
	dynamic->update();
	
	return dynamic;
}

int
main(int argc, char** argv)
{
	if (argc < 7)
	{
		std::cout << "Usage: rlDynamics1Planar2Demo Q1 Q2 QD1 QD2 QDD1 QDD2" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
		std::shared_ptr<rl::mdl::Dynamic> dynamic = createPlanar2();
		
		rl::math::Vector q(dynamic->getDofPosition());
		rl::math::Vector qd(dynamic->getDof());
		rl::math::Vector qdd(dynamic->getDof());
		
		for (std::size_t i = 0; i < dynamic->getDofPosition(); ++i)
		{
			q(i) = boost::lexical_cast<rl::math::Real>(argv[i + 1]);
		}
		
		for (std::size_t i = 0; i < dynamic->getDof(); ++i)
		{
			qd(i) = boost::lexical_cast<rl::math::Real>(argv[i + 1 + dynamic->getDofPosition()]);
			qdd(i) = boost::lexical_cast<rl::math::Real>(argv[i + 1 + dynamic->getDofPosition() + dynamic->getDof()]);
		}
		
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->setAcceleration(qdd);
		
		dynamic->inverseDynamics();
		std::cout << "tau = " << dynamic->getTorque().transpose() << std::endl;
		
		dynamic->forwardDynamics();
		std::cout << "qdd = " << dynamic->getAcceleration().transpose() << std::endl;
		
		rl::mdl::RungeKuttaNystromIntegrator integrator(dynamic.get());
		integrator.integrate(1);
		std::cout << "q = " << dynamic->getPosition().transpose() << std::endl;
		std::cout << "qd = " << dynamic->getVelocity().transpose() << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
