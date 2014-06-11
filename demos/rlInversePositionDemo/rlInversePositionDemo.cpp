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
#include <rl/hal/Coach.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/math/Unit.h>
#include <rl/util/Timer.h>

#define COACH

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlInversePositionDemo MODELFILE Q1 ... Qn" << std::endl;
		return 1;
	}
	
	try
	{
		srand(static_cast< unsigned int >(rl::util::Timer::now() * 1.0e9f));
		
		rl::mdl::XmlFactory factory;
		boost::shared_ptr< rl::mdl::Model > model(factory.create(argv[1]));
		
		rl::mdl::Kinematic* kinematic = dynamic_cast< rl::mdl::Kinematic* >(model.get());
		
		rl::math::Vector q(kinematic->getDof());
		
		for (std::ptrdiff_t i = 0; i < q.size(); ++i)
		{
			q(i) = boost::lexical_cast< rl::math::Real >(argv[i + 2]);
		}
		
		kinematic->setPosition(q);
		
		kinematic->forwardPosition();
		rl::math::Vector3 position = kinematic->getOperationalPosition(0).translation();
		rl::math::Vector3 orientation = kinematic->getOperationalPosition(0).rotation().eulerAngles(2, 1, 0).reverse();
		std::cout << "x: " << position.x() << " m, y: " << position.y() << " m, z: " << position.z() << " m, a: " << orientation.x() * rl::math::RAD2DEG << " deg, b: " << orientation.y() * rl::math::RAD2DEG << " deg, c: " << orientation.z() * rl::math::RAD2DEG << " deg" << std::endl;
		
		rl::math::Transform x = kinematic->getOperationalPosition(0);
		kinematic->setPosition(rl::math::Vector::Random(kinematic->getDof()));
		rl::util::Timer timer;
		timer.start();
		bool result = kinematic->calculateInversePosition(x);
		timer.stop();
		std::cout << (result ? "true" : "false") << " " << timer.elapsed() * 1000 << " ms" << std::endl;
		
		kinematic->forwardPosition();
		position = kinematic->getOperationalPosition(0).translation();
		orientation = kinematic->getOperationalPosition(0).rotation().eulerAngles(2, 1, 0).reverse();
		std::cout << "x: " << position.x() << " m, y: " << position.y() << " m, z: " << position.z() << " m, a: " << orientation.x() * rl::math::RAD2DEG << " deg, b: " << orientation.y() * rl::math::RAD2DEG << " deg, c: " << orientation.z() * rl::math::RAD2DEG << " deg" << std::endl;
		
		kinematic->getPosition(q);
		std::cout << "q: " << q.transpose() << std::endl;
		
#ifdef COACH
		rl::hal::Coach controller(kinematic->getDof(), 0.001f, 0, "localhost");
		controller.open();
		controller.start();
		controller.setJointPosition(q);
		controller.step();
		controller.stop();
		controller.close();
#endif // COACH
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return 1;
	}
	
	return 0;
}
