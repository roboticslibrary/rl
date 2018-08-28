//
// Copyright (c) 2013, Andre Gaschler
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
#include <rl/hal/RobotiqModelC.h>

int
main(int argc, char** argv)
{
	try
	{
#ifdef WIN32
		rl::hal::RobotiqModelC gripper("COM3");
#else
		rl::hal::RobotiqModelC gripper("/dev/ttyUSB0");
#endif
		
		gripper.open();
		gripper.start();
		
		do
		{
			gripper.step();
			std::cout << "activationStatus: " << gripper.getActivationStatus() << " - gripperStatus: " << gripper.getGripperStatus() << std::endl;
		}
		while (!gripper.isRunning());
		
		gripper.setForcePercentage(0.1f);
		gripper.setPositionPercentage(1.0f);
		gripper.setSpeedPercentage(0.25f);
		
		do
		{
			gripper.step();
			std::cout << "position: " << gripper.getPositionPercentage() << " - current: " << gripper.getCurrent() << std::endl;
		}
		while (rl::hal::RobotiqModelC::OBJECT_STATUS_MOTION == gripper.getObjectStatus());
		
		for (std::size_t i = 0; i < 100; ++i)
		{
			gripper.step();
			std::cout << "position: " << gripper.getPositionPercentage() << " - current: " << gripper.getCurrent() << std::endl;
		}
		
		gripper.setForcePercentage(0.1f);
		gripper.setPositionPercentage(0.0f);
		gripper.setSpeedPercentage(0.5f);
		
		do
		{
			gripper.step();
			std::cout << "position: " << gripper.getPositionPercentage() << " - current: " << gripper.getCurrent() << std::endl;
		}
		while (rl::hal::RobotiqModelC::OBJECT_STATUS_MOTION == gripper.getObjectStatus());
		
		gripper.stop();
		gripper.close();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
