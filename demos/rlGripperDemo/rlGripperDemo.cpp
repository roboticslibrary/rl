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
#include <signal.h>
#include <stdexcept>
#include <thread>
#include <rl/hal/WeissWsg50.h>

bool running = true;

void handler(int signum)
{
	running = false;
}

int
main(int argc, char** argv)
{
	signal(SIGINT, handler);
	
	try
	{
		rl::hal::WeissWsg50 gripper;
		
		gripper.open();
		
		std::chrono::steady_clock::time_point time = std::chrono::steady_clock::now();
		
		do
		{
			time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(gripper.getUpdateRate());
			std::this_thread::sleep_until(time);
			gripper.start();
		}
		while (!gripper.isRunning());
		
		for (std::size_t i = 0; running; ++i)
		{
			time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(gripper.getUpdateRate());
			std::this_thread::sleep_until(time);
			
			gripper.step();
			
			std::cout << "openingWidth: " << gripper.getOpeningWidth() << std::endl;
			std::cout << "force: " << gripper.getForce() << std::endl;
			std::cout << "speed: " << gripper.getSpeed() << std::endl;
			
			rl::hal::WeissWsg50::SystemState systemState = gripper.getSystemState();
			
			std::cout << "isReferenced: " << (rl::hal::WeissWsg50::SYSTEM_STATE_REFERENCED & systemState) << std::endl;
			std::cout << "isMoving: " << (rl::hal::WeissWsg50::SYSTEM_STATE_MOVING & systemState) << std::endl;
			std::cout << "isTargetPositionReached: " << (rl::hal::WeissWsg50::SYSTEM_STATE_TARGET_POSITION_REACHED & systemState) << std::endl;
			
			if (0 == i % 100)
			{
				if (gripper.getOpeningWidth() > 0.1f)
				{
					gripper.shut();
				}
				else
				{
					gripper.release();
				}
			}
		}
		
		do
		{
			time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(gripper.getUpdateRate());
			std::this_thread::sleep_until(time);
			gripper.stop();
		}
		while (gripper.isRunning());
		
		gripper.close();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
