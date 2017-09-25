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
#include <signal.h>
#include <rl/hal/Exception.h>

//#define LEUZE
//#define SCHMERSAL
#define SCHUNK
//#define SICK

#ifdef LEUZE
#include <rl/hal/LeuzeRs4.h>
#endif
#ifdef SCHMERSAL
#include <rl/hal/SchmersalLss300.h>
#endif
#ifdef SCHUNK
#include <rl/hal/SchunkFpsF5.h>
#endif
#ifdef SICK
#include <rl/hal/SickLms200.h>
#endif

bool running = true;

void handler(int signum)
{
	running = false;
}

int
main(int argc, char** argv)
{
	try
	{
#ifdef LEUZE
		rl::hal::LeuzeRs4 sensor(
			"/dev/ttyUSB0",
			rl::hal::LeuzeRs4::BAUDRATE_57600BPS
		);
#endif
#ifdef SCHMERSAL
		rl::hal::SchmersalLss300 sensor(
			"/dev/ttyUSB0",
			rl::hal::SchmersalLss300::BAUDRATE_9600BPS,
			rl::hal::SchmersalLss300::MONITORING_CONTINUOUS
		);
#endif
#ifdef SCHUNK
		rl::hal::SchunkFpsF5 sensor("/dev/ttyS0");
#endif
#ifdef SICK
		rl::hal::SickLms200 sensor(
			"/dev/ttyUSB0",
			rl::hal::SickLms200::BAUDRATE_9600BPS,
			rl::hal::SickLms200::MONITORING_CONTINUOUS
		);
#endif
		
		sensor.open();
		sensor.start();
		
		while (running)
		{
			sensor.step();
			rl::math::Vector data = sensor.getDistances();
			std::cout << data << std::endl;
		}
		
		sensor.stop();
		sensor.close();
		
		return EXIT_SUCCESS;
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}
