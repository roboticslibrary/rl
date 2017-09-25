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

#include <cstdio>
#include <iostream>
#include <signal.h>
#include <rl/hal/Exception.h>

//#define LEUZE
//#define SCHMERSAL
//#define SICKLMS200
#define SICKS300

#ifdef LEUZE
#include <rl/hal/LeuzeRs4.h>
#endif
#ifdef SCHMERSAL
#include <rl/hal/SchmersalLss300.h>
#endif
#ifdef SICKLMS200
#include <rl/hal/SickLms200.h>
#endif
#ifdef SICKS300
#include <rl/hal/SickS300.h>
#endif

#ifdef WIN32
#ifndef popen
#define popen _popen
#endif // popen
#ifndef pclose
#define pclose _pclose
#endif // pclose
#define GNUPLOT "pgnuplot.exe"
#else // WIN32
#define GNUPLOT "gnuplot"
#endif // WIN32

bool running = true;

void handler(int signum)
{
	running = false;
}

int
main(int argc, char** argv)
{
	signal(SIGINT, handler);
	
	FILE* gnuplot = popen(GNUPLOT, "w");
	
	if (nullptr == gnuplot)
	{
		std::cerr << "cannot open " << GNUPLOT << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
#ifdef LEUZE
		rl::hal::LeuzeRs4 laser(
			"/dev/ttyUSB0",
			rl::hal::LeuzeRs4::BAUDRATE_57600BPS
		);
#endif
#ifdef SCHMERSAL
		rl::hal::SchmersalLss300 laser(
			"/dev/ttyUSB0",
			rl::hal::SchmersalLss300::BAUDRATE_9600BPS,
			rl::hal::SchmersalLss300::MONITORING_CONTINUOUS
		);
#endif
#ifdef SICKLMS200
		rl::hal::SickLms200 laser(
			"/dev/ttyUSB0",
			rl::hal::SickLms200::BAUDRATE_9600BPS,
			rl::hal::SickLms200::MONITORING_CONTINUOUS
		);
#endif
#ifdef SICKS300
		rl::hal::SickS300 laser(
			"/dev/ttyS1",
			rl::hal::Serial::BAUDRATE_115200BPS
		);
#endif
		
		laser.open();
		
		fprintf(gnuplot, "set polar\n");
		fprintf(gnuplot, "set xrange [-5:5]\n");
		fprintf(gnuplot, "set yrange [0:5]\n");
		
		laser.start();
		
		while (running)
		{
			laser.step();
			rl::math::Vector data = laser.getDistances();
			
			fprintf(gnuplot, "plot '-' with lines\n");
			
			for (std::ptrdiff_t i = 0; i < data.size(); ++i)
			{
				fprintf(gnuplot, "%f %f\n", laser.getStartAngle() + laser.getResolution() * i, data(i));
			}
			
			fprintf(gnuplot, "e\n");
			fflush(gnuplot);
		}
		
		laser.stop();
		laser.close();
		
		pclose(gnuplot);
	}
	catch (const std::exception& e)
	{
		pclose(gnuplot);
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
