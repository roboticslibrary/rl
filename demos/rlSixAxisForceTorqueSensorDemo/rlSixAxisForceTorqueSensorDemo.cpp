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

#include <fstream>
#include <iostream>
#include <signal.h>
#include <rl/hal/Exception.h>
#include <rl/util/Timer.h>

//#define ATI
#define JR3

#ifdef ATI
#include <rl/hal/Ati.h>
#endif
#ifdef JR3
#include <rl/hal/Jr3.h>
#endif

bool running = true;

void handler(int signum)
{
	running = false;
}

int
main(int argc, char** argv)
{
	std::ofstream ft;
	ft.open("ft.txt", std::fstream::trunc);
	
	try
	{
#ifdef ATI
		rl::hal::Ati sensor("/usr/local/share/atidaq/left.cal", 0, "/dev/comedi0");
#endif
#ifdef JR3
		rl::hal::Jr3 sensor("/dev/comedi0");
#endif
		
		sensor.open();
		sensor.start();
		
		sensor.step();
		sensor.bias();
		
		rl::math::Vector data(6);
		::std::size_t t = 0;
		
		while (running)
		{
			sensor.step();
			sensor.getForcesTorques(data);
			
			std::cout << data << std::endl;
			ft << t++;
			
			for (::std::size_t i = 0; i < 6; ++i)
			{
				ft << " " << data(i);
			}
			
			ft << std::endl;
			ft.flush();
			
			rl::util::Timer::sleep(0.001f);
		}
		
		sensor.stop();
		sensor.close();
		
		ft.close();
	}
	catch (rl::hal::Exception& e)
	{
		ft.close();
		std::cout << e.what() << std::endl;
		return 1;
	}
	
	return 0;
}
