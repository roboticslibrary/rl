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

#define __STDC_WANT_LIB_EXT1__ 1

#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdexcept>
#include <thread>

//#define ATI
//#define JR3
#define WEISSKMS40

#ifdef ATI
#include <rl/hal/Ati.h>
#endif
#ifdef JR3
#include <rl/hal/Jr3.h>
#endif
#ifdef WEISSKMS40
#include <rl/hal/WeissKms40.h>
#endif

bool running = true;

void
handler(int signum)
{
	running = false;
}

std::string
toIsoString(const std::chrono::system_clock::time_point& timePoint)
{
	std::time_t time = std::chrono::system_clock::to_time_t(timePoint);
	std::array<char, 32> str;
	std::tm result;
#ifdef __STDC_LIB_EXT1__
	std::strftime(str.data(), str.size(), "%Y-%m-%d %H:%M:%S", gmtime_s(&time, &result));
#elif _MSC_VER
	if (NULL == gmtime_s(&result, &time))
	{
		std::strftime(str.data(), str.size(), "%Y-%m-%d %H:%M:%S", &result);
	}
#else
	std::strftime(str.data(), str.size(), "%Y-%m-%d %H:%M:%S", gmtime_r(&time, &result));
#endif
	return str.data();
}

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: rlSixAxisForceTorqueSensorDemo FILE" << std::endl;
		return EXIT_FAILURE;
	}
	
	signal(SIGINT, handler);
	
	std::ofstream file;
	file.open(argv[1], std::fstream::trunc);
	
	try
	{
#ifdef ATI
		rl::hal::Ati sensor("/usr/local/share/atidaq/left.cal", std::chrono::milliseconds(1), 0, "/dev/comedi0");
#endif
#ifdef JR3
		rl::hal::Jr3 sensor("/dev/comedi0", std::chrono::milliseconds(1));
#endif
#ifdef WEISSKMS40
		rl::hal::WeissKms40 sensor("192.168.29.21");
#endif
		
		sensor.open();
		
#ifdef WEISSKMS40
		std::cout << "Verbose level: " << sensor.doGetVerboseLevel() << std::endl;
		std::cout << "System type: " << sensor.doGetSystemType() << std::endl;
		std::cout << "Firmware version: " << sensor.doGetFirmwareVersion() << std::endl;
		std::cout << "Serial number: " << sensor.doGetSerialNumber() << std::endl;
		std::cout << "Descriptor string: " << sensor.doGetDescriptorString() << std::endl;
		std::cout << "System flags: " << sensor.doGetSystemFlags() << std::endl;
		std::cout << "Temperature: " << sensor.doGetTemperature() << std::endl;
		std::cout << "Single frame: " << sensor.doAcquireSingleFrame().transpose() << std::endl;
		std::cout << "Data acquisition mask: " << sensor.doGetDataAcquisitionMask() << std::endl;
		std::cout << "Frame send divider: " << sensor.doGetFrameSendDivider() << std::endl;
		std::cout << "Tare: " << sensor.doGetTare() << std::endl;
		std::cout << "Filter: " << sensor.doGetFilter() << std::endl;
		::std::pair<::std::chrono::system_clock::time_point, ::std::chrono::system_clock::duration> calibrationDateLifetime = sensor.doGetCalibrationDateLifetime();
		std::cout << "Calibration date: " << calibrationDateLifetime.first.time_since_epoch().count() << " " << toIsoString(calibrationDateLifetime.first) << std::endl;
		std::cout << "Calibration lifetime: " << calibrationDateLifetime.second.count() << " " << toIsoString(calibrationDateLifetime.first + calibrationDateLifetime.second) << std::endl;
#endif
		
		std::chrono::steady_clock::time_point time = std::chrono::steady_clock::now();
		
		do
		{
			time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(sensor.getUpdateRate());
			std::this_thread::sleep_until(time);
			sensor.start();
		}
		while (!sensor.isRunning());
		
		for (std::size_t i = 0; running; ++i)
		{
			time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(sensor.getUpdateRate());
			std::this_thread::sleep_until(time);
			
			sensor.step();
			rl::math::Vector data = sensor.getForcesTorques();
			
			file << i;
			
			for (std::size_t j = 0; j < 6; ++j)
			{
				file << " " << data(j);
			}
			
			file << std::endl;
		}
		
		do
		{
			time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(sensor.getUpdateRate());
			std::this_thread::sleep_until(time);
			sensor.stop();
		}
		while (sensor.isRunning());
		
		sensor.close();
		
		file.flush();
		file.close();
	}
	catch (const std::exception& e)
	{
		file.close();
		std::cout << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
