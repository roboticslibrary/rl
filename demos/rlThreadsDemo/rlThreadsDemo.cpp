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

#include <chrono>
#include <functional>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

void run(const std::size_t& i)
{
	std::function<double()> rand = std::bind(std::uniform_real_distribution<double>(0.0, 2.0), std::mt19937(std::random_device()()));
	
	for (std::size_t j = 0; j < 5; ++j)
	{
		double seconds = rand(); 
		std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
		std::cout << "Thread[" << i << "] - " << j << " - " << seconds << " seconds" << std::endl;
	}
}

int
main(int argc, char** argv)
{
	std::vector<std::thread> threads;
	
	for (std::size_t i = 0; i < 5; ++i)
	{
		std::cout << "Thread[" << i << "]" << std::endl;
		threads.push_back(std::thread(run, i));
	}
	
	std::cout << "before join()" << std::endl;
	
	for (std::size_t i = 0; i < threads.size(); ++i)
	{
		threads[i].join();
	}
	
	std::cout << "after join()" << std::endl;
	
	return EXIT_SUCCESS;
}
