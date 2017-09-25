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
#include <rl/hal/Socket.h>

#define IPV4
//#define IPV6

#define TCP
//#define UDP

int
main(int argc, char** argv)
{
	if (argc < 5)
	{
		std::cout << "Usage: rlSocketDemoClient ADDRESS PORT MESSAGE BUFFERSIZE" << std::endl;
		return EXIT_FAILURE;
	}
	
	try
	{
#ifdef IPV4
		rl::hal::Socket::Address address = rl::hal::Socket::Address::Ipv4(argv[1], argv[2]);
#endif
#ifdef IPV6
		rl::hal::Socket::Address address = rl::hal::Socket::Address::Ipv6(argv[1], argv[2]);
#endif
		
#ifdef TCP
		rl::hal::Socket socket = rl::hal::Socket::Tcp(address);
#endif
#ifdef UDP
		rl::hal::Socket socket = rl::hal::Socket::Udp(address);
#endif
		socket.open();
		socket.connect();
		
		std::string message = argv[3];
		std::size_t numbytes = socket.send(message.c_str(), message.length());
		std::cout << numbytes << " characters sent" << std::endl;
		std::cout << message.substr(0, numbytes) << std::endl;
		
		std::vector<char> buffer(boost::lexical_cast<std::size_t>(argv[4]) + 1);
		buffer.back() = '\0';
		numbytes = socket.recv(buffer.data(), buffer.size() - 1);
		std::cout << numbytes << " characters received" << std::endl;
		std::cout << buffer.data() << std::endl;
		
		socket.close();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}
