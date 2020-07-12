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

#ifndef RL_HAL_SOCKET_H
#define RL_HAL_SOCKET_H

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <rl/math/Real.h>

#include "Com.h"

namespace rl
{
	namespace hal
	{
		class RL_HAL_EXPORT Socket : public Com
		{
		public:
			class RL_HAL_EXPORT Address
			{
			public:
				struct Impl;
				
				Address();
				
				Address(const Address& address);
				
				virtual ~Address();
				
				static Address Ipv4(const ::std::string& string, const unsigned short int& port, const bool& asNumeric = false);
				
				static Address Ipv4(const ::std::string& string, const ::std::string& port, const bool& asNumeric = false);
				
				static Address Ipv6(const ::std::string& string, const unsigned short int& port, const bool& asNumeric = false);
				
				static Address Ipv6(const ::std::string& string, const ::std::string& port, const bool& asNumeric = false);
				
				Impl* get() const;
				
				::std::vector<unsigned char> getHexadecimal();
				
				::std::size_t getLength() const;
				
				::std::string getNameInfo(const bool& asNumeric = false) const;
				
				Socket::Address& operator=(const Socket::Address& other);
				
				void setInfo(const ::std::string& string, const unsigned short int& port, const bool& asNumeric = false);
				
				void setInfo(const ::std::string& string, const ::std::string& port, const bool& asNumeric = false);
				
			protected:
				
			private:
				Address(const int& family);
				
				::std::unique_ptr<Impl> impl;
			};
			
			enum class Option
			{
				keepalive,
				multicastLoop,
				multicastTtl,
#if defined(__APPLE__) || defined(__QNX__) || defined(WIN32) || defined(__CYGWIN__)
				nodelay
#else // __APPLE__ || __QNX__ || WIN32 || __CYGWIN__
				nodelay,
				quickack
#endif // __APPLE__ || __QNX__ || WIN32 || __CYGWIN__
			};
			
			RL_HAL_DEPRECATED static constexpr Option OPTION_KEEPALIVE = Option::keepalive;
			RL_HAL_DEPRECATED static constexpr Option OPTION_MULTICAST_LOOP = Option::multicastLoop;
			RL_HAL_DEPRECATED static constexpr Option OPTION_MULTICAST_TTL = Option::multicastTtl;
#if defined(__APPLE__) || defined(__QNX__) || defined(WIN32) || defined(__CYGWIN__)
			RL_HAL_DEPRECATED static constexpr Option OPTION_NODELAY = Option::nodelay;
#else // __APPLE__ || __QNX__ || WIN32 || __CYGWIN__
			RL_HAL_DEPRECATED static constexpr Option OPTION_NODELAY = Option::nodelay;
			RL_HAL_DEPRECATED static constexpr Option OPTION_QUICKACK = Option::quickack;
#endif // __APPLE__ || __QNX__ || WIN32 || __CYGWIN__
			
			Socket(const Socket& socket);
			
			virtual ~Socket();
			
			static Socket Tcp(const Address& address);
			
			static Socket Udp(const Address& address);
			
			Socket accept();
			
			void bind();
			
			void close();
			
			void connect();
			
			const Address& getAddress() const;
			
			int getOption(const Option& option) const;
			
			const int& getProtocol() const;
			
			const int& getType() const;
			
			void listen();
			
			void listen(const int& backlog);
			
			void open();
			
			Socket& operator=(const Socket& other);
			
			::std::size_t recv(void* buf, const ::std::size_t& count);
			
			::std::size_t recvfrom(void* buf, const ::std::size_t& count, Address& address);
			
			::std::size_t select(const bool& read, const bool& write, const ::std::chrono::nanoseconds& timeout);
			
			::std::size_t send(const void* buf, const ::std::size_t& count);
			
			::std::size_t sendto(const void* buf, const ::std::size_t& count, const Address& address);
			
			void setAddress(const Address& address);
			
			void setOption(const Option& option, const int& value);
			
			void shutdown(const bool& read = true, const bool& write = true);
			
		protected:
			
		private:
			Socket(const int& type, const int& protocol, const Address& address);
			
#ifdef WIN32
#ifdef _WIN64
			Socket(const int& type, const int& protocol, const Address& address, const unsigned __int64& fd);
#else // _WIN64
			Socket(const int& type, const int& protocol, const Address& address, const unsigned int& fd);
#endif // _WIN64
#else // WIN32
			Socket(const int& type, const int& protocol, const Address& address, const int& fd);
#endif // WIN32
			
#ifdef WIN32
			static void cleanup();
			
			static void startup();
#endif // WIN32
			
			Address address;
			
#ifdef WIN32
#ifdef _WIN64
			unsigned __int64 fd;
#else // _WIN64
			unsigned int fd;
#endif // _WIN64
#else // WIN32
			int fd;
#endif // WIN32
			
			int protocol;
			
			int type;
		};
	}
}

#endif // RL_HAL_SOCKET_H
