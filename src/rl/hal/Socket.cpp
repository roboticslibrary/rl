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

#ifdef WIN32
#include <windows.h>
#include <ws2tcpip.h>
#else // WIN32
#include <cerrno>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#ifdef __QNX__
#include <sys/select.h>
#endif // __QNX__
#include <sys/types.h>
#endif // WIN32

#include <cstring>
#include <sstream>
#include <boost/lexical_cast.hpp>

#include "ComException.h"
#include "Socket.h"
#include "TimeoutException.h"

namespace rl
{
	namespace hal
	{
		Socket::Socket(const Socket& socket) :
			fd(socket.fd),
			address(socket.address),
			protocol(socket.protocol),
			type(socket.type)
		{
#ifdef WIN32
			Socket::startup();
#endif // WIN32
		}
		
		Socket::Socket(const int& type, const int& protocol, const Address& address) :
			Com(),
#ifdef WIN32
			fd(INVALID_SOCKET),
#else // WIN32
			fd(-1),
#endif // WIN32
			address(address),
			protocol(protocol),
			type(type)
		{
#ifdef WIN32
			Socket::startup();
#endif // WIN32
		}
		
#ifdef WIN32
		Socket::Socket(const int& type, const int& protocol, const Address& address, const SOCKET& fd) :
#else // WIN32
		Socket::Socket(const int& type, const int& protocol, const Address& address, const int& fd) :
#endif // WIN32
			Com(),
			fd(fd),
			address(address),
			protocol(protocol),
			type(type)
		{
#ifdef WIN32
			Socket::startup();
#endif // WIN32
			
			this->setConnected(true);
		}
		
		Socket::~Socket()
		{
			if (this->isConnected())
			{
				this->close();
			}
			
#ifdef WIN32
			Socket::cleanup();
#endif // WIN32
		}
		
		Socket
		Socket::Tcp(const Address& address)
		{
			return Socket(SOCK_STREAM, IPPROTO_TCP, address);
		}
		
		Socket
		Socket::Udp(const Address& address)
		{
			return Socket(SOCK_DGRAM, IPPROTO_UDP, address);
		}
		
		Socket
		Socket::accept()
		{
			::sockaddr_storage addr;
			::socklen_t addrlen = sizeof(addr);
			
#ifdef WIN32
			SOCKET fd = ::accept(this->fd, reinterpret_cast< ::sockaddr*>(&addr), &addrlen);
#else // WIN32
			int fd = ::accept(this->fd, reinterpret_cast< ::sockaddr*>(&addr), &addrlen);
#endif // WIN32
			
#ifdef WIN32
			if (INVALID_SOCKET == fd)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == fd)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			return Socket(this->type, this->protocol, Address(addr), fd);
		}
		
		void
		Socket::bind()
		{
			int err = ::bind(this->fd, reinterpret_cast<const ::sockaddr*>(&this->address.get()), sizeof(this->address.get()));
			
#ifdef WIN32
			if (SOCKET_ERROR == err)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == err)
			{
				throw ComException(errno);
			}
#endif // WIN32
		}
		
#ifdef WIN32
		void
		Socket::cleanup()
		{
			if (0 != ::WSACleanup())
			{
				throw ComException(::WSAGetLastError());
			}
		}
#endif // WIN32
		
		void
		Socket::close()
		{
#ifdef WIN32
			if (SOCKET_ERROR == ::closesocket(this->fd))
			{
				throw ComException(::WSAGetLastError());
			}
			
			this->fd = INVALID_SOCKET;
#else // WIN32
			if (-1 == ::close(this->fd))
			{
				throw ComException(errno);
			}
			
			this->fd = -1;
#endif // WIN32
			
			this->setConnected(false);
		}
		
		void
		Socket::connect()
		{
			int err = ::connect(this->fd, reinterpret_cast<const ::sockaddr*>(&this->address.get()), sizeof(this->address.get()));
			
#ifdef WIN32
			if (SOCKET_ERROR == err)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == err)
			{
				throw ComException(errno);
			}
#endif // WIN32
		}
		
		const Socket::Address&
		Socket::getAddress() const
		{
			return this->address;
		}
		
		int
		Socket::getOption(const Option& option) const
		{
			int level;
			int optname;
			int optval;
#if WIN32
			int optlen = sizeof(optval);
#else // WIN32
			::socklen_t optlen = sizeof(optval);
#endif // WIN32
			
			switch (option)
			{
			case OPTION_KEEPALIVE:
				level = SOL_SOCKET;
				optname = SO_KEEPALIVE;
				break;
			case OPTION_MULTICAST_LOOP:
				if (AF_INET6 == this->address.get().ss_family)
				{
					level = IPPROTO_IPV6;
					optname = IPV6_MULTICAST_LOOP;
				}
				else
				{
					level = IPPROTO_IP;
					optname = IP_MULTICAST_LOOP;
				}
				break;
			case OPTION_MULTICAST_TTL:
				if (AF_INET6 == this->address.get().ss_family)
				{
					level = IPPROTO_IPV6;
					optname = IPV6_MULTICAST_HOPS;
				}
				else
				{
					level = IPPROTO_IP;
					optname = IP_MULTICAST_TTL;
				}
				break;
			case OPTION_NODELAY:
				level = IPPROTO_TCP;
				optname = TCP_NODELAY;
				break;
#if !defined(__APPLE__) && !defined(__QNX__) && !defined(WIN32)
			case OPTION_QUICKACK:
				level = IPPROTO_TCP;
				optname = TCP_QUICKACK;
				break;
#endif // __APPLE__ || __QNX__ || WIN32
			default:
				break;
			}
			
#if WIN32
			int result = ::getsockopt(this->fd, level, optname, reinterpret_cast<char*>(&optval), &optlen);
#else // WIN32
			int result = ::getsockopt(this->fd, level, optname, reinterpret_cast<void*>(&optval), &optlen);
#endif // WIN32
			
#ifdef WIN32
			if (SOCKET_ERROR == result)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == result)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			return optval;
		}
		
		const int&
		Socket::getProtocol() const
		{
			return this->protocol;
		}
		
		const int&
		Socket::getType() const
		{
			return this->type;
		}
		
		void
		Socket::listen()
		{
			this->listen(SOMAXCONN);
		}
		
		void
		Socket::listen(const int& backlog)
		{
			int err = ::listen(this->fd, backlog);
			
#ifdef WIN32
			if (SOCKET_ERROR == err)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == err)
			{
				throw ComException(errno);
			}
#endif // WIN32
		}
		
		void
		Socket::open()
		{
			this->fd = ::socket(this->address.get().ss_family, this->type, this->protocol);
			
#ifdef WIN32
			if (INVALID_SOCKET == this->fd)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == this->fd)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			this->setConnected(true);
		}
		
		::std::size_t
		Socket::recv(void* buf, const ::std::size_t& count)
		{
			::std::memset(buf, 0, count);
			
#ifdef WIN32
			int numbytes = ::recv(this->fd, static_cast<char*>(buf), count, 0);
#else // WIN32
			::ssize_t numbytes = ::recv(this->fd, buf, count, 0);
#endif // WIN32
			
#ifdef WIN32
			if (SOCKET_ERROR == numbytes)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == numbytes)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			return numbytes;
		}
		
		::std::size_t
		Socket::recvfrom(void* buf, const ::std::size_t& count, Address& address)
		{
			::sockaddr_storage addr;
			int addrlen = sizeof(addr);
			
			::std::memset(buf, 0, count);
			
#ifdef WIN32
			int numbytes = ::recvfrom(this->fd, static_cast<char*>(buf), count, 0, reinterpret_cast<::sockaddr*>(&addr), &addrlen);
#else // WIN32
			::ssize_t numbytes = ::recvfrom(this->fd, buf, count, 0, reinterpret_cast< ::sockaddr*>(&addr), reinterpret_cast< ::socklen_t*>(&addrlen));
#endif // WIN32
			
			address = Address(addr);
			
#ifdef WIN32
			if (SOCKET_ERROR == numbytes)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == numbytes)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			return numbytes;
		}
		
		::std::size_t
		Socket::select(const bool& read, const bool& write, const ::std::chrono::nanoseconds& timeout)
		{
			::timeval tv;
			tv.tv_sec = ::std::chrono::duration_cast< ::std::chrono::seconds>(timeout).count();
			tv.tv_usec = ::std::chrono::duration_cast< ::std::chrono::microseconds>(timeout - ::std::chrono::duration_cast< ::std::chrono::seconds>(timeout)).count();
			
			::fd_set readfds;
			FD_ZERO(&readfds);
			FD_SET(this->fd, &readfds);
			
			::fd_set writefds;
			FD_ZERO(&writefds);
			FD_SET(this->fd, &writefds);
			
#ifdef WIN32
			int numdescriptors = ::select(-1, read ? &readfds : nullptr, write ? &writefds : nullptr, nullptr, &tv);
#else // WIN32
			::ssize_t numdescriptors = ::select(this->fd + 1, read ? &readfds : nullptr, write ? &writefds : nullptr, nullptr, &tv);
#endif // WIN32
			
			if (0 == numdescriptors)
			{
				throw TimeoutException();
			}
#ifdef WIN32
			else if (SOCKET_ERROR == numdescriptors)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			else if (-1 == numdescriptors)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			return numdescriptors;
		}
		
		::std::size_t
		Socket::send(const void* buf, const ::std::size_t& count)
		{
#ifdef WIN32
			int numbytes = ::send(this->fd, static_cast<const char*>(buf), count, 0);
#else // WIN32
			::ssize_t numbytes = ::send(this->fd, buf, count, 0);
#endif // WIN32
			
#ifdef WIN32
			if (SOCKET_ERROR == numbytes)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == numbytes)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			return numbytes;
		}
		
		::std::size_t
		Socket::sendto(const void* buf, const ::std::size_t& count, const Address& address)
		{
#ifdef WIN32
			int numbytes = ::sendto(this->fd, static_cast<const char*>(buf), count, 0, reinterpret_cast<const ::sockaddr*>(&address.get()), sizeof(address.get()));
#else // WIN32
			::ssize_t numbytes = ::sendto(this->fd, buf, count, 0, reinterpret_cast<const ::sockaddr*>(&address.get()), sizeof(address.get()));
#endif // WIN32
			
#ifdef WIN32
			if (SOCKET_ERROR == numbytes)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == numbytes)
			{
				throw ComException(errno);
			}
#endif // WIN32
			
			return numbytes;
		}
		
		void
		Socket::setAddress(const Address& address)
		{
			this->address = address;
		}
		
		void
		Socket::setOption(const Option& option, const int& value)
		{
			int level;
			int optname;
			
			switch (option)
			{
			case OPTION_KEEPALIVE:
				level = SOL_SOCKET;
				optname = SO_KEEPALIVE;
				break;
			case OPTION_MULTICAST_LOOP:
				if (AF_INET6 == this->address.get().ss_family)
				{
					level = IPPROTO_IPV6;
					optname = IPV6_MULTICAST_LOOP;
				}
				else
				{
					level = IPPROTO_IP;
					optname = IP_MULTICAST_LOOP;
				}
				break;
			case OPTION_MULTICAST_TTL:
				if (AF_INET6 == this->address.get().ss_family)
				{
					level = IPPROTO_IPV6;
					optname = IPV6_MULTICAST_HOPS;
				}
				else
				{
					level = IPPROTO_IP;
					optname = IP_MULTICAST_TTL;
				}
				break;
			case OPTION_NODELAY:
				level = IPPROTO_TCP;
				optname = TCP_NODELAY;
				break;
#if !defined(__APPLE__) && !defined(__QNX__) && !defined(WIN32)
			case OPTION_QUICKACK:
				level = IPPROTO_TCP;
				optname = TCP_QUICKACK;
				break;
#endif // __APPLE__ || __QNX__ || WIN32
			default:
				break;
			}
			
#if WIN32
			int result = ::setsockopt(this->fd, level, optname, reinterpret_cast<const char*>(&value), sizeof(value));
#else // WIN32
			int result = ::setsockopt(this->fd, level, optname, reinterpret_cast<const void*>(&value), sizeof(value));
#endif // WIN32
			
#ifdef WIN32
			if (SOCKET_ERROR == result)
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == result)
			{
				throw ComException(errno);
			}
#endif // WIN32
		}
		
		void
		Socket::shutdown(const bool& read, const bool& write)
		{
			int how;
			
			if (read && write)
			{
#ifdef WIN32
				how = SD_BOTH;
#else // WIN32
				how = SHUT_RDWR;
#endif // WIN32
			}
			else if (read)
			{
#ifdef WIN32
				how = SD_RECEIVE;
#else // WIN32
				how = SHUT_RD;
#endif // WIN32
			}
			else if (write)
			{
#ifdef WIN32
				how = SD_SEND;
#else // WIN32
				how = SHUT_WR;
#endif // WIN32
			}
			else
			{
				return;
			}
			
#ifdef WIN32
			if (SOCKET_ERROR == ::shutdown(this->fd, how))
			{
				throw ComException(::WSAGetLastError());
			}
#else // WIN32
			if (-1 == ::shutdown(this->fd, how))
			{
				throw ComException(errno);
			}
#endif // WIN32
		}
		
#ifdef WIN32
		void
		Socket::startup()
		{
			::WSADATA wsaData;
			
			int err = ::WSAStartup(MAKEWORD(1, 1), &wsaData);
			
			if (0 != err)
			{
				throw ComException(err);
			}
			
			if (1 != LOBYTE(wsaData.wVersion) || 1 != HIBYTE(wsaData.wVersion))
			{
				if (0 != ::WSACleanup())
				{
					throw ComException(::WSAGetLastError());
				}
				
				throw ComException("Requested Windows Sockets API version not supported.");
			}
		}
#endif // WIN32
		
		Socket::Address::Address() :
			addr()
		{
#ifdef WIN32
			Socket::startup();
#endif // WIN32
		}
		
		Socket::Address::Address(const Address& address) :
			addr(address.addr)
		{
#ifdef WIN32
			Socket::startup();
#endif // WIN32
		}
		
		Socket::Address::Address(const ::sockaddr_storage& addr) :
			addr(addr)
		{
#ifdef WIN32
			Socket::startup();
#endif // WIN32
		}
		
		Socket::Address::Address(const int& family) :
			addr()
		{
#ifdef WIN32
			Socket::startup();
#endif // WIN32
			
			this->addr.ss_family = family;
		}
		
		Socket::Address::~Address()
		{
#ifdef WIN32
			Socket::cleanup();
#endif // WIN32
		}
		
		Socket::Address
		Socket::Address::Ipv4(const ::std::string& string, const unsigned short int& port, const bool& asNumeric)
		{
			Address address(AF_INET);
			address.setInfo(string, port, asNumeric);
			return address;
		}
		
		Socket::Address
		Socket::Address::Ipv4(const ::std::string& string, const ::std::string& port, const bool& asNumeric)
		{
			Address address(AF_INET);
			address.setInfo(string, port, asNumeric);
			return address;
		}
		
		Socket::Address
		Socket::Address::Ipv6(const ::std::string& string, const unsigned short int& port, const bool& asNumeric)
		{
			Address address(AF_INET6);
			address.setInfo(string, port, asNumeric);
			return address;
		}
		
		Socket::Address
		Socket::Address::Ipv6(const ::std::string& string, const ::std::string& port, const bool& asNumeric)
		{
			Address address(AF_INET6);
			address.setInfo(string, port, asNumeric);
			return address;
		}
		
		const ::sockaddr_storage&
		Socket::Address::get() const
		{
			return this->addr;
		}
		
		::std::vector<unsigned char>
		Socket::Address::getHexadecimal()
		{
			::std::vector<unsigned char> hexadecimal;
			
			switch (this->addr.ss_family)
			{
			case AF_INET:
				hexadecimal.resize(4);
				::std::memcpy(hexadecimal.data(), &reinterpret_cast< ::sockaddr_in*>(&this->addr)->sin_addr.s_addr, hexadecimal.size());
				break;
			case AF_INET6:
				hexadecimal.resize(16);
				::std::memcpy(hexadecimal.data(), &reinterpret_cast< ::sockaddr_in6*>(&this->addr)->sin6_addr.s6_addr, hexadecimal.size());
				break;
			default:
				break;
			}
			
			return hexadecimal;
		}
		
		::std::string
		Socket::Address::getNameInfo(const bool& asNumeric) const
		{
			char host[INET6_ADDRSTRLEN];
			
			int err = ::getnameinfo(reinterpret_cast<const ::sockaddr*>(&this->addr), sizeof(this->addr), host, INET6_ADDRSTRLEN, nullptr, 0, asNumeric ? NI_NUMERICHOST : 0);
			
			if (0 != err)
			{
				throw ComException(::gai_strerror(err));
			}
			
			return host;
		}
		
		void
		Socket::Address::setInfo(const ::std::string& string, const unsigned short int& port, const bool& asNumeric)
		{
			this->setInfo(string, ::boost::lexical_cast< ::std::string>(port), asNumeric);
		}
		
		void
		Socket::Address::setInfo(const ::std::string& string, const ::std::string& port, const bool& asNumeric)
		{
			::addrinfo hints;
			::std::memset(&hints, 0, sizeof(hints));
			hints.ai_family = this->addr.ss_family;
			hints.ai_flags = asNumeric ? AI_NUMERICHOST | AI_PASSIVE : AI_PASSIVE;
			hints.ai_socktype = 0;
			
			::addrinfo* res;
			
			int err = ::getaddrinfo(string.empty() ? nullptr : string.c_str(), port.empty() ? nullptr : port.c_str(), &hints, &res);
			
			if (0 != err)
			{
				throw ComException(::gai_strerror(err));
			}
			
			::std::memcpy(&this->addr, res->ai_addr, res->ai_addrlen);
			
			::freeaddrinfo(res);
		}
	}
}
