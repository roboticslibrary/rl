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

#include <cstring>

#ifdef WIN32
#include <winsock.h>
#else // WIN32
#include <errno.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#ifdef __QNX__
#include <sys/select.h>
#endif // __QNX__
#include <sys/socket.h>
#include <sys/types.h>
#endif // WIN32

#include "ComException.h"
#include "Socket.h"
#include "TimeoutException.h"

namespace rl
{
	namespace hal
	{
		Socket::Socket(const ::std::string& host, const unsigned short int& port) :
			Com(),
			host(host),
			port(port)
		{
		}
		
		Socket::~Socket()
		{
			if (this->isConnected())
			{
				this->close();
			}
		}
			
		void
		Socket::close()
		{
#ifdef WIN32
			if (SOCKET_ERROR == closesocket(this->sockfd))
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, WSAGetLastError(), 0, buffer, 0, NULL);
				ComException e(buffer);
				LocalFree(buffer);
				throw e;
			}
			
			WSACleanup();
#else // WIN32
			if (-1 == ::close(this->sockfd))
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			this->setConnected(false);
		}
		
		void
		Socket::connect(const int& domain, const int& type, const int& protocol)
		{
			int err;
			
#ifdef WIN32
			WSADATA wsaData;
			
			err = WSAStartup(MAKEWORD(1, 1), &wsaData);
			
			if (0 != err)
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, err, 0, buffer, 0, NULL);
				ComException e(buffer);
				LocalFree(buffer);
				throw e;
			}
			
			if ((1 != LOBYTE(wsaData.wVersion)) || (1 != HIBYTE(wsaData.wVersion)))
			{
				WSACleanup();
				throw ComException("The Windows Sockets version requested is not supported.");
			}
#endif // WIN32
			
			this->sockfd = socket(domain, type, protocol);
			
#ifdef WIN32
			if (SOCKET_ERROR == this->sockfd)
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, WSAGetLastError(), 0, buffer, 0, NULL);
				ComException e(buffer);
				LocalFree(buffer);
				WSACleanup();
				throw e;
			}
#else // WIN32
			if (-1 == this->sockfd)
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			struct hostent* server = gethostbyname(this->host.c_str());
			
			struct sockaddr_in servAddr;
			memset(&servAddr, 0, sizeof(servAddr));
			memcpy(&servAddr.sin_addr.s_addr, server->h_addr, server->h_length);
			servAddr.sin_family = AF_INET;
			servAddr.sin_port = htons(this->port);
			
			err = ::connect(this->sockfd, reinterpret_cast< const sockaddr* >(&servAddr), sizeof(servAddr));
			
#ifdef WIN32
			if (SOCKET_ERROR == err)
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, WSAGetLastError(), 0, buffer, 0, NULL);
				ComException e(buffer);
				LocalFree(buffer);
				WSACleanup();
				throw e;
			}
#else // WIN32
			if (-1 == err)
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			this->setConnected(true);
		}
		
		::std::string
		Socket::getHost() const
		{
			return this->host;
		}
		
		unsigned short int
		Socket::getPort() const
		{
			return this->port;
		}
		
		::std::size_t
		Socket::read(void* buf, const ::std::size_t& count)
		{
			memset(buf, 0, count);
			
#ifdef WIN32
			int numbytes = recv(this->sockfd, static_cast< char* >(buf), count, 0);
#else // WIN32
			ssize_t numbytes = recv(this->sockfd, buf, count, 0);
#endif // WIN32
			
#ifdef WIN32
			if (SOCKET_ERROR == numbytes)
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, WSAGetLastError(), 0, buffer, 0, NULL);
				ComException e(buffer);
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			if (-1 == numbytes)
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			return numbytes;
		}
		
		::std::size_t
		Socket::select(const bool& read, const bool& write, const ::rl::math::Real& timeout)
		{
			::rl::math::Real tmp = timeout;
			
			struct timeval tv;
			
			tv.tv_sec = static_cast< long int >(tmp);
			tmp -= tv.tv_sec;
			tv.tv_usec = static_cast< long int >(tmp * 1000000.0f);
			
			fd_set readfds;
			FD_ZERO(&readfds);
			FD_SET(this->sockfd, &readfds);
			
			fd_set writefds;
			FD_ZERO(&writefds);
			FD_SET(this->sockfd, &writefds);
			
#ifdef WIN32
			int numdescriptors;
#else // WIN32
			ssize_t numdescriptors;
#endif // WIN32
			
			if (read && write)
			{
				numdescriptors = ::select(this->sockfd + 1, &readfds, &writefds, NULL, &tv);
			}
			else if (read)
			{
				numdescriptors = ::select(this->sockfd + 1, &readfds, NULL, NULL, &tv);
			}
			else if (write)
			{
				numdescriptors = ::select(this->sockfd + 1, NULL, &writefds, NULL, &tv);
			}
			else
			{
				numdescriptors = ::select(this->sockfd + 1, NULL, NULL, NULL, &tv);
			}
			
			if (0 == numdescriptors)
			{
				throw TimeoutException();
			}
#ifdef WIN32
			else if (SOCKET_ERROR == numdescriptors)
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, WSAGetLastError(), 0, buffer, 0, NULL);
				ComException e(buffer);
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			else if (-1 == numdescriptors)
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			return numdescriptors;
		}
		
		::std::size_t
		Socket::write(const void* buf, const ::std::size_t& count)
		{
#ifdef WIN32
			int numbytes = send(this->sockfd, static_cast< const char* >(buf), count, 0);
#else // WIN32
			ssize_t numbytes = send(this->sockfd, buf, count, 0);
#endif // WIN32
			
#ifdef WIN32
			if (SOCKET_ERROR == numbytes)
			{
				LPTSTR buffer = NULL;
				FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, WSAGetLastError(), 0, buffer, 0, NULL);
				ComException e(buffer);
				LocalFree(buffer);
				throw e;
			}
#else // WIN32
			if (-1 == numbytes)
			{
				throw ComException(strerror(errno));
			}
#endif // WIN32
			
			return numbytes;
		}
	}
}
