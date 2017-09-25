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

#ifndef WIN32
#include <cifxlinux.h>
#endif // WIN32

#include "HilscherCifx.h"

namespace rl
{
	namespace hal
	{
		HilscherCifx::HilscherCifx(const ::std::string& board, const ::std::size_t& number) :
			Fieldbus(),
			board(board),
			channel(),
			driver(),
			number()
		{
		}
		
		HilscherCifx::~HilscherCifx()
		{
		}
		
		void
		HilscherCifx::close()
		{
			::std::int32_t error = ::xChannelBusState(this->channel, CIFX_BUS_STATE_OFF, nullptr, 1000);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
			error = ::xChannelConfigLock(this->channel, CIFX_CONFIGURATION_UNLOCK, nullptr, 1000);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
			error = ::xChannelHostState(this->channel, CIFX_HOST_STATE_NOT_READY, nullptr, 1000);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
			error = ::xChannelClose(&this->channel);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
			error = ::xDriverClose(&this->driver);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
#ifndef WIN32
			::cifXDriverDeinit();
#endif // WIN32
			
			this->setConnected(false);
		}
		
		void
		HilscherCifx::open()
		{
			::std::int32_t error;
			
#ifndef WIN32
			::CIFX_LINUX_INIT init;
			init.base_dir = nullptr;
			init.iCardNumber = 0;
			init.fEnableCardLocking = 0;
			init.init_options = CIFX_DRIVER_INIT_AUTOSCAN;
			init.poll_interval = 0;
			init.poll_StackSize = 0;
			init.trace_level = 255;
			init.user_card_cnt = 0;
			init.user_cards = nullptr;
			
			error = ::cifXDriverInit(&init);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
#endif // WIN32
			
			error = ::xDriverOpen(&this->driver);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
			error = ::xChannelOpen(nullptr, const_cast<char*>(this->board.c_str()), this->number, &this->channel);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
			error = ::xChannelHostState(this->channel, CIFX_HOST_STATE_READY, nullptr, 1000);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
			error = ::xChannelConfigLock(this->channel, CIFX_CONFIGURATION_LOCK, nullptr, 1000);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
			error = ::xChannelBusState(this->channel, CIFX_BUS_STATE_ON, nullptr, 1000);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
			
			this->setConnected(true);
		}
		
		void
		HilscherCifx::read(void* data, const ::std::size_t& offset, const ::std::size_t& length)
		{
			::std::int32_t error = ::xChannelIORead(this->channel, 0, offset, length, data, 10);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
		}
		
		void
		HilscherCifx::write(void* data, const ::std::size_t& offset, const ::std::size_t& length)
		{
			::std::int32_t error = ::xChannelIOWrite(this->channel, 0, offset, length, data, 10);
			
			if (CIFX_NO_ERROR != error)
			{
				throw Exception(error);
			}
		}
		
		HilscherCifx::Exception::Exception(const ::std::int32_t& error) :
			ComException(""),
			buffer(),
			error(error)
		{
			::xDriverGetErrorDescription(this->error, this->buffer, sizeof(this->buffer));
		}
		
		HilscherCifx::Exception::~Exception() throw()
		{
		}
		
		::std::int32_t
		HilscherCifx::Exception::getError() const
		{
			return this->error;
		}
		
		const char*
		HilscherCifx::Exception::what() const throw()
		{
			return this->buffer;
		}
	}
}
