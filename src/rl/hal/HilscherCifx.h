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

#ifndef RL_HAL_HILSCHERCIFX_H
#define RL_HAL_HILSCHERCIFX_H

#include <cifXErrors.h>
#include <cifXUser.h>
#include <cstdint>
#include <string>

#include "ComException.h"
#include "Fieldbus.h"

namespace rl
{
	namespace hal
	{
		class HilscherCifx : public Fieldbus
		{
		public:
			class Exception : public ComException
			{
			public:
				Exception(const ::std::int32_t& error);
				
				virtual ~Exception() throw();
				
				::std::int32_t getError() const;
				
				virtual const char* what() const throw();
				
			protected:
				
			private:
				char buffer[1024];
				
				::std::int32_t error;
			};
			
			HilscherCifx(const ::std::string& board = "cifX0", const ::std::size_t& number = 0);
			
			virtual ~HilscherCifx();
			
			void close();
			
			void open();
			
			void read(void* data, const ::std::size_t& offset, const ::std::size_t& length);
			
			void write(void* data, const ::std::size_t& offset, const ::std::size_t& length);
			
		protected:
			
		private:
			::std::string board;
			
			::CIFXHANDLE channel;
			
			::CIFXHANDLE driver;
			
			::std::size_t number;
		};
	}
}

#endif // RL_HAL_HILSCHERCIFX_H
