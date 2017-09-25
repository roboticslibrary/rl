//
// Copyright (c) 2016, Markus Rickert
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

#ifndef RL_HAL_WEISSEXCEPTION_H
#define RL_HAL_WEISSEXCEPTION_H

#include "DeviceException.h"

namespace rl
{
	namespace hal
	{
		class WeissException : public DeviceException
		{
		public:
			enum Code
			{
				CODE_SUCCESS = 0,
				CODE_NOT_AVAILABLE = 1,
				CODE_NO_SENSOR = 2,
				CODE_NOT_INITIALIZED = 3,
				CODE_ALREADY_RUNNING = 4,
				CODE_FEATURE_NOT_SUPPORTED = 5,
				CODE_INCONSISTENT_DATA = 6,
				CODE_TIMEOUT = 7,
				CODE_READ_ERROR = 8,
				CODE_WRITE_ERROR = 9,
				CODE_INSUFFICIENT_RESOURCES = 10,
				CODE_CHECKSUM_ERROR = 11,
				CODE_NO_PARAM_EXPECTED = 12,
				CODE_NOT_ENOUGH_PARAMS = 13,
				CODE_COMMAND_UNKNOWN = 14,
				CODE_COMMAND_FORMAT_ERROR = 15,
				CODE_ACCESS_DENIED = 16,
				CODE_ALREADY_OPEN = 17,
				CODE_COMMAND_FAILED = 18,
				CODE_COMMAND_ABORTED = 19,
				CODE_INVALID_HANDLE = 20,
				CODE_NOT_FOUND = 21,
				CODE_NOT_OPEN = 22,
				CODE_IO_ERROR = 23,
				CODE_INVALID_PARAMETER = 24,
				CODE_INDEX_OUT_OF_BOUNDS = 25,
				CODE_COMMAND_PENDING = 26,
				CODE_OVERRUN = 27,
				CODE_RANGE_ERROR = 28,
				CODE_AXIS_BLOCKED = 29,
				CODE_FILE_EXISTS = 30
			};
			
			WeissException(const Code& code);
			
			virtual ~WeissException() throw();
			
			Code getCode() const;
			
			virtual const char* what() const throw();
			
		protected:
			
		private:
			Code code;
		};
	};
}

#endif // RL_HAL_WEISSEXCEPTION_H
