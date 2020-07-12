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
		class RL_HAL_EXPORT WeissException : public DeviceException
		{
		public:
			enum class Code
			{
				success = 0,
				notAvailable = 1,
				noSensor = 2,
				notInitialized = 3,
				alreadyRunning = 4,
				featureNotSupported = 5,
				inconsistentData = 6,
				timeout = 7,
				readError = 8,
				writeError = 9,
				insufficientResources = 10,
				checksumError = 11,
				noParamExpected = 12,
				notEnoughParams = 13,
				commandUnknown = 14,
				commandFormatError = 15,
				accessDenied = 16,
				alreadyOpen = 17,
				commandFailed = 18,
				commandAborted = 19,
				invalidHandle = 20,
				notFound = 21,
				notOpen = 22,
				ioError = 23,
				invalidParameter = 24,
				indexOutOfBounds = 25,
				commandPending = 26,
				overrun = 27,
				rangeError = 28,
				axisBlocked = 29,
				fileExists = 30
			};
			
			RL_HAL_DEPRECATED static constexpr Code CODE_SUCCESS = Code::success;
			RL_HAL_DEPRECATED static constexpr Code CODE_NOT_AVAILABLE = Code::notAvailable;
			RL_HAL_DEPRECATED static constexpr Code CODE_NO_SENSOR = Code::noSensor;
			RL_HAL_DEPRECATED static constexpr Code CODE_NOT_INITIALIZED = Code::notInitialized;
			RL_HAL_DEPRECATED static constexpr Code CODE_ALREADY_RUNNING = Code::alreadyRunning;
			RL_HAL_DEPRECATED static constexpr Code CODE_FEATURE_NOT_SUPPORTED = Code::featureNotSupported;
			RL_HAL_DEPRECATED static constexpr Code CODE_INCONSISTENT_DATA = Code::inconsistentData;
			RL_HAL_DEPRECATED static constexpr Code CODE_TIMEOUT = Code::timeout;
			RL_HAL_DEPRECATED static constexpr Code CODE_READ_ERROR = Code::readError;
			RL_HAL_DEPRECATED static constexpr Code CODE_WRITE_ERROR = Code::writeError;
			RL_HAL_DEPRECATED static constexpr Code CODE_INSUFFICIENT_RESOURCES = Code::insufficientResources;
			RL_HAL_DEPRECATED static constexpr Code CODE_CHECKSUM_ERROR = Code::checksumError;
			RL_HAL_DEPRECATED static constexpr Code CODE_NO_PARAM_EXPECTED = Code::noParamExpected;
			RL_HAL_DEPRECATED static constexpr Code CODE_NOT_ENOUGH_PARAMS = Code::notEnoughParams;
			RL_HAL_DEPRECATED static constexpr Code CODE_COMMAND_UNKNOWN = Code::commandUnknown;
			RL_HAL_DEPRECATED static constexpr Code CODE_COMMAND_FORMAT_ERROR = Code::commandFormatError;
			RL_HAL_DEPRECATED static constexpr Code CODE_ACCESS_DENIED = Code::accessDenied;
			RL_HAL_DEPRECATED static constexpr Code CODE_ALREADY_OPEN = Code::alreadyOpen;
			RL_HAL_DEPRECATED static constexpr Code CODE_COMMAND_FAILED = Code::commandFailed;
			RL_HAL_DEPRECATED static constexpr Code CODE_COMMAND_ABORTED = Code::commandAborted;
			RL_HAL_DEPRECATED static constexpr Code CODE_INVALID_HANDLE = Code::invalidHandle;
			RL_HAL_DEPRECATED static constexpr Code CODE_NOT_FOUND = Code::notFound;
			RL_HAL_DEPRECATED static constexpr Code CODE_NOT_OPEN = Code::notOpen;
			RL_HAL_DEPRECATED static constexpr Code CODE_IO_ERROR = Code::ioError;
			RL_HAL_DEPRECATED static constexpr Code CODE_INVALID_PARAMETER = Code::invalidParameter;
			RL_HAL_DEPRECATED static constexpr Code CODE_INDEX_OUT_OF_BOUNDS = Code::indexOutOfBounds;
			RL_HAL_DEPRECATED static constexpr Code CODE_COMMAND_PENDING = Code::commandPending;
			RL_HAL_DEPRECATED static constexpr Code CODE_OVERRUN = Code::overrun;
			RL_HAL_DEPRECATED static constexpr Code CODE_RANGE_ERROR = Code::rangeError;
			RL_HAL_DEPRECATED static constexpr Code CODE_AXIS_BLOCKED = Code::axisBlocked;
			RL_HAL_DEPRECATED static constexpr Code CODE_FILE_EXISTS = Code::fileExists;
			
			WeissException(const Code& code);
			
			virtual ~WeissException() throw();
			
			Code getCode() const;
			
			virtual const char* what() const throw();
			
		protected:
			
		private:
			Code code;
		};
	}
}

#endif // RL_HAL_WEISSEXCEPTION_H
