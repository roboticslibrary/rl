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

#include "WeissException.h"

namespace rl
{
	namespace hal
	{
		constexpr WeissException::Code WeissException::CODE_SUCCESS;
		constexpr WeissException::Code WeissException::CODE_NOT_AVAILABLE;
		constexpr WeissException::Code WeissException::CODE_NO_SENSOR;
		constexpr WeissException::Code WeissException::CODE_NOT_INITIALIZED;
		constexpr WeissException::Code WeissException::CODE_ALREADY_RUNNING;
		constexpr WeissException::Code WeissException::CODE_FEATURE_NOT_SUPPORTED;
		constexpr WeissException::Code WeissException::CODE_INCONSISTENT_DATA;
		constexpr WeissException::Code WeissException::CODE_TIMEOUT;
		constexpr WeissException::Code WeissException::CODE_READ_ERROR;
		constexpr WeissException::Code WeissException::CODE_WRITE_ERROR;
		constexpr WeissException::Code WeissException::CODE_INSUFFICIENT_RESOURCES;
		constexpr WeissException::Code WeissException::CODE_CHECKSUM_ERROR;
		constexpr WeissException::Code WeissException::CODE_NO_PARAM_EXPECTED;
		constexpr WeissException::Code WeissException::CODE_NOT_ENOUGH_PARAMS;
		constexpr WeissException::Code WeissException::CODE_COMMAND_UNKNOWN;
		constexpr WeissException::Code WeissException::CODE_COMMAND_FORMAT_ERROR;
		constexpr WeissException::Code WeissException::CODE_ACCESS_DENIED;
		constexpr WeissException::Code WeissException::CODE_ALREADY_OPEN;
		constexpr WeissException::Code WeissException::CODE_COMMAND_FAILED;
		constexpr WeissException::Code WeissException::CODE_COMMAND_ABORTED;
		constexpr WeissException::Code WeissException::CODE_INVALID_HANDLE;
		constexpr WeissException::Code WeissException::CODE_NOT_FOUND;
		constexpr WeissException::Code WeissException::CODE_NOT_OPEN;
		constexpr WeissException::Code WeissException::CODE_IO_ERROR;
		constexpr WeissException::Code WeissException::CODE_INVALID_PARAMETER;
		constexpr WeissException::Code WeissException::CODE_INDEX_OUT_OF_BOUNDS;
		constexpr WeissException::Code WeissException::CODE_COMMAND_PENDING;
		constexpr WeissException::Code WeissException::CODE_OVERRUN;
		constexpr WeissException::Code WeissException::CODE_RANGE_ERROR;
		constexpr WeissException::Code WeissException::CODE_AXIS_BLOCKED;
		constexpr WeissException::Code WeissException::CODE_FILE_EXISTS;
		
		WeissException::WeissException(const Code& code) :
			DeviceException(""),
			code(code)
		{
		}
		
		WeissException::~WeissException() throw()
		{
		}
		
		WeissException::Code
		WeissException::getCode() const
		{
			return this->code;
		}
		
		const char*
		WeissException::what() const throw()
		{
			switch (this->code)
			{
			case Code::success:
				return "No error.";
				break;
			case Code::notAvailable:
				return "Device, service or data is not available.";
				break;
			case Code::noSensor:
				return "No sensor connected.";
				break;
			case Code::notInitialized:
				return "The device is not initialized.";
				break;
			case Code::alreadyRunning:
				return "Service is already running.";
				break;
			case Code::featureNotSupported:
				return "The asked feature is not supported.";
				break;
			case Code::inconsistentData:
				return "One or more dependent parameters mismatch.";
				break;
			case Code::timeout:
				return "Timeout error.";
				break;
			case Code::readError:
				return "Error while reading from a device.";
				break;
			case Code::writeError:
				return "Error while writing to a device.";
				break;
			case Code::insufficientResources:
				return "No memory available.";
				break;
			case Code::checksumError:
				return "Checksum error.";
				break;
			case Code::noParamExpected:
				return "No parameters expected.";
				break;
			case Code::notEnoughParams:
				return "Not enough parameters.";
				break;
			case Code::commandUnknown:
				return "Unknown command.";
				break;
			case Code::commandFormatError:
				return "Command format error.";
				break;
			case Code::accessDenied:
				return "Access denied.";
				break;
			case Code::alreadyOpen:
				return "The interface is already open.";
				break;
			case Code::commandFailed:
				return "Command failed.";
				break;
			case Code::commandAborted:
				return "Command aborted.";
				break;
			case Code::invalidHandle:
				return "invalid handle.";
				break;
			case Code::notFound:
				return "device not found.";
				break;
			case Code::notOpen:
				return "device not open.";
				break;
			case Code::ioError:
				return "I/O error.";
				break;
			case Code::invalidParameter:
				return "invalid parameter.";
				break;
			case Code::indexOutOfBounds:
				return "index out of bounds.";
				break;
			case Code::commandPending:
				return "Command was received correctly, but the execution needs more time. If the command was completely processed, another status message is returned indicating the command's result.";
				break;
			case Code::overrun:
				return "Data overrun.";
				break;
			case Code::rangeError:
				return "Range error.";
				break;
			case Code::axisBlocked:
				return "Axis is blocked.";
				break;
			case Code::fileExists:
				return "File already exists.";
				break;
			default:
				return "Unknown error.";
				break;
			}
		}
	}
}
