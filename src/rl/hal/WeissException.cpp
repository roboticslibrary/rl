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
			case CODE_SUCCESS:
				return "No error.";
				break;
			case CODE_NOT_AVAILABLE:
				return "Device, service or data is not available.";
				break;
			case CODE_NO_SENSOR:
				return "No sensor connected.";
				break;
			case CODE_NOT_INITIALIZED:
				return "The device is not initialized.";
				break;
			case CODE_ALREADY_RUNNING:
				return "Service is already running.";
				break;
			case CODE_FEATURE_NOT_SUPPORTED:
				return "The asked feature is not supported.";
				break;
			case CODE_INCONSISTENT_DATA:
				return "One or more dependent parameters mismatch.";
				break;
			case CODE_TIMEOUT:
				return "Timeout error.";
				break;
			case CODE_READ_ERROR:
				return "Error while reading from a device.";
				break;
			case CODE_WRITE_ERROR:
				return "Error while writing to a device.";
				break;
			case CODE_INSUFFICIENT_RESOURCES:
				return "No memory available.";
				break;
			case CODE_CHECKSUM_ERROR:
				return "Checksum error.";
				break;
			case CODE_NO_PARAM_EXPECTED:
				return "No parameters expected.";
				break;
			case CODE_NOT_ENOUGH_PARAMS:
				return "Not enough parameters.";
				break;
			case CODE_COMMAND_UNKNOWN:
				return "Unknown command.";
				break;
			case CODE_COMMAND_FORMAT_ERROR:
				return "Command format error.";
				break;
			case CODE_ACCESS_DENIED:
				return "Access denied.";
				break;
			case CODE_ALREADY_OPEN:
				return "The interface is already open.";
				break;
			case CODE_COMMAND_FAILED:
				return "Command failed.";
				break;
			case CODE_COMMAND_ABORTED:
				return "Command aborted.";
				break;
			case CODE_INVALID_HANDLE:
				return "invalid handle.";
				break;
			case CODE_NOT_FOUND:
				return "device not found.";
				break;
			case CODE_NOT_OPEN:
				return "device not open.";
				break;
			case CODE_IO_ERROR:
				return "I/O error.";
				break;
			case CODE_INVALID_PARAMETER:
				return "invalid parameter.";
				break;
			case CODE_INDEX_OUT_OF_BOUNDS:
				return "index out of bounds.";
				break;
			case CODE_COMMAND_PENDING:
				return "Command was received correctly, but the execution needs more time. If the command was completely processed, another status message is returned indicating the command's result.";
				break;
			case CODE_OVERRUN:
				return "Data overrun.";
				break;
			case CODE_RANGE_ERROR:
				return "Range error.";
				break;
			case CODE_AXIS_BLOCKED:
				return "Axis is blocked.";
				break;
			case CODE_FILE_EXISTS:
				return "File already exists.";
				break;
			default:
				return "Unknown error.";
				break;
			}
		}
	}
}
