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

#ifndef _RL_HAL_DC1394CAMERAEXCEPTION_H_
#define _RL_HAL_DC1394CAMERAEXCEPTION_H_

#if (LIBDC1394_VERSION_MAJOR > 10)
#include <dc1394/dc1394.h>
#else
#include <libdc1394/dc1394_control.h>
#endif

#include "DeviceException.h"

namespace rl
{
	namespace hal
	{
		class Dc1394CameraException : public DeviceException
		{
		public:
#if (LIBDC1394_VERSION_MAJOR > 10)
			Dc1394CameraException(const dc1394error_t& error);
#else
			Dc1394CameraException(const int& error);
#endif
			
			virtual ~Dc1394CameraException() throw();
			
#if (LIBDC1394_VERSION_MAJOR > 10)
			dc1394error_t getError() const;
#else
			int getError() const;
#endif
			
			virtual const char* what() const throw();
			
		protected:
			
		private:
#if (LIBDC1394_VERSION_MAJOR > 10)
			dc1394error_t error;
#else
			int error;
#endif
		};
	}
}

#endif // _RL_HAL_DC1394CAMERAEXCEPTION_H_
