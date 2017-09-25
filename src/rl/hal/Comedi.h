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

#ifndef RL_HAL_COMEDI_H
#define RL_HAL_COMEDI_H

#include <comedilib.h>
#include <string>

#include "Com.h"

namespace rl
{
	namespace hal
	{
		class Comedi : public Com
		{
		public:
			Comedi(const ::std::string& filename = "/dev/comedi0");
			
			virtual ~Comedi();
			
			void close();
			
			unsigned int getAref() const;
			
			::std::string getBoardName() const;
			
			comedi_t* getDevice() const;
			
			::std::string getDriverName() const;
			
			int getFileno() const;
			
			lsampl_t getMax(const ::std::size_t& subdevice, const ::std::size_t& channel) const;
			
			::std::size_t getNumChannels(const ::std::size_t& subdevice) const;
			
			::std::size_t getNumRanges(const ::std::size_t& subdevice, const ::std::size_t& channel) const;
			
			::std::size_t getNumSubdevices() const;
			
			unsigned int getRange() const;
			
			::std::size_t getSubdeviceFlags(const ::std::size_t& subdevice) const;
			
			void open();
			
			void read(const ::std::size_t& subdevice, const ::std::size_t& channel, double& data);
			
			void read(const ::std::size_t& subdevice, const ::std::size_t& channel, float& data);
			
			void setAref(const unsigned int& aref);
			
			void setRange(const unsigned int& range);
			
			void write(const ::std::size_t& subdevice, const ::std::size_t& channel, const double& data);
			
			void write(const ::std::size_t& subdevice, const ::std::size_t& channel, const float& data);
			
		protected:
			
		private:
			unsigned int aref;
			
			comedi_t* device;
			
			::std::string filename;
			
			unsigned int range;
		};
	}
}

#endif // RL_HAL_COMEDI_H
