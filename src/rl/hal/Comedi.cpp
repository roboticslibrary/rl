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

#include "Comedi.h"
#include "ComException.h"

namespace rl
{
	namespace hal
	{
		Comedi::Comedi(const ::std::string& filename) :
			Com(),
			aref(AREF_GROUND),
			device(nullptr),
			filename(filename),
			range(0)
		{
		}
		
		Comedi::~Comedi()
		{
			if (this->isConnected())
			{
				this->close();
			}
		}
		
		void
		Comedi::close()
		{
			if (nullptr != this->device)
			{
				comedi_close(this->device);
			}
			
			this->setConnected(false);
		}
		
		unsigned int
		Comedi::getAref() const
		{
			return this->aref;
		}
		
		::std::string
		Comedi::getBoardName() const
		{
			const char* name = comedi_get_board_name(this->device);
			
			if (nullptr == name)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			return name;
		}
		
		comedi_t*
		Comedi::getDevice() const
		{
			return this->device;
		}
		
		::std::string
		Comedi::getDriverName() const
		{
			const char* name = comedi_get_driver_name(this->device);
			
			if (nullptr == name)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			return name;
		}
		
		int
		Comedi::getFileno() const
		{
			int fileno = comedi_fileno(this->device);
			
			if (-1 == fileno)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			return fileno;
		}
		
		lsampl_t
		Comedi::getMax(const ::std::size_t& subdevice, const ::std::size_t& channel) const
		{
			lsampl_t max = comedi_get_maxdata(this->device, subdevice, channel);
			
			if (0 == max)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			return max;
		}
		
		::std::size_t
		Comedi::getNumChannels(const ::std::size_t& subdevice) const
		{
			int numChannels = comedi_get_n_channels(this->device, subdevice);
			
			if (-1 == numChannels)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			return numChannels;
		}
		
		::std::size_t
		Comedi::getNumRanges(const ::std::size_t& subdevice, const ::std::size_t& channel) const
		{
			int numRanges = comedi_get_n_ranges(this->device, subdevice, channel);
			
			if (-1 == numRanges)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			return numRanges;
		}
		
		::std::size_t
		Comedi::getNumSubdevices() const
		{
			return comedi_get_n_subdevices(this->device);
		}
		
		unsigned int
		Comedi::getRange() const
		{
			return this->range;
		}
		
		::std::size_t
		Comedi::getSubdeviceFlags(const ::std::size_t& subdevice) const
		{
			int subdeviceFlags = comedi_get_subdevice_flags(this->device, subdevice);
			
			if (-1 == subdeviceFlags)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			return subdeviceFlags;
		}
		
		void
		Comedi::open()
		{
			this->device = comedi_open(this->filename.c_str());
			
			if (nullptr == this->device)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			this->setConnected(true);
		}
		
		void
		Comedi::read(const ::std::size_t& subdevice, const ::std::size_t& channel, double& data)
		{
			lsampl_t maxdata = comedi_get_maxdata(this->device, subdevice, channel);
			
			if (0 == maxdata)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			lsampl_t sample;
			
			if (-1 == comedi_data_read(this->device, subdevice, channel, this->range, this->aref, &sample))
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			comedi_range* range = comedi_get_range(this->device, subdevice, channel, this->range);
			
			if (nullptr == range)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			data = comedi_to_phys(sample, range, maxdata);
		}
		
		void
		Comedi::read(const ::std::size_t& subdevice, const ::std::size_t& channel, float& data)
		{
			lsampl_t maxdata = comedi_get_maxdata(this->device, subdevice, channel);
			
			if (0 == maxdata)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			lsampl_t sample;
			
			if (-1 == comedi_data_read(this->device, subdevice, channel, this->range, this->aref, &sample))
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			comedi_range* range = comedi_get_range(this->device, subdevice, channel, this->range);
			
			if (nullptr == range)
			{
				throw ComException(comedi_strerror(comedi_errno()));
			}
			
			data = comedi_to_phys(sample, range, maxdata);
		}
		
		void
		Comedi::setAref(const unsigned int& aref)
		{
			this->aref = aref;
		}
		
		void
		Comedi::setRange(const unsigned int& range)
		{
			this->range = range;
		}
		
		void
		Comedi::write(const ::std::size_t& subdevice, const ::std::size_t& channel, const double& data)
		{
		}
		
		void
		Comedi::write(const ::std::size_t& subdevice, const ::std::size_t& channel, const float& data)
		{
		}
	}
}
