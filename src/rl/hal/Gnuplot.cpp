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

#include <cassert>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <rl/math/Unit.h>

#include "ComException.h"
#include "Gnuplot.h"

#ifdef WIN32
#ifndef popen
#define popen _popen
#endif // popen
#ifndef pclose
#define pclose _pclose
#endif // pclose
#define GNUPLOT "pgnuplot.exe"
#else // WIN32
#define GNUPLOT "gnuplot"
#endif // WIN32

namespace rl
{
	namespace hal
	{
		Gnuplot::Gnuplot(
			const ::std::size_t& dof,
			const ::rl::math::Real& updateRate,
			const ::rl::math::Real& ymin,
			const ::rl::math::Real& ymax,
			const ::std::size_t& max
		) :
			AxisController(dof, updateRate),
			JointPositionActuator(dof, updateRate),
			fp(NULL),
			history(),
			max(max),
			ymax(ymax),
			ymin(ymin)
		{
		}
		
		Gnuplot::~Gnuplot()
		{
		}
		
		void
		Gnuplot::close()
		{
			if (-1 == pclose(this->fp))
			{
				throw ComException(strerror(errno));
			}
			
			this->setConnected(false);
		}
		
		void
		Gnuplot::open()
		{
			this->fp = popen(GNUPLOT, "w");
			
			if (NULL == this->fp)
			{
				throw ComException(strerror(errno));
			}
			
			fprintf(this->fp, "set xrange [0:%zi]\n", this->max);
			
			this->setRange(this->ymin, this->ymax);
			
			this->setConnected(true);
		}
		
		void
		Gnuplot::setJointPosition(const ::rl::math::Vector& q)
		{
			assert(this->getDof() == q.size());
			
			this->history.push_back(q);
			
			if (this->history.size() > this->max)
			{
				this->history.pop_front();
			}
		}
		
		void
		Gnuplot::setRange(const ::rl::math::Real& ymin, const ::rl::math::Real& ymax)
		{
			this->ymin = ymin;
			this->ymax =  ymax;
			
			fprintf(this->fp, "set yrange [%f:%f]\n", this->ymin * ::rl::math::RAD2DEG, this->ymax * ::rl::math::RAD2DEG);
		}
		
		void
		Gnuplot::start()
		{
			this->history.clear();
		}
		
		void
		Gnuplot::step()
		{
			if (!this->history.empty())
			{
				fprintf(this->fp, "plot");
				
				for (::std::size_t i = 0; i < this->getDof(); ++i)
				{
					if (i > 0)
					{
						fprintf(this->fp, ",");
					}
					
					fprintf(this->fp, " '-' with lines");
				}
				
				fprintf(this->fp, "\n");
				
				for (::std::size_t i = 0; i < this->getDof(); ++i)
				{
					::std::size_t t = this->max - this->history.size();
					
					for (::std::list< ::rl::math::Vector >::iterator j = this->history.begin(); j != this->history.end(); ++j)
					{
						fprintf(this->fp, "%zi %f\n", t, (*j)(i) * ::rl::math::RAD2DEG);
						++t;
					}
					
					fprintf(this->fp, "e\n");
				}
				
				fflush(this->fp);
			}
		}
		
		void
		Gnuplot::stop()
		{
		}
	}
}
