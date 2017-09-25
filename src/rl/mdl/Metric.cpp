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

#include "Joint.h"
#include "Metric.h"

namespace rl
{
	namespace mdl
	{
		Metric::Metric() :
			Model()
		{
		}
		
		Metric::~Metric()
		{
		}
		
		void
		Metric::clip(::rl::math::Vector& q) const
		{
			assert(q.size() == this->getDofPosition());
			
			for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i)
			{
				::rl::math::Vector qi = q.segment(j, this->joints[i]->getDofPosition()); // TODO
				this->joints[i]->clip(qi);
				q.segment(j, this->joints[i]->getDofPosition()) = qi; // TODO
			}
		}
		
		Model*
		Metric::clone() const
		{
			return new Metric(*this);
		}
		
		::rl::math::Real
		Metric::distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			assert(q1.size() == this->getDofPosition());
			assert(q2.size() == this->getDofPosition());
			
			::rl::math::Real d = 0;
			
			for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i)
			{
				d += this->joints[i]->distance(
					q1.segment(j, this->joints[i]->getDofPosition()),
					q2.segment(j, this->joints[i]->getDofPosition())
				);
			}
			
			return d;
		}
		
		void
		Metric::interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const
		{
			assert(q1.size() == this->getDofPosition());
			assert(q2.size() == this->getDofPosition());
			assert(alpha >= 0.0f);
			assert(alpha <= 1.0f);
			assert(q.size() == this->getDofPosition());
			
			for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i)
			{
				::rl::math::Vector qi = q.segment(j, this->joints[i]->getDofPosition()); // TODO
				
				this->joints[i]->interpolate(
					q1.segment(j, this->joints[i]->getDofPosition()),
					q2.segment(j, this->joints[i]->getDofPosition()),
					alpha,
					qi
				);
				
				q.segment(j, this->joints[i]->getDofPosition()) = qi; // TODO
			}
		}
		
		::rl::math::Real
		Metric::inverseOfTransformedDistance(const ::rl::math::Real& d) const
		{
			return ::std::sqrt(d);
		}
		
		bool
		Metric::isValid(const ::rl::math::Vector& q) const
		{
			assert(q.size() == this->getDofPosition());
			
			for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i)
			{
				if (!this->joints[i]->isValid(q.segment(j, this->joints[i]->getDofPosition())))
				{
					return false;
				}
			}
			
			return true;
		}
		
		::rl::math::Real
		Metric::maxDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const
		{
			::rl::math::Real d = 0;
			
			::std::size_t k = 0;
			
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				for (::std::size_t j = 0; j < this->joints[i]->getDofPosition(); ++j)
				{
					::rl::math::Real delta = ::std::max(::std::abs(q(k) - min(k)), ::std::abs(q(k) - max(k)));
					
					if (this->joints[i]->wraparound(j))
					{
						::rl::math::Real range = ::std::abs(this->joints[i]->max(j) - this->joints[i]->min(j));
						d += this->transformedDistance(::std::max(delta, ::std::abs(range - delta)));
					}
					else
					{
						d += this->transformedDistance(delta);
					}
					
					++k;
				}
			}
			
			return d;
		}
		
		::rl::math::Real
		Metric::minDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const
		{
			::rl::math::Real d = 0;
			
			for (::std::size_t i = 0; i < this->getDofPosition(); ++i)
			{
				d += this->transformedDistance(this->minDistanceToRectangle(q(i), min(i), max(i), i));
			}
			
			return d;
		}
		
		::rl::math::Real
		Metric::minDistanceToRectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cuttingDimension) const
		{
			::rl::math::Real d = 0;
#if 0 // TODO
			
			if (q < min || q > max)
			{
				::rl::math::Real delta = ::std::min(::std::abs(q - min), ::std::abs(q - max));
				
				if (this->joints[cuttingDimension]->wraparound)
				{
					::rl::math::Real range = ::std::abs(this->joints[cuttingDimension]->max - this->joints[cuttingDimension]->min);
					::rl::math::Real size = ::std::abs(max - min);
					d += ::std::min(delta, ::std::abs(range - size - delta));
				}
				else
				{
					d += delta;
				}
			}
			
#endif
			return d;
		}
		
		::rl::math::Real
		Metric::newDistance(const ::rl::math::Real& dist, const ::rl::math::Real& oldOff, const ::rl::math::Real& newOff, const int& cuttingDimension) const
		{
			return dist - this->transformedDistance(oldOff) + this->transformedDistance(newOff);
		}
		
		void
		Metric::normalize(::rl::math::Vector& q) const
		{
			assert(q.size() == this->getDofPosition());
			
			for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i)
			{
				::rl::math::Vector qi = q.segment(j, this->joints[i]->getDofPosition()); // TODO
				this->joints[i]->normalize(qi);
				q.segment(j, this->joints[i]->getDofPosition()) = qi; // TODO
			}
		}
		
		void
		Metric::step(const ::rl::math::Vector& q1, const ::rl::math::Vector& qdot, ::rl::math::Vector& q2) const
		{
			assert(q1.size() == this->getDofPosition());
			assert(qdot.size() == this->getDof());
			assert(q2.size() == this->getDofPosition());
			
			for (::std::size_t i = 0, j = 0, k = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), k += this->joints[i]->getDof(), ++i)
			{
				::rl::math::Vector q2i = q2.segment(j, this->joints[i]->getDofPosition()); // TODO
				
				this->joints[i]->step(
					q1.segment(j, this->joints[i]->getDofPosition()),
					qdot.segment(k, this->joints[i]->getDof()),
					q2i
				);
				
				q2.segment(j, this->joints[i]->getDofPosition()) = q2i; // TODO
			}
		}
		
		::rl::math::Real
		Metric::transformedDistance(const ::rl::math::Real& d) const
		{
			return ::std::pow(d, 2);
		}
		
		::rl::math::Real
		Metric::transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			assert(q1.size() == this->getDofPosition());
			assert(q2.size() == this->getDofPosition());
			
			::rl::math::Real d = 0;
			
			for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i)
			{
				d += this->joints[i]->transformedDistance(
					q1.segment(j, this->joints[i]->getDofPosition()),
					q2.segment(j, this->joints[i]->getDofPosition())
				);
			}
			
			return d;
		}
		
		::rl::math::Real
		Metric::transformedDistance(const ::rl::math::Real& q1, const ::rl::math::Real& q2, const ::std::size_t& i) const
		{
			::rl::math::Real delta = ::std::abs(q1 - q2);
			
			if (this->joints[i]->wraparound(0))
			{
				::rl::math::Real range = ::std::abs(this->joints[i]->max(0) - this->joints[i]->min(0));
				return this->transformedDistance(::std::max(delta, ::std::abs(range - delta)));
			}
			else
			{
				return this->transformedDistance(delta);
			}
		}
	}
}
