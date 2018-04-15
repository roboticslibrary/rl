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
				::rl::math::VectorBlock qi = q.segment(j, this->joints[i]->getDofPosition());
				this->joints[i]->clip(qi);
				q.segment(j, this->joints[i]->getDofPosition()) = qi;
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
				d += this->joints[i]->transformedDistance(
					q1.segment(j, this->joints[i]->getDofPosition()),
					q2.segment(j, this->joints[i]->getDofPosition())
				);
			}
			
			return this->inverseOfTransformedDistance(d);
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
				::rl::math::VectorBlock qi = q.segment(j, this->joints[i]->getDofPosition());
				
				this->joints[i]->interpolate(
					q1.segment(j, this->joints[i]->getDofPosition()),
					q2.segment(j, this->joints[i]->getDofPosition()),
					alpha,
					qi
				);
				
				q.segment(j, this->joints[i]->getDofPosition()) = qi;
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
		
		void
		Metric::normalize(::rl::math::Vector& q) const
		{
			assert(q.size() == this->getDofPosition());
			
			for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i)
			{
				::rl::math::VectorBlock qi = q.segment(j, this->joints[i]->getDofPosition());
				this->joints[i]->normalize(qi);
				q.segment(j, this->joints[i]->getDofPosition()) = qi;
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
				::rl::math::VectorBlock q2i = q2.segment(j, this->joints[i]->getDofPosition());
				
				this->joints[i]->step(
					q1.segment(j, this->joints[i]->getDofPosition()),
					qdot.segment(k, this->joints[i]->getDof()),
					q2i
				);
				
				q2.segment(j, this->joints[i]->getDofPosition()) = q2i;
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
		
		void
		Metric::update()
		{
			Model::update();
			
			this->normalize(this->home);
		}
	}
}
