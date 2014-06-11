//
// Copyright (c) 2011, Arne Sieverling
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
// * Neither the name of  the Technische Universitaet Muenchen nor the names of
//   its contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
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

#ifndef _RL_PLAN_PRMUTILITYGUIDED_H_
#define _RL_PLAN_PRMUTILITYGUIDED_H_

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include "Prm.h"

namespace rl
{
	namespace plan
	{
		/**
		 * Probabilistic Roadmap using Utility-Guided Sampling.
		 */
		class PrmUtilityGuided : public Prm
		{
		public:
			PrmUtilityGuided();
			
			virtual ~PrmUtilityGuided();
			
			void construct(const ::std::size_t& steps);
			
			::std::string getName() const;
			
			void seed(const ::boost::mt19937::result_type& value);
			
			bool solve();
			
		private:
			class Sample
			{
			public:
				Sample(const ::std::size_t& d);
				
				virtual ~Sample();
				
				::rl::math::Real d;
				
				bool isColliding;
				
				::rl::math::Vector q;
			};
			
			/** A compare structure for nearest neighbor sorting. */
			struct CompareSample
			{
				bool operator()(const Sample* x, const Sample* y) const;
			};
			
			/** Samples a point near the middle (+/- variance) of two random nodes from unconnected components of the graph. */
			void generateEntropyGuidedSample(::rl::math::Vector& q);
			
			/**
			* Get an estimated probability that a sample is not colliding with the scene.
			* Here we look how many of the numNeigbors nearest neighbors of the sample
			* are colliding and return "#ofFreeNeighbors"/"#Neighbors".
			*/
			::rl::math::Real getFreeProbability(const Sample& sample);
			
			::std::size_t numNeighbors;
			
			::std::size_t numSamples;
			
			::boost::variate_generator< ::boost::mt19937, ::boost::uniform_real< ::rl::math::Real > > rand;
			
			::std::vector< Sample > samples;
			
			::rl::math::Real variance;
		};
	}
}

#endif // _RL_PLAN_PRMUTILITYGUIDED_H_
