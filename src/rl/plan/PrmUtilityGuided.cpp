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

#include <chrono>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "PrmUtilityGuided.h"
#include "Sampler.h"
#include "SimpleModel.h"

// If this flag is set to true, the implementation is equal to the Brendan Burns version.
//#define ORIGINAL_VERSION

namespace rl
{
	namespace plan
	{
		PrmUtilityGuided::PrmUtilityGuided() :
			Prm(),
			numNeighbors(5), //TODO check for optimal value
			numSamples(3), //TODO check for optimal value
			randDistribution(0, 1),
			randEngine(::std::random_device()()),
			variance(1) //TODO change this value dynamically or use xml-input
		{
		}
		
		PrmUtilityGuided::~PrmUtilityGuided()
		{
		}
		
		void
		PrmUtilityGuided::construct(const ::std::size_t& steps)
		{
			for (::std::size_t i = 0; i < steps; ++i)
			{
				Sample sample(this->model->getDofPosition());
				Sample bestSample(this->model->getDofPosition());
				::rl::math::Real pBest = -1.0f;
				
				// From numSamples samples, get the one with the best probability for being free.
				for (::std::size_t j = 0; j < this->numSamples; ++j)
				{
					sample.isColliding = true;
					
					this->generateEntropyGuidedSample(sample.q);
	
					::rl::math::Real pFree = this->getFreeProbability(sample);
	
#ifdef ORIGINAL_VERSION
					// Used in the paper: take the samples with the least collision probability
					if (pFree > pBest)
					{   
						pBest = pFree;
						bestSample = sample;
					}
#else
					// This works better in our examples. Here we define entropy by using samples where we are unsure, if they are colliding.
					if (::std::fabs(pFree - 0.5f) < ::std::fabs(pBest - 0.5f))
					{   
						pBest = pFree;
						bestSample = sample;
					}
#endif
				}
				
				// now do collision check
				this->model->setPosition(bestSample.q);
				this->model->updateFrames();
				
				if (!this->model->isColliding())
				{
					// store the sample in the graph
					bestSample.isColliding = false;
					Vertex v = this->addVertex(::std::make_shared< ::rl::math::Vector>(bestSample.q));
					this->insert(v);
				}
				
				// Store all previously drawn samples in a vector.
				this->samples.push_back(bestSample);
			}
		}
		
		void
		PrmUtilityGuided::generateEntropyGuidedSample(::rl::math::Vector& q)
		{
			// indices for two random vertices
			// the first sample uses the start ot the end component
#ifdef ORIGINAL_VERSION
			::std::size_t randIndex1 = static_cast< ::std::size_t>(::std::floor(this->rand() * this->getNumVertices()));
#else
			// here we always pick a component containing the beginning or end vertex
			// this prevents roadmap building in remote areas.
			::std::size_t randIndex1 = static_cast< ::std::size_t>(::std::floor(this->rand() * 2.0f));
#endif
			::std::size_t randIndex2 = static_cast< ::std::size_t>(::std::floor(this->rand() * this->getNumVertices()));
			
			// two random vertices
			Vertex sample1 = ::boost::vertex(randIndex1, this->graph);
			Vertex sample2;
			
			do
			{
				// Just take the next vertex to ensure that the same sample is not checked twice.
				++randIndex2;
				
				if (randIndex2 == this->getNumVertices())
				{
					randIndex2 = 0;
				}
				
				sample2 = ::boost::vertex(randIndex2, this->graph);
				// The two vertices have to be from two unconnected components.
			}
			while (this->ds.find_set(sample1) == this->ds.find_set(sample2));
			
			// The point in the middle of the two samples.
			::rl::math::Vector midPoint = 0.5f * (*this->graph[sample1].q + *this->graph[sample2].q);
			
			// add variance drawn randomly from [-variance, variance] to the point
			for (::std::ptrdiff_t i = 0; i < midPoint.size(); ++i)
			{
				q[i] = midPoint[i] + (2.0f * this->rand() - 1.0f) * this->variance;
			}
			
		}
		
		::rl::math::Real
		PrmUtilityGuided::getFreeProbability(const Sample& sample)
		{
			::std::size_t collisionCount = 0;
			
			// Sorted queue for finding the nearest neighbours
			::std::priority_queue<Sample*, ::std::vector<Sample*>, CompareSample> queue;
			
			::std::vector<Sample*>::iterator it;
			// iterate over vector
			// TODO use kd tree!!!
			for (::std::size_t i = 0; i < this->samples.size(); ++i)
			{
				this->samples[i].d = this->model->transformedDistance(sample.q, this->samples[i].q);
				queue.push(&this->samples[i]);
			}
			
			::std::size_t count;
			
			for (count = 0; count < this->numNeighbors && !queue.empty(); ++count)
			{
				if (queue.top()->isColliding)
				{
					++collisionCount;
				}
				
				queue.pop(); 
			}
			
			return 1.0f - static_cast< ::rl::math::Real>(collisionCount) / static_cast< ::rl::math::Real>(count);
		}
		
		::std::string
		PrmUtilityGuided::getName() const
		{
			return "PRM Utility Guided"; 
		}
		
		::std::uniform_real_distribution< ::rl::math::Real>::result_type
		PrmUtilityGuided::rand()
		{
			return this->randDistribution(this->randEngine);
		}
		
		void
		PrmUtilityGuided::seed(const ::std::mt19937::result_type& value)
		{
			this->randEngine.seed(value);
		}
		
		bool
		PrmUtilityGuided::solve()
		{
			// Add the start and end configurations as samples
			Sample first(this->model->getDofPosition());
			first.isColliding = false;
			first.q = *this->start;
			this->samples.push_back(first);
			
			Sample last(this->model->getDofPosition());
			last.isColliding = false;
			last.q = *this->goal;
			this->samples.push_back(last);
			
			return Prm::solve();
		}
		
		PrmUtilityGuided::Sample::Sample(const ::std::size_t& d) :
			d(0),
			isColliding(true),
			q(d)
		{
		}
		
		PrmUtilityGuided::Sample::~Sample()
		{
		}
		
		bool
		PrmUtilityGuided::CompareSample::operator()(const Sample* x, const Sample* y) const
		{
			return x->d > y->d;
		}
	}
}
