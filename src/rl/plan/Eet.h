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

#ifndef RL_PLAN_EET_H
#define RL_PLAN_EET_H

#include <random>
#include <rl/math/GnatNearestNeighbors.h>

#include "RrtCon.h"
#include "WorkspaceMetric.h"

namespace rl
{
	namespace plan
	{
		class WorkspaceSphereExplorer;
		
		/**
		 * Exploring/Exploiting Trees
		 * 
		 * Markus Rickert, Arne Sieverling, and Oliver Brock. Balancing exploration
		 * and exploitation in sampling-based motion planning. IEEE Transactions on
		 * Robotics, 30(6):1305-1317, December 2014.
		 * 
		 * http://dx.doi.org/10.1109/TRO.2014.2340191
		 */
		class Eet : public RrtCon
		{
		public:
			struct ExplorerSetup
			{
				/** Goal configuration reference for workspace explorer setup. */
				::rl::math::Vector* goalConfiguration;
				
				/** Goal frame for workspace explorer in goal configuration reference. */
				int goalFrame;
				
				/** Start configuration reference for workspace explorer setup. */
				::rl::math::Vector* startConfiguration;
				
				/** Start frame for workspace explorer in start configuration reference. */
				int startFrame;
			};
			
			Eet();
			
			virtual ~Eet();
			
			::std::chrono::steady_clock::duration getExplorationDuration() const;
			
			virtual ::std::string getName() const;
			
			virtual ::std::size_t getNumEdges() const;
			
			virtual ::std::size_t getNumVertices() const;
			
			VectorList getPath();
			
			void seed(const ::std::mt19937::result_type& value);
			
			void reset();
			
			bool solve();
			
			/** Control increase/decrease of exploitation. */
			::rl::math::Real alpha;
			
			/** Better performance in certain scenarios. */
			bool alternativeDistanceComputation;
			
			/** Threshold for switching to uniform orientation sampling. */
			::rl::math::Real beta;
			
			/** Weight factor translation vs.\ orientation. */
			::rl::math::Real distanceWeight;
			
			::std::vector<WorkspaceSphereExplorer*> explorers;
			
			::std::vector<ExplorerSetup> explorersSetup;
			
			/** Initialization value for exploration/exploitation balance. */
			::rl::math::Real gamma;
			
			/** Epsilon for goal state. */
			::rl::math::Real goalEpsilon;
			
			/** Include frame orientation in goal epsilon calculation. */
			bool goalEpsilonUseOrientation;
			
			/** Upper workspace limit for workspace frame sampling. */
			::rl::math::Vector3 max;
			
			/** Lower workspace limit for workspace frame sampling. */
			::rl::math::Vector3 min;
			
		protected:
			struct VertexBundle : Rrt::VertexBundle
			{
				TransformPtr t;
			};
			
			Edge addEdge(const Vertex& u, const Vertex& v, Tree& tree);
			
			Vertex addVertex(Tree& tree, const VectorPtr& q);
			
			using RrtCon::connect;
			
			Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Transform& chosen);
			
			::rl::math::Real distance(const ::rl::math::Transform& t1, const ::rl::math::Transform& t2) const;
			
			int expand(const VertexBundle* nearest, const ::rl::math::Transform& nearest2, const ::rl::math::Transform& chosen, const ::rl::math::Real& distance, VertexBundle& expanded);
			
			using RrtCon::extend;
			
			Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Transform& chosen);
			
			::std::normal_distribution< ::rl::math::Real>::result_type gauss();
			
			static VertexBundle* get(const Tree& tree, const Vertex& v);
			
			using RrtCon::nearest;
			
			Neighbor nearest(const Tree& tree, const ::rl::math::Transform& chosen);
			
			::std::uniform_real_distribution< ::rl::math::Real>::result_type rand();
			
			::std::normal_distribution< ::rl::math::Real> gaussDistribution;
			
			::std::mt19937 gaussEngine;
			
			::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
			
			::std::mt19937 randEngine;
			
		private:
			::std::chrono::steady_clock::time_point explorationTimeStart;
			
			::std::chrono::steady_clock::time_point explorationTimeStop;
			
			::rl::math::GnatNearestNeighbors<WorkspaceMetric> nn;
		};
	}
}

#endif // RL_PLAN_EET_H
