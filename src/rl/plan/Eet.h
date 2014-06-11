//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//

#ifndef _RL_PLAN_EET_H_
#define _RL_PLAN_EET_H_

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <rl/util/Timer.h>

#include "RrtCon.h"

namespace rl
{
	namespace plan
	{
		class WorkspaceSphereExplorer;
		
		/**
		 * Exploring/Exploiting Trees
		 * 
		 * Markus Rickert, Oliver Brock, and Alois Knoll. Balancing exploration
		 * and exploitation in motion planning. In Proceedings of the IEEE
		 * International Conference on Robotics and Automation, pages 2812-2817,
		 * Pasadena, CA, USA, May 2008.
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
			
			::rl::math::Real getExplorationTime() const;
			
			virtual ::std::string getName() const;
			
			virtual ::std::size_t getNumEdges() const;
			
			virtual ::std::size_t getNumVertices() const;
			
			void getPath(VectorList& path);
			
			void seed(const ::boost::mt19937::result_type& value);
			
			void reset();
			
			bool solve();
			
			/** Control increase/decrease of exploitation. */
			::rl::math::Real alpha;
			
			/** Better performance in certain scenarios. */
			bool alternativeDistanceComputation;
			
			/** Weight factor translation vs. orientation. */
			::rl::math::Real distanceWeight;
			
			::std::vector< WorkspaceSphereExplorer* > explorers;
			
			::std::vector< ExplorerSetup > explorersSetup;
			
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
			using RrtCon::connect;
			
			using RrtCon::extend;
			
			using RrtCon::nearest;
			
			Edge addEdge(const Vertex& u, const Vertex& v, Tree& tree);
			
			Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Transform& chosen);
			
			::rl::math::Real distance(const ::rl::math::Transform& t1, const ::rl::math::Transform& t2) const;
			
			int expand(const VertexBundle& nearest, const ::rl::math::Transform& nearest2, const ::rl::math::Transform& chosen, const ::rl::math::Real& distance, VertexBundle& expanded);
			
			Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Transform& chosen);
			
			Neighbor nearest(const Tree& tree, const ::rl::math::Transform& chosen);
			
			::boost::variate_generator< ::boost::mt19937, ::boost::normal_distribution< ::rl::math::Real > > gauss;
			
			::boost::variate_generator< ::boost::mt19937, ::boost::uniform_real< ::rl::math::Real > > rand;
			
		private:
			::rl::util::Timer exploration;
			
			/** Vertices for nearest neighbor search. */
			::std::vector< Vertex > selected;
			
			::rl::util::Timer timer;
		};
	}
}

#endif // _RL_PLAN_EET_H_
