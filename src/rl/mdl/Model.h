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

#ifndef RL_MDL_MODEL_H
#define RL_MDL_MODEL_H

#include <memory>
#include <string>
#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

#include "Frame.h"
#include "Transform.h"

namespace rl
{
	/**
	 * Rigid body kinematics and dynamics.
	 * 
	 * Roy Featherstone. Rigid Body Dynamics Algorithms. Springer, New
	 * York, NY, USA, 2008.
	 */
	namespace mdl
	{
		class Body;
		class Compound;
		class Joint;
		
		class Model
		{
		public:
			Model();
			
			virtual ~Model();
			
			void add(Compound* compound, const Frame* a, const Frame* b);
			
			void add(Frame* frame);
			
			void add(Transform* transform, const Frame* a, const Frame* b);
			
			bool areColliding(const ::std::size_t& i, const ::std::size_t& j) const;
			
			Model* clone() const;
			
			::rl::math::Vector generatePositionGaussian(const ::rl::math::Vector& rand, const ::rl::math::Vector& mean, const ::rl::math::Vector& sigma) const;
			
			::rl::math::Vector generatePositionUniform(const ::rl::math::Vector& rand) const;
			
			::rl::math::Vector getAcceleration() const;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> getAccelerationUnits() const;
			
			::std::size_t getBodies() const;
			
			Body* getBody(const ::std::size_t& i) const;
			
			::std::size_t getDof() const;
			
			::std::size_t getDofPosition() const;
			
			const ::rl::math::Transform& getFrame(const ::std::size_t& i) const;
			
			const ::rl::math::Matrix& getGammaPosition() const;
			
			const ::rl::math::Matrix& getGammaVelocity() const;
			
			const ::rl::math::Matrix& getGammaPositionInverse() const;
			
			const ::rl::math::Matrix& getGammaVelocityInverse() const;
			
			::rl::math::Vector getHomePosition() const;
			
			Joint* getJoint(const ::std::size_t& i) const;
			
			::std::size_t getJoints() const;
			
			const ::rl::math::MotionVector& getOperationalAcceleration(const ::std::size_t& i) const;
			
			::std::size_t getOperationalDof() const;
			
			const ::rl::math::ForceVector& getOperationalForce(const ::std::size_t& i) const;
			
			const ::rl::math::Transform& getOperationalPosition(const ::std::size_t& i) const;
			
			const ::rl::math::MotionVector& getOperationalVelocity(const ::std::size_t& i) const;
			
			const ::std::string& getManufacturer() const;
			
			::rl::math::Vector getMaximum() const;
			
			::rl::math::Vector getMinimum() const;
			
			const ::std::string& getName() const;
			
			::rl::math::Vector getPosition() const;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> getPositionUnits() const;
			
			::rl::math::Vector getTorque() const;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> getTorqueUnits() const;
			
			::rl::math::Vector getSpeed() const;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> getSpeedUnits() const;
			
			::rl::math::Vector getVelocity() const;
			
			::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1> getVelocityUnits() const;
			
			bool isColliding(const ::std::size_t& i) const;
			
			void replace(Compound* compound, Transform* transform);
			
			void replace(Transform* transform, Compound* compound);
			
			void remove(Compound* compound);
			
			void remove(Frame* frame);
			
			void remove(Transform* transform);
			
			void setAcceleration(const ::rl::math::Vector& qdd);
			
			void setGammaPosition(const ::rl::math::Matrix& gammaPosition);
			
			void setGammaVelocity(const ::rl::math::Matrix& gammaVelocity);
			
			void setHomePosition(const ::rl::math::Vector& home);
			
			void setManufacturer(const ::std::string& manufacturer);
			
			void setName(const ::std::string& name);
			
			void setOperationalVelocity(const ::std::size_t& i, const ::rl::math::MotionVector& v) const;
			
			void setPosition(const ::rl::math::Vector& q);
			
			void setTorque(const ::rl::math::Vector& tau);
			
			void setVelocity(const ::rl::math::Vector& qd);
			
			::rl::math::Transform& tool(const ::std::size_t& i = 0);
			
			const ::rl::math::Transform& tool(const ::std::size_t& i = 0) const;
			
			virtual void update();
			
			::rl::math::Transform& world();
			
			const ::rl::math::Transform& world() const;
			
		protected:
			friend class Compound;
			
			typedef ::boost::adjacency_list<
				::boost::listS,
				::boost::listS,
				::boost::bidirectionalS,
				::boost::property<
					::boost::vertex_color_t, Compound*,
					::std::shared_ptr<Frame>
				>,
				::boost::property<
					::boost::edge_weight_t, Compound*,
					::std::shared_ptr<Transform>
				>,
				::boost::no_property,
				::boost::listS
			> Tree;
			
			typedef ::boost::graph_traits<Tree>::edge_descriptor Edge;
			
			typedef ::boost::graph_traits<Tree>::edge_iterator EdgeIterator;
			
			typedef ::std::pair<EdgeIterator, EdgeIterator> EdgeIteratorPair;
			
			typedef ::boost::graph_traits<Tree>::in_edge_iterator InEdgeIterator;
			
			typedef ::std::pair<InEdgeIterator, InEdgeIterator> InEdgeIteratorPair;
			
			typedef ::boost::graph_traits<Tree>::out_edge_iterator OutEdgeIterator;
			
			typedef ::std::pair<OutEdgeIterator, OutEdgeIterator> OutEdgeIteratorPair;
			
			typedef ::boost::graph_traits<Tree>::vertex_descriptor Vertex;
			
			typedef ::boost::graph_traits<Tree>::vertex_iterator VertexIterator;
			
			typedef ::std::pair<VertexIterator, VertexIterator> VertexIteratorPair;
			
			void update(const Vertex& u);
			
			::std::vector<Body*> bodies;
			
			::std::vector<Element*> elements;
			
			::std::vector<Frame*> frames;
			
			::rl::math::Matrix gammaPosition;
			
			::rl::math::Matrix gammaVelocity;
			
			::rl::math::Vector home;
			
			::rl::math::Matrix invGammaPosition;
			
			::rl::math::Matrix invGammaVelocity;
			
			::std::vector<Joint*> joints;
			
			::std::vector<Vertex> leaves;
			
			::std::string manufacturer;
			
			::std::string name;
			
			Vertex root;
			
			::std::vector<Edge> tools;
			
			::std::vector<Transform*> transforms;
			
			Tree tree;
			
		private:
			
		};
	}
}

#endif // RL_MDL_MODEL_H
