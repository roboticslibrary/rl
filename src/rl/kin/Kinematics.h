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

#ifndef RL_KIN_KINEMATICS_H
#define RL_KIN_KINEMATICS_H

#include <memory>
#include <string>
#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>

namespace rl
{
	/**
	 * Denavit-Hartenberg kinematics.
	 */
	namespace kin
	{
		class Element;
		class Frame;
		class Link;
		class Joint;
		class Transform;
		
		class Kinematics
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			
			Kinematics();
			
			virtual ~Kinematics();
			
			/**
			 * See if specified bodies should be tested for collisions with each other.
			 */
			bool areColliding(const ::std::size_t& i, const ::std::size_t& j) const;
			
			/**
			 * Clip specified configuration to be within joint limits.
			 * 
			 * @param[out] q \f$\vec{q}\f$
			 */
			virtual void clip(::rl::math::Vector& q) const;
			
			virtual Kinematics* clone() const;
			
			static Kinematics* create(const ::std::string& filename);
			
			/**
			 * Calculate distance measure between specified configuration.
			 * 
			 * @param[in] q1 \f$\vec{q}_{1}\f$
			 * @param[in] q2 \f$\vec{q}_{2}\f$
			 */
			virtual ::rl::math::Real distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;
			
			/**
			 * Get forward position kinematics.
			 * 
			 * @param[in] i End effector
			 * 
			 * @pre updateFrames()
			 */
			virtual const ::rl::math::Transform& forwardPosition(const ::std::size_t& i = 0) const;
			
			/**
			 * Calculate forward force kinematics.
			 * 
			 * \f[ \vec{F} = \matr{J}^{-\mathrm{T}} \vec{\tau} \f]
			 * 
			 * @param[in] tau \f$\vec{\tau}\f$
			 * @param[out] f \f$\vec{F}\f$
			 * 
			 * @pre updateJacobian()
			 * @pre updateJacobianInverse()
			 */
			virtual void forwardForce(const ::rl::math::Vector& tau, ::rl::math::Vector& f) const;
			
			/**
			 * Calculate forward velocity kinematics.
			 * 
			 * \f[ \dot{\vec{x}} = \matr{J} \dot{\vec{q}} \f]
			 * 
			 * @param[in] qdot \f$\dot{\vec{q}}\f$
			 * @param[out] xdot \f$\dot{\vec{x}}\f$
			 * 
			 * @pre updateJacobian()
			 */
			virtual void forwardVelocity(const ::rl::math::Vector& qdot, ::rl::math::Vector& xdot) const;
			
			::rl::math::Vector generatePositionGaussian(const ::rl::math::Vector& rand, const ::rl::math::Vector& mean, const ::rl::math::Vector& sigma) const;
			
			::rl::math::Vector generatePositionUniform(const ::rl::math::Vector& rand) const;
			
			/**
			 * Get number of links.
			 */
			::std::size_t getBodies() const;
			
			/**
			 * Get number of degrees of freedom (DOF).
			 */
			::std::size_t getDof() const;
			
			/**
			 * Get link frame.
			 * 
			 * @pre updateFrames()
			 */
			const ::rl::math::Transform& getFrame(const ::std::size_t& i) const;
			
			/**
			 * Get Jacobian.
			 * 
			 * @pre updateJacobian()
			 */
			const ::rl::math::Matrix& getJacobian() const;
			
			/**
			 * Get Jacobian-Inverse.
			 * 
			 * @pre updateJacobianInverse()
			 */
			const ::rl::math::Matrix& getJacobianInverse() const;
			
			Joint* getJoint(const ::std::size_t& i) const;
			
			/**
			 * Get manipulability measure.
			 * 
			 * @pre updateJacobian()
			 */
			::rl::math::Real getManipulabilityMeasure() const;
			
			::std::string getManufacturer() const;
			
			::rl::math::Real getMaximum(const ::std::size_t& i) const;
			
			void getMaximum(::rl::math::Vector& max) const;
			
			::rl::math::Real getMinimum(const ::std::size_t& i) const;
			
			void getMinimum(::rl::math::Vector& min) const;
			
			::std::string getName() const;
			
			/**
			 * Get number of end effectors.
			 */
			::std::size_t getOperationalDof() const;
			
			/**
			 * Get current joint position.
			 * 
			 * @param[out] q \f$\vec{q}\f$
			 */
			void getPosition(::rl::math::Vector& q) const;
			
			void getPositionUnits(::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>& units) const;
			
			void getSpeed(::rl::math::Vector& speed) const;
			
			void getSpeedUnits(::Eigen::Matrix< ::rl::math::Unit, ::Eigen::Dynamic, 1>& units) const;
			
			virtual void interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const;
			
			/**
			 * Calculate inverse force kinematics.
			 * 
			 * \f[ \vec{\tau} = \matr{J}^{\mathrm{T}} \vec{F} \f]
			 * 
			 * @param[in] f \f$\vec{F}\f$
			 * @param[out] tau \f$\vec{\tau}\f$
			 * 
			 * @pre updateJacobian()
			 */
			virtual void inverseForce(const ::rl::math::Vector& f, ::rl::math::Vector& tau) const;
			
			virtual ::rl::math::Real inverseOfTransformedDistance(const ::rl::math::Real& d) const;
			
			/**
			 * Calculate inverse position kinematics.
			 */
			virtual bool inversePosition(
				const ::rl::math::Transform& x,
				::rl::math::Vector& q,
				const ::std::size_t& leaf = 0,
				const ::rl::math::Real& delta = ::std::numeric_limits< ::rl::math::Real>::infinity(),
				const ::rl::math::Real& epsilon = 1.0e-3f,
				const ::std::size_t& iterations = 1000
			);
			
			/**
			 * Calculate inverse velocity kinematics.
			 * 
			 * \f[ \dot{\vec{q}} = \matr{J}^{\dag} \dot{\vec{x}} \f]
			 * 
			 * @param[in] xdot \f$\dot{\vec{x}}\f$
			 * @param[out] qdot \f$\dot{\vec{q}}\f$
			 * 
			 * @pre updateJacobianInverse()
			 */
			virtual void inverseVelocity(const ::rl::math::Vector& xdot, ::rl::math::Vector& qdot) const;
			
			/**
			 * See if specified body should be tested for collisions with the environment.
			 */
			bool isColliding(const ::std::size_t& i) const;
			
			/**
			 * Check if current configuration is singular.
			 */
			virtual bool isSingular() const;
			
			/**
			 * Check if specified configuration is within joint limits.
			 * 
			 * @param[in] q \f$\vec{q}\f$
			 */
			virtual bool isValid(const ::rl::math::Vector& q) const;
			
			virtual ::rl::math::Real maxDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const;
			
			virtual ::rl::math::Real minDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const;
			
			virtual ::rl::math::Real minDistanceToRectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cuttingDimension) const;
			
			virtual ::rl::math::Real newDistance(const ::rl::math::Real& dist, const ::rl::math::Real& oldOff, const ::rl::math::Real& newOff, const int& cuttingDimension) const;
			
			/**
			 * Set if specified body should be tested for collisions with the environment.
			 */
			void setColliding(const ::std::size_t& i, const bool& doesCollide);
			
			/**
			 * Set if specified bodies should be tested for collisions with each other.
			 */
			void setColliding(const ::std::size_t& i, const ::std::size_t& j, const bool& doCollide);
			
			/**
			 * Update current joint position.
			 * 
			 * @param[in] q \f$\vec{q}\f$
			 */
			void setPosition(const ::rl::math::Vector& q);
			
			virtual void step(const ::rl::math::Vector& q1, const ::rl::math::Vector& qdot, ::rl::math::Vector& q2) const;
			
			::rl::math::Transform& tool(const ::std::size_t& i = 0);
			
			const ::rl::math::Transform& tool(const ::std::size_t& i = 0) const;
			
			virtual ::rl::math::Real transformedDistance(const ::rl::math::Real& d) const;
			
			virtual ::rl::math::Real transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;
			
			virtual ::rl::math::Real transformedDistance(const ::rl::math::Real& q1, const ::rl::math::Real& q2, const ::std::size_t& i) const;
			
			/**
			 * Update frames.
			 * 
			 * \f[ {_{0}^{n}\matr{T}} = {_{0}^{1}\matr{T}} \, {_{0}^{1}\matr{T}} \, \ldots \, {_{n-1}^{n}\matr{T}} \f]
			 * 
			 * @pre setPosition()
			 */
			virtual void updateFrames();
			
			/**
			 * Update Jacobian.
			 * 
			 * \f[ {^{0}\matr{J}} = \begin{pmatrix} {^{0}\matr{J}_{1}} & {^{0}\matr{J}_{2}} & \cdots & {^{0}\matr{J}_{n}} \end{pmatrix} \f]
			 * 
			 * @pre updateFrames()
			 */
			virtual void updateJacobian();
			
			/**
			 * Update Jacobian-Inverse.
			 * 
			 * \f[ \matr{J}^{\dag}(\vec{q}) = \sum_{i = 1}^{r} \frac{ \sigma_{i} }{ \sigma_{i}^{2} + \lambda^{2} } \, \vec{v}_{i} \, \vec{u}_{i}^{\mathrm{T}} \f]
			 * \f[ \matr{J}^{\dag}(\vec{q}) = \matr{J}^{\mathrm{T}}(\vec{q}) \, \bigl( \matr{J}(\vec{q}) \, \matr{J}^{\mathrm{T}}(\vec{q}) + \lambda^{2} \, \matr{1} \bigr)^{-1} \f]
			 * 
			 * @param[in] lambda Damping factor \f$\lambda\f$
			 * @param[in] doSvd Use singular value decomposition or damped least squares
			 * 
			 * @pre updateJacobian()
			 */
			virtual void updateJacobianInverse(const ::rl::math::Real& lambda = 0.0f, const bool& doSvd = true);
			
			::rl::math::Transform& world();
			
			const ::rl::math::Transform& world() const;
			
		protected:
			typedef ::boost::adjacency_list<
				::boost::vecS,
				::boost::vecS,
				::boost::bidirectionalS,
				::std::shared_ptr<Frame>,
				::std::shared_ptr<Transform>,
				::boost::no_property,
				::boost::vecS
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
			
			void update();
			
			void update(Vertex& u);
			
			::std::vector<Element*> elements;
			
			::std::vector<Frame*> frames;
			
			::std::vector<Vertex> leaves;
			
			::std::vector<Link*> links;
			
			::rl::math::Matrix jacobian;
			
			::rl::math::Matrix jacobianInverse;
			
			::std::vector<Joint*> joints;
			
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

#endif // RL_KIN_KINEMATICS_H
