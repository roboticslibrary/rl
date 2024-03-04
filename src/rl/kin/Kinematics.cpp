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

#include <algorithm>
#include <rl/std/algorithm.h>

#include "Exception.h"
#include "Frame.h"
#include "Kinematics.h"
#include "Joint.h"
#include "Link.h"
#include "World.h"
#include "XmlFactory.h"

namespace rl
{
	namespace kin
	{
		Kinematics::Kinematics() :
			elements(),
			frames(),
			leaves(),
			links(),
			jacobian(),
			jacobianInverse(),
			joints(),
			manufacturer(),
			name(),
			root(),
			tools(),
			transforms(),
			tree(),
			randDistribution(0, 1),
			randEngine(::std::random_device()())
		{
		}
		
		Kinematics::~Kinematics()
		{
		}
		
		void
		Kinematics::add(const ::std::shared_ptr<Frame>& frame)
		{
			Vertex vertex = ::boost::add_vertex(this->tree);
			frame->setVertexDescriptor(vertex);
			this->tree[vertex] = frame;
			
			if (::std::dynamic_pointer_cast<World>(frame))
			{
				this->root = vertex;
			}
		}
		
		void
		Kinematics::add(const ::std::shared_ptr<Transform>& transform, const Frame* a, const Frame* b)
		{
			Edge edge = ::boost::add_edge(a->getVertexDescriptor(), b->getVertexDescriptor(), this->tree).first;
			transform->setEdgeDescriptor(edge);
			this->tree[edge] = transform;
		}
		
		bool
		Kinematics::areColliding(const ::std::size_t& i, const ::std::size_t& j) const
		{
			assert(i < this->links.size());
			assert(j < this->links.size());
			
			if (this->links[i]->selfcollision.count(this->links[j]) > 0 || this->links[j]->selfcollision.count(this->links[i]) > 0)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		
		void
		Kinematics::clamp(::rl::math::Vector& q) const
		{
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				if (this->joints[i]->wraparound)
				{
					::rl::math::Real range = ::std::abs(this->joints[i]->max - this->joints[i]->min);
					
					while (q(i) > this->joints[i]->max)
					{
						q(i) -= range;
					}
					
					while (q(i) < this->joints[i]->min)
					{
						q(i) += range;
					}
				}
				else
				{
					q(i) = ::rl::std17::clamp(q(i), this->joints[i]->min, this->joints[i]->max);
				}
			}
		}
		
		::std::shared_ptr<Kinematics>
		Kinematics::create(const ::std::string& filename)
		{
			XmlFactory factory;
			return factory.create(filename);
		}
		
		::rl::math::Real
		Kinematics::distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			return this->inverseOfTransformedDistance(this->transformedDistance(q1, q2));
		}
		
		const ::rl::math::Transform&
		Kinematics::forwardPosition(const ::std::size_t& i) const
		{
			return this->tree[this->leaves[i]]->frame;
		}
		
		void
		Kinematics::forwardForce(const ::rl::math::Vector& tau, ::rl::math::Vector& f) const
		{
			assert(tau.size() <= this->getDof());
			assert(f.size() <= this->getOperationalDof() * 6);
			
			f = this->jacobianInverse.transpose() * tau;
		}
		
		void
		Kinematics::forwardVelocity(const ::rl::math::Vector& qdot, ::rl::math::Vector& xdot) const
		{
			assert(qdot.size() <= this->getDof());
			assert(xdot.size() <= this->getOperationalDof() * 6);
			
			xdot = this->jacobian * qdot;
		}
		
		::rl::math::Vector
		Kinematics::generatePositionGaussian(const ::rl::math::Vector& mean, const ::rl::math::Vector& sigma)
		{
			::rl::math::Vector rand(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				rand(i) = this->rand();
			}
			
			return this->generatePositionGaussian(rand, mean, sigma);
		}
		
		::rl::math::Vector
		Kinematics::generatePositionGaussian(const ::rl::math::Vector& rand, const ::rl::math::Vector& mean, const ::rl::math::Vector& sigma) const
		{
			::rl::math::Vector q(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				q(i) = mean(i) + rand(i) * sigma(i);
			}
			
			this->clamp(q);
			
			return q;
		}
		
		::rl::math::Vector
		Kinematics::generatePositionUniform()
		{
			::rl::math::Vector rand(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				rand(i) = this->rand();
			}
			
			return this->generatePositionUniform(rand);
		}
		
		::rl::math::Vector
		Kinematics::generatePositionUniform(const ::rl::math::Vector& rand) const
		{
			::rl::math::Vector q(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				q(i) = this->getMinimum(i) + rand(i) * (this->getMaximum(i) - this->getMinimum(i));
			}
			
			return q;
		}
		
		::rl::math::Vector
		Kinematics::generatePositionUniform(const ::rl::math::Vector& min, const ::rl::math::Vector& max)
		{
			::rl::math::Vector rand(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				rand(i) = this->rand();
			}
			
			return this->generatePositionUniform(rand, min, max);
		}
		
		::rl::math::Vector
		Kinematics::generatePositionUniform(const ::rl::math::Vector& rand, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const
		{
			::rl::math::Vector q(this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				q(i) = min(i) + rand(i) * (max(i) - min(i));
			}
			
			return q;
		}
		
		::std::size_t
		Kinematics::getBodies() const
		{
			return this->links.size();
		}
		
		::std::size_t
		Kinematics::getDof() const
		{
			return this->joints.size();
		}
		
		const ::rl::math::Transform&
		Kinematics::getFrame(const ::std::size_t& i) const
		{
			assert(i < this->links.size());
			
			return this->links[i]->frame;
		}
		
		const ::rl::math::Matrix&
		Kinematics::getJacobian() const
		{
			return this->jacobian;
		}
		
		const ::rl::math::Matrix&
		Kinematics::getJacobianInverse() const
		{
			return this->jacobianInverse;
		}
		
		Joint*
		Kinematics::getJoint(const ::std::size_t& i) const
		{
			assert(i < this->joints.size());
			
			return this->joints[i];
		}
		
		::rl::math::Real
		Kinematics::getManipulabilityMeasure() const
		{
			return ::std::sqrt((this->jacobian * this->jacobian.transpose()).determinant());
		}
		
		::std::string
		Kinematics::getManufacturer() const
		{
			return this->manufacturer;
		}
		
		::rl::math::Real
		Kinematics::getMaximum(const ::std::size_t& i) const
		{
			assert(i < this->joints.size());
			
			return this->joints[i]->max;
		}
		
		void
		Kinematics::getMaximum(::rl::math::Vector& max) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				max(i) = this->joints[i]->max;
			}
		}
		
		::rl::math::Real
		Kinematics::getMinimum(const ::std::size_t& i) const
		{
			assert(i < this->joints.size());
			
			return this->joints[i]->min;
		}
		
		void
		Kinematics::getMinimum(::rl::math::Vector& min) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				min(i) = this->joints[i]->min;
			}
		}
		
		::std::string
		Kinematics::getName() const
		{
			return this->name;
		}
		
		::std::size_t
		Kinematics::getOperationalDof() const
		{
			return this->leaves.size();
		}
		
		void
		Kinematics::getPosition(::rl::math::Vector& q) const
		{
			assert(q.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				q(i) = this->joints[i]->getPosition();
			}
		}
		
		void
		Kinematics::getPositionUnits(::Eigen::Matrix<::rl::math::Units, ::Eigen::Dynamic, 1>& units) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				units(i) = this->joints[i]->getPositionUnit();
			}
		}
		
		void
		Kinematics::getSpeed(::rl::math::Vector& speed) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				speed(i) = this->joints[i]->speed;
			}
		}
		
		void
		Kinematics::getSpeedUnits(::Eigen::Matrix<::rl::math::Units, ::Eigen::Dynamic, 1>& units) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				units(i) = this->joints[i]->getSpeedUnit();
			}
		}
		
		void
		Kinematics::getWraparounds(::Eigen::Matrix<bool, ::Eigen::Dynamic, 1>& wraparounds) const
		{
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				wraparounds(i) = this->joints[i]->wraparound;
			}
		}
		
		void
		Kinematics::interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const
		{
			assert(q1.size() == this->getDof());
			assert(q2.size() == this->getDof());
			assert(alpha >= 0);
			assert(alpha <= 1);
			assert(q.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				if (this->joints[i]->wraparound)
				{
					::rl::math::Real diff = ::std::abs(q2(i) - q1(i));
					::rl::math::Real range = ::std::abs(this->joints[i]->max - this->joints[i]->min);
					
					if (::std::abs(range - diff) < diff)
					{
						if (q1(i) > q2(i))
						{
							q(i) = (1 - alpha) * q1(i) + alpha * (q2(i) + range);
						}
						else
						{
							q(i) = (1 - alpha) * (q1(i) + range) + alpha * q2(i);
						}
						
						while (q(i) > this->joints[i]->max)
						{
							q(i) -= range;
						}
						
						while (q(i) < this->joints[i]->min)
						{
							q(i) += range;
						}
					}
					else
					{
						q(i) = (1 - alpha) * q1(i) + alpha * q2(i);
					}
				}
				else
				{
					q(i) = (1 - alpha) * q1(i) + alpha * q2(i);
				}
			}
		}
		
		void
		Kinematics::inverseForce(const ::rl::math::Vector& f, ::rl::math::Vector& tau) const
		{
			assert(f.size() <= this->getOperationalDof() * 6);
			assert(tau.size() <= this->getDof());
			
			tau = this->jacobian.transpose() * f;
		}
		
		::rl::math::Real
		Kinematics::inverseOfTransformedDistance(const ::rl::math::Real& d) const
		{
			return ::std::sqrt(d);
		}
		
		bool
		Kinematics::inversePosition(const ::rl::math::Transform& x, ::rl::math::Vector& q, const ::std::size_t& leaf, const ::rl::math::Real& delta, const ::rl::math::Real& epsilon, const ::std::size_t& iterations, const ::std::chrono::nanoseconds& duration, const ::std::size_t& steps, const ::std::size_t& restarts)
		{
			assert(q.size() == this->getDof());
			
			::std::chrono::steady_clock::time_point start = ::std::chrono::steady_clock::now();
			double remaining = ::std::chrono::duration<double>(duration).count();
			::std::size_t attempt = 0;
			::std::size_t iteration = 0;
			
			this->getPosition(q);
			::rl::math::Vector q2(this->getDof());
			::rl::math::Vector dq(this->getDof());
			::rl::math::Vector dx(6 * this->getOperationalDof());
			
			do
			{
				if (attempt > 0)
				{
					q = this->generatePositionUniform();
					this->setPosition(q);
				}
				
				for (::std::size_t i = 0; i < steps && remaining > 0 && iteration < iterations; ++i, ++iteration)
				{
					this->updateFrames();
					dx.setZero();
					
					::rl::math::VectorBlock dxi = dx.segment(6 * leaf, 6);
					dxi = this->forwardPosition(leaf).toDelta(x);
					
					if (dx.squaredNorm() < ::std::pow(epsilon, 2))
					{
						this->normalize(q);
						this->setPosition(q);
						
						if (this->isValid(q))
						{
							return true;
						}
					}
					
					this->updateJacobian();
					this->updateJacobianInverse();
					this->inverseVelocity(dx, dq);
					
					this->step(q, dq, q2);
					
					if (this->transformedDistance(q, q2) > ::std::pow(delta, 2))
					{
						this->interpolate(q, q2, delta, q2);
					}
					
					q = q2;
					this->setPosition(q);
					
					remaining = ::std::chrono::duration<double>(duration - (::std::chrono::steady_clock::now() - start)).count();
				}
				
				remaining = ::std::chrono::duration<double>(duration - (::std::chrono::steady_clock::now() - start)).count();
			}
			while (attempt++ < restarts && remaining > 0 && iteration < iterations);
			
			return false;
		}
		
		void
		Kinematics::inverseVelocity(const ::rl::math::Vector& xdot, ::rl::math::Vector& qdot) const
		{
			assert(xdot.size() <= this->getOperationalDof() * 6);
			assert(qdot.size() <= this->getDof());
			
			qdot = this->jacobianInverse * xdot;
		}
		
		bool
		Kinematics::isColliding(const ::std::size_t& i) const
		{
			assert(i < this->links.size());
			
			return this->links[i]->collision;
		}
		
		bool
		Kinematics::isSingular() const
		{
#if 0
			return !(this->getManipulabilityMeasure() > 0);
#else
			::Eigen::JacobiSVD<::rl::math::Matrix> svd(this->jacobian);
			return (::std::abs(svd.singularValues()(svd.singularValues().size() - 1)) > ::std::numeric_limits<::rl::math::Real>::epsilon()) ? false : true;
#endif
		}
		
		bool
		Kinematics::isValid(const ::rl::math::Vector& q) const
		{
			assert(q.size() == this->getDof());
			
			for (::std::ptrdiff_t i = 0; i < q.size(); ++i)
			{
				if (q(i) < this->joints[i]->min || q(i) > this->joints[i]->max)
				{
					return false;
				}
			}
			
			return true;
		}
		
		void
		Kinematics::normalize(::rl::math::Vector& q) const
		{
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->joints[i]->normalize(q(i));
			}
		}
		
		::std::uniform_real_distribution<::rl::math::Real>::result_type
		Kinematics::rand()
		{
			return this->randDistribution(this->randEngine);
		}
		
		void
		Kinematics::remove(Frame* frame)
		{
			::boost::clear_vertex(frame->getVertexDescriptor(), this->tree);
			
			if (dynamic_cast<World*>(frame))
			{
				this->root = 0;
			}
			
			::boost::remove_vertex(frame->getVertexDescriptor(), this->tree);
		}
		
		void
		Kinematics::remove(Transform* transform)
		{
			::boost::remove_edge(transform->getEdgeDescriptor(), this->tree);
		}
		
		void
		Kinematics::seed(const ::std::mt19937::result_type& value)
		{
			this->randEngine.seed(value);
		}
		
		void
		Kinematics::setColliding(const ::std::size_t& i, const bool& doesCollide)
		{
			assert(i < this->getBodies());
			
			this->links[i]->collision = doesCollide;
		}
		
		void
		Kinematics::setColliding(const ::std::size_t& i, const ::std::size_t& j, const bool& doCollide)
		{
			assert(i < this->getBodies());
			assert(j < this->getBodies());
			
			if (doCollide)
			{
				this->links[i]->selfcollision.erase(this->links[j]);
				this->links[j]->selfcollision.erase(this->links[i]);
			}
			else
			{
				this->links[i]->selfcollision.insert(this->links[j]);
				this->links[j]->selfcollision.insert(this->links[i]);
			}
		}
		
		void
		Kinematics::setManufacturer(const ::std::string& manufacturer)
		{
			this->manufacturer = manufacturer;
		}
		
		void
		Kinematics::setMaximum(const ::std::size_t& i, const ::rl::math::Real& max)
		{
			assert(i < this->joints.size());
			
			this->joints[i]->max = max;
		}
		
		void
		Kinematics::setMaximum(const ::rl::math::Vector& max)
		{
			assert(max.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				this->joints[i]->max = max(i);
			}
		}
		
		void
		Kinematics::setMinimum(const ::std::size_t& i, const ::rl::math::Real& min)
		{
			assert(i < this->joints.size());
			
			this->joints[i]->min = min;
		}
		
		void
		Kinematics::setMinimum(const ::rl::math::Vector& min)
		{
			assert(min.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				this->joints[i]->min = min(i);
			}
		}
		
		void
		Kinematics::setName(const ::std::string& name)
		{
			this->name = name;
		}
		
		void
		Kinematics::setPosition(const ::rl::math::Vector& q)
		{
			assert(q.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->joints.size(); ++i)
			{
				this->joints[i]->setPosition(q(i));
			}
		}
		
		void
		Kinematics::step(const ::rl::math::Vector& q1, const ::rl::math::Vector& dq, ::rl::math::Vector& q2) const
		{
			assert(q1.size() == this->getDof());
			assert(dq.size() == this->getDof());
			assert(q2.size() == this->getDof());
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				q2(i) = q1(i) + dq(i);
			}
			
			this->clamp(q2);
		}
		
		::rl::math::Transform&
		Kinematics::tool(const ::std::size_t& i)
		{
			assert(i < this->tools.size());
			
			return this->tree[this->tools[i]]->transform;
		}
		
		const ::rl::math::Transform&
		Kinematics::tool(const ::std::size_t& i) const
		{
			assert(i < this->tools.size());
			
			return this->tree[this->tools[i]]->transform;
		}
		
		::rl::math::Real
		Kinematics::transformedDistance(const ::rl::math::Real& d) const
		{
			return ::std::pow(d, 2);
		}
		
		::rl::math::Real
		Kinematics::transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			assert(q1.size() == this->getDof());
			assert(q2.size() == this->getDof());
			
			::rl::math::Real d = 0;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				::rl::math::Real delta = ::std::abs(q2(i) - q1(i));
				
				if (this->joints[i]->wraparound)
				{
					::rl::math::Real range = ::std::abs(this->joints[i]->max - this->joints[i]->min);
					d += this->transformedDistance(::std::min(delta, ::std::abs(range - delta)));
				}
				else
				{
					d += this->transformedDistance(delta);
				}
			}
			
			return d;
		}
		
		::rl::math::Real
		Kinematics::transformedDistance(const ::rl::math::Real& q1, const ::rl::math::Real& q2, const ::std::size_t& i) const
		{
			::rl::math::Real delta = ::std::abs(q1 - q2);
			
			if (this->joints[i]->wraparound)
			{
				::rl::math::Real range = ::std::abs(this->joints[i]->max - this->joints[i]->min);
				return this->transformedDistance(::std::max(delta, ::std::abs(range - delta)));
			}
			else
			{
				return this->transformedDistance(delta);
			}
		}
		
		void
		Kinematics::update()
		{
			this->elements.clear();
			this->joints.clear();
			this->leaves.clear();
			this->links.clear();
			this->tools.clear();
			this->transforms.clear();
			
			this->update(this->root);
			
			for (::std::vector<Vertex>::iterator i = this->leaves.begin(); i != this->leaves.end(); ++i)
			{
				Vertex v = *i;
				
				while (v != this->root)
				{
					Edge e = *::boost::in_edges(v, this->tree).first;
					Transform* transform = this->tree[e].get();
					
					if (Joint* joint = dynamic_cast<Joint*>(transform))
					{
						joint->leaves.insert(*i);
					}
					
					v = ::boost::source(e, this->tree);
				}
			}
			
			this->jacobian = ::rl::math::Matrix::Identity(this->leaves.size() * 6, this->joints.size());
			this->jacobianInverse = ::rl::math::Matrix::Identity(this->joints.size(), this->leaves.size() * 6);
		}
		
		void
		Kinematics::update(Vertex& u)
		{
			Frame* frame = this->tree[u].get();
			this->elements.push_back(frame);
			this->frames.push_back(frame);
			
			if (Link* link = dynamic_cast<Link*>(frame))
			{
				this->links.push_back(link);
			}
			
			if (::boost::out_degree(u, this->tree) > 0)
			{
				for (OutEdgeIteratorPair i = ::boost::out_edges(u, this->tree); i.first != i.second; ++i.first)
				{
					Edge e = *i.first;
					Vertex v = ::boost::target(e, this->tree);
					
					Transform* transform = this->tree[e].get();
					this->elements.push_back(transform);
					this->transforms.push_back(transform);
					transform->in = this->tree[u].get();
					transform->out = this->tree[v].get();
					
					if (Joint* joint = dynamic_cast<Joint*>(transform))
					{
						joint->leaves.clear();
						this->joints.push_back(joint);
					}
					
					this->update(v);
				}
			}
			else
			{
				this->leaves.push_back(u);
				
				for (InEdgeIteratorPair i = ::boost::in_edges(u, this->tree); i.first != i.second; ++i.first)
				{
					this->tools.push_back(*i.first);
				}
			}
		}
		
		void
		Kinematics::updateFrames()
		{
			for (::std::vector<Transform*>::iterator i = this->transforms.begin(); i != this->transforms.end(); ++i)
			{
				(*i)->updateFrames();
			}
		}
		
		void
		Kinematics::updateJacobian()
		{
			this->jacobian.setZero();
			
			for (::std::size_t i = 0; i < this->leaves.size(); ++i)
			{
				for (::std::size_t j = 0; j < this->joints.size(); ++j)
				{
					if (this->joints[j]->leaves.count(this->leaves[i]) > 0)
					{
						::rl::math::MatrixBlock jacobian = this->jacobian.block(6 * i, j, 6, 1);
						this->joints[j]->jacobian(this->tree[this->leaves[i]]->frame, jacobian);
					}
				}
			}
		}
		
		void
		Kinematics::updateJacobianInverse(const ::rl::math::Real& lambda, const bool& doSvd)
		{
			if (doSvd)
			{
				this->jacobianInverse.setZero();
				
				::Eigen::JacobiSVD<::rl::math::Matrix> svd(this->jacobian, ::Eigen::ComputeFullU | ::Eigen::ComputeFullV);
				
				::rl::math::Real wMin = svd.singularValues().minCoeff();
				::rl::math::Real lambdaSqr = wMin < static_cast<::rl::math::Real>(1.0e-9) ? (1 - ::std::pow((wMin / static_cast<::rl::math::Real>(1.0e-9)), 2)) * ::std::pow(lambda, 2) : 0;
				
				for (::std::ptrdiff_t i = 0; i < svd.nonzeroSingularValues(); ++i)
				{
					this->jacobianInverse.noalias() += (
						svd.singularValues()(i) / (::std::pow(svd.singularValues()(i), 2) + lambdaSqr) *
						svd.matrixV().col(i) * svd.matrixU().col(i).transpose()
					);
				}
			}
			else
			{
				this->jacobianInverse = this->jacobian.transpose() * (
					this->jacobian * this->jacobian.transpose() + ::std::pow(lambda, 2) *
					::rl::math::Matrix::Identity(this->getOperationalDof() * 6, this->getOperationalDof() * 6)
				).inverse();
			}
		}
		
		::rl::math::Transform&
		Kinematics::world()
		{
			return this->tree[this->root]->frame;
		}
		
		const ::rl::math::Transform&
		Kinematics::world() const
		{
			return this->tree[this->root]->frame;
		}
	}
}
