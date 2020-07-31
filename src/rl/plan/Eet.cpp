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

#include <chrono>
#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>

#include "DistanceModel.h"
#include "Eet.h"
#include "Exception.h"
#include "Sampler.h"
#include "SimpleModel.h"
#include "Viewer.h"
#include "WorkspaceSphereExplorer.h"
#include "WorkspaceSphereVector.h"

struct Compare
{
	bool operator()(const ::std::pair<void*, ::std::size_t>& lhs, const ::std::pair<void*, ::std::size_t>& rhs) const
	{
		return lhs.second < rhs.second;
	}
};

namespace rl
{
	namespace plan
	{
		Eet::Eet() :
			RrtCon(),
			alpha(static_cast<::rl::math::Real>(0.01)),
			alternativeDistanceComputation(false),
			beta(0),
			distanceWeight(static_cast<::rl::math::Real>(0.1)),
			explorers(),
			explorersSetup(),
			gamma(static_cast<::rl::math::Real>(1) / static_cast<::rl::math::Real>(3)),
			goalEpsilon(static_cast<::rl::math::Real>(0.1)),
			goalEpsilonUseOrientation(false),
			max(::rl::math::Vector3::Zero()),
			min(::rl::math::Vector3::Zero()),
			gaussDistribution(0, 1),
			gaussEngine(::std::random_device()()),
			randDistribution(0, 1),
			randEngine(::std::random_device()()),
			explorationTimeStart(),
			explorationTimeStop(),
			nn(WorkspaceMetric(&this->distanceWeight, &this->alternativeDistanceComputation))
		{
		}
		
		Eet::~Eet()
		{
		}
		
		Rrt::Edge
		Eet::addEdge(const Vertex& u, const Vertex& v, Tree& tree)
		{
			Edge e = ::boost::add_edge(u, v, tree).first;
			
			if (nullptr != this->getViewer())
			{
				this->getViewer()->drawConfigurationEdge(*get(tree, u)->q, *get(tree, v)->q);
			}
			
			return e;
		}
		
		void
		Eet::addExplorer(WorkspaceSphereExplorer* explorer)
		{
			this->explorers.push_back(explorer);
		}
		
		void
		Eet::addExplorerSetup(const ExplorerSetup& explorerSetup)
		{
			this->explorersSetup.push_back(explorerSetup);
		}
		
		Eet::Vertex
		Eet::addVertex(Tree& tree, const VectorPtr& q)
		{
			::std::shared_ptr<VertexBundle> bundle = ::std::make_shared<VertexBundle>();
			bundle->index = ::boost::num_vertices(tree) - 1;
			bundle->q = q;
			
			Vertex v = ::boost::add_vertex(tree);
			tree[v] = bundle;
			
			if (nullptr != this->getViewer())
			{
				this->getViewer()->drawConfigurationVertex(*get(tree, v)->q);
			}
			
			return v;
		}
		
		void
		Eet::clearExplorers()
		{
			this->explorers.clear();
		}
		
		void
		Eet::clearExplorersSetup()
		{
			this->explorersSetup.clear();
		}
		
		Rrt::Vertex
		Eet::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Transform& chosen)
		{
			::rl::math::Real distance = this->distance(*get(tree, nearest.second)->t, chosen);
			
			Vertex connected = nullptr;
			Vertex n = nearest.second;
			int state = 1;
			
			do
			{
				VertexBundle best;
				best.q = ::std::make_shared<::rl::math::Vector>(this->getModel()->getDofPosition());
				best.t = ::std::make_shared<::rl::math::Transform>();
				
				state = this->expand(get(tree, n), *get(tree, nearest.second)->t, chosen, distance, best); // TODO
				
				if (state >= 0)
				{
					connected = this->addVertex(tree, best.q);
					get(tree, connected)->t = best.t;
					this->addEdge(n, connected, tree);
					n = connected;
					
					::rl::math::Real distance2 = this->distance(*get(tree, n)->t, chosen);
					
					if (distance2 > distance)
					{
						break;
					}
					else if (::std::abs(distance - distance2) < 1e-6)
					{
						break;
					}
					
					distance = distance2;
				}
			}
			while (state > 0);
			
			if (nullptr != connected)
			{
				this->nn.push(WorkspaceMetric::Value(get(tree, connected)->t.get(), connected));
			}
			
			return connected;
		}
		
		::rl::math::Real
		Eet::distance(const ::rl::math::Transform& t1, const ::rl::math::Transform& t2) const
		{
			if (this->alternativeDistanceComputation)
			{
				::rl::math::Vector6 delta = t1.toDelta(t2, true);
				return delta.norm();
			}
			else
			{
				return t1.distance(t2, this->distanceWeight);
			}
		}
		
		int
		Eet::expand(const VertexBundle* nearest, const ::rl::math::Transform& nearest2, const ::rl::math::Transform& chosen, const ::rl::math::Real& distance, VertexBundle& expanded)
		{
			int state = 1;
			
			::rl::math::Vector6 tdot = nearest2.toDelta(chosen, true);
			
			this->getModel()->setPosition(*nearest->q);
			this->getModel()->updateFrames();
			this->getModel()->updateJacobian();
			this->getModel()->updateJacobianInverse();
			
			::rl::math::Vector qdot(this->getModel()->getDof());
			qdot.setZero();
			
			::rl::math::Vector qdot2(this->getModel()->getDof());
			this->getModel()->inverseVelocity(tdot, qdot2);
			qdot += qdot2;
			
			if (qdot.norm() <= this->getDelta())
			{
				state = 0;
			}
			else
			{
				qdot.normalize();
				qdot *= this->getDelta();
			}
			
			this->getModel()->step(*nearest->q, qdot, *expanded.q);
			
			if (this->getModel()->getManipulabilityMeasure() < static_cast<::rl::math::Real>(1.0e-3)) // within singularity
			{
				*expanded.q = this->getSampler()->generate(); // uniform sampling for singularities
				::rl::math::Real tmp = this->getModel()->distance(*nearest->q, *expanded.q);
				this->getModel()->interpolate(*nearest->q, *expanded.q, this->getDelta() / tmp, *expanded.q);
			}
			
			if (!this->getModel()->isValid(*expanded.q))
			{
				return -1;
			}
			
			if (nullptr != this->getViewer())
			{
				this->getViewer()->drawConfiguration(*expanded.q);
			}
			
			if (this->getModel()->isColliding(*expanded.q))
			{
				return -1;
			}
			
			*expanded.t = this->getModel()->forwardPosition();
			
			return state;
		}
		
		Rrt::Vertex
		Eet::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Transform& chosen)
		{
			::rl::math::Real distance = this->distance(*get(tree, nearest.second)->t, chosen);
			
			Vertex extended = nullptr;
			
			VertexBundle best;
			best.q = ::std::make_shared<::rl::math::Vector>(this->getModel()->getDofPosition());
			best.t = ::std::make_shared<::rl::math::Transform>();
			
			if (this->expand(get(tree, nearest.second), *get(tree, nearest.second)->t, chosen, distance, best) >= 0)
			{
				extended = this->addVertex(tree, best.q);
				get(tree, extended)->t = best.t;
				this->addEdge(nearest.second, extended, tree);
			}
			
			return extended;
		}
		
		::std::normal_distribution<::rl::math::Real>::result_type
		Eet::gauss()
		{
			return this->gaussDistribution(this->gaussEngine);
		}
		
		Eet::VertexBundle*
		Eet::get(const Tree& tree, const Vertex& v)
		{
			return static_cast<VertexBundle*>(tree[v].get());
		}
		
		::rl::math::Real
		Eet::getAlpha() const
		{
			return this->alpha;
		}
		
		bool
		Eet::getAlternativeDistanceComputation() const
		{
			return this->alternativeDistanceComputation;
		}
		
		::rl::math::Real
		Eet::getBeta() const
		{
			return this->beta;
		}
		
		::rl::math::Real
		Eet::getDistanceWeight() const
		{
			return this->distanceWeight;
		}
		
		::std::chrono::steady_clock::duration
		Eet::getExplorationDuration() const
		{
			return this->explorationTimeStop - this->explorationTimeStart;
		}
		
		const ::std::vector<WorkspaceSphereExplorer*>&
		Eet::getExplorers() const
		{
			return this->explorers;
		}
		
		const ::std::vector<Eet::ExplorerSetup>&
		Eet::getExplorersSetup() const
		{
			return this->explorersSetup;
		}
		
		::rl::math::Real
		Eet::getGamma() const
		{
			return this->gamma;
		}
		
		::rl::math::Real
		Eet::getGoalEpsilon() const
		{
			return this->goalEpsilon;
		}
		
		bool
		Eet::getGoalEpsilonUseOrientation() const
		{
			return this->goalEpsilonUseOrientation;
		}
		
		const ::rl::math::Vector3&
		Eet::getMax() const
		{
			return this->max;
		}
		
		const ::rl::math::Vector3&
		Eet::getMin() const
		{
			return this->min;
		}
		
		::std::string
		Eet::getName() const
		{
			return "EET";
		}
		
		::std::size_t
		Eet::getNumEdges() const
		{
			return this->nn.size() - 1;
		}
		
		::std::size_t
		Eet::getNumVertices() const
		{
			return this->nn.size();
		}
		
		VectorList
		Eet::getPath()
		{
			return Rrt::getPath();
		}
		
		Rrt::Neighbor
		Eet::nearest(const Tree& tree, const ::rl::math::Transform& chosen)
		{
			::std::vector<::rl::math::GnatNearestNeighbors<WorkspaceMetric>::Neighbor> neighbors = this->nn.nearest(WorkspaceMetric::Value(&chosen, Vertex()), 1);
			return Neighbor(neighbors.front().first, neighbors.front().second.second);
		}
		
		::std::uniform_real_distribution<::rl::math::Real>::result_type
		Eet::rand()
		{
			return this->randDistribution(this->randEngine);
		}
		
		void
		Eet::reset()
		{
			RrtCon::reset();
			
			for (::std::vector<WorkspaceSphereExplorer*>::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
			{
				(*i)->reset();
			}
			
			this->nn.clear();
		}
		
		void
		Eet::seed(const ::std::mt19937::result_type& value)
		{
			this->gaussEngine.seed(value);
			this->nn.seed(value);
			this->randEngine.seed(value);
		}
		
		void
		Eet::setAlpha(const ::rl::math::Real& alpha)
		{
			this->alpha = alpha;
		}
		
		void
		Eet::setAlternativeDistanceComputation(const bool& alternativeDistanceComputation)
		{
			this->alternativeDistanceComputation = alternativeDistanceComputation;
		}
		
		void
		Eet::setBeta(const ::rl::math::Real& beta)
		{
			this->beta = beta;
		}
		
		void
		Eet::setDistanceWeight(const ::rl::math::Real& distanceWeight)
		{
			this->distanceWeight = distanceWeight;
		}
		
		void
		Eet::setExplorers(const ::std::vector<WorkspaceSphereExplorer*>& explorers)
		{
			this->explorers = explorers;
		}
		
		void
		Eet::setExplorersSetup(const ::std::vector<ExplorerSetup>& explorersSetup)
		{
			this->explorersSetup = explorersSetup;
		}
		
		void
		Eet::setGamma(const ::rl::math::Real& gamma)
		{
			this->gamma = gamma;
		}
		
		void
		Eet::setGoalEpsilon(const ::rl::math::Real& goalEpsilon)
		{
			this->goalEpsilon = goalEpsilon;
		}
		
		void
		Eet::setGoalEpsilonUseOrientation(const bool& goalEpsilonUseOrientation)
		{
			this->goalEpsilonUseOrientation = goalEpsilonUseOrientation;
		}
		
		void
		Eet::setMax(const ::rl::math::Vector3& max)
		{
			this->max = max;
		}
		
		void
		Eet::setMin(const ::rl::math::Vector3& min)
		{
			this->min = min;
		}
		
		bool
		Eet::solve()
		{
			if (this->getModel()->getOperationalDof() > 1)
			{
				throw("::rl::plan::Eet::solve() - branched kinematics not supported");
			}
			
			this->time = ::std::chrono::steady_clock::now();
			
			this->explorationTimeStart = ::std::chrono::steady_clock::now();
			
			// initialize workspace explorers
			
			for (::std::size_t i = 0; i < this->explorersSetup.size(); ++i)
			{
				if (nullptr != this->explorersSetup[i].startConfiguration)
				{
					this->getModel()->setPosition(*this->explorersSetup[i].startConfiguration);
					this->getModel()->updateFrames(false);
					
					if (-1 == this->explorersSetup[i].startFrame)
					{
						*this->explorers[i]->getStart() = this->getModel()->forwardPosition().translation();
					}
					else
					{
						*this->explorers[i]->getStart() = this->getModel()->getFrame(this->explorersSetup[i].startFrame).translation();
					}
				}
				
				if (nullptr != this->explorersSetup[i].goalConfiguration)
				{
					this->getModel()->setPosition(*this->explorersSetup[i].goalConfiguration);
					this->getModel()->updateFrames(false);
					
					if (-1 == this->explorersSetup[i].goalFrame)
					{
						*this->explorers[i]->getGoal() = this->getModel()->forwardPosition().translation();
					}
					else
					{
						*this->explorers[i]->getGoal() = this->getModel()->getFrame(this->explorersSetup[i].goalFrame).translation();
					}
				}
			}
			
			// compute sphere tree
			
			WorkspaceSphereVector path;
			
			for (::std::vector<WorkspaceSphereExplorer*>::iterator j = this->explorers.begin(); j != this->explorers.end(); ++j)
			{
				if (!(*j)->explore())
				{
					return false;
				}
				
				WorkspaceSphereList path2 = (*j)->getPath();
				path.insert(path.end(), path2.begin(), path2.end());
			}
			
			this->explorationTimeStop = ::std::chrono::steady_clock::now();
			
			// tree initialization with start configuration
			
			this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared<::rl::math::Vector>(*this->getStart()));
			this->getModel()->setPosition(*this->getStart());
			this->getModel()->updateFrames();
			get(this->tree[0], this->begin[0])->t = ::std::make_shared<::rl::math::Transform>(this->getModel()->forwardPosition());
			this->nn.push(WorkspaceMetric::Value(get(this->tree[0], this->begin[0])->t.get(), begin[0]));
			
			::rl::math::Transform chosen;
			chosen.setIdentity();
			
			this->getModel()->setPosition(*this->getGoal());
			this->getModel()->updateFrames();
			::rl::math::Transform goal = this->getModel()->forwardPosition();
			
			WorkspaceSphereVector::iterator i = ++path.begin();
			::rl::math::Real sigma = gamma; // initialize exploration/exploitation balance
			
			while ((::std::chrono::steady_clock::now() - this->time) < this->getDuration()) // search until goal reached
			{
				if (sigma < 1) // sample is within current sphere
				{
					Neighbor nearest;
					
					if (path.end() == i + 1 && this->rand() < static_cast<::rl::math::Real>(0.5)) // within last sphere
					{
						chosen = goal; // select goal position and orientation
						nearest = this->nearest(this->tree[0], chosen); // nearest vertex in tree
					}
					else
					{
						// sample within sphere
						
						chosen.translation().x() = this->gauss() * sigma * i->radius + (*i->center).x();
						chosen.translation().y() = this->gauss() * sigma * i->radius + (*i->center).y();
						chosen.translation().z() = this->gauss() * sigma * i->radius + (*i->center).z();
						
						for (::std::size_t k = 0; k < 3; ++k)
						{
							if (chosen(k, 3) < this->min(k)) chosen(k, 3) = this->min(k);
							if (chosen(k, 3) > this->max(k)) chosen(k, 3) = this->max(k);
						}
						
						// uniform sampling for orientation
						
						chosen.linear() = ::rl::math::Quaternion::Random(::rl::math::Vector3(this->rand(), this->rand(), this->rand())).toRotationMatrix();
						
						nearest = this->nearest(this->tree[0], chosen); // nearest vertex in tree
						
						if (sigma < this->beta) // maintain orientation for small sigma
						{
							// Gauss sampling for orientation similar to nearest vertex
							
							chosen.linear() = ::rl::math::Quaternion::Random(
								::rl::math::Vector3(this->gauss(), this->gauss(), this->gauss()),
								::rl::math::Quaternion((*get(this->tree[0], nearest.second)->t).linear()),
								::rl::math::Vector3::Constant(sigma / this->beta)
							).toRotationMatrix();
						}
					}
					
					if (nullptr != this->getViewer())
					{
						this->getViewer()->drawWork(chosen);
					}
					
					Vertex connected = this->connect(this->tree[0], nearest, chosen);
					
					if (nullptr != connected)
					{
						if (this->goalEpsilonUseOrientation)
						{
							if (this->distance(*get(this->tree[0], connected)->t, goal) < this->goalEpsilon)
							{
								this->end[0] = connected;
								return true;
							}
						}
						else
						{
							if ((get(this->tree[0], connected)->t->translation() - goal.translation()).norm() < this->goalEpsilon)
							{
								this->end[0] = connected;
								return true;
							}
						}
						
						sigma *= 1 - this->alpha; // increase exploitation
						sigma = ::std::max(static_cast<::rl::math::Real>(0.1), sigma);
						
						for (WorkspaceSphereVector::reverse_iterator k = ++path.rbegin(); k.base() != i; ++k) // search spheres backwards
						{
							if (((*get(this->tree[0], connected)->t).translation() - *k->center).norm() < k->radius) // position is within sphere
							{
								i = k.base(); // advance to matching sphere
								sigma = gamma; // reset exploration/exploitation balance
								break;
							}
						}
					}
					else // expansion unsuccessful
					{
						sigma *= 1 + this->alpha; // decrease exploitation
					}
				}
				else // perform backtracking to previous sphere
				{
					if (i != path.begin())
					{
						--i; // select previous sphere
						sigma = gamma; // reset exploration/exploitation balance
					}
				}
			}
			
			return false;
		}
	}
}
