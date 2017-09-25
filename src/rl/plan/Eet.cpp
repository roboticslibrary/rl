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
#include <rl/math/Unit.h>

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
			alpha(0.01f),
			alternativeDistanceComputation(false),
			beta(0),
			distanceWeight(0.1f),
			explorers(),
			explorersSetup(),
			gamma(1.0f / 3.0f),
			goalEpsilon(0.1f),
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
			
			if (nullptr != this->viewer)
			{
				this->viewer->drawConfigurationEdge(*get(tree, u)->q, *get(tree, v)->q);
			}
			
			return e;
		}
		
		Eet::Vertex
		Eet::addVertex(Tree& tree, const VectorPtr& q)
		{
			::std::shared_ptr<VertexBundle> bundle = ::std::make_shared<VertexBundle>();
			bundle->index = ::boost::num_vertices(tree) - 1;
			bundle->q = q;
			
			Vertex v = ::boost::add_vertex(tree);
			tree[v] = bundle;
			
			if (nullptr != this->viewer)
			{
				this->viewer->drawConfigurationVertex(*get(tree, v)->q);
			}
			
			return v;
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
				best.q = ::std::make_shared< ::rl::math::Vector>(this->model->getDofPosition());
				best.t = ::std::make_shared< ::rl::math::Transform>();
				
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
			
			this->model->setPosition(*nearest->q);
			this->model->updateFrames();
			this->model->updateJacobian();
			this->model->updateJacobianInverse();
			
			::rl::math::Vector qdot(this->model->getDof());
			qdot.setZero();
			
			::rl::math::Vector qdot2(this->model->getDof());
			this->model->inverseVelocity(tdot, qdot2);
			qdot += qdot2;
			
			if (qdot.norm() <= this->delta)
			{
				state = 0;
			}
			else
			{
				qdot.normalize();
				qdot *= this->delta;
			}
			
			this->model->step(*nearest->q, qdot, *expanded.q);
			
			if (this->model->getManipulabilityMeasure() < 1.0e-3f) // within singularity
			{
				*expanded.q = this->sampler->generate(); // uniform sampling for singularities
				::rl::math::Real tmp = this->model->distance(*nearest->q, *expanded.q);
				this->model->interpolate(*nearest->q, *expanded.q, this->delta / tmp, *expanded.q);
			}
			
			if (!this->model->isValid(*expanded.q))
			{
				return -1;
			}
			
			if (nullptr != this->viewer)
			{
				this->viewer->drawConfiguration(*expanded.q);
			}
			
			this->model->setPosition(*expanded.q);
			this->model->updateFrames();
			
			if (this->model->isColliding())
			{
				return -1;
			}
			
			*expanded.t = this->model->forwardPosition();
			
			return state;
		}
		
		Rrt::Vertex
		Eet::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Transform& chosen)
		{
			::rl::math::Real distance = this->distance(*get(tree, nearest.second)->t, chosen);
			
			Vertex extended = nullptr;
			
			VertexBundle best;
			best.q = ::std::make_shared< ::rl::math::Vector>(this->model->getDofPosition());
			best.t = ::std::make_shared< ::rl::math::Transform>();
			
			if (this->expand(get(tree, nearest.second), *get(tree, nearest.second)->t, chosen, distance, best) >= 0)
			{
				extended = this->addVertex(tree, best.q);
				get(tree, extended)->t = best.t;
				this->addEdge(nearest.second, extended, tree);
			}
			
			return extended;
		}
		
		::std::normal_distribution< ::rl::math::Real>::result_type
		Eet::gauss()
		{
			return this->gaussDistribution(this->gaussEngine);
		}
		
		Eet::VertexBundle*
		Eet::get(const Tree& tree, const Vertex& v)
		{
			return static_cast<VertexBundle*>(tree[v].get());
		}
		
		::std::chrono::steady_clock::duration
		Eet::getExplorationDuration() const
		{
			return this->explorationTimeStop - this->explorationTimeStart;
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
			::std::vector< ::rl::math::GnatNearestNeighbors<WorkspaceMetric>::Neighbor> neighbors = this->nn.nearest(WorkspaceMetric::Value(&chosen, Vertex()), 1);
			return Neighbor(neighbors.front().first, neighbors.front().second.second);
		}
		
		::std::uniform_real_distribution< ::rl::math::Real>::result_type
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
		
		bool
		Eet::solve()
		{
			if (this->model->getOperationalDof() > 1)
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
					this->model->setPosition(*this->explorersSetup[i].startConfiguration);
					this->model->updateFrames(false);
					
					if (-1 == this->explorersSetup[i].startFrame)
					{
						*this->explorers[i]->start = this->model->forwardPosition().translation();
					}
					else
					{
						*this->explorers[i]->start = this->model->getFrame(this->explorersSetup[i].startFrame).translation();
					}
				}
				
				if (nullptr != this->explorersSetup[i].goalConfiguration)
				{
					this->model->setPosition(*this->explorersSetup[i].goalConfiguration);
					this->model->updateFrames(false);
					
					if (-1 == this->explorersSetup[i].goalFrame)
					{
						*this->explorers[i]->goal = this->model->forwardPosition().translation();
					}
					else
					{
						*this->explorers[i]->goal = this->model->getFrame(this->explorersSetup[i].goalFrame).translation();
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
			
			this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector>(*this->start));
			this->model->setPosition(*this->start);
			this->model->updateFrames();
			get(this->tree[0], this->begin[0])->t = ::std::make_shared< ::rl::math::Transform>(this->model->forwardPosition());
			this->nn.push(WorkspaceMetric::Value(get(this->tree[0], this->begin[0])->t.get(), begin[0]));
			
			::rl::math::Transform chosen;
			chosen.setIdentity();
			
			::rl::math::Quaternion quaternion;
			
			this->model->setPosition(*this->goal);
			this->model->updateFrames();
			::rl::math::Transform goal = this->model->forwardPosition();
			
			WorkspaceSphereVector::iterator i = ++path.begin();
			::rl::math::Real sigma = gamma; // initialize exploration/exploitation balance
			
			while ((::std::chrono::steady_clock::now() - this->time) < this->duration) // search until goal reached
			{
				if (sigma < 1.0f) // sample is within current sphere
				{
					Neighbor nearest;
					
					if (path.end() == i + 1 && this->rand() < 0.5f) // within last sphere
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
						
						quaternion.setFromUniform(::rl::math::Vector3(this->rand(), this->rand(), this->rand()));
						chosen.linear() = quaternion.toRotationMatrix();
						
						nearest = this->nearest(this->tree[0], chosen); // nearest vertex in tree
						
						if (sigma < this->beta) // maintain orientation for small sigma
						{
							// Gauss sampling for orientation
							
							::rl::math::Real x = this->gauss() * sigma * 180.0f * ::rl::math::DEG2RAD / this->beta;
							::rl::math::Real y = this->gauss() * sigma * 180.0f * ::rl::math::DEG2RAD / this->beta;
							::rl::math::Real z = this->gauss() * sigma * 180.0f * ::rl::math::DEG2RAD / this->beta;
							
							// orientation similar to nearest vertex
							
							chosen.linear() = (*get(this->tree[0], nearest.second)->t).linear() *
								::rl::math::AngleAxis(z, ::rl::math::Vector3::UnitZ()) *
								::rl::math::AngleAxis(y, ::rl::math::Vector3::UnitY()) *
								::rl::math::AngleAxis(x, ::rl::math::Vector3::UnitX());
						}
					}
					
					if (nullptr != this->viewer)
					{
						this->viewer->drawWork(chosen);
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
						
						sigma *= 1.0f - this->alpha; // increase exploitation
						sigma = ::std::max(static_cast< ::rl::math::Real>(0.1f), sigma);
						
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
						sigma *= 1.0f + this->alpha; // decrease exploitation
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
