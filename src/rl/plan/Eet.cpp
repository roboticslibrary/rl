//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//

//#define CSPACE

#include <boost/make_shared.hpp>
#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/util/Timer.h>

#include "DistanceModel.h"
#include "Eet.h"
#include "Exception.h"
#include "Sampler.h"
#include "SimpleModel.h"
#include "Viewer.h"
#include "WorkspaceSphereExplorer.h"
#include "WorkspaceSphereVector.h"

#if _MSC_VER < 1600
#define nullptr NULL // TODO
#endif

struct Compare
{
	bool operator()(const ::std::pair< void*, ::std::size_t >& lhs, const ::std::pair< void*, ::std::size_t >& rhs) const
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
			distanceWeight(0.1f),
			explorers(),
			explorersSetup(),
			gamma(1.0f / 3.0f),
			goalEpsilon(0.1f),
			goalEpsilonUseOrientation(false),
			max(::rl::math::Vector3::Zero()),
			min(::rl::math::Vector3::Zero()),
			gauss(
				::boost::mt19937(static_cast< ::boost::mt19937::result_type >(::rl::util::Timer::now() * 1000000.0f)),
				::boost::normal_distribution< ::rl::math::Real >(0.0f, 1.0f)
			),
			rand(
				::boost::mt19937(static_cast< ::boost::mt19937::result_type >(::rl::util::Timer::now() * 1000000.0f)),
				::boost::uniform_real< ::rl::math::Real >(0.0f, 1.0f)
			),
			exploration(),
			timer()
		{
			this->kd = false;
		}
		
		Eet::~Eet()
		{
		}
		
		Rrt::Edge
		Eet::addEdge(const Vertex& u, const Vertex& v, Tree& tree)
		{
			Edge e = ::boost::add_edge(u, v, tree).first;
			
			if (NULL != this->viewer)
			{
				this->viewer->drawConfigurationEdge(*tree[u].q, *tree[v].q);
			}
			
			return e;
		}
		
		Rrt::Vertex
		Eet::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Transform& chosen)
		{
			::rl::math::Real distance = this->distance(*tree[nearest.first].t, chosen);
			
			Vertex connected = NULL;
			Vertex n = nearest.first;
			int state = 1;
			
			do
			{
				VertexBundle best;
				best.q = ::boost::make_shared< ::rl::math::Vector >(this->model->getDof());
				best.t = ::boost::make_shared< ::rl::math::Transform >();
				
				state = this->expand(tree[n], *tree[nearest.first].t, chosen, distance, best); // TODO
				
				if (state >= 0)
				{
					connected = this->addVertex(tree, best.q);
					tree[connected].t = best.t;
					this->addEdge(n, connected, tree);
					n = connected;
					
					::rl::math::Real distance2 = this->distance(*tree[n].t, chosen);
					
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
			
			if (NULL != connected)
			{
				this->selected.push_back(connected);
			}
			
			return connected;
		}
		
		::rl::math::Real
		Eet::distance(const ::rl::math::Transform& t1, const ::rl::math::Transform& t2) const
		{
			if (this->alternativeDistanceComputation)
			{
				::rl::math::Vector6 delta;
				::rl::math::transform::toDelta(t1, t2, delta);
				return delta.norm();
			}
			else
			{
				return ::rl::math::transform::distance(t1, t2, this->distanceWeight);
			}
		}
		
		int
		Eet::expand(const VertexBundle& nearest, const ::rl::math::Transform& nearest2, const ::rl::math::Transform& chosen, const ::rl::math::Real& distance, VertexBundle& expanded)
		{
			int state = 1;
			
			::rl::math::Vector6 tdot;
			::rl::math::transform::toDelta(nearest2, chosen, tdot);
			
			this->model->setPosition(*nearest.q);
			this->model->updateFrames();
			this->model->updateJacobian();
			this->model->updateJacobianInverse();
			
			::rl::math::Vector qdot(this->model->getDof());
			qdot.setZero();
			
			::rl::math::Vector qdot2(this->model->getDof());
			this->model->inverseVelocity(tdot, qdot2);
			qdot += qdot2;
			
			::rl::math::Vector3 d;
			::rl::math::Vector3 omega;
			RealList distances;
			Vector3List points1;
			Vector3List points2;
			::rl::math::Vector3 v;
			
			if (qdot.norm() <= this->delta)
			{
				state = 0;
			}
			else
			{
				qdot.normalize();
				qdot *= this->delta;
			}
			
			this->model->step(*nearest.q, qdot, *expanded.q);
			
			if (this->model->getManipulabilityMeasure() < 1.0e-3f) // within singularity
			{
				this->sampler->generate(*expanded.q); // uniform sampling for singularities
				::rl::math::Real tmp = this->model->distance(*nearest.q, *expanded.q);
				this->model->interpolate(*nearest.q, *expanded.q, this->delta / tmp, *expanded.q);
			}
			
			if (!this->model->isValid(*expanded.q))
			{
				return -1;
			}
			
			if (NULL != this->viewer)
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
			::rl::math::Real distance = this->distance(*tree[nearest.first].t, chosen);
			
			Vertex extended = NULL;
			
			VertexBundle best;
			best.q = ::boost::make_shared< ::rl::math::Vector >(this->model->getDof());
			best.t = ::boost::make_shared< ::rl::math::Transform >();
			
			if (this->expand(tree[nearest.first], *tree[nearest.first].t, chosen, distance, best) >= 0)
			{
				extended = this->addVertex(tree, best.q);
				tree[extended].t = best.t;
				this->addEdge(nearest.first, extended, tree);
			}
			
			return extended;
		}
		
		::rl::math::Real
		Eet::getExplorationTime() const
		{
			return this->exploration.elapsed();
		}
		
		::std::string
		Eet::getName() const
		{
			return "EET";
		}
		
		::std::size_t
		Eet::getNumEdges() const
		{
			return this->selected.size() - 1;
		}
		
		::std::size_t
		Eet::getNumVertices() const
		{
			return this->selected.size();
		}
		
		void
		Eet::getPath(VectorList& path)
		{
			Rrt::getPath(path);
		}
		
		Rrt::Neighbor
		Eet::nearest(const Tree& tree, const ::rl::math::Transform& chosen)
		{
			Neighbor p(nullptr, ::std::numeric_limits< ::rl::math::Real >::max());
			
			for (std::vector< Vertex >::iterator i = this->selected.begin(); i != this->selected.end(); ++i)
			{
				::rl::math::Real d = this->distance(chosen, *tree[*i].t);
				
				if (d < p.second)
				{
					p.first = *i;
					p.second = d;
				}
			}
			
			return p;
		}
		
		void
		Eet::reset()
		{
			RrtCon::reset();
			
			for (::std::vector<WorkspaceSphereExplorer*>::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
			{
				(*i)->reset();
			}
			
			this->selected.clear();
		}
		
		void
		Eet::seed(const ::boost::mt19937::result_type& value)
		{
			this->gauss.engine().seed(value);
			this->rand.engine().seed(value);
		}
		
		bool
		Eet::solve()
		{
			if (this->model->getOperationalDof() > 1)
			{
				throw("::rl::plan::Eet::solve() - branched kinematics not supported");
			}
			
			this->timer.start();
			this->exploration.start();
			
			// initialize workspace explorers
			
			for (::std::size_t i = 0; i < this->explorersSetup.size(); ++i)
			{
				if (NULL != this->explorersSetup[i].startConfiguration)
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
				
				if (NULL != this->explorersSetup[i].goalConfiguration)
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
			
			for (::std::vector< WorkspaceSphereExplorer* >::iterator j = this->explorers.begin(); j != this->explorers.end(); ++j)
			{
				if (!(*j)->explore())
				{
					return false;
				}
				
				WorkspaceSphereList path2;
				(*j)->getPath(path2);
				path.insert(path.end(), path2.begin(), path2.end());
			}
			
			this->exploration.stop();
			
			// tree initialization with start configuration
			
			this->begin[0] = this->addVertex(this->tree[0], ::boost::make_shared< ::rl::math::Vector >(*this->start));
			this->selected.push_back(this->begin[0]);
			this->model->setPosition(*this->start);
			this->model->updateFrames();
			this->tree[0][this->begin[0]].t = ::boost::make_shared< ::rl::math::Transform >(this->model->forwardPosition());
			
			::rl::math::Transform chosen;
			chosen.setIdentity();
			
			::rl::math::Quaternion quaternion;
			
			this->model->setPosition(*this->goal);
			this->model->updateFrames();
			::rl::math::Transform goal = this->model->forwardPosition();
			
			WorkspaceSphereVector::iterator i = ++path.begin();
			::rl::math::Real sigma = gamma; // initialize exploration/exploitation balance
			
			while (timer.elapsed() < this->duration) // search until goal reached
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
						
						quaternion.fromUniform(this->rand(), this->rand(), this->rand());
						chosen.linear() = quaternion.toRotationMatrix();
						
						nearest = this->nearest(this->tree[0], chosen); // nearest vertex in tree
					}
					
					if (NULL != this->viewer)
					{
						this->viewer->drawWork(chosen);
					}
					
					Vertex connected = this->connect(this->tree[0], nearest, chosen);
					
					if (NULL != connected)
					{
						if (this->goalEpsilonUseOrientation)
						{
							if (this->distance(*this->tree[0][connected].t, goal) < this->goalEpsilon)
							{
								this->end[0] = connected;
								return true;
							}
						}
						else
						{
							if ((this->tree[0][connected].t->translation() - goal.translation()).norm() < this->goalEpsilon)
							{
								this->end[0] = connected;
								return true;
							}
						}
						
						sigma *= 1.0f - this->alpha; // increase exploitation
						sigma = ::std::max(static_cast< ::rl::math::Real >(0.1f), sigma);
						
						for (WorkspaceSphereVector::reverse_iterator k = ++path.rbegin(); k.base() != i; ++k) // search spheres backwards
						{
							if (((*this->tree[0][connected].t).translation() - *k->center).norm() < k->radius) // position is within sphere
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
				
				timer.stop();
			}
			
			return false;
		}
	}
}
