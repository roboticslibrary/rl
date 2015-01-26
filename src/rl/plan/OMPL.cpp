/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Mark Moll */

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/tools/config/MagicConstants.h>

#include "OMPL.h"
#include "SimpleModel.h"

namespace ompl
{
    namespace base
    {
        class OMPLStateSampler : public StateSampler
        {
        public:
            OMPLStateSampler(const StateSpace *space) : ompl::base::StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            virtual void sampleUniformNear(State *state, const ompl::base::State *near, const double distance);
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        class OMPLStateSpace : public StateSpace
        {
        public:
            class StateType : public State
            {
            public:
                StateType(::std::size_t ndofs) : rlState(ndofs)
                {
                }

                ::rl::math::Vector rlState;
            };

            OMPLStateSpace(::rl::plan::SimpleModel*& model) : StateSpace(), model(model), epsilon(1e-3)
            {
                setName("OMPLStateSpace" + getName());
            }
            virtual unsigned int getDimension() const
            {
                return model->getDof();
            }
            virtual double getMaximumExtent() const
            {
                ::std::size_t ndofs = this->model->getDof();
                ::rl::math::Vector maximum(ndofs), minimum(ndofs);
                model->getMaximum(maximum);
                model->getMinimum(minimum);
                return model->distance(minimum, maximum);
            }
            virtual double getMeasure() const
            {
                ::std::size_t ndofs = this->model->getDof();
                ::rl::math::Vector maximum(ndofs), minimum(ndofs);
                model->getMaximum(maximum);
                model->getMinimum(minimum);
                double measure = 1.;
                for (::std::size_t i = 0; i<ndofs; ++i)
                    measure *= maximum[i] - minimum[i];
                return measure;
            }
            virtual void enforceBounds(State *state) const
            {
                ::std::size_t ndofs = this->model->getDof();
                ::rl::math::Vector maximum(ndofs), minimum(ndofs);
                model->getMaximum(maximum);
                model->getMinimum(minimum);
                ::rl::math::Vector& v = state->as<StateType>()->rlState;
                for (::std::size_t i = 0; i<ndofs; ++i)
                {
                    if (v[i] < minimum[i])
                        v[i] = minimum[i];
                    else if (v[i] > maximum[i])
                        v[i] = maximum[i];
                }
            }
            virtual bool satisfiesBounds(const State *state) const
            {
                ::std::size_t ndofs = this->model->getDof();
                ::rl::math::Vector maximum(ndofs), minimum(ndofs);
                model->getMaximum(maximum);
                model->getMinimum(minimum);
                const ::rl::math::Vector& v = state->as<StateType>()->rlState;
                for (::std::size_t i = 0; i<ndofs; ++i)
                    if (v[i] < minimum[i] || v[i] > maximum[i])
                        return false;
                return true;
            }
            virtual void copyState(State *destination, const State *source) const
            {
                destination->as<StateType>()->rlState =
                    source->as<StateType>()->rlState;
            }
            virtual double distance(const State *state1, const State *state2) const
            {
                return model->distance(
                    state1->as<StateType>()->rlState,
                    state2->as<StateType>()->rlState);
            }
            virtual bool equalStates(const State *state1, const State *state2) const
            {
                return distance(state1, state2) < epsilon;
            }
            virtual void interpolate(const State *from, const State *to, const double t, State *state) const
            {
                model->interpolate(from->as<StateType>()->rlState,
                    to->as<StateType>()->rlState, t, state->as<StateType>()->rlState);
            }
            virtual StateSamplerPtr allocDefaultStateSampler() const
            {
                return StateSamplerPtr(new OMPLStateSampler(this));
            }
            virtual State* allocState() const
            {
                return new StateType(model->getDof());
            }
            virtual void freeState(State *state) const
            {
                delete state->as<StateType>();
            }
            virtual void registerProjections();

            ::rl::plan::SimpleModel*& model;
            ::rl::math::Real epsilon;
        };

        class OMPLStateValidityChecker : public ompl::base::StateValidityChecker
        {
        public:
            OMPLStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                ::rl::plan::SimpleModel*& model)
                : ompl::base::StateValidityChecker(si), model(model)
            {
            }
            virtual ~OMPLStateValidityChecker()
            {
            }
            virtual bool isValid(const State *state) const
            {
                model->setPosition(state->as<OMPLStateSpace::StateType>()->rlState);
                model->updateFrames();
                return !model->isColliding();
            }
            ::rl::plan::SimpleModel*& model;
        };

        class OMPLProjectionEvaluator : public ompl::base::ProjectionEvaluator
        {
        public:
            OMPLProjectionEvaluator(const StateSpace *space, unsigned int ndim)
                : ProjectionEvaluator(space), ndim(ndim)
            {
            }
            OMPLProjectionEvaluator(const StateSpacePtr &space, unsigned int ndim)
                : ProjectionEvaluator(space), ndim(ndim)
            {
            }
            virtual ~OMPLProjectionEvaluator()
            {
            }
            virtual unsigned int getDimension() const
            {
                return ndim;
            }
            virtual void project(const State *state, EuclideanProjection &projection) const
            {
                // TODO: compute a better projection based on, e.g., end effector position
                // instead of first couple joint angles.
                for (unsigned int i = 0; i < ndim; ++i)
                    projection(i) = state->as<OMPLStateSpace::StateType>()->rlState[i];
            }
            virtual void defaultCellSizes()
            {
                ::std::size_t ndofs = space_->as<OMPLStateSpace>()->model->getDof();
                ::rl::math::Vector maximum(ndofs), minimum(ndofs);
                space_->as<OMPLStateSpace>()->model->getMaximum(maximum);
                space_->as<OMPLStateSpace>()->model->getMinimum(minimum);
                bounds_.resize(ndim);
                cellSizes_.resize(ndim);
                for (unsigned int i = 0 ; i < cellSizes_.size() ; ++i)
                {
                    bounds_.low[i] = minimum[i];
                    bounds_.high[i] = maximum[i];
                    cellSizes_[i] = (bounds_.high[i] - bounds_.low[i]) / magic::PROJECTION_DIMENSION_SPLITS;
                }
            }

        protected:
            unsigned int ndim;
        };

        void OMPLStateSpace::registerProjections()
        {
            ::std::size_t ndofs = model->getDof();
            if (ndofs> 0)
            {
                if (ndofs > 2)
                {
                    int p = std::max(2, (int)ceil(log((double)ndofs)));
                    registerDefaultProjection(ProjectionEvaluatorPtr(new OMPLProjectionEvaluator(this, p)));
                }
                else
                    registerDefaultProjection(ProjectionEvaluatorPtr(new OMPLProjectionEvaluator(this, ndofs)));
            }
        }

        void OMPLStateSampler::sampleUniform(State *state)
        {
            const ::rl::plan::SimpleModel* model = space_->as<OMPLStateSpace>()->model;
            ::std::size_t ndofs = model->getDof();
            ::rl::math::Vector maximum(ndofs), minimum(ndofs);
            model->getMaximum(maximum);
            model->getMinimum(minimum);

            ::rl::math::Vector& v = state->as<OMPLStateSpace::StateType>()->rlState;
            for (::std::size_t i = 0; i < ndofs; ++i)
                v[i] = rng_.uniformReal(minimum[i], maximum[i]);
        }

        void OMPLStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
        {
            const ::rl::plan::SimpleModel* model = space_->as<OMPLStateSpace>()->model;
            ::std::size_t ndofs = model->getDof();
            ::rl::math::Vector maximum(ndofs), minimum(ndofs);
            model->getMaximum(maximum);
            model->getMinimum(minimum);

            ::rl::math::Vector& v = state->as<OMPLStateSpace::StateType>()->rlState;
            const ::rl::math::Vector& vNear = near->as<OMPLStateSpace::StateType>()->rlState;
            for (::std::size_t i = 0; i < ndofs; ++i)
                v[i] = rng_.uniformReal(std::max(minimum[i], vNear[i] - distance),
            std::min(maximum[i], vNear[i] + distance));
        }

        void OMPLStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
        {
            const ::rl::plan::SimpleModel* model = space_->as<OMPLStateSpace>()->model;
            ::std::size_t ndofs = model->getDof();
            ::rl::math::Vector maximum(ndofs), minimum(ndofs);
            model->getMaximum(maximum);
            model->getMinimum(minimum);

            ::rl::math::Vector& v = state->as<OMPLStateSpace::StateType>()->rlState;
            const ::rl::math::Vector& vMean = mean->as<OMPLStateSpace::StateType>()->rlState;
            for (::std::size_t i = 0; i < ndofs; ++i)
            {
                double r = rng_.gaussian(vMean[i], stdDev);
                if (r < minimum[i])
                    r = minimum[i];
                else if (r > maximum[i])
                    r = maximum[i];
                v[i] = r;
            }
        }
    }
}

namespace rl
{
    namespace plan
    {

        OMPLPlanner::OMPLPlanner(PlannerType plannerType) : Planner(),
            plannerType(plannerType),
            space(ompl::base::StateSpacePtr(new ompl::base::OMPLStateSpace(this->model))),
            setup(space)
        {
            setup.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
                new ompl::base::OMPLStateValidityChecker(
                    setup.getSpaceInformation(), model)));
        }

        OMPLPlanner::~OMPLPlanner()
        {
        }

        ::std::string OMPLPlanner::getName() const
        {
            return setup.getPlanner()->getName();
        }
        void OMPLPlanner::getPath(VectorList& path)
        {
            if (setup.haveSolutionPath())
            {
                setup.simplifySolution();
                ompl::geometric::PathGeometric& p(setup.getSolutionPath());
                path.clear();
                for (unsigned int i = 0; i < p.getStateCount(); ++i)
                    path.push_back(
                        static_cast<ompl::base::OMPLStateSpace::StateType*>(p.getState(i))->rlState);
            }
        }

        void OMPLPlanner::reset()
        {
            setup.clear();
        }

        bool OMPLPlanner::solve()
        {
            ompl::base::ScopedState<ompl::base::OMPLStateSpace> start(space), goal(space);
            start->rlState = *this->start;
            goal->rlState = *this->goal;
            setup.setStartAndGoalStates(start, goal);
            if (!setup.getPlanner())
            {
                const ompl::base::SpaceInformationPtr &si = setup.getSpaceInformation();

                switch (plannerType)
                {
                case PRM:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::PRM(si)));
                    break;
                case PRMSTAR:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si)));
                    break;
                case LAZYPRM:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LazyPRM(si)));
                    break;
                case LAZYPRMSTAR:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LazyPRMstar(si)));
                    break;
                case SPARS:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::SPARS(si)));
                    break;
                case SPARS2:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::SPARStwo(si)));
                    break;
                case RRT:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRT(si)));
                    break;
                case RRTCONNECT:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(si)));
                    break;
                case RRTSTAR:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si)));
                    break;
                case TRRT:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::TRRT(si)));
                    break;
                case LBTRRT:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LBTRRT(si)));
                    break;
                case LAZYRRT:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LazyRRT(si)));
                    break;
                case EST:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::EST(si)));
                    break;
                case SBL:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::SBL(si)));
                    break;
                case STRIDE:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::STRIDE(si)));
                    break;
                case KPIECE:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(si)));
                    break;
                case BKPIECE:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::BKPIECE1(si)));
                    break;
                case LBKPIECE:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LBKPIECE1(si)));
                    break;
                case PDST:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::PDST(si)));
                    break;
                case CFOREST:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::CForest(si)));
                    break;
                case FMT:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::FMT(si)));
                    break;
                case ANYTIMEPATHSHORTENING:
                    setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::AnytimePathShortening(si)));
                    break;
                default:
                    ; // nothing: autoconfigure
                }
            }
            return setup.solve(this->duration);
        }

        // ompl::base::ParamSet& OMPLPlanner::params()
        // {
        //     return setup.getPlanner();
        // }
    }
}
