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

#ifndef _RL_PLAN_OMPL_H
#define _RL_PLAN_OMPL_H

#include "Planner.h"
#include <ompl/geometric/SimpleSetup.h>

namespace rl
{
    namespace plan
    {

        class OMPLPlanner : public ::rl::plan::Planner
        {
        public:
            enum PlannerType { AUTOCONFIG, // let OMPL choose one
                PRM, PRMSTAR, LAZYPRM, LAZYPRMSTAR, SPARS, SPARS2, // roadmap-based planners
                RRT, RRTCONNECT, RRTSTAR, TRRT, LBTRRT, LAZYRRT, // RRT variants
                EST, SBL, STRIDE, // EST-like planners
                KPIECE, BKPIECE, LBKPIECE, // KPIECE variants
                PDST, CFOREST, FMT, ANYTIMEPATHSHORTENING // other planners
            };

            OMPLPlanner(PlannerType plannerType = AUTOCONFIG);
            virtual ~OMPLPlanner();
            virtual ::std::string getName() const;
            virtual void getPath(VectorList& path);
            virtual void reset();
            virtual bool solve();

        protected:
            PlannerType plannerType;
            ompl::base::StateSpacePtr space;
            ompl::geometric::SimpleSetup setup;
        };
    }
}

#endif
