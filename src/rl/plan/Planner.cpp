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

#include "Planner.h"
#include "SimpleModel.h"
#include "Viewer.h"

namespace rl
{
	namespace plan
	{
		Planner::Planner() :
			time(),
			duration(::std::chrono::steady_clock::duration::max()),
			goal(nullptr),
			model(nullptr),
			start(nullptr),
			viewer(nullptr)
		{
		}
		
		Planner::~Planner()
		{
		}
		
		::std::chrono::steady_clock::duration
		Planner::getDuration() const
		{
			return this->duration;
		}
		
		::rl::math::Vector*
		Planner::getGoal() const
		{
			return this->goal;
		}
		
		SimpleModel*
		Planner::getModel() const
		{
			return this->model;
		}
		
		::rl::math::Vector*
		Planner::getStart() const
		{
			return this->start;
		}
		
		Viewer*
		Planner::getViewer() const
		{
			return this->viewer;
		}
		
		void
		Planner::setDuration(const ::std::chrono::steady_clock::duration& duration)
		{
			this->duration = duration;
		}
		
		void
		Planner::setGoal(::rl::math::Vector* goal)
		{
			this->goal = goal;
		}
		
		void
		Planner::setModel(SimpleModel* model)
		{
			this->model = model;
		}
		
		void
		Planner::setStart(::rl::math::Vector* start)
		{
			this->start = start;
		}
		
		void
		Planner::setViewer(Viewer* viewer)
		{
			this->viewer = viewer;
		}
		
		bool
		Planner::verify()
		{
			if (!this->model->isValid(*this->start))
			{
				if (nullptr != this->viewer)
				{
					this->viewer->showMessage("Invalid start configuration.");
				}
				
				return false;
			}
			
			if (!this->model->isValid(*this->goal))
			{
				if (nullptr != this->viewer)
				{
					this->viewer->showMessage("Invalid goal configuration.");
				}
				
				return false;
			}
			
			if (this->model->isColliding(*this->start))
			{
				if (nullptr != this->viewer)
				{
					this->viewer->showMessage("Colliding start configuration in body " + ::std::to_string(this->model->getCollidingBody()) + ".");
				}
				
				return false;
			}
			
			if (this->model->isColliding(*this->goal))
			{
				if (nullptr != this->viewer)
				{
					this->viewer->showMessage("Colliding goal configuration in body " + ::std::to_string(this->model->getCollidingBody()) + ".");
				}
				
				return false;
			}
			
			return true;
		}
	}
}
