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

#include <QMutexLocker>
#include <rl/plan/SimpleModel.h>

#include "ConfigurationSpaceThread.h"
#include "MainWindow.h"

ConfigurationSpaceThread::ConfigurationSpaceThread(QObject* parent) :
	QThread(parent),
	delta(1.0f),
	model(NULL),
	x(0),
	y(1),
	running(false)
{
}

ConfigurationSpaceThread::~ConfigurationSpaceThread()
{
}

void
ConfigurationSpaceThread::run()
{
	QMutexLocker lock(&MainWindow::instance()->mutex);
	
	this->running = true;
	
	if (rl::plan::SimpleModel* model = dynamic_cast< rl::plan::SimpleModel* >(this->model))
	{
		rl::math::Vector maximum(model->getDof());
		model->getMaximum(maximum);
		rl::math::Vector minimum(model->getDof());
		model->getMinimum(minimum);
		
		rl::math::Real range0 = std::abs(maximum(this->x) - minimum(this->x));
		rl::math::Real range1 = std::abs(maximum(this->y) - minimum(this->y));
		
		rl::math::Real delta0 = range0 / std::ceil(range0 / this->delta);
		rl::math::Real delta1 = range1 / std::ceil(range1 / this->delta);
		
		std::size_t steps0 = static_cast< std::size_t >(std::ceil(range0 / delta0));
		std::size_t steps1 = static_cast< std::size_t >(std::ceil(range1 / delta1));
		
		rl::math::Vector q(*MainWindow::instance()->start);
		
		for (std::size_t i = 0; i < steps1 + 1 && this->running; ++i)
		{
			q(this->y) = maximum(this->y) - i * delta1;
			
			for (std::size_t j = 0; j < steps0 + 1 && this->running; ++j)
			{
				q(this->x) = minimum(this->x) + j * delta0;
				
				model->setPosition(q);
				model->updateFrames();
				
				if (model->isColliding())
				{
					emit addCollision(
						q(this->x),
						q(this->y),
						delta0,
						delta1,
						0
					);
				}
			}
		}
	}
	
	this->running = false;
}

void
ConfigurationSpaceThread::stop()
{
	if (this->running)
	{
		this->running = false;
		
		while (!this->isFinished())
		{
			QThread::usleep(0);
		}
	}
}
