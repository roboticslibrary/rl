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

#include "ConfigurationSpaceScene.h"
#include "ConfigurationSpaceThread.h"
#include "MainWindow.h"

ConfigurationSpaceThread::ConfigurationSpaceThread(QObject* parent) :
	QThread(parent),
	scene(nullptr),
	running(false)
{
}

ConfigurationSpaceThread::~ConfigurationSpaceThread()
{
}

void
ConfigurationSpaceThread::run()
{
	if (nullptr == this->scene)
	{
		return;
	}
	
	QMutexLocker lock(&MainWindow::instance()->mutex);
	
	this->running = true;
	
	if (rl::plan::SimpleModel* model = dynamic_cast<rl::plan::SimpleModel*>(this->scene->model))
	{
		rl::math::Vector q(*MainWindow::instance()->q);
		
		for (int i = 0; i < this->scene->steps[0] && this->running; ++i)
		{
			q(this->scene->axis[0]) = this->scene->minimum[0] + i * this->scene->delta[0];
			
			for (int j = 0; j < this->scene->steps[1] && this->running; ++j)
			{
				q(this->scene->axis[1]) = this->scene->minimum[1] + j * this->scene->delta[1];
				unsigned char rgb = model->isColliding(q) ? 0 : 255;
				emit addCollision(i, j, rgb);
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
			QThread::yieldCurrentThread();
		}
	}
}
