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

#include <QGraphicsRectItem>
#include <QGraphicsSceneMouseEvent>

#include "ConfigurationModel.h"
#include "ConfigurationSpaceScene.h"
#include "ConfigurationSpaceThread.h"
#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

ConfigurationSpaceScene::ConfigurationSpaceScene(QObject* parent) :
	QGraphicsScene(parent),
	delta(1.0f),
	model(NULL),
	x(0),
	y(1),
	edges(),
	path(),
	thread(new ConfigurationSpaceThread(this))
{
	QObject::connect(
		this->thread,
		SIGNAL(addCollision(const qreal&, const qreal&, const qreal&, const qreal&, const int&)),
		this,
		SLOT(addCollision(const qreal&, const qreal&, const qreal&, const qreal&, const int&))
	);
}

ConfigurationSpaceScene::~ConfigurationSpaceScene()
{
	this->thread->stop();
}

void
ConfigurationSpaceScene::addCollision(const qreal& x, const qreal& y, const qreal& w, const qreal& h, const int& rgb)
{
	QGraphicsRectItem* rect = this->addRect(
		x - w / 2.0f,
		-y - h / 2.0f,
		w,
		h,
		QPen(Qt::NoPen),
		QBrush(QColor(rgb, rgb, rgb))
	);
	
	rect->setZValue(1);
}

void
ConfigurationSpaceScene::clear()
{
	QList<QGraphicsItem*> items = this->items();
	
	while (!items.isEmpty())
	{
		delete items.takeFirst();
	}
	
	this->edges.clear();
	this->path.clear();
}

void
ConfigurationSpaceScene::drawConfiguration(const rl::math::Vector& q)
{
}

void
ConfigurationSpaceScene::drawConfigurationEdge(const rl::math::Vector& u, const rl::math::Vector& v, const bool& free)
{
	QGraphicsLineItem* line = this->addLine(
		u(this->x),
		-u(this->y),
		v(this->x),
		-v(this->y),
		free ? QPen(QBrush(QColor(0, 128, 0)), 0.0f) : QPen(QBrush(QColor(128, 0, 0)), 0.0f)
	);
	
	line->setZValue(2);
	
	this->edges.push_back(line);
}

void
ConfigurationSpaceScene::drawConfigurationVertex(const rl::math::Vector& q, const bool& free)
{
}

void
ConfigurationSpaceScene::drawConfigurationPath(const rl::plan::VectorList& path)
{
	this->resetPath();
	
	rl::plan::VectorList::const_iterator i = path.begin();
	rl::plan::VectorList::const_iterator j = ++path.begin();
	
	while (i != path.end() && j != path.end())
	{
		QGraphicsLineItem* line = this->addLine(
			(*i)(this->x),
			-(*i)(this->y),
			(*j)(this->x),
			-(*j)(this->y),
			QPen(QBrush(QColor(0, 255, 0)), 0.0f)
		);
		
		line->setZValue(3);
		
		this->path.push_back(line);
		
		++i;
		++j;
	}
}

void
ConfigurationSpaceScene::drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1)
{
}

void
ConfigurationSpaceScene::drawPoint(const rl::math::Vector& xyz)
{
}

void
ConfigurationSpaceScene::drawSphere(const rl::math::Vector& center, const rl::math::Real& radius)
{
}

void
ConfigurationSpaceScene::drawSweptVolume(const rl::plan::VectorList& path)
{
}

void
ConfigurationSpaceScene::drawWork(const rl::math::Transform& t)
{
}

void
ConfigurationSpaceScene::drawWorkEdge(const rl::math::Vector& u, const rl::math::Vector& v)
{
}

void
ConfigurationSpaceScene::drawWorkPath(const rl::plan::VectorList& path)
{
}

void
ConfigurationSpaceScene::drawWorkVertex(const rl::math::Vector& q)
{
}

void
ConfigurationSpaceScene::eval()
{
	this->thread->stop();
	
	this->clear();
	
	rl::math::Vector maximum(this->model->getDof());
	this->model->getMaximum(maximum);
	rl::math::Vector minimum(this->model->getDof());
	this->model->getMinimum(minimum);
	
	QGraphicsRectItem* rect = this->addRect(
		minimum(this->x),
		-maximum(this->y),
		std::abs(maximum(this->x) - minimum(this->x)),
		std::abs(maximum(this->y) - minimum(this->y)),
		QPen(Qt::NoPen),
		QBrush(Qt::white)
	);
	
	rect->setZValue(0);
	
	this->thread->delta = this->delta;
	this->thread->model = this->model;
	this->thread->x = this->x;
	this->thread->y = this->y;
	
	this->thread->start();
}

void
ConfigurationSpaceScene::mouseMoveEvent(QGraphicsSceneMouseEvent* mouseEvent)
{
	this->mousePressEvent(mouseEvent);
}

void
ConfigurationSpaceScene::mousePressEvent(QGraphicsSceneMouseEvent* mouseEvent)
{
	if (Qt::LeftButton == mouseEvent->buttons())
	{
		if (!MainWindow::instance()->thread->isRunning())
		{
			rl::math::Vector maximum(this->model->getDof());
			this->model->getMaximum(maximum);
			rl::math::Vector minimum(this->model->getDof());
			this->model->getMinimum(minimum);
			
			if (mouseEvent->scenePos().x() > minimum(this->x) &&
				mouseEvent->scenePos().x() < maximum(this->x))
			{
				(*MainWindow::instance()->q)(this->x) = mouseEvent->scenePos().x();
			}
			
			if (mouseEvent->scenePos().y() > minimum(this->y) &&
				mouseEvent->scenePos().y() < maximum(this->y))
			{
				(*MainWindow::instance()->q)(this->y) = -mouseEvent->scenePos().y();
			}
			
			MainWindow::instance()->configurationModel->invalidate();
			MainWindow::instance()->viewer->drawConfiguration(*MainWindow::instance()->q);
		}
	}
}

void
ConfigurationSpaceScene::reset()
{
	this->resetEdges();
	this->resetLines();
	this->resetPath();
}

void
ConfigurationSpaceScene::resetEdges()
{
	while (!this->edges.isEmpty())
	{
		delete this->edges.takeFirst();
	}
}

void
ConfigurationSpaceScene::resetLines()
{
}

void
ConfigurationSpaceScene::resetPath()
{
	while (!this->path.isEmpty())
	{
		delete this->path.takeFirst();
	}
}

void
ConfigurationSpaceScene::resetPoints()
{
}

void
ConfigurationSpaceScene::resetSpheres()
{
}

void
ConfigurationSpaceScene::resetVertices()
{
}
