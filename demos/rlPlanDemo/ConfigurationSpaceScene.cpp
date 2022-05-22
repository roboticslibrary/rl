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
	axis(),
	delta(),
	maximum(),
	minimum(),
	model(nullptr),
	range(),
	steps(),
	collisions(nullptr),
	edges(nullptr),
	path(nullptr),
	scene(nullptr),
	thread(new ConfigurationSpaceThread(this))
{
	this->axis[0] = 0;
	this->axis[1] = 1;
	this->delta[0] = 1;
	this->delta[1] = 1;
	
	this->thread->scene = this;
	
	this->scene = this->addRect(0, 0, 0, 0, QPen(Qt::NoPen), QBrush(Qt::white));
	this->scene->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
	this->scene->setZValue(0);
	
	this->collisions = this->createItemGroup(QList<QGraphicsItem*>());
	this->collisions->setZValue(1);
	
	this->edges = this->createItemGroup(QList<QGraphicsItem*>());
	this->edges->setZValue(2);
	
	this->path = this->createItemGroup(QList<QGraphicsItem*>());
	this->path->setZValue(3);
	
	QObject::connect(
		this->thread,
		SIGNAL(addCollision(const qreal&, const qreal&, const qreal&, const qreal&, const int&)),
		this,
		SLOT(addCollision(const qreal&, const qreal&, const qreal&, const qreal&, const int&))
	);
	
	QObject::connect(this->thread, SIGNAL(finished()), this, SIGNAL(evalFinished()));
}

ConfigurationSpaceScene::~ConfigurationSpaceScene()
{
	this->thread->stop();
}

void
ConfigurationSpaceScene::addCollision(const qreal& x, const qreal& y, const qreal& w, const qreal& h, const int& rgb)
{
	QGraphicsRectItem* rect = this->addRect(
		x - w * static_cast<qreal>(0.5),
		-y - h * static_cast<qreal>(0.5),
		w,
		h,
		QPen(Qt::NoPen),
		QBrush(QColor(rgb, rgb, rgb))
	);
	
	this->collisions->addToGroup(rect);
}

void
ConfigurationSpaceScene::clear()
{
	this->reset();
	this->resetCollisions();
}

void
ConfigurationSpaceScene::drawConfiguration(const rl::math::Vector& q)
{
}

void
ConfigurationSpaceScene::drawConfigurationEdge(const rl::math::Vector& u, const rl::math::Vector& v, const bool& free)
{
	QGraphicsLineItem* line = this->addLine(
		u(this->axis[0]),
		-u(this->axis[1]),
		v(this->axis[0]),
		-v(this->axis[1]),
		free ? QPen(QBrush(QColor(0, 128, 0)), 0) : QPen(QBrush(QColor(128, 0, 0)), 0)
	);
	
	this->edges->addToGroup(line);
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
	
	for (; i != path.end() && j != path.end(); ++i, ++j)
	{
		QGraphicsLineItem* line = this->addLine(
			(*i)(this->axis[0]),
			-(*i)(this->axis[1]),
			(*j)(this->axis[0]),
			-(*j)(this->axis[1]),
			QPen(QBrush(QColor(0, 255, 0)), 0)
		);
		
		this->path->addToGroup(line);
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
	if (nullptr == this->model)
	{
		return;
	}
	
	if (this->model->getDofPosition() < 2)
	{
		return;
	}
	
	this->resetCollisions();
	this->thread->start();
}

void
ConfigurationSpaceScene::init()
{
	this->clear();
	
	if (nullptr == this->model)
	{
		return;
	}
	
	if (this->model->getDofPosition() < 2)
	{
		return;
	}
	
	rl::math::Vector maximum = this->model->getMaximum();
	rl::math::Vector minimum = this->model->getMinimum();
	
	this->maximum[0] = maximum(this->axis[0]);
	this->maximum[1] = maximum(this->axis[1]);
	
	this->minimum[0] = minimum(this->axis[0]);
	this->minimum[1] = minimum(this->axis[1]);
	
	this->range[0] = std::abs(this->maximum[0] - this->minimum[0]);
	this->range[1] = std::abs(this->maximum[1] - this->minimum[1]);
	
	this->steps[0] = static_cast<int>(std::ceil(this->range[0] / this->delta[0]));
	this->steps[1] = static_cast<int>(std::ceil(this->range[1] / this->delta[1]));
	
	this->scene->setRect(
		this->minimum[0],
		-this->maximum[1],
		this->range[0],
		this->range[1]
	);
	
	this->setSceneRect(this->scene->boundingRect());
}

void
ConfigurationSpaceScene::mouseMoveEvent(QGraphicsSceneMouseEvent* mouseEvent)
{
	this->mousePressEvent(mouseEvent);
}

void
ConfigurationSpaceScene::mousePressEvent(QGraphicsSceneMouseEvent* mouseEvent)
{
	if (nullptr == this->model)
	{
		return;
	}
	
	if (this->model->getDofPosition() < 2)
	{
		return;
	}
	
	if (Qt::LeftButton == mouseEvent->buttons())
	{
		if (!MainWindow::instance()->thread->isRunning())
		{
			if (mouseEvent->scenePos().x() > this->minimum[0] &&
				mouseEvent->scenePos().x() < this->maximum[0])
			{
				(*MainWindow::instance()->q)(this->axis[0]) = mouseEvent->scenePos().x();
			}
			
			if (-mouseEvent->scenePos().y() > this->minimum[1] &&
				-mouseEvent->scenePos().y() < this->maximum[1])
			{
				(*MainWindow::instance()->q)(this->axis[1]) = -mouseEvent->scenePos().y();
			}
			
			MainWindow::instance()->configurationModel->invalidate();
			MainWindow::instance()->viewer->drawConfiguration(*MainWindow::instance()->q);
		}
	}
}

void
ConfigurationSpaceScene::reset()
{
	this->thread->stop();
	this->resetEdges();
	this->resetLines();
	this->resetPath();
}

void
ConfigurationSpaceScene::resetCollisions()
{
	qDeleteAll(this->collisions->childItems());
}

void
ConfigurationSpaceScene::resetEdges()
{
	qDeleteAll(this->edges->childItems());
}

void
ConfigurationSpaceScene::resetLines()
{
}

void
ConfigurationSpaceScene::resetPath()
{
	qDeleteAll(this->path->childItems());
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

void
ConfigurationSpaceScene::showMessage(const std::string& message)
{
}

void
ConfigurationSpaceScene::toggleCollisions(const bool& doOn)
{
	this->collisions->setVisible(doOn);
}

void
ConfigurationSpaceScene::toggleConfigurationEdges(const bool& doOn)
{
	this->edges->setVisible(doOn);
}

void
ConfigurationSpaceScene::togglePathEdges(const bool& doOn)
{
	this->path->setVisible(doOn);
}
