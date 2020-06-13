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
	axis0(0),
	axis1(1),
	delta0(1),
	delta1(1),
	model(nullptr),
	collisions(nullptr),
	edges(nullptr),
	path(nullptr),
	scene(nullptr),
	thread(new ConfigurationSpaceThread(this))
{
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
	QGraphicsScene::clear();
	this->collisions = nullptr;
	this->edges = nullptr;
	this->path = nullptr;
	this->scene = nullptr;
}

void
ConfigurationSpaceScene::drawConfiguration(const rl::math::Vector& q)
{
}

void
ConfigurationSpaceScene::drawConfigurationEdge(const rl::math::Vector& u, const rl::math::Vector& v, const bool& free)
{
	QGraphicsLineItem* line = this->addLine(
		u(this->axis0),
		-u(this->axis1),
		v(this->axis0),
		-v(this->axis1),
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
			(*i)(this->axis0),
			-(*i)(this->axis1),
			(*j)(this->axis0),
			-(*j)(this->axis1),
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
	
	this->thread->axis0 = this->axis0;
	this->thread->axis1 = this->axis1;
	this->thread->delta0 = this->delta0;
	this->thread->delta1 = this->delta1;
	this->thread->model = this->model;
	
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
	
	this->scene = this->addRect(
		minimum(this->axis0),
		-maximum(this->axis1),
		std::abs(maximum(this->axis0) - minimum(this->axis0)),
		std::abs(maximum(this->axis1) - minimum(this->axis1)),
		QPen(Qt::NoPen),
		QBrush(Qt::white)
	);
	
	this->scene->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
	this->scene->setZValue(0);
	
	this->setSceneRect(this->scene->boundingRect());
	
	this->collisions = this->createItemGroup(QList<QGraphicsItem*>());
	this->collisions->setZValue(1);
	
	this->edges = this->createItemGroup(QList<QGraphicsItem*>());
	this->edges->setZValue(2);
	
	this->path = this->createItemGroup(QList<QGraphicsItem*>());
	this->path->setZValue(3);
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
			rl::math::Vector maximum = this->model->getMaximum();
			rl::math::Vector minimum = this->model->getMinimum();
			
			if (mouseEvent->scenePos().x() > minimum(this->axis0) &&
				mouseEvent->scenePos().x() < maximum(this->axis0))
			{
				(*MainWindow::instance()->q)(this->axis0) = mouseEvent->scenePos().x();
			}
			
			if (-mouseEvent->scenePos().y() > minimum(this->axis1) &&
				-mouseEvent->scenePos().y() < maximum(this->axis1))
			{
				(*MainWindow::instance()->q)(this->axis1) = -mouseEvent->scenePos().y();
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
ConfigurationSpaceScene::resetEdges()
{
	if (nullptr == this->edges)
	{
		return;
	}
	
	QList<QGraphicsItem*> items = this->edges->childItems();
	
	while (!items.isEmpty())
	{
		delete items.takeFirst();
	}
}

void
ConfigurationSpaceScene::resetLines()
{
}

void
ConfigurationSpaceScene::resetPath()
{
	if (nullptr == this->path)
	{
		return;
	}
	
	QList<QGraphicsItem*> items = this->path->childItems();
	
	while (!items.isEmpty())
	{
		delete items.takeFirst();
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

void
ConfigurationSpaceScene::showMessage(const std::string& message)
{
}
