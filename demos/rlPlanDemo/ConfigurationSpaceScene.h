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

#ifndef CONFIGURATIONSPACESCENE_H
#define CONFIGURATIONSPACESCENE_H

#include <QGraphicsItemGroup>
#include <QGraphicsScene>
#include <rl/plan/Model.h>
#include <rl/plan/Viewer.h>

class ConfigurationSpaceThread;

class ConfigurationSpaceScene : public QGraphicsScene, public rl::plan::Viewer
{
	Q_OBJECT
	
public:
	ConfigurationSpaceScene(QObject* parent = nullptr);
	
	virtual ~ConfigurationSpaceScene();
	
	std::size_t axis0;
	
	std::size_t axis1;
	
	rl::math::Real delta0;
	
	rl::math::Real delta1;
	
	rl::plan::Model* model;
	
public slots:
	void addCollision(const qreal& x, const qreal& y, const qreal& w, const qreal& h, const int& rgb);
	
	void clear();
	
	void drawConfiguration(const rl::math::Vector& q);
	
	void drawConfigurationEdge(const rl::math::Vector& u, const rl::math::Vector& v, const bool& free = true);
	
	void drawConfigurationPath(const rl::plan::VectorList& path);
	
	void drawConfigurationVertex(const rl::math::Vector& q, const bool& free = true);
	
	void drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void drawPoint(const rl::math::Vector& xyz);
	
	void drawSphere(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void drawSweptVolume(const rl::plan::VectorList& path);
	
	void drawWork(const rl::math::Transform& t);
	
	void drawWorkEdge(const rl::math::Vector& u, const rl::math::Vector& v);
	
	void drawWorkPath(const rl::plan::VectorList& path);
	
	void drawWorkVertex(const rl::math::Vector& q);
	
	void eval();
	
	void init();
	
	void reset();
	
	void resetEdges();
	
	void resetLines();
	
	void resetPath();
	
	void resetPoints();
	
	void resetSpheres();
	
	void resetVertices();
	
	void showMessage(const std::string& message);
	
protected:
	void mouseMoveEvent(QGraphicsSceneMouseEvent* mouseEvent);
	
	void mousePressEvent(QGraphicsSceneMouseEvent* mouseEvent);
	
private:
	QGraphicsItemGroup* collisions;
	
	QGraphicsItemGroup* edges;
	
	QGraphicsItemGroup* path;
	
	QGraphicsRectItem* scene;
	
	ConfigurationSpaceThread* thread;
	
signals:
	void evalFinished();
};

#endif // CONFIGURATIONSPACESCENE_H
