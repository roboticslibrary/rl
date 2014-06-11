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

#include <iostream>
#include <QApplication>
#include <QDockWidget>
#include <QFileDialog>
#include <boost/make_shared.hpp>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <rl/sg/Body.h>
#include <rl/sg/DistanceScene.h>
#include <rl/sg/Model.h>
#include <rl/sg/Shape.h>
#include <rl/sg/SimpleScene.h>

#ifdef RL_SG_HAVE_BULLET
#include <rl/sg/bullet/Scene.h>
#endif // RL_SG_HAVE_BULLET
#ifdef RL_SG_HAVE_ODE
#include <rl/sg/ode/Scene.h>
#endif // RL_SG_HAVE_ODE
#ifdef RL_SG_HAVE_PQP
#include <rl/sg/pqp/Scene.h>
#endif // RL_SG_HAVE_PQP
#ifdef RL_SG_HAVE_SOLID
#include <rl/sg/solid/Scene.h>
#endif // RL_SG_HAVE_SOLID

#include "MainWindow.h"
#include "TestWidget.h"

MainWindow::MainWindow(QWidget* parent, Qt::WFlags f) :
	QMainWindow(parent, f),
	collisionScene(),
	viewScene(),
	body(0),
	engine(),
	filename(),
	model(0)
{
	MainWindow::singleton = this;
	
	SoQt::init(this);
	SoDB::init();
	
	QStringList engines;
#ifdef RL_SG_HAVE_ODE
	engines.push_back("ode");
	this->engine = "ode";
#endif // RL_SG_HAVE_ODE
#ifdef RL_SG_HAVE_PQP
	engines.push_back("pqp");
	this->engine = "pqp";
#endif // RL_SG_HAVE_PQP
#ifdef RL_SG_HAVE_BULLET
	engines.push_back("bullet");
	this->engine = "bullet";
#endif // RL_SG_HAVE_BULLET
#ifdef RL_SG_HAVE_SOLID
	engines.push_back("solid");
	this->engine = "solid";
#endif // RL_SG_HAVE_SOLID
	engines.sort();
	
	QRegExp bodyRegExp("--body=(\\d*)");
	QRegExp engineRegExp("--engine=(" + engines.join("|") + ")");
	QRegExp helpRegExp("--help");
	QRegExp modelRegExp("--model=(\\d*)");
	
	for (int i = 1; i < QApplication::arguments().size(); ++i)
	{
		if (-1 != bodyRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->body = bodyRegExp.cap(1).toInt();
		}
		else if (-1 != engineRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->engine = engineRegExp.cap(1);
		}
		else if (-1 != helpRegExp.indexIn(QApplication::arguments()[i]))
		{
			std::cout << "Usage: rlCollisionDemo [SCENEFILE] [--engine=" << engines.join("|").toStdString() << "] [--help] [--model=MODEL] [--body=BODY]" << std::endl;
		}
		else if (-1 != modelRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->model = modelRegExp.cap(1).toInt();
		}
		else
		{
			this->filename = QApplication::arguments()[i];
		}
	}
	
	while (this->filename.isEmpty())
	{
		this->filename = QFileDialog::getOpenFileName(this, "", this->filename, "All Formats (*.xml)");
	}
	
	this->resize(1024, 768);
	
#ifdef RL_SG_HAVE_BULLET
	if ("bullet" == this->engine)
	{
		this->collisionScene = boost::make_shared< rl::sg::bullet::Scene >();
	}
#endif // RL_SG_HAVE_BULLET
#ifdef RL_SG_HAVE_ODE
	if ("ode" == this->engine)
	{
		this->collisionScene = boost::make_shared< rl::sg::ode::Scene >();
	}
#endif // RL_SG_HAVE_ODE
#ifdef RL_SG_HAVE_PQP
	if ("pqp" == this->engine)
	{
		this->collisionScene = boost::make_shared< rl::sg::pqp::Scene >();
	}
#endif // RL_SG_HAVE_PQP
#ifdef RL_SG_HAVE_SOLID
	if ("solid" == this->engine)
	{
		this->collisionScene = boost::make_shared< rl::sg::solid::Scene >();
	}
#endif // RL_SG_HAVE_SOLID
	
	this->viewScene = boost::make_shared< rl::sg::so::Scene >();
	
	this->viewer = new SoQtExaminerViewer(this, NULL, true, SoQtFullViewer::BUILD_POPUP);
	this->viewer->setFeedbackVisibility(true);
	this->viewer->setSceneGraph(this->viewScene->root);
	this->viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	
	this->setCentralWidget(this->viewer->getWidget());
	this->setFocusProxy(this->viewer->getWidget());
	
	this->setWindowTitle(filename + " - " + this->engine + " - rlCollisionDemo");
	
	this->collisionScene->load(this->filename.toStdString());
	this->viewScene->load(this->filename.toStdString());
	
	this->viewer->viewAll();
	
	TestWidget* testWidget = new TestWidget(
		this->viewScene->getModel(this->model)->getBody(this->body),
		this->collisionScene->getModel(this->model)->getBody(this->body),
		this
	);
	
	QDockWidget* testDockWidget = new QDockWidget(this);
	testDockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
	testDockWidget->setFloating(true);
	testDockWidget->setWidget(testWidget);
	testDockWidget->show();
	
	SoVRMLShape* lineShape = new SoVRMLShape();
	this->lineCoordinate = new SoVRMLCoordinate();
	this->lineSet = new SoVRMLIndexedLineSet();
	SoVRMLAppearance* lineAppearance = new SoVRMLAppearance();
	SoVRMLMaterial* lineMaterial = new SoVRMLMaterial();
	lineMaterial->diffuseColor.setValue(1.0f, 0.0f, 0.0f);
	lineAppearance->material = lineMaterial;
	lineShape->appearance = lineAppearance;
	this->lineSet->coord = this->lineCoordinate;
	lineShape->geometry = this->lineSet;
	this->viewScene->root->addChild(lineShape);
	
	this->test();
}

MainWindow::~MainWindow()
{
	MainWindow::singleton = NULL;
}

MainWindow*
MainWindow::instance()
{
	if (NULL == MainWindow::singleton)
	{
		new MainWindow();
	}
	
	return MainWindow::singleton;
}

void
MainWindow::test()
{
	if (rl::sg::SimpleScene* simpleScene = dynamic_cast< rl::sg::SimpleScene* >(this->collisionScene.get()))
	{
		bool test = simpleScene->isColliding();
		
		if (test)
		{
			this->viewer->setBackgroundColor(SbColor(1, 0, 0));
		}
		else
		{
			this->viewer->setBackgroundColor(SbColor(0, 0, 0));
		}
	}
	
	if (rl::sg::DistanceScene* distanceScene = dynamic_cast< rl::sg::DistanceScene* >(this->collisionScene.get()))
	{
		rl::math::Vector3 point1;
		rl::math::Vector3 point2;
		
		rl::math::Real distance = distanceScene->distance(
			this->collisionScene->getModel(0)->getBody(0),
			this->collisionScene->getModel(1)->getBody(0),
			point1,
			point2
		);
		
		this->lineCoordinate->point.setNum(0);
		this->lineSet->coordIndex.setNum(0);
		this->lineCoordinate->point.set1Value(
			this->lineCoordinate->point.getNum(),
			point1.x(), point1.y(), point1.z()
		);
		this->lineSet->coordIndex.set1Value(
			this->lineSet->coordIndex.getNum(),
			this->lineCoordinate->point.getNum() - 1
		);
		this->lineCoordinate->point.set1Value(
			this->lineCoordinate->point.getNum(),
			point2.x(), point2.y(), point2.z()
		);
		this->lineSet->coordIndex.set1Value(
			this->lineSet->coordIndex.getNum(),
			this->lineCoordinate->point.getNum() - 1
		);
	}
}
