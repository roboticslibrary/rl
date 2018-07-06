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

#include <QApplication>
#include <QDockWidget>
#include <QFileDialog>
#include <QHeaderView>
#include <QMessageBox>
#include <QTableView>
#include <QStatusBar>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <rl/sg/Body.h>
#include <rl/sg/DepthScene.h>
#include <rl/sg/DistanceScene.h>
#include <rl/sg/Model.h>
#include <rl/sg/Shape.h>
#include <rl/sg/SimpleScene.h>
#include <rl/sg/XmlFactory.h>

#ifdef RL_SG_BULLET
#include <rl/sg/bullet/Scene.h>
#endif // RL_SG_BULLET
#ifdef RL_SG_FCL
#include <rl/sg/fcl/Scene.h>
#endif // RL_SG_FCL
#ifdef RL_SG_ODE
#include <rl/sg/ode/Scene.h>
#endif // RL_SG_ODE
#ifdef RL_SG_PQP
#include <rl/sg/pqp/Scene.h>
#endif // RL_SG_PQP
#ifdef RL_SG_SOLID
#include <rl/sg/solid/Scene.h>
#endif // RL_SG_SOLID

#include "BodyDelegate.h"
#include "BodyModel.h"
#include "MainWindow.h"
#include "SoGradientBackground.h"

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags f) :
	QMainWindow(parent, f),
	collisionScene(),
	viewScene(),
	body(0),
	depthCoordinate(nullptr),
	depthLabel(new QLabel(this)),
	depthLineSet(nullptr),
	depthPointSet(nullptr),
	distanceCoordinate(nullptr),
	distanceLabel(new QLabel(this)),
	distanceLineSet(nullptr),
	distancePointSet(nullptr),
	engine(),
	filename(),
	gradientBackground(),
	model(0),
	simpleLabel(new QLabel(this)),
	viewer(nullptr)
{
	MainWindow::singleton = this;
	
	SoQt::init(this);
	SoDB::init();
	SoGradientBackground::initClass();
	
	QStringList engines;
#ifdef RL_SG_FCL
	engines.push_back("fcl");
	this->engine = "fcl";
#endif // RL_SG_FCL
#ifdef RL_SG_ODE
	engines.push_back("ode");
	this->engine = "ode";
#endif // RL_SG_ODE
#ifdef RL_SG_PQP
	engines.push_back("pqp");
	this->engine = "pqp";
#endif // RL_SG_PQP
#ifdef RL_SG_BULLET
	engines.push_back("bullet");
	this->engine = "bullet";
#endif // RL_SG_BULLET
#ifdef RL_SG_SOLID
	engines.push_back("solid");
	this->engine = "solid";
#endif // RL_SG_SOLID
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
			QMessageBox::information(this, "Usage", "rlCollisionDemo [SCENEFILE] [--engine=" + engines.join("|") + "] [--help] [--model=MODEL] [--body=BODY]");
			exit(0);
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
	
#ifdef RL_SG_BULLET
	if ("bullet" == this->engine)
	{
		this->collisionScene = std::make_shared<rl::sg::bullet::Scene>();
	}
#endif // RL_SG_BULLET
#ifdef RL_SG_FCL
	if ("fcl" == this->engine)
	{
		this->collisionScene = std::make_shared<rl::sg::fcl::Scene>();
	}
#endif // RL_SG_FCL
#ifdef RL_SG_ODE
	if ("ode" == this->engine)
	{
		this->collisionScene = std::make_shared<rl::sg::ode::Scene>();
	}
#endif // RL_SG_ODE
#ifdef RL_SG_PQP
	if ("pqp" == this->engine)
	{
		this->collisionScene = std::make_shared<rl::sg::pqp::Scene>();
	}
#endif // RL_SG_PQP
#ifdef RL_SG_SOLID
	if ("solid" == this->engine)
	{
		this->collisionScene = std::make_shared<rl::sg::solid::Scene>();
	}
#endif // RL_SG_SOLID
	this->viewScene = std::make_shared<rl::sg::so::Scene>();
	
	this->gradientBackground = new SoGradientBackground();
	this->gradientBackground->ref();
	this->gradientBackground->color0.setValue(0.8f, 0.8f, 0.8f);
	this->gradientBackground->color1.setValue(1.0f, 1.0f, 1.0f);
	this->viewScene->root->insertChild(this->gradientBackground, 0);
	
	this->viewer = new SoQtExaminerViewer(this, nullptr, true, SoQtFullViewer::BUILD_POPUP);
	this->viewer->setFeedbackVisibility(true);
	this->viewer->setSceneGraph(this->viewScene->root);
	this->viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	
	this->setCentralWidget(this->viewer->getWidget());
	this->setFocusProxy(this->viewer->getWidget());
	
	this->setWindowTitle(filename + " - " + this->engine.toUpper() + " - rlCollisionDemo");
	
	rl::sg::XmlFactory factory;
	factory.load(this->filename.toStdString(), this->collisionScene.get());
	factory.load(this->filename.toStdString(), this->viewScene.get());
	
	this->viewer->viewAll();
	
	BodyDelegate* bodyDelegate = new BodyDelegate(this);
	
	BodyModel* bodyModel = new BodyModel(this);
	bodyModel->setBody(
		this->viewScene->getModel(this->model)->getBody(this->body),
		this->collisionScene->getModel(this->model)->getBody(this->body)
	);
	
	QTableView* bodyView = new QTableView(this);
#if QT_VERSION >= 0x050000
	bodyView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	bodyView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	bodyView->horizontalHeader()->hide();
	bodyView->setAlternatingRowColors(true);
	bodyView->setItemDelegate(bodyDelegate);
	bodyView->setModel(bodyModel);
	
	QDockWidget* bodyDockWidget = new QDockWidget(this);
	bodyDockWidget->resize(160, 240);
	bodyDockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
	bodyDockWidget->setFloating(true);
	bodyDockWidget->setWidget(bodyView);
	bodyDockWidget->setWindowTitle("Model[" + QString::number(this->model) + "]->Body[" + QString::number(this->body) + "]");
	
	SoDrawStyle* drawStyle = new SoDrawStyle();
	drawStyle->lineWidth = 0.0f;
	drawStyle->pointSize = 10.0f;
	this->viewScene->root->addChild(drawStyle);
	
	this->depthCoordinate = new SoVRMLCoordinate();
	this->depthLineSet = new SoVRMLIndexedLineSet();
	this->depthPointSet = new SoVRMLPointSet();
	
	SoVRMLAppearance* depthAppearance = new SoVRMLAppearance();
	SoVRMLMaterial* depthMaterial = new SoVRMLMaterial();
	depthMaterial->diffuseColor.setValue(1.0f, 0.0f, 0.0f);
	depthMaterial->emissiveColor.setValue(1.0f, 0.0f, 0.0f);
	depthAppearance->material = depthMaterial;
	
	SoVRMLShape* depthLineShape = new SoVRMLShape();
	depthLineShape->appearance = depthAppearance;
	this->depthLineSet->coord = this->depthCoordinate;
	depthLineShape->geometry = this->depthLineSet;
	this->viewScene->root->addChild(depthLineShape);
	
	SoVRMLShape* depthPointShape = new SoVRMLShape();
	depthPointShape->appearance = depthAppearance;
	this->depthPointSet->coord = this->depthCoordinate;
	depthPointShape->geometry = this->depthPointSet;
	this->viewScene->root->addChild(depthPointShape);
	
	this->distanceCoordinate = new SoVRMLCoordinate();
	this->distanceLineSet = new SoVRMLIndexedLineSet();
	this->distancePointSet = new SoVRMLPointSet();
	
	SoVRMLAppearance* distanceAppearance = new SoVRMLAppearance();
	SoVRMLMaterial* distanceMaterial = new SoVRMLMaterial();
	distanceMaterial->diffuseColor.setValue(0.0f, 1.0f, 0.0f);
	distanceMaterial->emissiveColor.setValue(0.0f, 1.0f, 0.0f);
	distanceAppearance->material = distanceMaterial;
	
	SoVRMLShape* distanceLineShape = new SoVRMLShape();
	distanceLineShape->appearance = distanceAppearance;
	this->distanceLineSet->coord = this->distanceCoordinate;
	distanceLineShape->geometry = this->distanceLineSet;
	this->viewScene->root->addChild(distanceLineShape);
	
	SoVRMLShape* distancePointShape = new SoVRMLShape();
	distancePointShape->appearance = distanceAppearance;
	this->distancePointSet->coord = this->distanceCoordinate;
	distancePointShape->geometry = this->distancePointSet;
	this->viewScene->root->addChild(distancePointShape);
	
	if (dynamic_cast<rl::sg::SimpleScene*>(this->collisionScene.get()))
	{
		this->statusBar()->addWidget(this->simpleLabel);
	}
	
	if (dynamic_cast<rl::sg::DistanceScene*>(this->collisionScene.get()))
	{
		this->statusBar()->addWidget(this->distanceLabel);
	}
	
	if (dynamic_cast<rl::sg::DepthScene*>(this->collisionScene.get()))
	{
		this->statusBar()->addWidget(this->depthLabel);
	}
	
	this->test();
}

MainWindow::~MainWindow()
{
	this->gradientBackground->unref();
	MainWindow::singleton = nullptr;
}

MainWindow*
MainWindow::instance()
{
	if (nullptr == MainWindow::singleton)
	{
		new MainWindow();
	}
	
	return MainWindow::singleton;
}

void
MainWindow::test()
{
	if (rl::sg::SimpleScene* simpleScene = dynamic_cast<rl::sg::SimpleScene*>(this->collisionScene.get()))
	{
		bool test = simpleScene->isColliding();
		
		if (test)
		{
			this->simpleLabel->setText("Collision: true");
			this->viewer->setBackgroundColor(SbColor(0.5, 0, 0));
			this->gradientBackground->color0.setValue(0.5f, 0.0f, 0.0f);
			this->gradientBackground->color1.setValue(1.0f, 1.0f, 1.0f);
		}
		else
		{
			this->simpleLabel->setText("Collision: false");
			this->viewer->setBackgroundColor(SbColor(0, 0, 0));
			this->gradientBackground->color0.setValue(0.8f, 0.8f, 0.8f);
			this->gradientBackground->color1.setValue(1.0f, 1.0f, 1.0f);
		}
	}
	
	if (rl::sg::DistanceScene* distanceScene = dynamic_cast<rl::sg::DistanceScene*>(this->collisionScene.get()))
	{
		this->distanceCoordinate->point.setNum(0);
		this->distanceLineSet->coordIndex.setNum(0);
		this->distanceCoordinate->point.setNum(0);
		
		rl::math::Vector3 point1;
		rl::math::Vector3 point2;
		
		rl::math::Real distance = distanceScene->distance(
			this->collisionScene->getModel(0)->getBody(0),
			this->collisionScene->getModel(1)->getBody(0),
			point1,
			point2
		);
		
		this->distanceLabel->setText("Distance: " + QString::number(distance));
		
		if (distance > std::numeric_limits<rl::math::Real>::epsilon())
		{
			this->distanceCoordinate->point.set1Value(
				this->distanceCoordinate->point.getNum(),
				point1.x(), point1.y(), point1.z()
			);
			this->distanceLineSet->coordIndex.set1Value(
				this->distanceLineSet->coordIndex.getNum(),
				this->distanceCoordinate->point.getNum() - 1
			);
			this->distanceCoordinate->point.set1Value(
				this->distanceCoordinate->point.getNum(),
				point2.x(), point2.y(), point2.z()
			);
			this->distanceLineSet->coordIndex.set1Value(
				this->distanceLineSet->coordIndex.getNum(),
				this->distanceCoordinate->point.getNum() - 1
			);
			
			this->distanceCoordinate->point.set1Value(
				this->distanceCoordinate->point.getNum(),
				point1.x(), point1.y(), point1.z()
			);
			this->distanceCoordinate->point.set1Value(
				this->distanceCoordinate->point.getNum(),
				point2.x(), point2.y(), point2.z()
			);
		}
	}
	
	if (rl::sg::DepthScene* depthScene = dynamic_cast<rl::sg::DepthScene*>(this->collisionScene.get()))
	{
		this->depthCoordinate->point.setNum(0);
		this->depthLineSet->coordIndex.setNum(0);
		this->depthCoordinate->point.setNum(0);
		
		rl::math::Vector3 point1;
		rl::math::Vector3 point2;
		
		rl::math::Real depth = depthScene->depth(
			this->collisionScene->getModel(0)->getBody(0),
			this->collisionScene->getModel(1)->getBody(0),
			point1,
			point2
		);
		
		this->depthLabel->setText("Depth: " + QString::number(depth));
		
		if (depth > std::numeric_limits<rl::math::Real>::epsilon())
		{
			this->depthCoordinate->point.set1Value(
				this->depthCoordinate->point.getNum(),
				point1.x(), point1.y(), point1.z()
			);
			this->depthLineSet->coordIndex.set1Value(
				this->depthLineSet->coordIndex.getNum(),
				this->depthCoordinate->point.getNum() - 1
			);
			this->depthCoordinate->point.set1Value(
				this->depthCoordinate->point.getNum(),
				point2.x(), point2.y(), point2.z()
			);
			this->depthLineSet->coordIndex.set1Value(
				this->depthLineSet->coordIndex.getNum(),
				this->depthCoordinate->point.getNum() - 1
			);
			
			this->depthCoordinate->point.set1Value(
				this->depthCoordinate->point.getNum(),
				point1.x(), point1.y(), point1.z()
			);
			this->depthCoordinate->point.set1Value(
				this->depthCoordinate->point.getNum(),
				point2.x(), point2.y(), point2.z()
			);
		}
	}
}
