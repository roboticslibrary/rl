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

#include <chrono>
#include <QApplication>
#include <QDateTime>
#include <QDockWidget>
#include <QFileDialog>
#include <QGraphicsView>
#include <QHeaderView>
#include <QLayout>
#include <QMenuBar>
#include <QMessageBox>
#include <QMutexLocker>
#include <QPainter>
#include <QPrinter>
#include <QStatusBar>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Qt/SoQt.h>
#include <rl/math/Constants.h>
#include <rl/math/Rotation.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/plan/AddRrtConCon.h>
#include <rl/plan/AdvancedOptimizer.h>
#include <rl/plan/BridgeSampler.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/Eet.h>
#include <rl/plan/GaussianSampler.h>
#include <rl/plan/GnatNearestNeighbors.h>
#include <rl/plan/KdtreeBoundingBoxNearestNeighbors.h>
#include <rl/plan/KdtreeNearestNeighbors.h>
#include <rl/plan/LinearNearestNeighbors.h>
#include <rl/plan/Prm.h>
#include <rl/plan/PrmUtilityGuided.h>
#include <rl/plan/RecursiveVerifier.h>
#include <rl/plan/Rrt.h>
#include <rl/plan/RrtCon.h>
#include <rl/plan/RrtConCon.h>
#include <rl/plan/RrtDual.h>
#include <rl/plan/RrtExtCon.h>
#include <rl/plan/RrtExtExt.h>
#include <rl/plan/RrtGoalBias.h>
#include <rl/plan/SequentialVerifier.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/SimpleOptimizer.h>
#include <rl/plan/UniformSampler.h>
#include <rl/plan/WorkspaceSphereExplorer.h>
#include <rl/sg/Body.h>
#include <rl/sg/UrdfFactory.h>
#include <rl/sg/XmlFactory.h>
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>
#include <rl/xml/Stylesheet.h>

#if QT_VERSION >= 0x050200
#include <QCommandLineParser>
#endif

#if QT_VERSION >= 0x060000
#include <QSurfaceFormat>
#else
#include <QGLFormat>
#endif

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

#include "ConfigurationDelegate.h"
#include "ConfigurationModel.h"
#include "ConfigurationSpaceModel.h"
#include "ConfigurationSpaceScene.h"
#include "GraphicsView.h"
#include "MainWindow.h"
#include "PlannerModel.h"
#include "SoGradientBackground.h"
#include "Thread.h"
#include "Viewer.h"

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags f) :
	QMainWindow(parent, f),
	configurationModel(new ConfigurationModel(this)),
	configurationSpaceModel(new ConfigurationSpaceModel(this)),
	engine(),
	explorerGoals(),
	explorers(),
	explorerStarts(),
	goal(),
	kin(),
	kin2(),
	mdl(),
	mdl2(),
	model(),
	model2(),
	mutex(),
	nearestNeighbors(),
	planner(),
	plannerModel(new PlannerModel(this)),
	q(),
	sampler(),
	sampler2(),
	sigma(),
	scene(),
	scene2(),
	sceneModel(nullptr),
	sceneModel2(nullptr),
	start(),
	thread(new Thread(this)),
	verifier(),
	verifier2(),
	viewer(nullptr),
	configurationDelegate(new ConfigurationDelegate(this)),
	configurationDockWidget(new QDockWidget(this)),
	configurationSpaceDockWidget(new QDockWidget(this)),
	configurationSpaceScene(new ConfigurationSpaceScene(this)),
	configurationSpaceSceneDockWidget(new QDockWidget(this)),
	configurationSpaceSceneView(new GraphicsView(this)),
	configurationSpaceView(new QTableView(this)),
	configurationView(new QTableView(this)),
	evalAction(new QAction(this)),
	exitAction(new QAction(this)),
	filename(),
	getGoalConfigurationAction(new QAction(this)),
	getRandomConfigurationAction(new QAction(this)),
	getRandomFreeConfigurationAction(new QAction(this)),
	getStartConfigurationAction(new QAction(this)),
	openAction(new QAction(this)),
	plannerDockWidget(new QDockWidget(this)),
	plannerView(new QTableView(this)),
	resetAction(new QAction(this)),
	saveImageWithAlphaAction(new QAction(this)),
	saveImageWithoutAlphaAction(new QAction(this)),
	savePdfAction(new QAction(this)),
	saveSceneAction(new QAction(this)),
	seed(),
	setGoalConfigurationAction(new QAction(this)),
	setStartConfigurationAction(new QAction(this)),
	startThreadAction(new QAction(this)),
	toggleAnimationAction(new QAction(this)),
	toggleCameraAction(new QAction(this)),
	toggleConfigurationAction(new QAction(this)),
	toggleConfigurationEdgesAction(new QAction(this)),
	toggleConfigurationSpaceAction(new QAction(this)),
	toggleConfigurationSpaceActiveAction(new QAction(this)),
	toggleConfigurationSpaceSceneAction(new QAction(this)),
	toggleConfigurationVerticesAction(new QAction(this)),
	toggleLinesAction(new QAction(this)),
	togglePathEdgesAction(new QAction(this)),
	togglePathVerticesAction(new QAction(this)),
	togglePlannerAction(new QAction(this)),
	togglePointsAction(new QAction(this)),
	toggleSpheresAction(new QAction(this)),
	toggleSweptVolumeAction(new QAction(this)),
	toggleViewActiveAction(new QAction(this)),
	toggleWorkFramesAction(new QAction(this)),
	wait(true)
{
	MainWindow::singleton = this;
	
	SoQt::init(this);
	SoDB::init();
	SoGradientBackground::initClass();
	
#if QT_VERSION >= 0x060000
	QSurfaceFormat format;
	format.setSamples(8);
	QSurfaceFormat::setDefaultFormat(format);
#else
	QGLFormat format;
	format.setAlpha(true);
	format.setSampleBuffers(true);
	QGLFormat::setDefaultFormat(format);
#endif
	
	this->viewer = new Viewer(this);
	this->setCentralWidget(this->viewer);
	
	this->configurationSpaceModel->configurationSpaceScene = this->configurationSpaceScene;
	
#if QT_VERSION >= 0x050000
	this->configurationSpaceView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	this->configurationSpaceView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	this->configurationSpaceView->horizontalHeader()->hide();
	this->configurationSpaceView->setAlternatingRowColors(true);
	this->configurationSpaceView->setModel(this->configurationSpaceModel);
	
	this->configurationSpaceSceneView->setBackgroundBrush(QBrush(QWidget::palette().color(QWidget::backgroundRole())));
	this->configurationSpaceSceneView->setScene(this->configurationSpaceScene);
	
#if QT_VERSION >= 0x050000
	this->configurationView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	this->configurationView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	this->configurationView->horizontalHeader()->hide();
	this->configurationView->setAlternatingRowColors(true);
	this->configurationView->setItemDelegate(this->configurationDelegate);
	this->configurationView->setModel(this->configurationModel);
	
#if QT_VERSION >= 0x050000
	this->plannerView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	this->plannerView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	this->plannerView->horizontalHeader()->hide();
	this->plannerView->setAlternatingRowColors(true);
	this->plannerView->setModel(this->plannerModel);
	this->plannerView->setWordWrap(false);
	
	this->configurationDockWidget->resize(160, 320);
	this->configurationDockWidget->setWidget(this->configurationView);
	this->configurationDockWidget->setWindowTitle("Configuration");
	
	this->configurationSpaceDockWidget->resize(160, 320);
	this->configurationSpaceDockWidget->setWidget(this->configurationSpaceView);
	this->configurationSpaceDockWidget->setWindowTitle("Configuration Space");
	
	this->configurationSpaceSceneDockWidget->resize(320, 320);
	this->configurationSpaceSceneDockWidget->setWidget(this->configurationSpaceSceneView);
	this->configurationSpaceSceneDockWidget->setWindowTitle("Configuration Space");
	
	this->plannerDockWidget->resize(160, 320);
	this->plannerDockWidget->setWidget(this->plannerView);
	this->plannerDockWidget->setWindowTitle("Planner");
	
	this->addDockWidget(Qt::LeftDockWidgetArea, this->configurationSpaceDockWidget);
	this->tabifyDockWidget(this->configurationSpaceDockWidget, this->configurationSpaceSceneDockWidget);
	this->addDockWidget(Qt::LeftDockWidgetArea, this->configurationDockWidget);
	this->addDockWidget(Qt::LeftDockWidgetArea, this->plannerDockWidget);
	
	QObject::connect(this->configurationSpaceModel, SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)), this->configurationSpaceSceneView, SLOT(adjust(const QModelIndex&, const QModelIndex&)));
	QObject::connect(this->configurationSpaceScene, SIGNAL(evalFinished()), this, SLOT(evalDone()));
	
	this->init();
	
	QObject::connect(this->thread, SIGNAL(statusChanged(const QString&)), this->statusBar(), SLOT(showMessage(const QString&)));
	
	this->parseCommandLine();
	
	if (this->filename.isEmpty())
	{
		this->open();
		
		if (this->filename.isEmpty())
		{
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		this->load(this->filename);
	}
}

MainWindow::~MainWindow()
{
	this->reset();
	
	MainWindow::singleton = nullptr;
}

void
MainWindow::clear()
{
	this->configurationSpaceScene->clear();
	this->explorerGoals.clear();
	this->explorers.clear();
	this->explorerStarts.clear();
	this->goal.reset();
	this->kin.reset();
	this->kin2.reset();
	this->mdl.reset();
	this->mdl2.reset();
	this->model.reset();
	this->model2.reset();
	this->nearestNeighbors.clear();
	this->optimizer.reset();
	this->planner.reset();
	this->q.reset();
	this->sampler.reset();
	this->sampler2.reset();
	this->sigma.reset();
	this->scene.reset();
	this->scene2.reset();
	this->sceneModel = nullptr;
	this->sceneModel2 = nullptr;
	this->start.reset();
	this->verifier.reset();
	this->verifier2.reset();
}

void
MainWindow::connect(const QObject* sender, const QObject* receiver)
{
	QObject::connect(
		sender,
		SIGNAL(configurationRequested(const rl::math::Vector&)),
		receiver,
		SLOT(drawConfiguration(const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(configurationEdgeRequested(const rl::math::Vector&, const rl::math::Vector&, const bool&)),
		receiver,
		SLOT(drawConfigurationEdge(const rl::math::Vector&, const rl::math::Vector&, const bool&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(configurationPathRequested(const rl::plan::VectorList&)),
		receiver,
		SLOT(drawConfigurationPath(const rl::plan::VectorList&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(configurationVertexRequested(const rl::math::Vector&, const bool&)),
		receiver,
		SLOT(drawConfigurationVertex(const rl::math::Vector&, const bool&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(edgeResetRequested()),
		receiver,
		SLOT(resetEdges())
	);
	
	QObject::connect(
		sender,
		SIGNAL(lineRequested(const rl::math::Vector&, const rl::math::Vector&)),
		receiver,
		SLOT(drawLine(const rl::math::Vector&, const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(lineResetRequested()),
		receiver,
		SLOT(resetLines())
	);
	
	QObject::connect(
		sender,
		SIGNAL(messageRequested(const std::string&)),
		receiver,
		SLOT(showMessage(const std::string&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(pointRequested(const rl::math::Vector&)),
		receiver,
		SLOT(drawPoint(const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(pointResetRequested()),
		receiver,
		SLOT(resetPoints())
	);
	
	QObject::connect(
		sender,
		SIGNAL(resetRequested()),
		receiver,
		SLOT(reset())
	);
	
	QObject::connect(
		sender,
		SIGNAL(sphereRequested(const rl::math::Vector&, const rl::math::Real&)),
		receiver,
		SLOT(drawSphere(const rl::math::Vector&, const rl::math::Real&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(sphereResetRequested()),
		receiver,
		SLOT(resetSpheres())
	);
	
	QObject::connect(
		sender,
		SIGNAL(sweptVolumeRequested(const rl::plan::VectorList&)),
		receiver,
		SLOT(drawSweptVolume(const rl::plan::VectorList&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(vertexResetRequested()),
		receiver,
		SLOT(resetVertices())
	);
	
	QObject::connect(
		sender,
		SIGNAL(workRequested(const rl::math::Transform&)),
		receiver,
		SLOT(drawWork(const rl::math::Transform&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(workEdgeRequested(const rl::math::Vector&, const rl::math::Vector&)),
		receiver,
		SLOT(drawWorkEdge(const rl::math::Vector&, const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(workPathRequested(const rl::plan::VectorList&)),
		receiver,
		SLOT(drawWorkPath(const rl::plan::VectorList&))
	);
}

void
MainWindow::disconnect(const QObject* sender, const QObject* receiver)
{
	QObject::disconnect(sender, nullptr, receiver, nullptr);
}

void
MainWindow::eval()
{
	this->configurationView->setEnabled(false);
	this->configurationSpaceSceneView->setEnabled(false);
	this->configurationSpaceView->setEnabled(false);
	this->evalAction->setEnabled(false);
	this->getGoalConfigurationAction->setEnabled(false);
	this->getRandomConfigurationAction->setEnabled(false);
	this->getRandomFreeConfigurationAction->setEnabled(false);
	this->getStartConfigurationAction->setEnabled(false);
	this->openAction->setEnabled(false);
	this->plannerView->setEnabled(false);
	this->setGoalConfigurationAction->setEnabled(false);
	this->setStartConfigurationAction->setEnabled(false);
	this->startThreadAction->setEnabled(false);
	this->toggleConfigurationSpaceActiveAction->setEnabled(false);
	this->toggleViewActiveAction->setEnabled(false);
	
	this->model->reset();
	
	MainWindow::instance()->statusBar()->showMessage("Calculating configuration space...");
	
	this->configurationSpaceScene->eval();
}

void
MainWindow::evalDone()
{
	this->configurationView->setEnabled(true);
	this->configurationSpaceSceneView->setEnabled(true);
	this->configurationSpaceView->setEnabled(true);
	this->evalAction->setEnabled(true);
	this->getGoalConfigurationAction->setEnabled(true);
	this->getRandomConfigurationAction->setEnabled(true);
	this->getRandomFreeConfigurationAction->setEnabled(true);
	this->getStartConfigurationAction->setEnabled(true);
	this->openAction->setEnabled(true);
	this->plannerView->setEnabled(true);
	this->setGoalConfigurationAction->setEnabled(true);
	this->setStartConfigurationAction->setEnabled(true);
	this->startThreadAction->setEnabled(true);
	this->toggleAnimationAction->setEnabled(true);
	this->toggleConfigurationSpaceActiveAction->setEnabled(true);
	this->toggleSweptVolumeAction->setEnabled(true);
	this->toggleViewActiveAction->setEnabled(true);
	
	this->statusBar()->showMessage("Finished calculating configuration space.", 1000);
}

void
MainWindow::getGoalConfiguration()
{
	*this->q = *this->goal;
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
	this->statusBar()->showMessage("Showing goal configuration.", 1000);
}

void
MainWindow::getRandomConfiguration()
{
	*this->q = this->sampler2->generate();
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
	this->statusBar()->showMessage("Showing random configuration.", 1000);
}

void
MainWindow::getRandomFreeConfiguration()
{
	*this->q = this->sampler2->generateCollisionFree();
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
	this->statusBar()->showMessage("Showing random collision-free configuration.", 1000);
}

void
MainWindow::getStartConfiguration()
{
	*this->q = *this->start;
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
	this->statusBar()->showMessage("Showing start configuration.", 1000);
}

void
MainWindow::init()
{
	this->statusBar();
	
	QMenu* fileMenu = this->menuBar()->addMenu("File");
	
	this->openAction->setText("Open...");
	this->openAction->setShortcut(QKeySequence::Open);
	QObject::connect(this->openAction, SIGNAL(triggered()), this, SLOT(open()));
	this->addAction(this->openAction);
	fileMenu->addAction(this->openAction);
	
	fileMenu->addSeparator();
	
	this->saveImageWithoutAlphaAction->setText("Save as PNG w/o Alpha");
	this->saveImageWithoutAlphaAction->setShortcut(QKeySequence("Return"));
	QObject::connect(this->saveImageWithoutAlphaAction, SIGNAL(triggered()), this, SLOT(saveImageWithoutAlpha()));
	this->addAction(this->saveImageWithoutAlphaAction);
	fileMenu->addAction(this->saveImageWithoutAlphaAction);
	
	this->saveImageWithAlphaAction->setText("Save as PNG w/ Alpha");
	this->saveImageWithAlphaAction->setShortcut(QKeySequence("Shift+Return"));
	QObject::connect(this->saveImageWithAlphaAction, SIGNAL(triggered()), this, SLOT(saveImageWithAlpha()));
	this->addAction(this->saveImageWithAlphaAction);
	fileMenu->addAction(this->saveImageWithAlphaAction);
	
	this->saveSceneAction->setText("Save as VRML");
	this->saveSceneAction->setShortcut(QKeySequence("Ctrl+Return"));
	QObject::connect(this->saveSceneAction, SIGNAL(triggered()), this, SLOT(saveScene()));
	this->addAction(this->saveSceneAction);
	fileMenu->addAction(this->saveSceneAction);
	
	this->savePdfAction->setText("Save as PDF");
	this->savePdfAction->setShortcut(QKeySequence("Alt+Return"));
	QObject::connect(this->savePdfAction, SIGNAL(triggered()), this, SLOT(savePdf()));
	this->addAction(this->savePdfAction);
	fileMenu->addAction(this->savePdfAction);
	
	fileMenu->addSeparator();
	
	this->exitAction->setText("Exit");
	QObject::connect(this->exitAction, SIGNAL(triggered()), qApp, SLOT(quit()));
	this->addAction(this->exitAction);
	fileMenu->addAction(this->exitAction);
	
	QMenu* configurationMenu = this->menuBar()->addMenu("Configuration");
	
	this->toggleConfigurationAction->setText("Parameters");
	this->toggleConfigurationAction->setShortcut(QKeySequence("F5"));
	QObject::connect(this->toggleConfigurationAction, SIGNAL(triggered()), this, SLOT(toggleConfiguration()));
	this->addAction(this->toggleConfigurationAction);
	configurationMenu->addAction(this->toggleConfigurationAction);
	
	configurationMenu->addSeparator();
	
	this->getRandomConfigurationAction->setText("Random");
	this->getRandomConfigurationAction->setShortcut(QKeySequence("F3"));
	QObject::connect(this->getRandomConfigurationAction, SIGNAL(triggered()), this, SLOT(getRandomConfiguration()));
	this->addAction(this->getRandomConfigurationAction);
	configurationMenu->addAction(this->getRandomConfigurationAction);
	
	this->getRandomFreeConfigurationAction->setText("Random (Collision-Free)");
	this->getRandomFreeConfigurationAction->setShortcut(QKeySequence("F4"));
	QObject::connect(this->getRandomFreeConfigurationAction, SIGNAL(triggered()), this, SLOT(getRandomFreeConfiguration()));
	this->addAction(this->getRandomFreeConfigurationAction);
	configurationMenu->addAction(this->getRandomFreeConfigurationAction);
	
	QMenu* configurationSpaceMenu = this->menuBar()->addMenu("Configuration Space");
	
	this->toggleConfigurationSpaceAction->setText("Parameters");
	this->toggleConfigurationSpaceAction->setShortcut(QKeySequence("F6"));
	QObject::connect(this->toggleConfigurationSpaceAction, SIGNAL(triggered()), this, SLOT(toggleConfigurationSpace()));
	this->addAction(this->toggleConfigurationSpaceAction);
	configurationSpaceMenu->addAction(this->toggleConfigurationSpaceAction);
	
	this->toggleConfigurationSpaceSceneAction->setText("Visualization");
	this->toggleConfigurationSpaceSceneAction->setShortcut(QKeySequence("F10"));
	QObject::connect(this->toggleConfigurationSpaceSceneAction, SIGNAL(triggered()), this, SLOT(toggleConfigurationSpaceScene()));
	this->addAction(this->toggleConfigurationSpaceSceneAction);
	configurationSpaceMenu->addAction(this->toggleConfigurationSpaceSceneAction);
	
	configurationSpaceMenu->addSeparator();
	
	this->toggleConfigurationSpaceActiveAction->setCheckable(true);
	this->toggleConfigurationSpaceActiveAction->setChecked(false);
	this->toggleConfigurationSpaceActiveAction->setText("Active");
	QObject::connect(this->toggleConfigurationSpaceActiveAction, SIGNAL(toggled(bool)), this, SLOT(toggleConfigurationSpaceActive(bool)));
	this->addAction(this->toggleConfigurationSpaceActiveAction);
	configurationSpaceMenu->addAction(this->toggleConfigurationSpaceActiveAction);
	
	configurationSpaceMenu->addSeparator();
	
	this->evalAction->setText("Evaluate");
	this->evalAction->setShortcut(QKeySequence("F11"));
	QObject::connect(this->evalAction, SIGNAL(triggered()), this, SLOT(eval()));
	this->addAction(this->evalAction);
	configurationSpaceMenu->addAction(this->evalAction);
	
	QMenu* plannerMenu = this->menuBar()->addMenu("Planner");
	
	this->togglePlannerAction->setText("Parameters");
	this->togglePlannerAction->setShortcut(QKeySequence("F7"));
	QObject::connect(this->togglePlannerAction, SIGNAL(triggered()), this, SLOT(togglePlanner()));
	this->addAction(this->togglePlannerAction);
	plannerMenu->addAction(this->togglePlannerAction);
	
	plannerMenu->addSeparator();
	
	this->getStartConfigurationAction->setText("Get Start Configuration");
	this->getStartConfigurationAction->setShortcut(QKeySequence("F1"));
	QObject::connect(this->getStartConfigurationAction, SIGNAL(triggered()), this, SLOT(getStartConfiguration()));
	this->addAction(this->getStartConfigurationAction);
	plannerMenu->addAction(this->getStartConfigurationAction);
	
	this->setStartConfigurationAction->setText("Set Start Configuration");
	this->setStartConfigurationAction->setShortcut(QKeySequence("CTRL+F1"));
	QObject::connect(this->setStartConfigurationAction, SIGNAL(triggered()), this, SLOT(setStartConfiguration()));
	this->addAction(this->setStartConfigurationAction);
	plannerMenu->addAction(this->setStartConfigurationAction);
	
	plannerMenu->addSeparator();
	
	this->getGoalConfigurationAction->setText("Get Goal Configuration");
	this->getGoalConfigurationAction->setShortcut(QKeySequence("F2"));
	QObject::connect(this->getGoalConfigurationAction, SIGNAL(triggered()), this, SLOT(getGoalConfiguration()));
	this->addAction(this->getGoalConfigurationAction);
	plannerMenu->addAction(this->getGoalConfigurationAction);
	
	this->setGoalConfigurationAction->setText("Set Goal Configuration");
	this->setGoalConfigurationAction->setShortcut(QKeySequence("CTRL+F2"));
	QObject::connect(this->setGoalConfigurationAction, SIGNAL(triggered()), this, SLOT(setGoalConfiguration()));
	this->addAction(this->setGoalConfigurationAction);
	plannerMenu->addAction(this->setGoalConfigurationAction);
	
	plannerMenu->addSeparator();
	
	this->startThreadAction->setText("Start");
	this->startThreadAction->setShortcut(QKeySequence("Space"));
	QObject::connect(this->startThreadAction, SIGNAL(triggered()), this, SLOT(startThread()));
	this->addAction(this->startThreadAction);
	plannerMenu->addAction(this->startThreadAction);
	
	this->resetAction->setText("Reset");
	this->resetAction->setShortcut(QKeySequence("F12"));
	QObject::connect(this->resetAction, SIGNAL(triggered()), this, SLOT(reset()));
	this->addAction(this->resetAction);
	plannerMenu->addAction(this->resetAction);
	
	QMenu* viewMenu = this->menuBar()->addMenu("View");
	
	this->toggleViewActiveAction->setCheckable(true);
	this->toggleViewActiveAction->setChecked(true);
	this->toggleViewActiveAction->setText("Active");
	QObject::connect(this->toggleViewActiveAction, SIGNAL(toggled(bool)), this, SLOT(toggleViewActive(bool)));
	this->addAction(this->toggleViewActiveAction);
	viewMenu->addAction(this->toggleViewActiveAction);
	
	viewMenu->addSeparator();
	
	this->togglePathEdgesAction->setCheckable(true);
	this->togglePathEdgesAction->setChecked(true);
	this->togglePathEdgesAction->setText("Path Edges");
	QObject::connect(this->togglePathEdgesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(togglePathEdges(bool)));
	this->addAction(this->togglePathEdgesAction);
	viewMenu->addAction(this->togglePathEdgesAction);
	
	this->togglePathVerticesAction->setCheckable(true);
	this->togglePathVerticesAction->setChecked(false);
	this->togglePathVerticesAction->setText("Path Vertices");
	QObject::connect(this->togglePathVerticesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(togglePathVertices(bool)));
	this->addAction(this->togglePathVerticesAction);
	viewMenu->addAction(this->togglePathVerticesAction);
	
	this->toggleAnimationAction->setCheckable(true);
	this->toggleAnimationAction->setChecked(true);
	this->toggleAnimationAction->setText("Animation");
	QObject::connect(this->toggleAnimationAction, SIGNAL(toggled(bool)), this, SLOT(toggleAnimation(bool)));
	this->addAction(this->toggleAnimationAction);
	viewMenu->addAction(this->toggleAnimationAction);
	
	this->toggleSweptVolumeAction->setCheckable(true);
	this->toggleSweptVolumeAction->setChecked(false);
	this->toggleSweptVolumeAction->setText("Swept Volume");
	QObject::connect(this->toggleSweptVolumeAction, SIGNAL(toggled(bool)), this, SLOT(toggleSweptVolume(bool)));
	QObject::connect(this->toggleSweptVolumeAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleSweptVolume(bool)));
	this->addAction(this->toggleSweptVolumeAction);
	viewMenu->addAction(this->toggleSweptVolumeAction);
	
	viewMenu->addSeparator();
	
	this->toggleConfigurationEdgesAction->setCheckable(true);
	this->toggleConfigurationEdgesAction->setChecked(true);
	this->toggleConfigurationEdgesAction->setText("Configuration Edges");
	QObject::connect(this->toggleConfigurationEdgesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleConfigurationEdges(bool)));
	this->addAction(this->toggleConfigurationEdgesAction);
	viewMenu->addAction(this->toggleConfigurationEdgesAction);
	
	this->toggleConfigurationVerticesAction->setCheckable(true);
	this->toggleConfigurationVerticesAction->setChecked(false);
	this->toggleConfigurationVerticesAction->setText("Configuration Vertices");
	QObject::connect(this->toggleConfigurationVerticesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleConfigurationVertices(bool)));
	this->addAction(this->toggleConfigurationVerticesAction);
	viewMenu->addAction(this->toggleConfigurationVerticesAction);
	
	viewMenu->addSeparator();
	
	this->toggleWorkFramesAction->setCheckable(true);
	this->toggleWorkFramesAction->setChecked(false);
	this->toggleWorkFramesAction->setText("Work Frames");
	QObject::connect(this->toggleWorkFramesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleWorkFrames(bool)));
	this->addAction(this->toggleWorkFramesAction);
	viewMenu->addAction(this->toggleWorkFramesAction);
	
	viewMenu->addSeparator();
	
	this->toggleLinesAction->setCheckable(true);
	this->toggleLinesAction->setChecked(true);
	this->toggleLinesAction->setText("Lines");
	QObject::connect(this->toggleLinesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleLines(bool)));
	this->addAction(this->toggleLinesAction);
	viewMenu->addAction(this->toggleLinesAction);
	
	this->togglePointsAction->setCheckable(true);
	this->togglePointsAction->setChecked(true);
	this->togglePointsAction->setText("Points");
	QObject::connect(this->togglePointsAction, SIGNAL(toggled(bool)), this->viewer, SLOT(togglePoints(bool)));
	this->addAction(this->togglePointsAction);
	viewMenu->addAction(this->togglePointsAction);
	
	this->toggleSpheresAction->setCheckable(true);
	this->toggleSpheresAction->setChecked(true);
	this->toggleSpheresAction->setText("Spheres");
	QObject::connect(this->toggleSpheresAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleSpheres(bool)));
	this->addAction(this->toggleSpheresAction);
	viewMenu->addAction(this->toggleSpheresAction);
	
	viewMenu->addSeparator();
	
	this->toggleCameraAction->setText("Perspective/Orthographic");
	this->toggleCameraAction->setShortcut(QKeySequence("F9"));
	QObject::connect(this->toggleCameraAction, SIGNAL(triggered()), this, SLOT(toggleCamera()));
	this->addAction(this->toggleCameraAction);
	viewMenu->addAction(this->toggleCameraAction);
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
MainWindow::load(const QString& filename)
{
	this->statusBar()->showMessage("Loading '" + filename + "'...");
	
	QMutexLocker lock(&this->mutex);
	
	this->clear();
	
	rl::xml::DomParser parser;
	
	rl::xml::Document document = parser.readFile(filename.toStdString(), "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	document.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	
	if ("stylesheet" == document.getRootElement().getName() || "transform" == document.getRootElement().getName())
	{
		if ("1.0" == document.getRootElement().getProperty("version"))
		{
			if (document.getRootElement().hasNamespace() && "http://www.w3.org/1999/XSL/Transform" == document.getRootElement().getNamespace().getHref())
			{
				rl::xml::Stylesheet stylesheet(document);
				document = stylesheet.apply();
			}
		}
	}
	
	this->filename = filename;
	this->setWindowTitle(filename + " - " + this->engine.toUpper() + " - rlPlanDemo");
	
	rl::xml::Path path(document);
	
#ifdef RL_SG_BULLET
	if ("bullet" == this->engine)
	{
		this->scene = std::make_shared<rl::sg::bullet::Scene>();
	}
#endif // RL_SG_BULLET
#ifdef RL_SG_FCL
	if ("fcl" == this->engine)
	{
		this->scene = std::make_shared<rl::sg::fcl::Scene>();
	}
#endif // RL_SG_FCL
#ifdef RL_SG_ODE
	if ("ode" == this->engine)
	{
		this->scene = std::make_shared<rl::sg::ode::Scene>();
	}
#endif // RL_SG_ODE
#ifdef RL_SG_PQP
	if ("pqp" == this->engine)
	{
		this->scene = std::make_shared<rl::sg::pqp::Scene>();
	}
#endif // RL_SG_PQP
#ifdef RL_SG_SOLID
	if ("solid" == this->engine)
	{
		this->scene = std::make_shared<rl::sg::solid::Scene>();
	}
#endif // RL_SG_SOLID
	
	rl::xml::NodeSet modelScene = path.eval("(/rl/plan|/rlplan)//model/scene").getValue<rl::xml::NodeSet>();
	std::string modelSceneFilename = modelScene[0].getLocalPath(modelScene[0].getProperty("href"));
	
	if ("urdf" == modelSceneFilename.substr(modelSceneFilename.length() - 4, 4))
	{
		rl::sg::UrdfFactory sceneFactory;
		sceneFactory.load(modelSceneFilename, this->scene.get());
		this->sceneModel = this->scene->getModel(
			path.eval("number((/rl/plan|/rlplan)//model/model)").getValue<std::size_t>()
		);
	}
	else
	{
		rl::sg::XmlFactory sceneFactory;
		sceneFactory.load(modelSceneFilename, this->scene.get());
		this->sceneModel = this->scene->getModel(
			path.eval("number((/rl/plan|/rlplan)//model/model)").getValue<std::size_t>()
		);
	}
	
	rl::xml::NodeSet modelKinematics = path.eval("(/rl/plan|/rlplan)//model/kinematics").getValue<rl::xml::NodeSet>();
	std::string modelKinematicsFilename = modelKinematics[0].getLocalPath(modelKinematics[0].getProperty("href"));
	
	if ("urdf" == modelKinematicsFilename.substr(modelKinematicsFilename.length() - 4, 4))
	{
		rl::mdl::UrdfFactory modelFactory;
		this->mdl = std::dynamic_pointer_cast<rl::mdl::Kinematic>(modelFactory.create(modelKinematicsFilename));
	}
	else if ("mdl" == modelKinematics[0].getProperty("type"))
	{
		rl::mdl::XmlFactory modelFactory;
		this->mdl = std::dynamic_pointer_cast<rl::mdl::Kinematic>(modelFactory.create(modelKinematicsFilename));
	}
	else
	{
		this->kin = rl::kin::Kinematics::create(modelKinematicsFilename);
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//model/kinematics/world) > 0").getValue<bool>())
	{
		rl::math::Transform* world = nullptr;
		
		if (nullptr != this->kin.get())
		{
			world = &this->kin->world();
		}
		else if (nullptr != this->mdl.get())
		{
			world = &this->mdl->world();
		}
		
		world->linear() = rl::math::AngleAxis(
			path.eval("number((/rl/plan|/rlplan)//model/kinematics/world/rotation/z)").getValue<rl::math::Real>(0) * rl::math::constants::deg2rad,
			rl::math::Vector3::UnitZ()
		) * rl::math::AngleAxis(
			path.eval("number((/rl/plan|/rlplan)//model/kinematics/world/rotation/y)").getValue<rl::math::Real>(0) * rl::math::constants::deg2rad,
			rl::math::Vector3::UnitY()
		) * rl::math::AngleAxis(
			path.eval("number((/rl/plan|/rlplan)//model/kinematics/world/rotation/x)").getValue<rl::math::Real>(0) * rl::math::constants::deg2rad,
			rl::math::Vector3::UnitX()
		).toRotationMatrix();
		
		world->translation().x() = path.eval("number((/rl/plan|/rlplan)//model/kinematics/world/translation/x)").getValue<rl::math::Real>(0);
		world->translation().y() = path.eval("number((/rl/plan|/rlplan)//model/kinematics/world/translation/y)").getValue<rl::math::Real>(0);
		world->translation().z() = path.eval("number((/rl/plan|/rlplan)//model/kinematics/world/translation/z)").getValue<rl::math::Real>(0);
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//model/kinematics/max) > 0").getValue<bool>())
	{
		rl::xml::NodeSet max = path.eval("(/rl/plan|/rlplan)//model/kinematics/max/q").getValue<rl::xml::NodeSet>();
		
		rl::math::Vector maximum(max.size());
		
		if (nullptr != this->kin.get())
		{
			this->kin->getMaximum(maximum);
		}
		else if (nullptr != this->mdl.get())
		{
			maximum = this->mdl->getMaximum();
		}
		
		for (int i = 0; i < max.size(); ++i)
		{
			std::string content = max[i].getContent();
			
			if (!content.empty())
			{
				maximum(i) = std::atof(content.c_str());
				
				if ("deg" == max[i].getProperty("unit"))
				{
					maximum(i) *= rl::math::constants::deg2rad;
				}
			}
		}
		
		if (nullptr != this->kin.get())
		{
			this->kin->setMaximum(maximum);
		}
		else if (nullptr != this->mdl.get())
		{
			this->mdl->setMaximum(maximum);
		}
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//model/kinematics/min) > 0").getValue<bool>())
	{
		rl::xml::NodeSet min = path.eval("(/rl/plan|/rlplan)//model/kinematics/min/q").getValue<rl::xml::NodeSet>();
		
		rl::math::Vector minimum(min.size());
		
		if (nullptr != this->kin.get())
		{
			this->kin->getMinimum(minimum);
		}
		else if (nullptr != this->mdl.get())
		{
			minimum = this->mdl->getMinimum();
		}
		
		for (int i = 0; i < min.size(); ++i)
		{
			std::string content = min[i].getContent();
			
			if (!content.empty())
			{
				minimum(i) = std::atof(content.c_str());
				
				if ("deg" == min[i].getProperty("unit"))
				{
					minimum(i) *= rl::math::constants::deg2rad;
				}
			}
		}
		
		if (nullptr != this->kin.get())
		{
			this->kin->setMinimum(minimum);
		}
		else if (nullptr != this->mdl.get())
		{
			this->mdl->setMinimum(minimum);
		}
	}
	
	if (rl::sg::DistanceScene* scene = dynamic_cast<rl::sg::DistanceScene*>(this->scene.get()))
	{
		this->model = std::make_shared<rl::plan::DistanceModel>();
	}
	else if (rl::sg::SimpleScene* scene = dynamic_cast<rl::sg::SimpleScene*>(this->scene.get()))
	{
		this->model = std::make_shared<rl::plan::SimpleModel>();
	}
	else
	{
		throw std::runtime_error("selected engine does not support collision queries");
	}
	
	if (nullptr != this->kin)
	{
		this->model->kin = this->kin.get();
	}
	else if (nullptr != this->mdl)
	{
		this->model->mdl = this->mdl.get();
	}
	
	this->model->model = this->sceneModel;
	this->model->scene = this->scene.get();
	
	this->q = std::make_shared<rl::math::Vector>(this->model->getDofPosition());
	
	if (nullptr != this->scene2)
	{
		this->viewer->sceneGroup->removeChild(this->scene2->root);
	}
	
	this->scene2 = std::make_shared<rl::sg::so::Scene>();
	
	rl::xml::NodeSet viewerScene = path.eval("(/rl/plan|/rlplan)//viewer/model/scene").getValue<rl::xml::NodeSet>();
	std::string viewerSceneFilename = viewerScene[0].getLocalPath(viewerScene[0].getProperty("href"));
	
	if ("urdf" == viewerSceneFilename.substr(viewerSceneFilename.length() - 4, 4))
	{
		rl::sg::UrdfFactory sceneFactory;
		sceneFactory.load(viewerSceneFilename, this->scene2.get());
		this->sceneModel2 = static_cast<rl::sg::so::Model*>(this->scene2->getModel(
			path.eval("number((/rl/plan|/rlplan)//viewer/model/model)").getValue<std::size_t>()
		));
	}
	else
	{
		rl::sg::XmlFactory sceneFactory;
		sceneFactory.load(viewerSceneFilename, this->scene2.get());
		this->sceneModel2 = static_cast<rl::sg::so::Model*>(this->scene2->getModel(
			path.eval("number((/rl/plan|/rlplan)//viewer/model/model)").getValue<std::size_t>()
		));
	}
	
	rl::xml::NodeSet viewerKinematics = path.eval("(/rl/plan|/rlplan)//viewer/model/kinematics").getValue<rl::xml::NodeSet>();
	std::string viewerKinematicsFilename = viewerKinematics[0].getLocalPath(viewerKinematics[0].getProperty("href"));
	
	if ("urdf" == viewerKinematicsFilename.substr(viewerKinematicsFilename.length() - 4, 4))
	{
		rl::mdl::UrdfFactory modelFactory;
		this->mdl2 = std::dynamic_pointer_cast<rl::mdl::Kinematic>(modelFactory.create(viewerKinematicsFilename));
	}
	else if ("mdl" == viewerKinematics[0].getProperty("type"))
	{
		rl::mdl::XmlFactory modelFactory;
		this->mdl2 = std::dynamic_pointer_cast<rl::mdl::Kinematic>(modelFactory.create(viewerKinematicsFilename));
	}
	else
	{
		rl::xml::NodeSet kin2 = path.eval("(/rl/plan|/rlplan)//viewer/model/kinematics").getValue<rl::xml::NodeSet>();
		this->kin2 = rl::kin::Kinematics::create(viewerKinematicsFilename);
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//viewer/model/kinematics/world) > 0").getValue<bool>())
	{
		rl::math::Transform* world = nullptr;
		
		if (nullptr != this->kin2.get())
		{
			world = &this->kin2->world();
		}
		else if (nullptr != this->mdl2.get())
		{
			world = &this->mdl2->world();
		}
		
		world->linear() = rl::math::AngleAxis(
			path.eval("number((/rl/plan|/rlplan)//viewer/model/kinematics/world/rotation/z)").getValue<rl::math::Real>(0) * rl::math::constants::deg2rad,
			rl::math::Vector3::UnitZ()
		) * rl::math::AngleAxis(
			path.eval("number((/rl/plan|/rlplan)//viewer/model/kinematics/world/rotation/y)").getValue<rl::math::Real>(0) * rl::math::constants::deg2rad,
			rl::math::Vector3::UnitY()
		) * rl::math::AngleAxis(
			path.eval("number((/rl/plan|/rlplan)//viewer/model/kinematics/world/rotation/x)").getValue<rl::math::Real>(0) * rl::math::constants::deg2rad,
			rl::math::Vector3::UnitX()
		).toRotationMatrix();
		
		world->translation().x() = path.eval("number((/rl/plan|/rlplan)//viewer/model/kinematics/world/translation/x)").getValue<rl::math::Real>(0);
		world->translation().y() = path.eval("number((/rl/plan|/rlplan)//viewer/model/kinematics/world/translation/y)").getValue<rl::math::Real>(0);
		world->translation().z() = path.eval("number((/rl/plan|/rlplan)//viewer/model/kinematics/world/translation/z)").getValue<rl::math::Real>(0);
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//viewer/model/kinematics/max) > 0").getValue<bool>())
	{
		rl::xml::NodeSet max = path.eval("(/rl/plan|/rlplan)//viewer/model/kinematics/max/q").getValue<rl::xml::NodeSet>();
		
		rl::math::Vector maximum(max.size());
		
		if (nullptr != this->kin2.get())
		{
			this->kin2->getMaximum(maximum);
		}
		else if (nullptr != this->mdl2.get())
		{
			maximum = this->mdl2->getMaximum();
		}
		
		for (int i = 0; i < max.size(); ++i)
		{
			std::string content = max[i].getContent();
			
			if (!content.empty())
			{
				maximum(i) = std::atof(content.c_str());
				
				if ("deg" == max[i].getProperty("unit"))
				{
					maximum(i) *= rl::math::constants::deg2rad;
				}
			}
		}
		
		if (nullptr != this->kin2.get())
		{
			this->kin2->setMaximum(maximum);
		}
		else if (nullptr != this->mdl2.get())
		{
			this->mdl2->setMaximum(maximum);
		}
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//viewer/model/kinematics/min) > 0").getValue<bool>())
	{
		rl::xml::NodeSet min = path.eval("(/rl/plan|/rlplan)//viewer/model/kinematics/min/q").getValue<rl::xml::NodeSet>();
		
		rl::math::Vector minimum(min.size());
		
		if (nullptr != this->kin2.get())
		{
			this->kin2->getMinimum(minimum);
		}
		else if (nullptr != this->mdl2.get())
		{
			minimum = this->mdl2->getMinimum();
		}
		
		for (int i = 0; i < min.size(); ++i)
		{
			std::string content = min[i].getContent();
			
			if (!content.empty())
			{
				minimum(i) = std::atof(content.c_str());
				
				if ("deg" == min[i].getProperty("unit"))
				{
					minimum(i) *= rl::math::constants::deg2rad;
				}
			}
		}
		
		if (nullptr != this->kin2.get())
		{
			this->kin2->setMinimum(minimum);
		}
		else if (nullptr != this->mdl2.get())
		{
			this->mdl2->setMinimum(minimum);
		}
	}
	
	this->model2 = std::make_shared<rl::plan::Model>();
	
	if (nullptr != this->kin2)
	{
		this->model2->kin = this->kin2.get();
	}
	else if (nullptr != this->mdl2)
	{
		this->model2->mdl = this->mdl2.get();
	}
	
	this->model2->model = this->sceneModel2;
	this->model2->scene = this->scene2.get();
	
	rl::xml::NodeSet start = path.eval("(/rl/plan|/rlplan)//start/q").getValue<rl::xml::NodeSet>();
	this->start = std::make_shared<rl::math::Vector>(start.size());
	
	for (int i = 0; i < start.size(); ++i)
	{
		(*this->start)(i) = std::atof(start[i].getContent().c_str());
		
		if ("deg" == start[i].getProperty("unit"))
		{
			(*this->start)(i) *= rl::math::constants::deg2rad;
		}
	}
	
	*this->q = *this->start;
	
	rl::xml::NodeSet goal = path.eval("(/rl/plan|/rlplan)//goal/q").getValue<rl::xml::NodeSet>();
	this->goal = std::make_shared<rl::math::Vector>(goal.size());
	
	for (int i = 0; i < goal.size(); ++i)
	{
		(*this->goal)(i) = std::atof(goal[i].getContent().c_str());
		
		if ("deg" == goal[i].getProperty("unit"))
		{
			(*this->goal)(i) *= rl::math::constants::deg2rad;
		}
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//sigma) > 0").getValue<bool>())
	{
		rl::xml::NodeSet sigma = path.eval("(/rl/plan|/rlplan)//sigma/q").getValue<rl::xml::NodeSet>();
		this->sigma = std::make_shared<rl::math::Vector>(sigma.size());
		
		for (int i = 0; i < sigma.size(); ++i)
		{
			(*this->sigma)(i) = std::atof(sigma[i].getContent().c_str());
			
			if ("deg" == sigma[i].getProperty("unit"))
			{
				(*this->sigma)(i) *= rl::math::constants::deg2rad;
			}
		}
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//uniformSampler) > 0").getValue<bool>())
	{
		this->sampler = std::make_shared<rl::plan::UniformSampler>();
		rl::plan::UniformSampler* uniformSampler = static_cast<rl::plan::UniformSampler*>(this->sampler.get());
		
		if (path.eval("count((/rl/plan|/rlplan)//uniformSampler/seed) > 0").getValue<bool>())
		{
			uniformSampler->seed(
				path.eval("number((/rl/plan|/rlplan)//uniformSampler/seed)").getValue<std::mt19937::result_type>(std::random_device()())
			);
		}
		else if (this->seed)
		{
			uniformSampler->seed(*this->seed);
		}
	}
	else if (path.eval("count((/rl/plan|/rlplan)//gaussianSampler) > 0").getValue<bool>())
	{
		this->sampler = std::make_shared<rl::plan::GaussianSampler>();
		rl::plan::GaussianSampler* gaussianSampler = static_cast<rl::plan::GaussianSampler*>(this->sampler.get());
		
		if (path.eval("count((/rl/plan|/rlplan)//gaussianSampler/seed) > 0").getValue<bool>())
		{
			gaussianSampler->seed(
				path.eval("number((/rl/plan|/rlplan)//gaussianSampler/seed)").getValue<std::mt19937::result_type>(std::random_device()())
			);
		}
		else if (this->seed)
		{
			gaussianSampler->seed(*this->seed);
		}
		
		gaussianSampler->setSigma(this->sigma.get());
	}
	else if (path.eval("count((/rl/plan|/rlplan)//bridgeSampler) > 0").getValue<bool>())
	{
		this->sampler = std::make_shared<rl::plan::BridgeSampler>();
		rl::plan::BridgeSampler* bridgeSampler = static_cast<rl::plan::BridgeSampler*>(this->sampler.get());
		bridgeSampler->setRatio(path.eval("number((/rl/plan|/rlplan)//bridgeSampler/ratio)").getValue<rl::math::Real>(static_cast<rl::math::Real>(5) / static_cast<rl::math::Real>(6)));
		
		if (path.eval("count((/rl/plan|/rlplan)//bridgeSampler/seed) > 0").getValue<bool>())
		{
			bridgeSampler->seed(
				path.eval("number((/rl/plan|/rlplan)//bridgeSampler/seed)").getValue<std::mt19937::result_type>(std::random_device()())
			);
		}
		else if (this->seed)
		{
			bridgeSampler->seed(*this->seed);
		}
		
		bridgeSampler->setSigma(this->sigma.get());
	}
	
	if (nullptr != this->sampler)
	{
		this->sampler->setModel(this->model.get());
	}
	
	this->sampler2 = std::make_shared<rl::plan::UniformSampler>();
	this->sampler2->setModel(this->model.get());
	
	if (path.eval("count((/rl/plan|/rlplan)//recursiveVerifier) > 0").getValue<bool>())
	{
		this->verifier = std::make_shared<rl::plan::RecursiveVerifier>();
		rl::math::Real delta = path.eval("number((/rl/plan|/rlplan)//recursiveVerifier/delta)").getValue<rl::math::Real>(1);
		
		if ("deg" == path.eval("string((/rl/plan|/rlplan)//recursiveVerifier/delta/@unit)").getValue<std::string>())
		{
			delta *= rl::math::constants::deg2rad;
		}
		
		this->verifier->setDelta(delta);
	}
	else if (path.eval("count((/rl/plan|/rlplan)//sequentialVerifier) > 0").getValue<bool>())
	{
		this->verifier = std::make_shared<rl::plan::SequentialVerifier>();
		rl::math::Real delta = path.eval("number((/rl/plan|/rlplan)//sequentialVerifier/delta)").getValue<rl::math::Real>(1);
		
		if ("deg" == path.eval("string((/rl/plan|/rlplan)//sequentialVerifier/delta/@unit)").getValue<std::string>())
		{
			delta *= rl::math::constants::deg2rad;
		}
		
		this->verifier->setDelta(delta);
	}
	
	if (nullptr != this->verifier)
	{
		this->verifier->setModel(this->model.get());
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//simpleOptimizer/recursiveVerifier) > 0").getValue<bool>())
	{
		this->verifier2 = std::make_shared<rl::plan::RecursiveVerifier>();
		rl::math::Real delta = path.eval("number((/rl/plan|/rlplan)//simpleOptimizer/recursiveVerifier/delta)").getValue<rl::math::Real>(1);
		
		if ("deg" == path.eval("string((/rl/plan|/rlplan)//simpleOptimizer/recursiveVerifier/delta/@unit)").getValue<std::string>())
		{
			delta *= rl::math::constants::deg2rad;
		}
		
		this->verifier2->setDelta(delta);
	}
	else if (path.eval("count((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier) > 0").getValue<bool>())
	{
		this->verifier2 = std::make_shared<rl::plan::RecursiveVerifier>();
		rl::math::Real delta = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier/delta)").getValue<rl::math::Real>(1);
		
		if ("deg" == path.eval("string((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier/delta/@unit)").getValue<std::string>())
		{
			delta *= rl::math::constants::deg2rad;
		}
		
		this->verifier2->setDelta(delta);
	}
	
	if (nullptr != this->verifier2)
	{
		this->verifier2->setModel(this->model.get());
	}
	
	this->optimizer.reset();
	
	if (path.eval("count((/rl/plan|/rlplan)//simpleOptimizer) > 0").getValue<bool>())
	{
		this->optimizer = std::make_shared<rl::plan::SimpleOptimizer>();
	}
	else if (path.eval("count((/rl/plan|/rlplan)//advancedOptimizer) > 0").getValue<bool>())
	{
		this->optimizer = std::make_shared<rl::plan::AdvancedOptimizer>();
		rl::plan::AdvancedOptimizer* advancedOptimizer = static_cast<rl::plan::AdvancedOptimizer*>(this->optimizer.get());
		rl::math::Real length = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/length)").getValue<rl::math::Real>(1);
		
		if ("deg" == path.eval("string((/rl/plan|/rlplan)//advancedOptimizer/length/@unit)").getValue<std::string>())
		{
			length *= rl::math::constants::deg2rad;
		}
		
		advancedOptimizer->setLength(length);
		advancedOptimizer->setRatio(path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/ratio)").getValue<rl::math::Real>(static_cast<rl::math::Real>(0.1)));
	}
	
	if (nullptr != this->optimizer)
	{
		this->optimizer->setModel(this->model.get());
		this->optimizer->setVerifier(this->verifier2.get());
	}
	
	rl::xml::NodeSet planners = path.eval("(/rl/plan|/rlplan)//addRrtConCon|(/rl/plan|/rlplan)//eet|(/rl/plan|/rlplan)//prm|(/rl/plan|/rlplan)//prmUtilityGuided|(/rl/plan|/rlplan)//rrt|(/rl/plan|/rlplan)//rrtCon|(/rl/plan|/rlplan)//rrtConCon|(/rl/plan|/rlplan)//rrtConExt|(/rl/plan|/rlplan)//rrtDual|(/rl/plan|/rlplan)//rrtGoalBias|(/rl/plan|/rlplan)//rrtExtCon|(/rl/plan|/rlplan)//rrtExtExt").getValue<rl::xml::NodeSet>();
	
	for (int i = 0; i < std::min(1, planners.size()); ++i)
	{
		rl::xml::Path path(document, planners[i]);
		
		if ("addRrtConCon" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::AddRrtConCon>();
			rl::plan::AddRrtConCon* addRrtConCon = static_cast<rl::plan::AddRrtConCon*>(this->planner.get());
			addRrtConCon->setAlpha(path.eval("number(alpha)").getValue<rl::math::Real>(static_cast<rl::math::Real>(0.05)));
			rl::math::Real delta = path.eval("number(delta)").getValue<rl::math::Real>(1);
			
			if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
			{
				delta *= rl::math::constants::deg2rad;
			}
			
			addRrtConCon->setDelta(delta);
			rl::math::Real epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1.0e-3));
			
			if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
			{
				epsilon *= rl::math::constants::deg2rad;
			}
			
			addRrtConCon->setEpsilon(epsilon);
			rl::math::Real lower = path.eval("number(lower)").getValue<rl::math::Real>(2);
			
			if ("deg" == path.eval("string(lower/@unit)").getValue<std::string>())
			{
				lower *= rl::math::constants::deg2rad;
			}
			
			addRrtConCon->setLower(lower);
			rl::math::Real radius = path.eval("number(radius)").getValue<rl::math::Real>(20);
			
			if ("deg" == path.eval("string(radius/@unit)").getValue<std::string>())
			{
				radius *= rl::math::constants::deg2rad;
			}
			
			addRrtConCon->setRadius(radius);
			addRrtConCon->setSampler(this->sampler.get());
		}
		else if ("eet" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::Eet>();
			rl::plan::Eet* eet = static_cast<rl::plan::Eet*>(this->planner.get());
			eet->setAlpha(path.eval("number(alpha)").getValue<rl::math::Real>(static_cast<rl::math::Real>(0.01)));
			eet->setAlternativeDistanceComputation(path.eval("count(alternativeDistanceComputation) > 0").getValue<bool>() ? true : false);
			eet->setBeta(path.eval("number(beta)").getValue<rl::math::Real>(0));
			rl::math::Real delta = path.eval("number(delta)").getValue<rl::math::Real>(1);
			
			if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
			{
				delta *= rl::math::constants::deg2rad;
			}
			
			eet->setDelta(delta);
			eet->setDistanceWeight(path.eval("number(distanceWeight)").getValue<rl::math::Real>(static_cast<rl::math::Real>(0.1)));
			rl::math::Real epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1.0e-3));
			
			if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
			{
				epsilon *= rl::math::constants::deg2rad;
			}
			
			eet->setEpsilon(epsilon);
			eet->setGamma(path.eval("number(gamma)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1) / static_cast<rl::math::Real>(3)));
			eet->setGoalEpsilon(path.eval("number(goalEpsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(0.1)));
			
			if (path.eval("translate(string(goalEpsilon/@orientation), 'TRUE', 'true') = 'true' or string(goalEpsilon/@orientation) = '1'").getValue<bool>())
			{
				eet->setGoalEpsilonUseOrientation(true);
			}
			else
			{
				eet->setGoalEpsilonUseOrientation(false);
			}
			
			eet->setMax(
				rl::math::Vector3(
					path.eval("number(max/x)").getValue<rl::math::Real>(0),
					path.eval("number(max/y)").getValue<rl::math::Real>(0),
					path.eval("number(max/z)").getValue<rl::math::Real>(0)
				)
			);
			eet->setMin(
				rl::math::Vector3(
					path.eval("number(min/x)").getValue<rl::math::Real>(0),
					path.eval("number(min/y)").getValue<rl::math::Real>(0),
					path.eval("number(min/z)").getValue<rl::math::Real>(0)
				)
			);
			eet->setSampler(this->sampler.get());
			
			if (path.eval("count(seed) > 0").getValue<bool>())
			{
				eet->seed(
					path.eval("number(seed)").getValue<std::mt19937::result_type>(std::random_device()())
				);
			}
			else if (this->seed)
			{
				eet->seed(*this->seed);
			}
			
			rl::xml::NodeSet explorers = path.eval("explorer").getValue<rl::xml::NodeSet>();
			
			for (int i = 0; i < explorers.size(); ++i)
			{
				rl::xml::Path path(document, explorers[i]);
				
				std::shared_ptr<rl::plan::WorkspaceSphereExplorer> explorer = std::make_shared<rl::plan::WorkspaceSphereExplorer>();
				this->explorers.push_back(explorer);
				eet->addExplorer(explorer.get());
				
				rl::plan::Eet::ExplorerSetup explorerSetup;
				
				std::shared_ptr<rl::math::Vector3> explorerStart = std::make_shared<rl::math::Vector3>();
				this->explorerStarts.push_back(explorerStart);
				explorer->setStart(explorerStart.get());
				
				(*explorerStart).x() = path.eval("number(start/x)").getValue<rl::math::Real>(0);
				(*explorerStart).y() = path.eval("number(start/y)").getValue<rl::math::Real>(0);
				(*explorerStart).z() = path.eval("number(start/z)").getValue<rl::math::Real>(0);
				
				if (path.eval("count(start/goal) > 0").getValue<bool>())
				{
					explorerSetup.startConfiguration = this->goal.get();
				}
				else if (path.eval("count(start/start) > 0").getValue<bool>())
				{
					explorerSetup.startConfiguration = this->start.get();
				}
				else
				{
					explorerSetup.startConfiguration = nullptr;
				}
				
				if (path.eval("count(start//frame) > 0").getValue<bool>())
				{
					explorerSetup.startFrame = path.eval("number(start//frame)").getValue<std::size_t>();
				}
				else if (path.eval("count(start//tcp) > 0").getValue<bool>())
				{
					explorerSetup.startFrame = -1;
				}
				
				std::shared_ptr<rl::math::Vector3> explorerGoal = std::make_shared<rl::math::Vector3>();
				this->explorerGoals.push_back(explorerGoal);
				explorer->setGoal(explorerGoal.get());
				
				(*explorerGoal).x() = path.eval("number(goal/x)").getValue<rl::math::Real>(0);
				(*explorerGoal).y() = path.eval("number(goal/y)").getValue<rl::math::Real>(0);
				(*explorerGoal).z() = path.eval("number(goal/z)").getValue<rl::math::Real>(0);
				
				if (path.eval("count(goal/goal) > 0").getValue<bool>())
				{
					explorerSetup.goalConfiguration = this->goal.get();
				}
				else if (path.eval("count(goal/start) > 0").getValue<bool>())
				{
					explorerSetup.goalConfiguration = this->start.get();
				}
				else
				{
					explorerSetup.goalConfiguration = nullptr;
				}
				
				if (path.eval("count(goal//frame) > 0").getValue<bool>())
				{
					explorerSetup.goalFrame = path.eval("number(goal//frame)").getValue<std::size_t>();
				}
				else if (path.eval("count(goal//tcp) > 0").getValue<bool>())
				{
					explorerSetup.goalFrame = -1;
				}
				
				explorer->setBoundingBox(
					rl::math::AlignedBox3(
						rl::math::Vector3(
							path.eval("number(boundingBox/min/x)").getValue<rl::math::Real>(-std::numeric_limits<rl::math::Real>::max()),
							path.eval("number(boundingBox/min/y)").getValue<rl::math::Real>(-std::numeric_limits<rl::math::Real>::max()),
							path.eval("number(boundingBox/min/z)").getValue<rl::math::Real>(-std::numeric_limits<rl::math::Real>::max())
						),
						rl::math::Vector3(
							path.eval("number(boundingBox/max/x)").getValue<rl::math::Real>(std::numeric_limits<rl::math::Real>::max()),
							path.eval("number(boundingBox/max/y)").getValue<rl::math::Real>(std::numeric_limits<rl::math::Real>::max()),
							path.eval("number(boundingBox/max/z)").getValue<rl::math::Real>(std::numeric_limits<rl::math::Real>::max())
						)
					)
				);
				
				if (path.eval("count(distance) > 0").getValue<bool>())
				{
					explorer->setGreedy(rl::plan::WorkspaceSphereExplorer::Greedy::distance);
				}
				else if (path.eval("count(sourceDistance) > 0").getValue<bool>())
				{
					explorer->setGreedy(rl::plan::WorkspaceSphereExplorer::Greedy::sourceDistance);
				}
				else if (path.eval("count(space) > 0").getValue<bool>())
				{
					explorer->setGreedy(rl::plan::WorkspaceSphereExplorer::Greedy::space);
				}
				
				if (rl::plan::DistanceModel* model = dynamic_cast<rl::plan::DistanceModel*>(this->model.get()))
				{
					explorer->setModel(model);
				}
				else
				{
					throw std::runtime_error("selected engine does not support distance queries");
				}
				
				explorer->setRadius(path.eval("number(radius)").getValue<rl::math::Real>(0));
				explorer->setRange(path.eval("number(range)").getValue<rl::math::Real>(std::numeric_limits<rl::math::Real>::max()));
				explorer->setSamples(path.eval("number(samples)").getValue<std::size_t>(10));
				
				if (path.eval("count(seed) > 0").getValue<bool>())
				{
					explorer->seed(
						path.eval("number(seed)").getValue<std::mt19937::result_type>(std::random_device()())
					);
				}
				else if (this->seed)
				{
					explorer->seed(*this->seed);
				}
				
				eet->addExplorerSetup(explorerSetup);
			}
		}
		else if ("prm" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::Prm>();
			rl::plan::Prm* prm = static_cast<rl::plan::Prm*>(this->planner.get());
			prm->setMaxDegree(path.eval("number(degree)").getValue<std::size_t>(std::numeric_limits<std::size_t>::max()));
			
			if (path.eval("count(dijkstra) > 0").getValue<bool>())
			{
				prm->setSearch(rl::plan::Prm::Search::dijkstra);
			}
			
			prm->setMaxNeighbors(path.eval("number(k)").getValue<std::size_t>(30));
			rl::math::Real radius = path.eval("number(radius)").getValue<rl::math::Real>(std::numeric_limits<rl::math::Real>::max());
			
			if ("deg" == path.eval("string(radius/@unit)").getValue<std::string>())
			{
				radius *= rl::math::constants::deg2rad;
			}
			
			prm->setMaxRadius(radius);
			prm->setSampler(this->sampler.get());
			prm->setVerifier(this->verifier.get());
		}
		else if ("prmUtilityGuided" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::PrmUtilityGuided>();
			rl::plan::PrmUtilityGuided* prmUtilityGuided = static_cast<rl::plan::PrmUtilityGuided*>(this->planner.get());
			prmUtilityGuided->setMaxDegree(path.eval("number(degree)").getValue<std::size_t>(std::numeric_limits<std::size_t>::max()));
			
			if (path.eval("count(dijkstra) > 0").getValue<bool>())
			{
				prmUtilityGuided->setSearch(rl::plan::Prm::Search::dijkstra);
			}
			
			prmUtilityGuided->setMaxNeighbors(path.eval("number(k)").getValue<std::size_t>(30));
			rl::math::Real radius = path.eval("number(radius)").getValue<rl::math::Real>(std::numeric_limits<rl::math::Real>::max());
			
			if ("deg" == path.eval("string(radius/@unit)").getValue<std::string>())
			{
				radius *= rl::math::constants::deg2rad;
			}
			
			prmUtilityGuided->setMaxRadius(radius);
			
			if (path.eval("count(seed) > 0").getValue<bool>())
			{
				prmUtilityGuided->seed(
					path.eval("number(seed)").getValue<std::mt19937::result_type>(std::random_device()())
				);
			}
			else if (this->seed)
			{
				prmUtilityGuided->seed(*this->seed);
			}
			
			prmUtilityGuided->setSampler(this->sampler.get());
			prmUtilityGuided->setVerifier(this->verifier.get());
		}
		else if ("rrt" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::Rrt>();
			rl::plan::Rrt* rrt = static_cast<rl::plan::Rrt*>(this->planner.get());
			rl::math::Real delta = path.eval("number(delta)").getValue<rl::math::Real>(1);
			
			if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
			{
				delta *= rl::math::constants::deg2rad;
			}
			
			rrt->setDelta(delta);
			rl::math::Real epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1.0e-3));
			
			if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
			{
				epsilon *= rl::math::constants::deg2rad;
			}
			
			rrt->setEpsilon(epsilon);
			rrt->setSampler(this->sampler.get());
		}
		else if ("rrtCon" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::RrtCon>();
			rl::plan::RrtCon* rrtCon = static_cast<rl::plan::RrtCon*>(this->planner.get());
			rl::math::Real delta = path.eval("number(delta)").getValue<rl::math::Real>(1);
			
			if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
			{
				delta *= rl::math::constants::deg2rad;
			}
			
			rrtCon->setDelta(delta);
			rl::math::Real epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1.0e-3));
			
			if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
			{
				epsilon *= rl::math::constants::deg2rad;
			}
			
			rrtCon->setEpsilon(epsilon);
			rrtCon->setProbability(path.eval("number(probability)").getValue<rl::math::Real>(static_cast<rl::math::Real>(0.05)));
			rrtCon->setSampler(this->sampler.get());
			
			if (path.eval("count(seed) > 0").getValue<bool>())
			{
				rrtCon->seed(
					path.eval("number(seed)").getValue<std::mt19937::result_type>(std::random_device()())
				);
			}
			else if (this->seed)
			{
				rrtCon->seed(*this->seed);
			}
		}
		else if ("rrtConCon" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::RrtConCon>();
			rl::plan::RrtConCon* rrtConCon = static_cast<rl::plan::RrtConCon*>(this->planner.get());
			rl::math::Real delta = path.eval("number(delta)").getValue<rl::math::Real>(1);
			
			if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
			{
				delta *= rl::math::constants::deg2rad;
			}
			
			rrtConCon->setDelta(delta);
			rl::math::Real epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1.0e-3));
			
			if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
			{
				epsilon *= rl::math::constants::deg2rad;
			}
			
			rrtConCon->setEpsilon(epsilon);
			rrtConCon->setSampler(this->sampler.get());
		}
		else if ("rrtDual" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::RrtDual>();
			rl::plan::RrtDual* rrtDual = static_cast<rl::plan::RrtDual*>(this->planner.get());
			rl::math::Real delta = path.eval("number(delta)").getValue<rl::math::Real>(1);
			
			if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
			{
				delta *= rl::math::constants::deg2rad;
			}
			
			rrtDual->setDelta(delta);
			rl::math::Real epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1.0e-3));
			
			if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
			{
				epsilon *= rl::math::constants::deg2rad;
			}
			
			rrtDual->setEpsilon(epsilon);
			rrtDual->setSampler(this->sampler.get());
		}
		else if ("rrtExtCon" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::RrtExtCon>();
			rl::plan::RrtExtCon* rrtExtCon = static_cast<rl::plan::RrtExtCon*>(this->planner.get());
			rl::math::Real delta = path.eval("number(delta)").getValue<rl::math::Real>(1);
			
			if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
			{
				delta *= rl::math::constants::deg2rad;
			}
			
			rrtExtCon->setDelta(delta);
			rl::math::Real epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1.0e-3));
			
			if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
			{
				epsilon *= rl::math::constants::deg2rad;
			}
			
			rrtExtCon->setEpsilon(epsilon);
			rrtExtCon->setSampler(this->sampler.get());
		}
		else if ("rrtExtExt" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::RrtExtExt>();
			rl::plan::RrtExtExt* rrtExtExt = static_cast<rl::plan::RrtExtExt*>(this->planner.get());
			rl::math::Real delta = path.eval("number(delta)").getValue<rl::math::Real>(1);
			
			if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
			{
				delta *= rl::math::constants::deg2rad;
			}
			
			rrtExtExt->setDelta(delta);
			rl::math::Real epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1.0e-3));
			
			if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
			{
				epsilon *= rl::math::constants::deg2rad;
			}
			
			rrtExtExt->setEpsilon(epsilon);
			rrtExtExt->setSampler(this->sampler.get());
		}
		else if ("rrtGoalBias" == planners[i].getName())
		{
			this->planner = std::make_shared<rl::plan::RrtGoalBias>();
			rl::plan::RrtGoalBias* rrtGoalBias = static_cast<rl::plan::RrtGoalBias*>(this->planner.get());
			rl::math::Real delta = path.eval("number(delta)").getValue<rl::math::Real>(1);
			
			if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>())
			{
				delta *= rl::math::constants::deg2rad;
			}
			
			rrtGoalBias->setDelta(delta);
			rl::math::Real epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(static_cast<rl::math::Real>(1.0e-3));
			
			if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>())
			{
				epsilon *= rl::math::constants::deg2rad;
			}
			
			rrtGoalBias->setEpsilon(epsilon);
			rrtGoalBias->setProbability(path.eval("number(probability)").getValue<rl::math::Real>(static_cast<rl::math::Real>(0.05)));
			rrtGoalBias->setSampler(this->sampler.get());
			
			if (path.eval("count(seed) > 0").getValue<bool>())
			{
				rrtGoalBias->seed(
					path.eval("number(seed)").getValue<std::mt19937::result_type>(std::random_device()())
				);
			}
			else if (this->seed)
			{
				rrtGoalBias->seed(*this->seed);
			}
		}
	}
	
	std::size_t nearestNeighborsSize = 1;
	
	if (rl::plan::RrtDual* rrtDual = dynamic_cast<rl::plan::RrtDual*>(this->planner.get()))
	{
		nearestNeighborsSize = 2;
	}
	
	for (::std::size_t i = 0; i < nearestNeighborsSize; ++i)
	{
		std::shared_ptr<rl::plan::NearestNeighbors> nearestNeighbors;
		
		if (path.eval("count((/rl/plan|/rlplan)//gnatNearestNeighbors) > 0").getValue<bool>())
		{
			std::shared_ptr<rl::plan::GnatNearestNeighbors> gnatNearestNeighbors = std::make_shared<rl::plan::GnatNearestNeighbors>(this->model.get());
			
			if (path.eval("count((/rl/plan|/rlplan)//gnatNearestNeighbors/checks) > 0").getValue<bool>())
			{
				gnatNearestNeighbors->setChecks(
					path.eval("number((/rl/plan|/rlplan)//gnatNearestNeighbors/checks)").getValue<std::size_t>(0)
				);
			}
			
			if (path.eval("count((/rl/plan|/rlplan)//gnatNearestNeighbors/node/data/@max) > 0").getValue<bool>())
			{
				gnatNearestNeighbors->setNodeDataMax(
					path.eval("number((/rl/plan|/rlplan)//gnatNearestNeighbors/node/data/@max)").getValue<std::size_t>(50)
				);
			}
			
			if (path.eval("count((/rl/plan|/rlplan)//gnatNearestNeighbors/node/degree) > 0").getValue<bool>())
			{
				gnatNearestNeighbors->setNodeDegree(
					path.eval("number((/rl/plan|/rlplan)//gnatNearestNeighbors/node/degree)").getValue<std::size_t>(8)
				);
			}
			
			if (path.eval("count((/rl/plan|/rlplan)//gnatNearestNeighbors/node/degree/@max) > 0").getValue<bool>())
			{
				gnatNearestNeighbors->setNodeDegreeMax(
					path.eval("number((/rl/plan|/rlplan)//gnatNearestNeighbors/node/degree/@max)").getValue<std::size_t>(12)
				);
			}
			
			if (path.eval("count((/rl/plan|/rlplan)//gnatNearestNeighbors/node/degree/@min) > 0").getValue<bool>())
			{
				gnatNearestNeighbors->setNodeDegreeMin(
					path.eval("number((/rl/plan|/rlplan)//gnatNearestNeighbors/node/degree/@min)").getValue<std::size_t>(4)
				);
			}
			
			if (path.eval("count((/rl/plan|/rlplan)//gnatNearestNeighbors/seed) > 0").getValue<bool>())
			{
				gnatNearestNeighbors->seed(
					path.eval("number((/rl/plan|/rlplan)//gnatNearestNeighbors/seed)").getValue<std::mt19937::result_type>(std::random_device()())
				);
			}
			else if (this->seed)
			{
				gnatNearestNeighbors->seed(*this->seed);
			}
			
			nearestNeighbors = gnatNearestNeighbors;
		}
		else if (path.eval("count((/rl/plan|/rlplan)//kdtreeBoundingBoxNearestNeighbors) > 0").getValue<bool>())
		{
			std::shared_ptr<rl::plan::KdtreeBoundingBoxNearestNeighbors> kdtreeBoundingBoxNearestNeighbors = std::make_shared<rl::plan::KdtreeBoundingBoxNearestNeighbors>(this->model.get());
			
			if (path.eval("count((/rl/plan|/rlplan)//kdtreeBoundingBoxNearestNeighbors/checks) > 0").getValue<bool>())
			{
				kdtreeBoundingBoxNearestNeighbors->setChecks(
					path.eval("number((/rl/plan|/rlplan)//kdtreeBoundingBoxNearestNeighbors/checks)").getValue<std::size_t>(0)
				);
			}
			
			if (path.eval("count((/rl/plan|/rlplan)//kdtreeBoundingBoxNearestNeighbors/node/data/@max) > 0").getValue<bool>())
			{
				kdtreeBoundingBoxNearestNeighbors->setNodeDataMax(
					path.eval("number((/rl/plan|/rlplan)//kdtreeBoundingBoxNearestNeighbors/node/data/@max)").getValue<std::size_t>(10)
				);
			}
			
			nearestNeighbors = kdtreeBoundingBoxNearestNeighbors;
		}
		else if (path.eval("count((/rl/plan|/rlplan)//kdtreeNearestNeighbors) > 0").getValue<bool>())
		{
			std::shared_ptr<rl::plan::KdtreeNearestNeighbors> kdtreeNearestNeighbors = std::make_shared<rl::plan::KdtreeNearestNeighbors>(this->model.get());
			
			if (path.eval("count((/rl/plan|/rlplan)//kdtreeNearestNeighbors/checks) > 0").getValue<bool>())
			{
				kdtreeNearestNeighbors->setChecks(
					path.eval("number((/rl/plan|/rlplan)//kdtreeNearestNeighbors/checks)").getValue<std::size_t>(0)
				);
			}
			
			if (path.eval("count((/rl/plan|/rlplan)//kdtreeNearestNeighbors/samples) > 0").getValue<bool>())
			{
				kdtreeNearestNeighbors->setSamples(
					path.eval("number((/rl/plan|/rlplan)//kdtreeNearestNeighbors/samples)").getValue<std::size_t>(100)
				);
			}
			
			nearestNeighbors = kdtreeNearestNeighbors;
		}
		else
		{
			std::shared_ptr<rl::plan::LinearNearestNeighbors> linearNearestNeighbors = std::make_shared<rl::plan::LinearNearestNeighbors>(this->model.get());
			nearestNeighbors = linearNearestNeighbors;
		}
		
		this->nearestNeighbors.push_back(nearestNeighbors);
		
		if (rl::plan::Prm* prm = dynamic_cast<rl::plan::Prm*>(this->planner.get()))
		{
			prm->setNearestNeighbors(nearestNeighbors.get());
		}
		else if (rl::plan::Rrt* rrt = dynamic_cast<rl::plan::Rrt*>(this->planner.get()))
		{
			rrt->setNearestNeighbors(nearestNeighbors.get(), i);
		}
	}
	
	this->planner->setDuration(
		std::chrono::duration_cast<std::chrono::steady_clock::duration>(
			std::chrono::duration<float>(
				path.eval("number((/rl/plan|/rlplan)//duration)").getValue<rl::math::Real>(std::numeric_limits<float>::max())
			)
		)
	);
	
	this->planner->setGoal(this->goal.get());
	this->planner->setModel(this->model.get());
	this->planner->setStart(this->start.get());
	
	this->viewer->delta = path.eval("number((/rl/plan|/rlplan)//viewer/delta)").getValue<rl::math::Real>();
	
	if ("deg" == path.eval("string((/rl/plan|/rlplan)//viewer/delta/@unit)").getValue<std::string>())
	{
		this->viewer->delta *= rl::math::constants::deg2rad;
	}
	
	this->viewer->deltaSwept = path.eval("number((/rl/plan|/rlplan)//viewer/swept)").getValue<rl::math::Real>(this->viewer->delta * 100);
	
	if ("deg" == path.eval("string((/rl/plan|/rlplan)//viewer/swept/@unit)").getValue<std::string>())
	{
		this->viewer->deltaSwept *= rl::math::constants::deg2rad;
	}
	
	this->viewer->sceneGroup->addChild(this->scene2->root);
	this->viewer->model = this->model2.get();
	
	this->configurationSpaceScene->model = this->model.get();
	
	if (this->toggleViewActiveAction->isChecked())
	{
		this->toggleViewActive(true);
	}
	else
	{
		this->toggleViewActive(false);
	}
	
	if (this->toggleConfigurationSpaceActiveAction->isChecked())
	{
		this->toggleConfigurationSpaceActive(true);
	}
	else
	{
		this->toggleConfigurationSpaceActive(false);
	}
	
	if (path.eval("count((/rl/plan|/rlplan)//viewer/camera/orthographic) > 0").getValue<bool>())
	{
		this->viewer->viewer->setCameraType(SoOrthographicCamera::getClassTypeId());
	}
	else
	{
		this->viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
	}
	
	this->viewer->viewer->getCamera()->setToDefaults();
	
	this->viewer->viewer->viewAll();
	
	this->viewer->viewer->getCamera()->position.setValue(
		path.eval("number((/rl/plan|/rlplan)//viewer/camera/position/x)").getValue<rl::math::Real>(this->viewer->viewer->getCamera()->position.getValue()[0]),
		path.eval("number((/rl/plan|/rlplan)//viewer/camera/position/y)").getValue<rl::math::Real>(this->viewer->viewer->getCamera()->position.getValue()[1]),
		path.eval("number((/rl/plan|/rlplan)//viewer/camera/position/z)").getValue<rl::math::Real>(this->viewer->viewer->getCamera()->position.getValue()[2])
	);
	
	if (path.eval("count((/rl/plan|/rlplan)//viewer/camera/target) > 0").getValue<bool>())
	{
		this->viewer->viewer->getCamera()->pointAt(
			SbVec3f(
				path.eval("number((/rl/plan|/rlplan)//viewer/camera/target/x)").getValue<rl::math::Real>(0),
				path.eval("number((/rl/plan|/rlplan)//viewer/camera/target/y)").getValue<rl::math::Real>(0),
				path.eval("number((/rl/plan|/rlplan)//viewer/camera/target/z)").getValue<rl::math::Real>(0)
			),
			SbVec3f(
				path.eval("number((/rl/plan|/rlplan)//viewer/camera/up/x)").getValue<rl::math::Real>(0),
				path.eval("number((/rl/plan|/rlplan)//viewer/camera/up/y)").getValue<rl::math::Real>(0),
				path.eval("number((/rl/plan|/rlplan)//viewer/camera/up/z)").getValue<rl::math::Real>(1)
			)
		);
	}
	
	this->viewer->viewer->getCamera()->scaleHeight(
		path.eval("number((/rl/plan|/rlplan)//viewer/camera/scale)").getValue<rl::math::Real>(1)
	);
	
	this->viewer->drawConfiguration(*this->start);
	
	this->configurationModel->invalidate();
	this->configurationSpaceModel->invalidate();
	this->plannerModel->invalidate();
	
	this->configurationSpaceScene->init();
	this->configurationSpaceSceneView->adjust();
	
	this->plannerDockWidget->setWindowTitle(QString::fromStdString(this->planner->getName()));
	
	this->statusBar()->showMessage("Finished loading '" + filename + "'.", 1000);
	
	if (!this->wait)
	{
		this->startThread();
	}
}

void
MainWindow::open()
{
	if (nullptr != this->planner)
	{
		this->reset();
	}
	
	QString filename = QFileDialog::getOpenFileName(this, "", this->filename, "All Formats (*.xml)");
	
	if (!filename.isEmpty())
	{
		this->load(filename);
	}
}

void
MainWindow::parseCommandLine()
{
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
	
	int width = 1024;
	int height = 768;
	
	bool doSetMinimumSize = false;
	
#if QT_VERSION >= 0x050200
	QCommandLineOption backgroundOption(QStringList("background"), "Sets background color of 3D viewer.", "color");
	QCommandLineOption benchmarkOption(QStringList("benchmark"), "Save benchmark statistics in file.", "filename");
	QCommandLineOption disableViewerOption(QStringList("disable-viewer"), "Disables viewer during planning.");
	QCommandLineOption disableWaitOption(QStringList("disable-wait"), "Disables wait before planning.");
	QCommandLineOption enableQuitOption(QStringList("enable-quit"), "Exits after planning.");
	QCommandLineOption engineOption(QStringList("engine"), "Sets collision engine.", engines.join("|"));
	QCommandLineOption heightOption(QStringList("height"), "Sets minimum height of 3D viewer.", "height");
	QCommandLineOption seedOption(QStringList("seed"), "Sets seed value.", "seed");
	QCommandLineOption widthOption(QStringList("width"), "Sets minimum width of 3D viewer.", "width");
	
	QCommandLineParser parser;
	parser.addOption(backgroundOption);
	parser.addOption(benchmarkOption);
	parser.addOption(disableViewerOption);
	parser.addOption(disableWaitOption);
	parser.addOption(enableQuitOption);
	parser.addOption(engineOption);
	parser.addOption(heightOption);
	QCommandLineOption helpOption = parser.addHelpOption();
	parser.addOption(seedOption);
	parser.addOption(widthOption);
	parser.addPositionalArgument("filename", "", "[filename]");
	
	parser.process(QCoreApplication::arguments());
	
	if (parser.isSet(backgroundOption))
	{
		QString background = parser.value(backgroundOption);
		
		if (!QColor::isValidColor(background))
		{
			parser.showHelp();
		}
		
		this->viewer->setBackgroundColor(QColor(background));
	}
	
	if (parser.isSet(benchmarkOption))
	{
		this->thread->benchmark = parser.value(benchmarkOption).toStdString();
	}
	
	if (parser.isSet(disableViewerOption))
	{
		QObject::disconnect(this->toggleViewActiveAction, SIGNAL(toggled(bool)), this, SLOT(toggleViewActive(bool)));
		this->toggleViewActiveAction->setChecked(false);
		QObject::connect(this->toggleViewActiveAction, SIGNAL(toggled(bool)), this, SLOT(toggleViewActive(bool)));
	}
	
	if (parser.isSet(disableWaitOption))
	{
		this->wait = false;
	}
	
	if (parser.isSet(enableQuitOption))
	{
		this->thread->quit = true;
	}
	
	if (parser.isSet(engineOption))
	{
		QString engine = parser.value(engineOption);
		
		if (!engines.contains(engine, Qt::CaseInsensitive))
		{
			parser.showHelp();
		}
		
		this->engine = engine;
	}
	
	if (parser.isSet(heightOption))
	{
		bool ok;
		height = parser.value(heightOption).toInt(&ok);
		
		if (!ok)
		{
			parser.showHelp();
		}
		
		doSetMinimumSize = true;
	}
	
	if (parser.isSet(seedOption))
	{
		bool ok;
		this->seed = parser.value(seedOption).toUInt(&ok);
		
		if (!ok)
		{
			parser.showHelp();
		}
	}
	
	if (parser.isSet(widthOption))
	{
		bool ok;
		width = parser.value(widthOption).toInt(&ok);
		
		if (!ok)
		{
			parser.showHelp();
		}
		
		doSetMinimumSize = true;
	}
	
	if (parser.positionalArguments().size() > 1)
	{
		parser.showHelp();
	}
	
	if (!parser.positionalArguments().empty())
	{
		this->filename = parser.positionalArguments()[0];
	}
#else
	QRegExp backgroundRegExp("--background=(\\S*)");
	QRegExp benchmarkRegExp("--benchmark=([\\s\\S]*)");
	QRegExp engineRegExp("--engine=(" + engines.join("|") + ")");
	QRegExp helpRegExp("--help");
	QRegExp heightRegExp("--height=(\\d*)");
	QRegExp quitRegExp("--enable-quit");
	QRegExp seedRegExp("--seed=(\\d*)");
	QRegExp viewerRegExp("--disable-viewer");
	QRegExp waitRegExp("--disable-wait");
	QRegExp widthRegExp("--width=(\\d*)");
	
	for (int i = 1; i < QApplication::arguments().size(); ++i)
	{
		if (-1 != backgroundRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->viewer->setBackgroundColor(backgroundRegExp.cap(1));
		}
		else if (-1 != benchmarkRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->thread->benchmark = benchmarkRegExp.cap(1).toStdString();
		}
		else if (-1 != engineRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->engine = engineRegExp.cap(1);
		}
		else if (-1 != helpRegExp.indexIn(QApplication::arguments()[i]))
		{
			QMessageBox::information(this, "Usage", "rlPlanDemo [--background=<color>] [--benchmark=<filename>] [--disable-viewer] [--disable-wait] [--enable-quit] [--engine=<" + engines.join("|") + ">] [--height=<height>] [--help] [--seed=<seed>] [--width=<width>] [filename]");
			exit(EXIT_SUCCESS);
		}
		else if (-1 != heightRegExp.indexIn(QApplication::arguments()[i]))
		{
			height = heightRegExp.cap(1).toInt();
			doSetMinimumSize = true;
		}
		else if (-1 != quitRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->thread->quit = true;
		}
		else if (-1 != seedRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->seed = seedRegExp.cap(1).toUInt();
		}
		else if (-1 != viewerRegExp.indexIn(QApplication::arguments()[i]))
		{
			QObject::disconnect(this->toggleViewActiveAction, SIGNAL(toggled(bool)), this, SLOT(toggleViewActive(bool)));
			this->toggleViewActiveAction->setChecked(false);
			QObject::connect(this->toggleViewActiveAction, SIGNAL(toggled(bool)), this, SLOT(toggleViewActive(bool)));
		}
		else if (-1 != waitRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->wait = false;
		}
		else if (-1 != widthRegExp.indexIn(QApplication::arguments()[i]))
		{
			width = widthRegExp.cap(1).toInt();
			doSetMinimumSize = true;
		}
		else
		{
			this->filename = QApplication::arguments()[i];
		}
	}
#endif
	
	this->resize(width, height);
	
	if (doSetMinimumSize)
	{
		this->viewer->setMinimumSize(width, height);
	}
}

void
MainWindow::reset()
{
	std::chrono::steady_clock::duration duration = this->planner->getDuration();
	
	this->thread->blockSignals(true);
	QCoreApplication::processEvents();
	this->planner->setDuration(std::chrono::steady_clock::duration::zero());
	this->thread->stop();
	this->planner->setDuration(duration);
	this->thread->blockSignals(false);
	
	this->planner->reset();
	this->model->reset();
	this->viewer->reset();
	this->configurationSpaceScene->reset();
	
	this->statusBar()->showMessage("Successfully reset.", 1000);
	
	this->configurationView->setEnabled(true);
	this->configurationSpaceSceneView->setEnabled(true);
	this->configurationSpaceView->setEnabled(true);
	this->evalAction->setEnabled(true);
	this->getGoalConfigurationAction->setEnabled(true);
	this->getRandomConfigurationAction->setEnabled(true);
	this->getRandomFreeConfigurationAction->setEnabled(true);
	this->getStartConfigurationAction->setEnabled(true);
	this->openAction->setEnabled(true);
	this->plannerView->setEnabled(true);
	this->setGoalConfigurationAction->setEnabled(true);
	this->setStartConfigurationAction->setEnabled(true);
	this->startThreadAction->setEnabled(true);
	this->toggleAnimationAction->setEnabled(true);
	this->toggleConfigurationSpaceActiveAction->setEnabled(true);
	this->toggleSweptVolumeAction->setEnabled(true);
	this->toggleViewActiveAction->setEnabled(true);
}

void
MainWindow::saveImageWithAlpha()
{
	this->viewer->saveImage(true);
}

void
MainWindow::saveImageWithoutAlpha()
{
	this->viewer->saveImage(false);
}

void
MainWindow::savePdf()
{
	QString filename = "rlPlanDemo-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".pdf";
	
	QPrinter printer;
	printer.setOutputFileName(filename);
	printer.setOutputFormat(QPrinter::PdfFormat);
#if QT_VERSION >= 0x050E00
	printer.setPageMargins(QMarginsF(0, 0, 0, 0), QPageLayout::Millimeter);
	printer.setPageSize(QPageSize(this->configurationSpaceScene->sceneRect().size(), QPageSize::Millimeter));
#else
	printer.setPageMargins(0, 0, 0, 0, QPrinter::Millimeter);
	printer.setPaperSize(this->configurationSpaceScene->sceneRect().size(), QPrinter::Millimeter);
#endif
	
	QPainter painter(&printer);
	
	this->configurationSpaceScene->render(&painter);
	
	this->statusBar()->showMessage("Successfully saved PDF in '" + filename + "'.", 1000);
}

void
MainWindow::saveScene()
{
	this->viewer->saveScene();
}

void
MainWindow::setGoalConfiguration()
{
	*this->goal = *this->q;
	this->statusBar()->showMessage("Successfully set goal configuration.", 1000);
}

void
MainWindow::setStartConfiguration()
{
	*this->start = *this->q;
	this->statusBar()->showMessage("Successfully set start configuration.", 1000);
}

void
MainWindow::startThread()
{
	this->configurationView->setEnabled(false);
	this->configurationSpaceSceneView->setEnabled(false);
	this->configurationSpaceView->setEnabled(false);
	this->evalAction->setEnabled(false);
	this->getGoalConfigurationAction->setEnabled(false);
	this->getRandomConfigurationAction->setEnabled(false);
	this->getRandomFreeConfigurationAction->setEnabled(false);
	this->getStartConfigurationAction->setEnabled(false);
	this->openAction->setEnabled(false);
	this->plannerView->setEnabled(false);
	this->setGoalConfigurationAction->setEnabled(false);
	this->setStartConfigurationAction->setEnabled(false);
	this->startThreadAction->setEnabled(false);
	this->toggleAnimationAction->setEnabled(false);
	this->toggleConfigurationSpaceActiveAction->setEnabled(false);
	this->toggleSweptVolumeAction->setEnabled(false);
	this->toggleViewActiveAction->setEnabled(false);
	
	this->model->reset();
	this->thread->start();
}

void
MainWindow::toggleAnimation(const bool& doOn)
{
	if (doOn)
	{
		this->thread->animate = true;
	}
	else
	{
		this->thread->animate = false;
	}
}

void
MainWindow::toggleCamera()
{
	if (SoPerspectiveCamera::getClassTypeId() == this->viewer->viewer->getCameraType())
	{
		this->viewer->viewer->setCameraType(SoOrthographicCamera::getClassTypeId());
	}
	else
	{
		this->viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
	}
	
	SbVec3f position = this->viewer->viewer->getCamera()->position.getValue();
	SbRotation orientation = this->viewer->viewer->getCamera()->orientation.getValue();
	this->viewer->viewer->getCamera()->setToDefaults();
	this->viewer->viewer->getCamera()->position.setValue(position);
	this->viewer->viewer->getCamera()->orientation.setValue(orientation);
	this->viewer->viewer->viewAll();
}

void
MainWindow::toggleConfiguration()
{
	if (this->configurationDockWidget->isVisible())
	{
		this->configurationDockWidget->hide();
	}
	else
	{
		this->configurationDockWidget->show();
	}
}

void
MainWindow::toggleConfigurationSpace()
{
	if (this->configurationSpaceDockWidget->isVisible())
	{
		this->configurationSpaceDockWidget->hide();
	}
	else
	{
		this->configurationSpaceDockWidget->show();
	}
}

void
MainWindow::toggleConfigurationSpaceActive(const bool& doOn)
{
	if (doOn)
	{
		this->connect(this->thread, this->configurationSpaceScene);
	}
	else
	{
		this->disconnect(this->thread, this->configurationSpaceScene);
	}
}

void
MainWindow::toggleConfigurationSpaceScene()
{
	if (this->configurationSpaceSceneDockWidget->isVisible())
	{
		this->configurationSpaceSceneDockWidget->hide();
	}
	else
	{
		this->configurationSpaceSceneDockWidget->show();
	}
}

void
MainWindow::togglePlanner()
{
	if (this->plannerDockWidget->isVisible())
	{
		this->plannerDockWidget->hide();
	}
	else
	{
		this->plannerDockWidget->show();
	}
}

void
MainWindow::toggleSweptVolume(const bool& doOn)
{
	if (doOn)
	{
		this->thread->swept = true;
	}
	else
	{
		this->thread->swept = false;
	}
}

void
MainWindow::toggleViewActive(const bool& doOn)
{
	if (doOn)
	{
		this->planner->setViewer(this->thread);
		
		if (nullptr != this->optimizer)
		{
			this->optimizer->setViewer(this->thread);
		}
		
		for (std::vector<std::shared_ptr<rl::plan::WorkspaceSphereExplorer>>::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
		{
			(*i)->setViewer(this->thread);
		}
		
		this->connect(this->thread, this->viewer);
	}
	else
	{
		this->disconnect(this->thread, this->viewer);
		
		this->planner->setViewer(nullptr);
		
		if (nullptr != this->optimizer)
		{
			this->optimizer->setViewer(nullptr);
		}
		
		for (std::vector<std::shared_ptr<rl::plan::WorkspaceSphereExplorer>>::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
		{
			(*i)->setViewer(nullptr);
		}
	}
}
