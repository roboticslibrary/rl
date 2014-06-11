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
#include <QDateTime>
#include <QDockWidget>
#include <QFileDialog>
#include <QGLWidget>
#include <QGraphicsView>
#include <QHeaderView>
#include <QLayout>
#include <QMenuBar>
#include <QMutexLocker>
#include <QPainter>
#include <QPrinter>
#include <QRegExp>
#include <boost/make_shared.hpp>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Qt/SoQt.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/plan/AddRrtConCon.h>
#include <rl/plan/AdvancedOptimizer.h>
#include <rl/plan/BridgeSampler.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/Eet.h>
#include <rl/plan/GaussianSampler.h>
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
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>

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

#include "ConfigurationDelegate.h"
#include "ConfigurationModel.h"
#include "ConfigurationSpaceScene.h"
#include "MainWindow.h"
#include "PlannerModel.h"
#include "Thread.h"
#include "Viewer.h"

MainWindow::MainWindow(QWidget* parent, Qt::WFlags f) :
	QMainWindow(parent, f),
	configurationModel(new ConfigurationModel(this)),
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
	planner(),
	plannerModel(new PlannerModel(this)),
	q(),
	sampler(),
	sampler2(),
	sigma(),
	scene(),
	scene2(),
	sceneModel(NULL),
	sceneModel2(NULL),
	start(),
	thread(new Thread(this)),
	verifier(),
	verifier2(),
	viewer(NULL),
	configurationDelegate(new ConfigurationDelegate(this)),
	configurationDockWidget(new QDockWidget(this)),
	configurationSpaceDockWidget(new QDockWidget(this)),
	configurationSpaceScene(new ConfigurationSpaceScene(this)),
	configurationSpaceView(new QGraphicsView(this)),
	configurationView(new QTableView(this)),
	engine(),
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
	saveImageAction(new QAction(this)),
	savePdfAction(new QAction(this)),
	saveSceneAction(new QAction(this)),
	setGoalConfigurationAction(new QAction(this)),
	setStartConfigurationAction(new QAction(this)),
	startThreadAction(new QAction(this)),
	toggleCameraAction(new QAction(this)),
	toggleConfigurationAction(new QAction(this)),
	toggleConfigurationEdgesAction(new QAction(this)),
	toggleConfigurationSpaceAction(new QAction(this)),
	toggleConfigurationVerticesAction(new QAction(this)),
	toggleLinesAction(new QAction(this)),
	togglePlannerAction(new QAction(this)),
	togglePointsAction(new QAction(this)),
	toggleSpheresAction(new QAction(this)),
	toggleViewAction(new QAction(this)),
	toggleWorkFramesAction(new QAction(this)),
	wait(true)
{
	MainWindow::singleton = this;
	
	SoQt::init(this);
	SoDB::init();
	
	this->viewer = new Viewer(this);
	this->setCentralWidget(this->viewer);
	
	this->configurationSpaceView->setEnabled(false);
	this->configurationSpaceView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	this->configurationSpaceView->setScene(this->configurationSpaceScene);
	this->configurationSpaceView->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	this->configurationSpaceView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	
	this->configurationView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
	this->configurationView->horizontalHeader()->hide();
	this->configurationView->setAlternatingRowColors(true);
	this->configurationView->setItemDelegate(this->configurationDelegate);
	this->configurationView->setModel(this->configurationModel);
	this->configurationView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
	
	this->configurationDockWidget->hide();
	this->configurationDockWidget->resize(160, 320);
	this->configurationDockWidget->setFloating(true);
	this->configurationDockWidget->setWidget(this->configurationView);
	this->configurationDockWidget->setWindowTitle("Configuration");
	
	this->configurationSpaceDockWidget->hide();
	this->configurationSpaceDockWidget->setFloating(true);
	this->configurationSpaceDockWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	this->configurationSpaceDockWidget->setWidget(this->configurationSpaceView);
	this->configurationSpaceDockWidget->setWindowTitle("C-Space");
	
	this->plannerView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
	this->plannerView->setAlternatingRowColors(true);
	this->plannerView->setModel(this->plannerModel);
	this->plannerView->setWordWrap(false);
	this->plannerView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
	
	this->plannerDockWidget->hide();
	this->plannerDockWidget->resize(160, 320);
	this->plannerDockWidget->setFloating(true);
	this->plannerDockWidget->setWidget(this->plannerView);
	this->plannerDockWidget->setWindowTitle("Planner");
	
	this->init();
	
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
	
	QRegExp engineRegExp("--engine=(" + engines.join("|") + ")");
	QRegExp helpRegExp("--help");
	QRegExp heightRegExp("--height=(\\d*)");
	QRegExp viewerRegExp("--disable-viewer");
	QRegExp waitRegExp("--disable-wait");
	QRegExp widthRegExp("--width=(\\d*)");
	QRegExp quitRegExp("--enable-quit");
	
	int width = 1024;
	int height = 768;
	
	for (int i = 1; i < QApplication::arguments().size(); ++i)
	{
		if (-1 != engineRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->engine = engineRegExp.cap(1);
		}
		else if (-1 != helpRegExp.indexIn(QApplication::arguments()[i]))
		{
			std::cout << "Usage: rlPlanDemo [PLANFILE] [--engine=" << engines.join("|").toStdString() << "] [--help] [--disable-viewer] [--disable-wait] [--enable-quit] [--width=WIDTH] [--height=HEIGHT]" << std::endl;
		}
		else if (-1 != heightRegExp.indexIn(QApplication::arguments()[i]))
		{
			height = heightRegExp.cap(1).toInt();
		}
		else if (-1 != viewerRegExp.indexIn(QApplication::arguments()[i]))
		{
			QObject::disconnect(this->toggleViewAction, SIGNAL(toggled(bool)), this, SLOT(toggleView(bool)));
			this->toggleViewAction->setChecked(false);
			QObject::connect(this->toggleViewAction, SIGNAL(toggled(bool)), this, SLOT(toggleView(bool)));
		}
		else if (-1 != waitRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->wait = false;
		}
		else if (-1 != widthRegExp.indexIn(QApplication::arguments()[i]))
		{
			width = widthRegExp.cap(1).toInt();
		}
		else if (-1 != quitRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->thread->quit = true;
		}
		else
		{
			this->filename = QApplication::arguments()[i];
		}
	}
	
	this->resize(width, height);
	this->viewer->setMinimumSize(width, height);
	
	if (this->filename.isEmpty())
	{
		this->open();
	}
	else
	{
		this->load(this->filename);
	}
}

MainWindow::~MainWindow()
{
	this->thread->stop();
	
	MainWindow::singleton = NULL;
}

void
MainWindow::clear()
{
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
	this->optimizer.reset();
	this->planner.reset();
	this->q.reset();
	this->sampler.reset();
	this->sampler2.reset();
	this->sigma.reset();
	this->scene.reset();
	this->scene2.reset();
	this->sceneModel = NULL;
	this->sceneModel2 = NULL;
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
	QObject::disconnect(sender, NULL, receiver, NULL);
}

void
MainWindow::eval()
{
	this->configurationSpaceScene->eval();
}

void
MainWindow::getGoalConfiguration()
{
	*this->q = *this->goal;
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::getRandomConfiguration()
{
	this->sampler2->generate(*this->q);
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::getRandomFreeConfiguration()
{
	this->sampler2->generateCollisionFree(*this->q);
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::getStartConfiguration()
{
	*this->q = *this->start;
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::init()
{
	QMenu* fileMenu = this->menuBar()->addMenu("File");
	
	this->openAction->setText("Open...");
	this->openAction->setShortcut(QKeySequence::Open);
	QObject::connect(this->openAction, SIGNAL(triggered()), this, SLOT(open()));
	this->addAction(this->openAction);
	fileMenu->addAction(this->openAction);
	
	fileMenu->addSeparator();
	
	this->saveImageAction->setText("Save as PNG");
	this->saveImageAction->setShortcut(QKeySequence("Return"));
	QObject::connect(this->saveImageAction, SIGNAL(triggered()), this, SLOT(saveImage()));
	this->addAction(this->saveImageAction);
	fileMenu->addAction(this->saveImageAction);
	
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
	
	this->toggleConfigurationAction->setText("Show/Hide");
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
	
	QMenu* cSpaceMenu = this->menuBar()->addMenu("C-Space");
	
	this->toggleConfigurationSpaceAction->setText("Show/Hide");
	this->toggleConfigurationSpaceAction->setShortcut(QKeySequence("F6"));
	QObject::connect(this->toggleConfigurationSpaceAction, SIGNAL(triggered()), this, SLOT(toggleConfigurationSpace()));
	this->addAction(this->toggleConfigurationSpaceAction);
	cSpaceMenu->addAction(this->toggleConfigurationSpaceAction);
	
	cSpaceMenu->addSeparator();
	
	this->evalAction->setText("Evaluate");
	this->evalAction->setShortcut(QKeySequence("F11"));
	QObject::connect(this->evalAction, SIGNAL(triggered()), this, SLOT(eval()));
	this->addAction(this->evalAction);
	cSpaceMenu->addAction(this->evalAction);
	
	QMenu* plannerMenu = this->menuBar()->addMenu("Planner");
	
	this->togglePlannerAction->setText("Show/Hide");
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
	
	this->toggleViewAction->setCheckable(true);
	this->toggleViewAction->setChecked(true);
	this->toggleViewAction->setText("Active");
	QObject::connect(this->toggleViewAction, SIGNAL(toggled(bool)), this, SLOT(toggleView(bool)));
	this->addAction(this->toggleViewAction);
	viewMenu->addAction(this->toggleViewAction);
	
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
	if (NULL == MainWindow::singleton)
	{
		new MainWindow();
	}
	
	return MainWindow::singleton;
}

void
MainWindow::load(const QString& filename)
{
	QMutexLocker lock(&this->mutex);
	
	this->clear();
	
	rl::xml::DomParser parser;
	
	rl::xml::Document doc = parser.readFile(filename.toStdString(), "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	
	this->filename = filename;
	this->setWindowTitle(filename + " - " + this->engine + " - rlPlanDemo");
	
	doc.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	
	rl::xml::Path path(doc);
	
#ifdef RL_SG_HAVE_BULLET
	if ("bullet" == this->engine)
	{
		this->scene = boost::make_shared< rl::sg::bullet::Scene >();
	}
#endif // RL_SG_HAVE_BULLET
#ifdef RL_SG_HAVE_ODE
	if ("ode" == this->engine)
	{
		this->scene = boost::make_shared< rl::sg::ode::Scene >();
	}
#endif // RL_SG_HAVE_ODE
#ifdef RL_SG_HAVE_PQP
	if ("pqp" == this->engine)
	{
		this->scene = boost::make_shared< rl::sg::pqp::Scene >();
	}
#endif // RL_SG_HAVE_PQP
#ifdef RL_SG_HAVE_SOLID
	if ("solid" == this->engine)
	{
		this->scene = boost::make_shared< rl::sg::solid::Scene >();
	}
#endif // RL_SG_HAVE_SOLID
	
	rl::xml::Object modelScene = path.eval("//model/scene");
	this->scene->load(modelScene.getNodeTab(0).getUri(modelScene.getNodeTab(0).getAttribute("href").getValue()));
	this->sceneModel = this->scene->getModel(
		static_cast< int >(path.eval("number(//model/model)").getFloatval())
	);
	
	if (path.eval("count(//model/kinematics) > 0").getBoolval())
	{
		rl::xml::Object kin = path.eval("//model/kinematics");
		this->kin.reset(rl::kin::Kinematics::create(
			kin.getNodeTab(0).getUri(kin.getNodeTab(0).getAttribute("href").getValue())
		));
		
		if (path.eval("count(//model/kinematics/world) > 0").getBoolval())
		{
			this->kin->world() = ::rl::math::AngleAxis(
				path.eval("number(//model/kinematics/world/rotation/z)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitZ()
			) * ::rl::math::AngleAxis(
				path.eval("number(//model/kinematics/world/rotation/y)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitY()
			) * ::rl::math::AngleAxis(
				path.eval("number(//model/kinematics/world/rotation/x)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitX()
			);
			
			this->kin->world().translation().x() = path.eval("number(//model/kinematics/world/translation/x)").getFloatval(0);
			this->kin->world().translation().y() = path.eval("number(//model/kinematics/world/translation/y)").getFloatval(0);
			this->kin->world().translation().z() = path.eval("number(//model/kinematics/world/translation/z)").getFloatval(0);
		}
	}
	
	if (path.eval("count(//model/mdl) > 0").getBoolval())
	{
		rl::xml::Object mdl = path.eval("//model/mdl");
		rl::mdl::XmlFactory factory;
		this->mdl.reset(dynamic_cast< rl::mdl::Dynamic* >(factory.create(
			mdl.getNodeTab(0).getUri(mdl.getNodeTab(0).getAttribute("href").getValue())
		)));
		
		if (path.eval("count(//model/mdl/world) > 0").getBoolval())
		{
			this->mdl->world() = ::rl::math::AngleAxis(
				path.eval("number(//model/mdl/world/rotation/z)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitZ()
			) * ::rl::math::AngleAxis(
				path.eval("number(//model/mdl/world/rotation/y)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitY()
			) * ::rl::math::AngleAxis(
				path.eval("number(//model/mdl/world/rotation/x)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitX()
			);
			
			this->mdl->world().translation().x() = path.eval("number(//model/mdl/world/translation/x)").getFloatval(0);
			this->mdl->world().translation().y() = path.eval("number(//model/mdl/world/translation/y)").getFloatval(0);
			this->mdl->world().translation().z() = path.eval("number(//model/mdl/world/translation/z)").getFloatval(0);
		}
	}
	
	this->model = boost::make_shared< rl::plan::DistanceModel >();
	
	if (NULL != this->kin)
	{
		this->model->kin = this->kin.get();
	}
	else if (NULL != this->mdl)
	{
		this->model->mdl = this->mdl.get();
	}
	
	this->model->model = this->sceneModel;
	this->model->scene = this->scene.get();
	
	this->q = boost::make_shared< rl::math::Vector >(this->model->getDof());
	
	if (NULL != this->scene2)
	{
		this->viewer->sceneGroup->removeChild(this->scene2->root);
	}
	
	this->scene2 = boost::make_shared< rl::sg::so::Scene >();
	
	rl::xml::Object viewerScene = path.eval("//viewer/model/scene");
	this->scene2->load(viewerScene.getNodeTab(0).getUri(viewerScene.getNodeTab(0).getAttribute("href").getValue()));
	this->sceneModel2 = static_cast< rl::sg::so::Model* >(this->scene2->getModel(
		static_cast< int >(path.eval("number(//viewer/model/model)").getFloatval())
	));
	
	if (path.eval("count(//viewer/model/kinematics) > 0").getBoolval())
	{
		rl::xml::Object kin2 = path.eval("//viewer/model/kinematics");
		this->kin2.reset(rl::kin::Kinematics::create(
			kin2.getNodeTab(0).getUri(kin2.getNodeTab(0).getAttribute("href").getValue())
		));
		
		if (path.eval("count(//viewer/model/kinematics/world) > 0").getBoolval())
		{
			this->kin2->world() = ::rl::math::AngleAxis(
				path.eval("number(//viewer/model/kinematics/world/rotation/z)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitZ()
			) * ::rl::math::AngleAxis(
				path.eval("number(//viewer/model/kinematics/world/rotation/y)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitY()
			) * ::rl::math::AngleAxis(
				path.eval("number(//viewer/model/kinematics/world/rotation/x)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitX()
			);
			
			this->kin2->world().translation().x() = path.eval("number(//viewer/model/kinematics/world/translation/x)").getFloatval(0);
			this->kin2->world().translation().y() = path.eval("number(//viewer/model/kinematics/world/translation/y)").getFloatval(0);
			this->kin2->world().translation().z() = path.eval("number(//viewer/model/kinematics/world/translation/z)").getFloatval(0);
		}
	}
	
	if (path.eval("count(//viewer/model/mdl) > 0").getBoolval())
	{
		rl::xml::Object mdl2 = path.eval("//viewer/model/mdl");
		rl::mdl::XmlFactory factory;
		this->mdl2.reset(dynamic_cast< rl::mdl::Dynamic* >(factory.create(
			mdl2.getNodeTab(0).getUri(mdl2.getNodeTab(0).getAttribute("href").getValue())
		)));
		
		if (path.eval("count(//viewer/model/mdl/world) > 0").getBoolval())
		{
			this->mdl2->world() = ::rl::math::AngleAxis(
				path.eval("number(//viewer/model/mdl/world/rotation/z)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitZ()
			) * ::rl::math::AngleAxis(
				path.eval("number(//viewer/model/mdl/world/rotation/y)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitY()
			) * ::rl::math::AngleAxis(
				path.eval("number(//viewer/model/mdl/world/rotation/x)").getFloatval(0) * rl::math::DEG2RAD,
				::rl::math::Vector3::UnitX()
			);
			
			this->mdl2->world().translation().x() = path.eval("number(//viewer/model/mdl/world/translation/x)").getFloatval(0);
			this->mdl2->world().translation().y() = path.eval("number(//viewer/model/mdl/world/translation/y)").getFloatval(0);
			this->mdl2->world().translation().z() = path.eval("number(//viewer/model/mdl/world/translation/z)").getFloatval(0);
		}
	}
	
	this->model2 = boost::make_shared< rl::plan::Model >();
	
	if (NULL != this->kin2)
	{
		this->model2->kin = this->kin2.get();
	}
	else if (NULL != this->mdl2)
	{
		this->model2->mdl = this->mdl2.get();
	}
	
	this->model2->model = this->sceneModel2;
	this->model2->scene = this->scene2.get();
	
	rl::xml::Object start = path.eval("//start//q");
	this->start = boost::make_shared< rl::math::Vector >(start.getNodeNr());
	
	for (int i = 0; i < start.getNodeNr(); ++i)
	{
		(*this->start)(i) = std::atof(start.getNodeTab(i).getContent().c_str());
		
		if (start.getNodeTab(i).hasAttribute("unit"))
		{
			if ("deg" == start.getNodeTab(i).getAttribute("unit").getValue())
			{
				(*this->start)(i) *= rl::math::DEG2RAD;
			}
		}
	}
	
	*this->q = *this->start;
	
	rl::xml::Object goal = path.eval("//goal//q");
	this->goal = boost::make_shared< rl::math::Vector >(goal.getNodeNr());
	
	for (int i = 0; i < goal.getNodeNr(); ++i)
	{
		(*this->goal)(i) = std::atof(goal.getNodeTab(i).getContent().c_str());
		
		if (goal.getNodeTab(i).hasAttribute("unit"))
		{
			if ("deg" == goal.getNodeTab(i).getAttribute("unit").getValue())
			{
				(*this->goal)(i) *= rl::math::DEG2RAD;
			}
		}
	}
	
	rl::xml::Object sigma = path.eval("//sigma//q");
	this->sigma = boost::make_shared< rl::math::Vector >(sigma.getNodeNr());
	
	for (int i = 0; i < sigma.getNodeNr(); ++i)
	{
		(*this->sigma)(i) = std::atof(sigma.getNodeTab(i).getContent().c_str());
		
		if (sigma.getNodeTab(i).hasAttribute("unit"))
		{
			if ("deg" == sigma.getNodeTab(i).getAttribute("unit").getValue())
			{
				(*this->sigma)(i) *= rl::math::DEG2RAD;
			}
		}
	}
	
	if (path.eval("count(//uniformSampler) > 0").getBoolval())
	{
		this->sampler = boost::make_shared< rl::plan::UniformSampler >();
		rl::plan::UniformSampler* uniformSampler = static_cast< rl::plan::UniformSampler* >(this->sampler.get());
		
		if (path.eval("count(//uniformSampler/seed) > 0").getBoolval())
		{
			uniformSampler->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(//uniformSampler/seed)").getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
	}
	else if (path.eval("count(//gaussianSampler) > 0").getBoolval())
	{
		this->sampler = boost::make_shared< rl::plan::GaussianSampler >();
		rl::plan::GaussianSampler* gaussianSampler = static_cast< rl::plan::GaussianSampler* >(this->sampler.get());
		
		if (path.eval("count(//gaussianSampler/seed) > 0").getBoolval())
		{
			gaussianSampler->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(//gaussianSampler/seed)").getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
		
		gaussianSampler->sigma = this->sigma.get();
	}
	else if (path.eval("count(//bridgeSampler) > 0").getBoolval())
	{
		this->sampler = boost::make_shared< rl::plan::BridgeSampler >();
		rl::plan::BridgeSampler* bridgeSampler = static_cast< rl::plan::BridgeSampler* >(this->sampler.get());
		bridgeSampler->ratio = path.eval("number(//bridgeSampler/ratio)").getFloatval(5.0f / 6.0f);
		
		if (path.eval("count(//bridgeSampler/seed) > 0").getBoolval())
		{
			bridgeSampler->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(//bridgeSampler/seed)").getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
		
		bridgeSampler->sigma = this->sigma.get();
	}
	
	if (NULL != this->sampler)
	{
		this->sampler->model = this->model.get();
	}
	
	this->sampler2 = boost::make_shared< rl::plan::UniformSampler >();
	this->sampler2->model = this->model.get();
	
	if (path.eval("count(//recursiveVerifier) > 0").getBoolval())
	{
		this->verifier = boost::make_shared< rl::plan::RecursiveVerifier >();
		this->verifier->delta = path.eval("number(//recursiveVerifier/delta)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//recursiveVerifier/delta/@unit)").getStringval())
		{
			this->verifier->delta *= rl::math::DEG2RAD;
		}
	}
	else if (path.eval("count(//sequentialVerifier) > 0").getBoolval())
	{
		this->verifier = boost::make_shared< rl::plan::SequentialVerifier >();
		this->verifier->delta = path.eval("number(//sequentialVerifier/delta)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//sequentialVerifier/delta/@unit)").getStringval())
		{
			this->verifier->delta *= rl::math::DEG2RAD;
		}
	}
	
	if (NULL != this->verifier)
	{
		this->verifier->model = this->model.get();
	}
	
	if (path.eval("count(//simpleOptimizer/recursiveVerifier) > 0").getBoolval())
	{
		this->verifier2 = boost::make_shared< rl::plan::RecursiveVerifier >();
		this->verifier2->delta = path.eval("number(//simpleOptimizer/recursiveVerifier/delta)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//simpleOptimizer/recursiveVerifier/delta/@unit)").getStringval())
		{
			this->verifier2->delta *= rl::math::DEG2RAD;
		}
	}
	else if (path.eval("count(//advancedOptimizer/recursiveVerifier) > 0").getBoolval())
	{
		this->verifier2 = boost::make_shared< rl::plan::RecursiveVerifier >();
		this->verifier2->delta = path.eval("number(//advancedOptimizer/recursiveVerifier/delta)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//advancedOptimizer/recursiveVerifier/delta/@unit)").getStringval())
		{
			this->verifier2->delta *= rl::math::DEG2RAD;
		}
	}
	
	if (NULL != this->verifier2)
	{
		this->verifier2->model = this->model.get();
	}
	
	this->optimizer.reset();
	
	if (path.eval("count(//simpleOptimizer) > 0").getBoolval())
	{
		this->optimizer = boost::make_shared< rl::plan::SimpleOptimizer >();
	}
	else if (path.eval("count(//advancedOptimizer) > 0").getBoolval())
	{
		this->optimizer = boost::make_shared< rl::plan::AdvancedOptimizer >();
		rl::plan::AdvancedOptimizer* advancedOptimizer = static_cast< rl::plan::AdvancedOptimizer* >(this->optimizer.get());
		advancedOptimizer->length = path.eval("number(//advancedOptimizer/length)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//advancedOptimizer/length/@unit)").getStringval())
		{
			advancedOptimizer->length *= rl::math::DEG2RAD;
		}
		
		advancedOptimizer->ratio = path.eval("number(//advancedOptimizer/ratio)").getFloatval(0.1f);
	}
	
	if (NULL != this->optimizer)
	{
		this->optimizer->model = this->model.get();
		this->optimizer->verifier = this->verifier2.get();
	}
	
	rl::xml::Object planner = path.eval("//addRrtConCon|//eet|//prm|//prmUtilityGuided|//rrt|//rrtCon|//rrtConCon|//rrtConExt|//rrtDual|//rrtGoalBias|//rrtExtCon|//rrtExtExt");
	
	if ("addRrtConCon" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::AddRrtConCon >();
		rl::plan::AddRrtConCon* addRrtConCon = static_cast< rl::plan::AddRrtConCon* >(this->planner.get());
		addRrtConCon->alpha = path.eval("number(alpha)", planner.getNodeTab(0)).getFloatval(0.05f);
		addRrtConCon->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			addRrtConCon->delta *= rl::math::DEG2RAD;
		}
		
		addRrtConCon->epsilon = path.eval("number(epsilon)", planner.getNodeTab(0)).getFloatval(1.0e-3f);
		
		if ("deg" == path.eval("string(epsilon/@unit)", planner.getNodeTab(0)).getStringval())
		{
			addRrtConCon->epsilon *= rl::math::DEG2RAD;
		}
		
		addRrtConCon->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		addRrtConCon->lower = path.eval("number(lower)", planner.getNodeTab(0)).getFloatval(2.0f);
		
		if ("deg" == path.eval("string(lower/@unit)", planner.getNodeTab(0)).getStringval())
		{
			addRrtConCon->lower *= rl::math::DEG2RAD;
		}
		
		addRrtConCon->radius = path.eval("number(radius)", planner.getNodeTab(0)).getFloatval(20.0f);
		
		if ("deg" == path.eval("string(radius/@unit)", planner.getNodeTab(0)).getStringval())
		{
			addRrtConCon->radius *= rl::math::DEG2RAD;
		}
		
		addRrtConCon->sampler = this->sampler.get();
	}
	else if ("eet" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::Eet >();
		rl::plan::Eet* eet = static_cast< rl::plan::Eet* >(this->planner.get());
		eet->alpha = path.eval("number(alpha)", planner.getNodeTab(0)).getFloatval(0.01f);
		eet->alternativeDistanceComputation = path.eval("count(alternativeDistanceComputation) > 0", planner.getNodeTab(0)).getBoolval() ? true : false;
		eet->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			eet->delta *= rl::math::DEG2RAD;
		}
		
		eet->distanceWeight = path.eval("number(distanceWeight)", planner.getNodeTab(0)).getFloatval(0.1f);
		eet->epsilon = path.eval("number(epsilon)", planner.getNodeTab(0)).getFloatval(1.0e-3f);
		
		if ("deg" == path.eval("string(epsilon/@unit)", planner.getNodeTab(0)).getStringval())
		{
			eet->epsilon *= rl::math::DEG2RAD;
		}
		
		eet->gamma = path.eval("number(gamma)", planner.getNodeTab(0)).getFloatval(1.f/3.f);
		eet->goalEpsilon = path.eval("number(goalEpsilon)", planner.getNodeTab(0)).getFloatval(0.1f);
		
		if (path.eval("translate(string(goalEpsilon/@orientation), 'TRUE', 'true') = 'true' or string(goalEpsilon/@orientation) = '1'", planner.getNodeTab(0)).getBoolval())
		{
			eet->goalEpsilonUseOrientation = true;
		}
		else
		{
			eet->goalEpsilonUseOrientation = false;
		}
		
		eet->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		eet->max.x() = path.eval("number(max/x)", planner.getNodeTab(0)).getFloatval(0.0f);
		eet->max.y() = path.eval("number(max/y)", planner.getNodeTab(0)).getFloatval(0.0f);
		eet->max.z() = path.eval("number(max/z)", planner.getNodeTab(0)).getFloatval(0.0f);
		eet->min.x() = path.eval("number(min/x)", planner.getNodeTab(0)).getFloatval(0.0f);
		eet->min.y() = path.eval("number(min/y)", planner.getNodeTab(0)).getFloatval(0.0f);
		eet->min.z() = path.eval("number(min/z)", planner.getNodeTab(0)).getFloatval(0.0f);
		eet->sampler = this->sampler.get();
		
		if (path.eval("count(seed) > 0", planner.getNodeTab(0)).getBoolval())
		{
			eet->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(seed)", planner.getNodeTab(0)).getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
		
		rl::xml::Object explorers = path.eval("explorer", planner.getNodeTab(0));
		
		for (int i = 0; i < explorers.getNodeNr(); ++i)
		{
			boost::shared_ptr< rl::plan::WorkspaceSphereExplorer > explorer = boost::make_shared< rl::plan::WorkspaceSphereExplorer >();
			this->explorers.push_back(explorer);
			eet->explorers.push_back(explorer.get());
			
			rl::plan::Eet::ExplorerSetup explorerSetup;
			
			boost::shared_ptr< rl::math::Vector3 > explorerStart = boost::make_shared< rl::math::Vector3 >();
			this->explorerStarts.push_back(explorerStart);
			explorer->start = explorerStart.get();
			
			(*explorerStart).x() = path.eval("number(start/x)", explorers.getNodeTab(i)).getFloatval(0.0f);
			(*explorerStart).y() = path.eval("number(start/y)", explorers.getNodeTab(i)).getFloatval(0.0f);
			(*explorerStart).z() = path.eval("number(start/z)", explorers.getNodeTab(i)).getFloatval(0.0f);
			
			if (path.eval("count(start/goal) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorerSetup.startConfiguration = this->goal.get();
			}
			else if (path.eval("count(start/start) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorerSetup.startConfiguration = this->start.get();
			}
			else
			{
				explorerSetup.startConfiguration = NULL;
			}
			
			if (path.eval("count(start//frame) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorerSetup.startFrame = static_cast< int >(path.eval("number(start//frame)", explorers.getNodeTab(i)).getFloatval());
			}
			else if (path.eval("count(start//tcp) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorerSetup.startFrame = -1;
			}
			
			boost::shared_ptr< rl::math::Vector3 > explorerGoal = boost::make_shared< rl::math::Vector3 >();
			this->explorerGoals.push_back(explorerGoal);
			explorer->goal = explorerGoal.get();
			
			(*explorerGoal).x() = path.eval("number(goal/x)", explorers.getNodeTab(i)).getFloatval(0.0f);
			(*explorerGoal).y() = path.eval("number(goal/y)", explorers.getNodeTab(i)).getFloatval(0.0f);
			(*explorerGoal).z() = path.eval("number(goal/z)", explorers.getNodeTab(i)).getFloatval(0.0f);
			
			if (path.eval("count(goal/goal) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorerSetup.goalConfiguration = this->goal.get();
			}
			else if (path.eval("count(goal/start) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorerSetup.goalConfiguration = this->start.get();
			}
			else
			{
				explorerSetup.goalConfiguration = NULL;
			}
			
			if (path.eval("count(goal//frame) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorerSetup.goalFrame = static_cast< int >(path.eval("number(goal//frame)", explorers.getNodeTab(i)).getFloatval());
			}
			else if (path.eval("count(goal//tcp) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorerSetup.goalFrame = -1;
			}
			
			if (path.eval("count(distance) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorer->greedy = rl::plan::WorkspaceSphereExplorer::GREEDY_DISTANCE;
			}
			else if (path.eval("count(sourcedistance) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorer->greedy = rl::plan::WorkspaceSphereExplorer::GREEDY_SOURCE_DISTANCE;
			}
			else if (path.eval("count(space) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorer->greedy = rl::plan::WorkspaceSphereExplorer::GREEDY_SPACE;
			}
			
			explorer->model = this->model.get();
			explorer->radius = path.eval("number(radius)", explorers.getNodeTab(i)).getFloatval(0.0f);
			explorer->range = path.eval("number(range)", explorers.getNodeTab(i)).getFloatval(std::numeric_limits< rl::math::Real >::max());
			explorer->samples = static_cast< int >(path.eval("number(samples)", explorers.getNodeTab(i)).getFloatval(10.0f));
			
			if (path.eval("count(seed) > 0", explorers.getNodeTab(i)).getBoolval())
			{
				explorer->seed(
					static_cast< boost::mt19937::result_type >(path.eval("number(seed)", explorers.getNodeTab(i)).getFloatval(rl::util::Timer::now() * 1000000.0f))
				);
			}
			
			eet->explorersSetup.push_back(explorerSetup);
		}
	}
	else if ("prm" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::Prm >();
		rl::plan::Prm* prm = static_cast< rl::plan::Prm* >(this->planner.get());
		prm->degree = static_cast< int >(path.eval("number(degree)", planner.getNodeTab(0)).getFloatval(std::numeric_limits< std::size_t >::max()));
		prm->k = static_cast< int >(path.eval("number(k)", planner.getNodeTab(0)).getFloatval(30.0f));
		prm->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		prm->radius = path.eval("number(radius)", planner.getNodeTab(0)).getFloatval(std::numeric_limits< rl::math::Real >::max());
		
		if ("deg" == path.eval("string(radius/@unit)", planner.getNodeTab(0)).getStringval())
		{
			prm->radius *= rl::math::DEG2RAD;
		}
		
		prm->sampler = this->sampler.get();
		prm->verifier = this->verifier.get();
	}
	else if ("prmUtilityGuided" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::PrmUtilityGuided >();
		rl::plan::PrmUtilityGuided* prmUtilityGuided = static_cast< rl::plan::PrmUtilityGuided* >(this->planner.get());
		prmUtilityGuided->degree = static_cast< int >(path.eval("number(degree)", planner.getNodeTab(0)).getFloatval(std::numeric_limits< std::size_t >::max()));
		prmUtilityGuided->k = static_cast< int >(path.eval("number(k)", planner.getNodeTab(0)).getFloatval(30.0f));
		prmUtilityGuided->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		prmUtilityGuided->radius = path.eval("number(radius)", planner.getNodeTab(0)).getFloatval(std::numeric_limits< rl::math::Real >::max());
		
		if ("deg" == path.eval("string(radius/@unit)", planner.getNodeTab(0)).getStringval())
		{
			prmUtilityGuided->radius *= rl::math::DEG2RAD;
		}
		
		if (path.eval("count(seed) > 0", planner.getNodeTab(0)).getBoolval())
		{
			prmUtilityGuided->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(seed)", planner.getNodeTab(0)).getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
		
		prmUtilityGuided->sampler = this->sampler.get();
		prmUtilityGuided->verifier = this->verifier.get();
	}
	else if ("rrt" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::Rrt >();
		rl::plan::Rrt* rrt = static_cast< rl::plan::Rrt* >(this->planner.get());
		rrt->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrt->delta *= rl::math::DEG2RAD;
		}
		
		rrt->epsilon = path.eval("number(epsilon)", planner.getNodeTab(0)).getFloatval(1.0e-3f);
		
		if ("deg" == path.eval("string(epsilon/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrt->epsilon *= rl::math::DEG2RAD;
		}
		
		rrt->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrt->sampler = this->sampler.get();
	}
	else if ("rrtCon" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::RrtCon >();
		rl::plan::RrtCon* rrtCon = static_cast< rl::plan::RrtCon* >(this->planner.get());
		rrtCon->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtCon->delta *= rl::math::DEG2RAD;
		}
		
		rrtCon->epsilon = path.eval("number(epsilon)", planner.getNodeTab(0)).getFloatval(1.0e-3f);
		
		if ("deg" == path.eval("string(epsilon/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtCon->epsilon *= rl::math::DEG2RAD;
		}
		
		rrtCon->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtCon->probability = path.eval("number(probability)", planner.getNodeTab(0)).getFloatval(0.05f);
		rrtCon->sampler = this->sampler.get();
		
		if (path.eval("count(seed) > 0", planner.getNodeTab(0)).getBoolval())
		{
			rrtCon->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(seed)", planner.getNodeTab(0)).getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
	}
	else if ("rrtConCon" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::RrtConCon >();
		rl::plan::RrtConCon* rrtConCon = static_cast< rl::plan::RrtConCon* >(this->planner.get());
		rrtConCon->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtConCon->delta *= rl::math::DEG2RAD;
		}
		
		rrtConCon->epsilon = path.eval("number(epsilon)", planner.getNodeTab(0)).getFloatval(1.0e-3f);
		
		if ("deg" == path.eval("string(epsilon/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtConCon->epsilon *= rl::math::DEG2RAD;
		}
		
		rrtConCon->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtConCon->sampler = this->sampler.get();
	}
	else if ("rrtDual" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::RrtDual >();
		rl::plan::RrtDual* rrtDual = static_cast< rl::plan::RrtDual* >(this->planner.get());
		rrtDual->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtDual->delta *= rl::math::DEG2RAD;
		}
		
		rrtDual->epsilon = path.eval("number(epsilon)", planner.getNodeTab(0)).getFloatval(1.0e-3f);
		
		if ("deg" == path.eval("string(epsilon/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtDual->epsilon *= rl::math::DEG2RAD;
		}
		
		rrtDual->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtDual->sampler = this->sampler.get();
	}
	else if ("rrtExtCon" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::RrtExtCon >();
		rl::plan::RrtExtCon* rrtExtCon = static_cast< rl::plan::RrtExtCon* >(this->planner.get());
		rrtExtCon->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtExtCon->delta *= rl::math::DEG2RAD;
		}
		
		rrtExtCon->epsilon = path.eval("number(epsilon)", planner.getNodeTab(0)).getFloatval(1.0e-3f);
		
		if ("deg" == path.eval("string(epsilon/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtExtCon->epsilon *= rl::math::DEG2RAD;
		}
		
		rrtExtCon->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtExtCon->sampler = this->sampler.get();
	}
	else if ("rrtExtExt" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::RrtExtExt >();
		rl::plan::RrtExtExt* rrtExtExt = static_cast< rl::plan::RrtExtExt* >(this->planner.get());
		rrtExtExt->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtExtExt->delta *= rl::math::DEG2RAD;
		}
		
		rrtExtExt->epsilon = path.eval("number(epsilon)", planner.getNodeTab(0)).getFloatval(1.0e-3f);
		
		if ("deg" == path.eval("string(epsilon/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtExtExt->epsilon *= rl::math::DEG2RAD;
		}
		
		rrtExtExt->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtExtExt->sampler = this->sampler.get();
	}
	else if ("rrtGoalBias" == planner.getNodeTab(0).getName())
	{
		this->planner = boost::make_shared< rl::plan::RrtGoalBias >();
		rl::plan::RrtGoalBias* rrtGoalBias = static_cast< rl::plan::RrtGoalBias* >(this->planner.get());
		rrtGoalBias->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtGoalBias->delta *= rl::math::DEG2RAD;
		}
		
		rrtGoalBias->epsilon = path.eval("number(epsilon)", planner.getNodeTab(0)).getFloatval(1.0e-3f);
		
		if ("deg" == path.eval("string(epsilon/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtGoalBias->epsilon *= rl::math::DEG2RAD;
		}
		
		rrtGoalBias->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtGoalBias->probability = path.eval("number(probability)", planner.getNodeTab(0)).getFloatval(0.05f);
		rrtGoalBias->sampler = this->sampler.get();
		
		if (path.eval("count(seed) > 0", planner.getNodeTab(0)).getBoolval())
		{
			rrtGoalBias->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(seed)", planner.getNodeTab(0)).getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
	}
	
	this->planner->duration = path.eval("number(//duration)").getFloatval(std::numeric_limits< rl::math::Real >::max());
	this->planner->goal = this->goal.get();
	this->planner->model = this->model.get();
	this->planner->start = this->start.get();
	
	this->viewer->delta = path.eval("number(//viewer/delta)").getFloatval();
	
	if ("deg" == path.eval("string(//viewer/delta/@unit)").getStringval())
	{
		this->viewer->delta *= rl::math::DEG2RAD;
	}
	
	this->viewer->sceneGroup->addChild(this->scene2->root);
	this->viewer->model = this->model2.get();
	
	this->configurationSpaceScene->model = this->model.get();
	
	if (this->toggleViewAction->isChecked())
	{
		this->toggleView(true);
	}
	else
	{
		this->toggleView(false);
	}
	
	if (path.eval("count(//viewer/swept) > 0").getBoolval())
	{
		this->thread->swept = true;
	}
	
	this->viewer->viewer->setBackgroundColor(SbColor(
		path.eval("number(//viewer/background/r)").getFloatval(0.0f),
		path.eval("number(//viewer/background/g)").getFloatval(0.0f),
		path.eval("number(//viewer/background/b)").getFloatval(0.0f)
	));
	
	if (path.eval("count(//viewer/camera/orthographic) > 0").getBoolval())
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
		path.eval("number(//viewer/camera/position/x)").getFloatval(this->viewer->viewer->getCamera()->position.getValue()[0]),
		path.eval("number(//viewer/camera/position/y)").getFloatval(this->viewer->viewer->getCamera()->position.getValue()[1]),
		path.eval("number(//viewer/camera/position/z)").getFloatval(this->viewer->viewer->getCamera()->position.getValue()[2])
	);
	
	if (path.eval("count(//viewer/camera/target) > 0").getBoolval())
	{
		this->viewer->viewer->getCamera()->pointAt(
			SbVec3f(
				path.eval("number(//viewer/camera/target/x)").getFloatval(0),
				path.eval("number(//viewer/camera/target/y)").getFloatval(0),
				path.eval("number(//viewer/camera/target/z)").getFloatval(0)
			),
			SbVec3f(
				path.eval("number(//viewer/camera/up/x)").getFloatval(0),
				path.eval("number(//viewer/camera/up/y)").getFloatval(0),
				path.eval("number(//viewer/camera/up/z)").getFloatval(1)
			)
		);
	}
	
	this->viewer->viewer->getCamera()->scaleHeight(
		path.eval("number(//viewer/camera/scale)").getFloatval(1.0f)
	);
	
	if (path.eval("count(//viewer/cspace) > 0").getBoolval())
	{
		this->evalAction->setEnabled(true);
		this->savePdfAction->setEnabled(true);
		this->toggleConfigurationSpaceAction->setEnabled(true);
		
		this->configurationSpaceScene->delta = path.eval("number(//viewer/cspace/delta)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//viewer/cspace/delta/@unit)").getStringval())
		{
			this->configurationSpaceScene->delta *= rl::math::DEG2RAD;
		}
		
		this->configurationSpaceScene->x = static_cast< std::size_t >(path.eval("number(//viewer/cspace/x)").getFloatval(0));
		this->configurationSpaceScene->y = static_cast< std::size_t >(path.eval("number(//viewer/cspace/y)").getFloatval(1));
		
		this->configurationSpaceScene->eval();
		
		qreal scale = static_cast< std::size_t >(path.eval("number(//viewer/cspace/scale)").getFloatval(1.0f));
		
		this->configurationSpaceView->setEnabled(true);
		
		rl::math::Vector maximum(this->planner->model->getDof());
		this->planner->model->getMaximum(maximum);
		rl::math::Vector minimum(this->planner->model->getDof());
		this->planner->model->getMinimum(minimum);
		
		this->configurationSpaceView->setSceneRect(
			minimum(this->configurationSpaceScene->x),
			-maximum(this->configurationSpaceScene->y),
			std::abs(maximum(this->configurationSpaceScene->x) - minimum(this->configurationSpaceScene->x)),
			std::abs(maximum(this->configurationSpaceScene->y) - minimum(this->configurationSpaceScene->y))
		);
		
		this->configurationSpaceView->resetMatrix();
		this->configurationSpaceView->scale(scale, scale);
		
		this->configurationSpaceView->adjustSize();
		this->configurationSpaceDockWidget->adjustSize();
		
		this->configurationSpaceDockWidget->setUpdatesEnabled(false);
		this->configurationSpaceDockWidget->setFloating(!this->configurationSpaceDockWidget->isFloating());
		this->configurationSpaceDockWidget->setFloating(!this->configurationSpaceDockWidget->isFloating());
		this->configurationSpaceDockWidget->setUpdatesEnabled(true);
	}
	else
	{
		this->evalAction->setEnabled(false);
		this->savePdfAction->setEnabled(false);
		this->toggleConfigurationSpaceAction->setEnabled(false);
		
		this->configurationSpaceScene->clear();
		this->configurationSpaceDockWidget->hide();
		this->configurationSpaceView->setEnabled(false);
		this->disconnect(this->thread, this->configurationSpaceScene);
	}
	
	this->viewer->drawConfiguration(*this->start);
	
	this->configurationModel->invalidate();
	this->plannerModel->invalidate();
	
	if (!this->wait)
	{
		this->startThread();
	}
}

void
MainWindow::open()
{
	if (NULL != this->planner)
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
MainWindow::reset()
{
	rl::math::Real duration = this->planner->duration;
	
	this->thread->blockSignals(true);
	QCoreApplication::processEvents();
	this->planner->duration = 0;
	this->thread->stop();
	this->planner->duration = duration;
	this->thread->blockSignals(false);
	
	this->planner->reset();
	this->model->reset();
	this->viewer->reset();
	this->configurationSpaceScene->reset();
	
	this->configurationView->setEnabled(true);
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
	this->toggleViewAction->setEnabled(true);
}

void
MainWindow::saveImage()
{
	this->viewer->saveImage("planDemo-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".png");
}

void
MainWindow::savePdf()
{
	QPrinter printer;
	printer.setOutputFileName("planDemo-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".pdf");
	printer.setOutputFormat(QPrinter::PdfFormat);
	printer.setPageSize(QPrinter::A4);
	
	QPainter painter(&printer);
	
	this->configurationSpaceScene->render(&painter);
}

void
MainWindow::saveScene()
{
	this->viewer->saveScene("planDemo-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".wrl");
}

void
MainWindow::setGoalConfiguration()
{
	*this->goal = *this->q;
}

void
MainWindow::setStartConfiguration()
{
	*this->start = *this->q;
}

void
MainWindow::startThread()
{
	this->configurationView->setEnabled(false);
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
	this->toggleViewAction->setEnabled(false);
	
	this->model->reset();
	this->thread->start();
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
	if (this->configurationSpaceView->isEnabled())
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
MainWindow::toggleView(const bool& doOn)
{
	if (doOn)
	{
		this->planner->viewer = this->thread;
		
		if (NULL != this->optimizer)
		{
			this->optimizer->viewer = this->thread;
		}
		
		for (std::vector< boost::shared_ptr< rl::plan::WorkspaceSphereExplorer > >::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
		{
			(*i)->viewer = this->thread;
		}
		
		this->connect(this->thread, this->configurationSpaceScene);
		this->connect(this->thread, this->viewer);
	}
	else
	{
		this->disconnect(this->thread, this->configurationSpaceScene);
		this->disconnect(this->thread, this->viewer);
		
		this->planner->viewer = NULL;
		
		if (NULL != this->optimizer)
		{
			this->optimizer->viewer = NULL;
		}
		
		for (std::vector< boost::shared_ptr< rl::plan::WorkspaceSphereExplorer > >::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
		{
			(*i)->viewer = NULL;
		}
	}
}
