//
// Copyright (c) 2009, Andre Gaschler
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
#include <iostream>
#include <QApplication>
#include <QDateTime>
#include <QGLWidget>
#include <QHeaderView>
#include <QGroupBox>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSpacerItem>
#include <QVBoxLayout>
#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/Qt/SoQt.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/sg/Body.h>
#include <rl/sg/XmlFactory.h>

#include "ConfigurationDelegate.h"
#include "ConfigurationModel.h"
#include "MainWindow.h"
#include "OperationalDelegate.h"
#include "OperationalModel.h"
#include "Server.h"
#include "SoGradientBackground.h"

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags f) :
	QMainWindow(parent, f),
	accelerationModel(nullptr),
	externalTorque(),
	geometryModels(nullptr),
	kinematicModels(),
	operationalModel(nullptr),
	positionModel(nullptr),
	scene(),
	torqueModel(nullptr),
	velocityModel(nullptr),
	accelerationDelegate(nullptr),
	accelerationDockWidget(new QDockWidget(this)),
	accelerationView(nullptr),
	gradientBackground(),
	operationalDelegate(nullptr),
	operationalDockWidget(new QDockWidget(this)),
	operationalViews(nullptr),
	positionDelegate(nullptr),
	positionDockWidget(new QDockWidget(this)),
	positionView(nullptr),
	saveImageAction(new QAction(this)),
	saveSceneAction(new QAction(this)),
	server(new Server(this)),
	serverConnectionStatus(nullptr),
	simulationDampingFactor(nullptr),
	simulationDampingValue(static_cast<rl::math::Real>(0.001)),
	simulationDockWidget(new QDockWidget(this)),
	simulationGravity(nullptr),
	simulationGravityValue(rl::math::GRAVITY),
	simulationIsRunning(true),
	simulationPause(nullptr),
	simulationReset(nullptr),
	simulationResultsEnergy(nullptr),
	simulationResultsTime(nullptr),
	simulationResetQ(),
	simulationResetQd(),
	simulationResetQdd(),
	simulationStart(nullptr),
	simulationStepsPerFrame(50),
	simulationTime(nullptr),
	simulationTimeElapsed(0),
	simulationTimeStep(static_cast<rl::math::Real>(0.001)),
	timer(),
	torqueDelegate(nullptr),
	torqueDockWidget(new QDockWidget(this)),
	torqueView(nullptr),
	velocityDelegate(nullptr),
	velocityDockWidget(new QDockWidget(this)),
	velocityView(nullptr),
	viewer(nullptr)
{
	MainWindow::singleton = this;
	
	SoQt::init(this);
	SoDB::init();
	SoGradientBackground::initClass();
	
	this->scene = std::make_shared<rl::sg::so::Scene>();
	rl::sg::XmlFactory geometryFactory;
	geometryFactory.load(QApplication::arguments()[1].toStdString(), this->scene.get());
	
	rl::mdl::XmlFactory kinematicFactory;
	
	this->geometryModels = this->scene->getModel(0);
	this->kinematicModels.reset(kinematicFactory.create(QApplication::arguments()[2].toStdString()));
	this->simulationResetQ = rl::math::Vector::Zero(kinematicModels->getDof());
	this->simulationResetQd = rl::math::Vector::Zero(kinematicModels->getDof());
	this->simulationResetQdd = rl::math::Vector::Zero(kinematicModels->getDof());
	
	this->positionDelegate = new PositionDelegate(this);
	this->positionModel = new PositionModel(this);
	this->positionView = new QTableView(this);
#if QT_VERSION >= 0x050000
	this->positionView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	this->positionView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	this->positionView->horizontalHeader()->hide();
	this->positionView->setAlternatingRowColors(true);
	this->positionView->setItemDelegate(positionDelegate);
	this->positionView->setModel(positionModel);
#if QT_VERSION >= 0x050000
	this->positionView->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
#else // QT_VERSION
	this->positionView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
#endif // QT_VERSION
	
	this->velocityDelegate = new VelocityDelegate(this);
	this->velocityModel = new VelocityModel(this);
	this->velocityView = new QTableView(this);
#if QT_VERSION >= 0x050000
	this->velocityView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	this->velocityView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	this->velocityView->horizontalHeader()->hide();
	this->velocityView->setAlternatingRowColors(true);
	this->velocityView->setItemDelegate(velocityDelegate);
	this->velocityView->setModel(velocityModel);
#if QT_VERSION >= 0x050000
	this->velocityView->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
#else // QT_VERSION
	this->velocityView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
#endif // QT_VERSION
	
	this->accelerationDelegate = new AccelerationDelegate(this);
	this->accelerationModel = new AccelerationModel(this);
	this->accelerationView = new QTableView(this);
#if QT_VERSION >= 0x050000
	this->accelerationView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	this->accelerationView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	this->accelerationView->horizontalHeader()->hide();
	this->accelerationView->setAlternatingRowColors(true);
	this->accelerationView->setItemDelegate(accelerationDelegate);
	this->accelerationView->setModel(accelerationModel);
#if QT_VERSION >= 0x050000
	this->accelerationView->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
#else // QT_VERSION
	this->accelerationView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
#endif // QT_VERSION
	
	this->torqueDelegate = new TorqueDelegate(this);
	this->torqueModel = new TorqueModel(this);
	this->torqueView = new QTableView(this);
#if QT_VERSION >= 0x050000
	this->torqueView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	this->torqueView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	this->torqueView->horizontalHeader()->hide();
	this->torqueView->setAlternatingRowColors(true);
	this->torqueView->setItemDelegate(torqueDelegate);
	this->torqueView->setModel(torqueModel);
#if QT_VERSION >= 0x050000
	this->torqueView->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
#else // QT_VERSION
	this->torqueView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
#endif // QT_VERSION
		
	OperationalDelegate* operationalDelegate = new OperationalDelegate(this);
	this->operationalDelegate = operationalDelegate;
		
	OperationalModel* operationalModel = new OperationalModel(this);
	this->operationalModel = operationalModel;
		
	QTableView* operationalView = new QTableView(this);
#if QT_VERSION >= 0x050000
	operationalView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	operationalView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	operationalView->setAlternatingRowColors(true);
	operationalView->setItemDelegate(operationalDelegate);
	operationalView->setModel(operationalModel);
#if QT_VERSION >= 0x050000
	operationalView->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
#else // QT_VERSION
	operationalView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
#endif // QT_VERSION
	this->operationalViews = operationalView;
		
	QObject::connect(
		positionModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		operationalModel,
		SLOT(configurationChanged(const QModelIndex&, const QModelIndex&))
	);
	
	QObject::connect(
		operationalModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		positionModel,
		SLOT(operationalChanged(const QModelIndex&, const QModelIndex&))
	);
		
	rl::math::Vector q(this->kinematicModels->getDof());
	q.setZero();
	positionModel->setData(q); //TODO?
	
	rl::math::Vector externalTorque = rl::math::Vector::Zero(this->kinematicModels->getDof());
	this->externalTorque = externalTorque;
	torqueModel->setData(externalTorque);
	
	QWidget* simulationDockWidgetContent = new QWidget(simulationDockWidget);
	QGroupBox* simulationGroup = new QGroupBox("Simulation", simulationDockWidgetContent);
	QVBoxLayout* simulationVerticalLayout = new QVBoxLayout();
	QHBoxLayout* simulationHorizontalLayout = new QHBoxLayout();
	
	this->addDockWidget(Qt::LeftDockWidgetArea, simulationDockWidget);
	this->simulationDockWidget->setWindowTitle("Control");
	simulationDockWidget->setWidget(simulationDockWidgetContent);
	simulationDockWidgetContent->setLayout(simulationVerticalLayout);
	simulationVerticalLayout->addWidget(simulationGroup);
	simulationGroup->setLayout(simulationHorizontalLayout);
	simulationStart = new QPushButton("&Start", simulationGroup);
	simulationPause = new QPushButton("&Pause", simulationGroup);
	simulationReset = new QPushButton("&Reset", simulationGroup);
	QObject::connect(simulationStart, SIGNAL(clicked()), this, SLOT(clickSimulationStart()));
	QObject::connect(simulationPause, SIGNAL(clicked()), this, SLOT(clickSimulationPause()));
	QObject::connect(simulationReset, SIGNAL(clicked()), this, SLOT(clickSimulationReset()));
	simulationHorizontalLayout->addWidget(simulationStart);
	simulationHorizontalLayout->addWidget(simulationPause);
	simulationHorizontalLayout->addWidget(simulationReset);
	
	QGridLayout* simulationSettingsLayout = new QGridLayout();
	QGroupBox* simulationSettings = new QGroupBox("Settings", simulationDockWidgetContent);
	simulationVerticalLayout->addWidget(simulationSettings);
	simulationSettings->setLayout(simulationSettingsLayout);
	simulationGravity = new QDoubleSpinBox(simulationSettings);
	simulationSettingsLayout->addWidget(new QLabel("Gravity [m/s^2]", simulationSettings), 0, 0);
	simulationSettingsLayout->addWidget(simulationGravity, 0, 1);
	simulationGravity->setRange(0, 100);
	simulationGravity->setSingleStep(0.1);
	simulationGravity->setDecimals(2);
	simulationGravity->setValue(simulationGravityValue);
	QObject::connect(simulationGravity, SIGNAL(valueChanged(double)), this, SLOT(changeSimulationGravity(double)));
	simulationDampingFactor = new QDoubleSpinBox(simulationSettings);
	simulationSettingsLayout->addWidget(new QLabel("Damping Factor", simulationSettings), 1, 0);
	simulationSettingsLayout->addWidget(simulationDampingFactor, 1, 1);
	simulationDampingFactor->setRange(0, 1.5);
	simulationDampingFactor->setSingleStep(0.0001);
	simulationDampingFactor->setDecimals(5);
	simulationDampingFactor->setValue(simulationDampingValue);
	simulationDampingFactor->setToolTip(QString("0.0 gives a frictionless system, 1.0 damps almost all motion; try 0.001--0.005 for typical viscous friction"));
	QObject::connect(simulationDampingFactor, SIGNAL(valueChanged(double)), this, SLOT(changeSimulationDampingFactor(double)));
	
	QGroupBox* simulationResults = new QGroupBox("Results", simulationDockWidgetContent);
	QGridLayout* simulationResultsLayout = new QGridLayout();
	simulationVerticalLayout->addWidget(simulationResults);
	simulationResults->setLayout(simulationResultsLayout);
	this->simulationTime = new QLineEdit(simulationResults);
	this->simulationResultsEnergy = new QLineEdit(simulationResults);
	this->simulationResultsTime = new QLineEdit(simulationResults);
	simulationResultsLayout->addWidget(new QLabel("Simulated Time [s]", simulationResults), 0, 0);
	simulationResultsLayout->addWidget(simulationTime, 0, 1);
	simulationTime->setReadOnly(true);
	simulationResultsLayout->addWidget(new QLabel("Frame Computation Time [s]", simulationResults), 1, 0);
	simulationResultsLayout->addWidget(simulationResultsTime, 1, 1);
	simulationResultsTime->setToolTip(QString("should be under 0.05 s for good interactive results"));
	simulationResultsTime->setReadOnly(true);
	simulationResultsLayout->addWidget(new QLabel("Kinetic Energy [kJ]", simulationResults), 2, 0);
	simulationResultsLayout->addWidget(simulationResultsEnergy, 2, 1);
	simulationResultsEnergy->setReadOnly(true);
	
	QGroupBox* serverBox = new QGroupBox("rlCoach Server", simulationDockWidgetContent);
	QGridLayout* serverBoxLayout = new QGridLayout();
	serverBox->setLayout(serverBoxLayout);
	simulationVerticalLayout->addWidget(serverBox);
	this->serverConnectionStatus = new QLineEdit(serverBox);
	serverBoxLayout->addWidget(new QLabel("Connection Status", serverBox), 0, 0);
	serverBoxLayout->addWidget(serverConnectionStatus, 0, 1);
	serverConnectionStatus->setReadOnly(true);
	//simulationVerticalLayout->addItem(new QSpacerItem(20, 10, QSizePolicy::Minimum, QSizePolicy::Expanding));
	
	this->addDockWidget(Qt::RightDockWidgetArea, positionDockWidget);
	this->positionDockWidget->setWidget(this->positionView);
	this->positionDockWidget->setWindowTitle("Position");
	
	this->addDockWidget(Qt::RightDockWidgetArea, velocityDockWidget);
	this->velocityDockWidget->setWidget(this->velocityView);
	this->velocityDockWidget->setWindowTitle("Velocity");
	
	this->addDockWidget(Qt::RightDockWidgetArea, accelerationDockWidget);
	this->accelerationDockWidget->setWidget(this->accelerationView);
	this->accelerationDockWidget->setWindowTitle("Acceleration");
	
	this->addDockWidget(Qt::LeftDockWidgetArea, torqueDockWidget);
	this->torqueDockWidget->setWidget(this->torqueView);
	this->torqueDockWidget->setWindowTitle("Motor Torque");
	
	this->addDockWidget(Qt::BottomDockWidgetArea, operationalDockWidget);
	this->operationalDockWidget->setWidget(this->operationalViews);
	this->operationalDockWidget->setWindowTitle("Operational Position");
	
	this->gradientBackground = new SoGradientBackground();
	this->gradientBackground->ref();
	this->gradientBackground->color0.setValue(0.8f, 0.8f, 0.8f);
	this->gradientBackground->color1.setValue(1.0f, 1.0f, 1.0f);
	this->scene->root->insertChild(this->gradientBackground, 0);
	
	QWidget* viewerContainer = new QWidget(this);
	this->viewer = new SoQtExaminerViewer(viewerContainer, nullptr, true, SoQtFullViewer::BUILD_POPUP);
	this->viewer->setSceneGraph(this->scene->root);
	this->viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	this->viewer->viewAll();
	
	this->setCentralWidget(viewerContainer);
	//this->viewer->getWidget()->resize(400, 300);
	this->resize(1000, 750);
	this->setWindowIconText("rlSimulator");
	this->setWindowTitle("rlSimulator");
	
	this->init();
	
	this->server->listen(QHostAddress::Any, 11235);
	this->setServerConnectionStatus("Listening on port 11235");
	
	this->timer.start(simulationTimeStep * simulationStepsPerFrame * 1000.f, this);
}

MainWindow::~MainWindow()
{
	this->gradientBackground->unref();
	MainWindow::singleton = nullptr;
}

void 
MainWindow::changeSimulationGravity(double value)
{
	this->simulationGravityValue = value;
}

void 
MainWindow::changeSimulationDampingFactor(double value)
{
	this->simulationDampingValue = value;
}

void 
MainWindow::clickSimulationStart()
{
	this->simulationIsRunning = true;
}

void
MainWindow::clickSimulationPause()
{
	this->simulationIsRunning = false;
}

void
MainWindow::clickSimulationReset()
{
	this->simulationIsRunning = false;
	this->simulationTimeElapsed = 0;
	this->simulationTime->setText(QString::number(this->simulationTimeElapsed));
	this->simulationResultsTime->setText("");
	this->simulationResultsEnergy->setText("");
	kinematicModels->setPosition(simulationResetQ);
	kinematicModels->setVelocity(simulationResetQd);
	kinematicModels->setAcceleration(simulationResetQdd);
	positionModel->setData(simulationResetQ);
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
MainWindow::init()
{
	this->positionDockWidget->toggleViewAction()->setShortcut(QKeySequence("F5"));
	this->addAction(this->positionDockWidget->toggleViewAction());
	
	this->saveImageAction->setShortcut(QKeySequence("Return"));
	QObject::connect(this->saveImageAction, SIGNAL(triggered()), this, SLOT(saveImage()));
	this->addAction(this->saveImageAction);
	
	this->saveSceneAction->setShortcut(QKeySequence("Ctrl+Return"));
	QObject::connect(this->saveSceneAction, SIGNAL(triggered()), this, SLOT(saveScene()));
	this->addAction(this->saveSceneAction);
}

void
MainWindow::timerEvent(QTimerEvent *event)
{
	if (!simulationIsRunning)
	{
		return;
	}
	
	rl::mdl::Dynamic* dynamic = dynamic_cast<rl::mdl::Dynamic*>(this->kinematicModels.get());
	
	rl::math::Vector3 g(0, 0, this->simulationGravityValue);
	dynamic->setWorldGravity(g);
	
	std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
	rl::math::Vector q(dynamic->getDof());
	rl::math::Vector qd(dynamic->getDof());
	rl::math::Vector qdd(dynamic->getDof());
	rl::math::Vector torque(dynamic->getDof());
	rl::math::Matrix M(dynamic->getDof(), dynamic->getDof());
	
#if 1 
	for (int i = 0; i < simulationStepsPerFrame; ++i)
	{
		dynamic->setTorque(externalTorque);
		dynamic->forwardDynamics();
#if 1
		q = dynamic->getPosition();
		qd = dynamic->getVelocity();
		qdd = dynamic->getAcceleration();
		qdd -= simulationDampingValue * qd / simulationTimeStep;
		// TODO: this is a very simple integration, slightly better than Euler, could be replaced by a *functional-style* Runge-Kutta
		q += simulationTimeStep * (qd + qdd * simulationTimeStep / 2);
		qd += simulationTimeStep * qdd;
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->setAcceleration(qdd);
#else
		dynamic->rungeKuttaNystrom(simulationTimeStep); // its side effects do not work with damping 
#endif
	}
	
	q = dynamic->getPosition();
	qd = dynamic->getVelocity();
	qdd = dynamic->getAcceleration();
	torque = dynamic->getTorque();
	dynamic->calculateMassMatrix(M);
	dynamic->setPosition(q);
	dynamic->setVelocity(qd);
	dynamic->setAcceleration(qdd);
	dynamic->setTorque(torque);
#else
	for (int i = 0; i < simulationStepsPerFrame; ++i)
	{
		dynamic->getPosition(q);
		dynamic->getVelocity(qd);
		dynamic->getAcceleration(qdd);
		dynamic->getTorque(torque);
		
		// V
		rl::math::Vector V(dynamic->getDof());
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
		dynamic->calculateCentrifugalCoriolis(V);
		
		// G
		rl::math::Vector G(dynamic->getDof());
		dynamic->setPosition(q);
		dynamic->calculateGravity(G);
		
		// M^{-1}
		rl::math::Matrix invM(dynamic->getDof(), dynamic->getDof());
		dynamic->calculateMassMatrixInverse(invM);
		
		// M^{-1} * ( tau - V - G )
		//qdd = invM * (torque - V - G);
		qdd = invM * (- V - G);
		
		qdd -= simulationDampingValue * qd / simulationTimeStep;
		
		// simple Euler-Cauchy integration
		dynamic->setAcceleration(qdd);
		q += simulationTimeStep * qd;
		qd += simulationTimeStep * qdd;
		dynamic->setPosition(q);
		dynamic->setVelocity(qd);
	}
#endif
	
	dynamic->forwardPosition();
	std::chrono::steady_clock::time_point stopTime = std::chrono::steady_clock::now();
	this->simulationTimeElapsed += simulationStepsPerFrame * simulationTimeStep;
	this->simulationTime->setText(QString::number(this->simulationTimeElapsed));
	this->simulationResultsTime->setText(QString::number(std::chrono::duration_cast<std::chrono::duration<double>>(stopTime - startTime).count()));
	rl::math::Real energy = 0.5 * qd.transpose() * M * qd;
	this->simulationResultsEnergy->setText(QString::number(energy));
	this->positionModel->setData(q);
	this->velocityModel->setData(qd);
	this->accelerationModel->setData(qdd);
}

void
MainWindow::saveImage()
{
	QImage image = static_cast<QGLWidget*>(this->viewer->getGLWidget())->grabFrameBuffer(true);
	image.save("coach-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".png", "PNG");
}

void
MainWindow::saveScene()
{
	SoOutput output;
	
	if (!output.openFile(QString("coach-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".wrl").toStdString().c_str()))
	{
		return;
	}
	
	output.setHeaderString("#VRML V2.0 utf8");
	
	SoWriteAction writeAction(&output);
	writeAction.apply(this->scene->root);
	
	output.closeFile();
}

void 
MainWindow::setServerConnectionStatus(QString str)
{
	this->serverConnectionStatus->setText(str);
}
