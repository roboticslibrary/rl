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
#include <QApplication>
#include <QDateTime>
#include <QGLWidget>
#include <QHeaderView>
#include <QGroupBox>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSpacerItem>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/Qt/SoQt.h>
#include <rl/math/Constants.h>
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
	dynamicModel(),
	externalTorque(),
	geometryModels(nullptr),
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
	simulationDampingFactor(nullptr),
	simulationDampingValue(static_cast<rl::math::Real>(0.001)),
	simulationDockWidget(new QDockWidget(this)),
	simulationGravity(nullptr),
	simulationGravityValue(rl::math::constants::gravity),
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
	this->dynamicModel = std::dynamic_pointer_cast<rl::mdl::Dynamic>(kinematicFactory.create(QApplication::arguments()[2].toStdString()));
	this->simulationResetQ = rl::math::Vector::Zero(this->dynamicModel->getDofPosition());
	this->simulationResetQd = rl::math::Vector::Zero(this->dynamicModel->getDof());
	this->simulationResetQdd = rl::math::Vector::Zero(this->dynamicModel->getDof());
	
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
	this->positionView->setItemDelegate(this->positionDelegate);
	this->positionView->setModel(this->positionModel);
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
	this->velocityView->setItemDelegate(this->velocityDelegate);
	this->velocityView->setModel(this->velocityModel);
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
	this->accelerationView->setItemDelegate(this->accelerationDelegate);
	this->accelerationView->setModel(this->accelerationModel);
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
	this->torqueView->setItemDelegate(this->torqueDelegate);
	this->torqueView->setModel(this->torqueModel);
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
		this->positionModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		operationalModel,
		SLOT(configurationChanged(const QModelIndex&, const QModelIndex&))
	);
	
	QObject::connect(
		operationalModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		this->positionModel,
		SLOT(operationalChanged(const QModelIndex&, const QModelIndex&))
	);
		
	rl::math::Vector q(this->dynamicModel->getDofPosition());
	q.setZero();
	this->positionModel->setData(q); //TODO?
	
	rl::math::Vector externalTorque = rl::math::Vector::Zero(this->dynamicModel->getDof());
	this->externalTorque = externalTorque;
	this->torqueModel->setData(externalTorque);
	
	QWidget* simulationDockWidgetContent = new QWidget(this->simulationDockWidget);
	QGroupBox* simulationGroup = new QGroupBox("Simulation", simulationDockWidgetContent);
	QVBoxLayout* simulationVerticalLayout = new QVBoxLayout();
	QHBoxLayout* simulationHorizontalLayout = new QHBoxLayout();
	
	this->addDockWidget(Qt::LeftDockWidgetArea, this->simulationDockWidget);
	this->simulationDockWidget->setWindowTitle("Control");
	this->simulationDockWidget->setWidget(simulationDockWidgetContent);
	simulationDockWidgetContent->setLayout(simulationVerticalLayout);
	simulationVerticalLayout->addWidget(simulationGroup);
	simulationGroup->setLayout(simulationHorizontalLayout);
	this->simulationStart = new QPushButton("&Start", simulationGroup);
	this->simulationStart->setEnabled(false);
	this->simulationPause = new QPushButton("&Pause", simulationGroup);
	this->simulationReset = new QPushButton("&Reset", simulationGroup);
	QObject::connect(this->simulationStart, SIGNAL(clicked()), this, SLOT(clickSimulationStart()));
	QObject::connect(this->simulationPause, SIGNAL(clicked()), this, SLOT(clickSimulationPause()));
	QObject::connect(this->simulationReset, SIGNAL(clicked()), this, SLOT(clickSimulationReset()));
	simulationHorizontalLayout->addWidget(this->simulationStart);
	simulationHorizontalLayout->addWidget(this->simulationPause);
	simulationHorizontalLayout->addWidget(this->simulationReset);
	
	QGridLayout* simulationSettingsLayout = new QGridLayout();
	QGroupBox* simulationSettings = new QGroupBox("Settings", simulationDockWidgetContent);
	simulationVerticalLayout->addWidget(simulationSettings);
	simulationSettings->setLayout(simulationSettingsLayout);
	this->simulationGravity = new QDoubleSpinBox(simulationSettings);
	simulationSettingsLayout->addWidget(new QLabel("Gravity [m/s^2]", simulationSettings), 0, 0);
	simulationSettingsLayout->addWidget(this->simulationGravity, 0, 1);
	this->simulationGravity->setRange(0, 100);
	this->simulationGravity->setSingleStep(0.1);
	this->simulationGravity->setDecimals(2);
	this->simulationGravity->setValue(this->simulationGravityValue);
	QObject::connect(this->simulationGravity, SIGNAL(valueChanged(double)), this, SLOT(changeSimulationGravity(double)));
	this->simulationDampingFactor = new QDoubleSpinBox(simulationSettings);
	simulationSettingsLayout->addWidget(new QLabel("Damping Factor", simulationSettings), 1, 0);
	simulationSettingsLayout->addWidget(this->simulationDampingFactor, 1, 1);
	this->simulationDampingFactor->setRange(0, 1.5);
	this->simulationDampingFactor->setSingleStep(0.0001);
	this->simulationDampingFactor->setDecimals(5);
	this->simulationDampingFactor->setValue(this->simulationDampingValue);
	this->simulationDampingFactor->setToolTip(QString("0.0 gives a frictionless system, 1.0 damps almost all motion; try 0.001--0.005 for typical viscous friction"));
	QObject::connect(this->simulationDampingFactor, SIGNAL(valueChanged(double)), this, SLOT(changeSimulationDampingFactor(double)));
	
	QGroupBox* simulationResults = new QGroupBox("Results", simulationDockWidgetContent);
	QGridLayout* simulationResultsLayout = new QGridLayout();
	simulationVerticalLayout->addWidget(simulationResults);
	simulationResults->setLayout(simulationResultsLayout);
	this->simulationTime = new QLineEdit(simulationResults);
	this->simulationResultsEnergy = new QLineEdit(simulationResults);
	this->simulationResultsTime = new QLineEdit(simulationResults);
	simulationResultsLayout->addWidget(new QLabel("Simulated Time [s]", simulationResults), 0, 0);
	simulationResultsLayout->addWidget(this->simulationTime, 0, 1);
	this->simulationTime->setReadOnly(true);
	simulationResultsLayout->addWidget(new QLabel("Frame Computation Time [s]", simulationResults), 1, 0);
	simulationResultsLayout->addWidget(this->simulationResultsTime, 1, 1);
	this->simulationResultsTime->setToolTip(QString("should be under 0.05 s for good interactive results"));
	this->simulationResultsTime->setReadOnly(true);
	simulationResultsLayout->addWidget(new QLabel("Kinetic Energy [kJ]", simulationResults), 2, 0);
	simulationResultsLayout->addWidget(this->simulationResultsEnergy, 2, 1);
	this->simulationResultsEnergy->setReadOnly(true);
	
	this->addDockWidget(Qt::RightDockWidgetArea, this->positionDockWidget);
	this->positionDockWidget->setWidget(this->positionView);
	this->positionDockWidget->setWindowTitle("Position");
	
	this->addDockWidget(Qt::RightDockWidgetArea, this->velocityDockWidget);
	this->velocityDockWidget->setWidget(this->velocityView);
	this->velocityDockWidget->setWindowTitle("Velocity");
	
	this->addDockWidget(Qt::RightDockWidgetArea, this->accelerationDockWidget);
	this->accelerationDockWidget->setWidget(this->accelerationView);
	this->accelerationDockWidget->setWindowTitle("Acceleration");
	
	this->addDockWidget(Qt::LeftDockWidgetArea, this->torqueDockWidget);
	this->torqueDockWidget->setWidget(this->torqueView);
	this->torqueDockWidget->setWindowTitle("Torque");
	
	this->addDockWidget(Qt::BottomDockWidgetArea, this->operationalDockWidget);
	this->operationalDockWidget->setWidget(this->operationalViews);
	this->operationalDockWidget->setWindowTitle("Operational Position");
	
#if QT_VERSION >= 0x050600
	QList<QDockWidget*> resizeDocksWidgets;
	resizeDocksWidgets.append(this->operationalDockWidget);
	resizeDocksWidgets.append(this->simulationDockWidget);
	QList<int> resizeDocksSizes;
	resizeDocksSizes.append(1);
	resizeDocksSizes.append(1);
	this->resizeDocks(resizeDocksWidgets, resizeDocksSizes, Qt::Vertical);
#endif // QT_VERSION
	
	this->statusBar();
	
	this->gradientBackground = new SoGradientBackground();
	this->gradientBackground->ref();
	this->gradientBackground->color0.setValue(0.8f, 0.8f, 0.8f);
	this->gradientBackground->color1.setValue(1.0f, 1.0f, 1.0f);
	this->scene->root->insertChild(this->gradientBackground, 0);
	
	this->viewer = new SoQtExaminerViewer(this, nullptr, true, SoQtFullViewer::BUILD_POPUP);
	this->viewer->setSceneGraph(this->scene->root);
	this->viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	this->viewer->viewAll();
	
	this->resize(1024, 768);
	this->setCentralWidget(this->viewer->getWidget());
	this->setWindowIconText("rlSimulator");
	this->setWindowTitle("rlSimulator");
	
	this->init();
	
	this->server->listen(QHostAddress::Any, 11235);
	this->statusBar()->showMessage("Listening on port 11235");
	
	this->timer.start(this->simulationTimeStep * this->simulationStepsPerFrame * 1000, this);
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
	this->simulationStart->setEnabled(false);
	this->simulationPause->setEnabled(true);
}

void
MainWindow::clickSimulationPause()
{
	this->simulationIsRunning = false;
	this->simulationStart->setEnabled(true);
	this->simulationPause->setEnabled(false);
}

void
MainWindow::clickSimulationReset()
{
	this->simulationIsRunning = false;
	this->simulationTimeElapsed = 0;
	this->simulationTime->setText(QString::number(this->simulationTimeElapsed));
	this->simulationResultsTime->setText("");
	this->simulationResultsEnergy->setText("");
	this->dynamicModel->setPosition(this->simulationResetQ);
	this->dynamicModel->setVelocity(this->simulationResetQd);
	this->dynamicModel->setAcceleration(this->simulationResetQdd);
	this->positionModel->setData(this->simulationResetQ);
	this->simulationStart->setEnabled(true);
	this->simulationPause->setEnabled(false);
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
	if (!this->simulationIsRunning)
	{
		return;
	}
	
	rl::math::Vector3 g(0, 0, this->simulationGravityValue);
	this->dynamicModel->setWorldGravity(g);
	
	std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
	rl::math::Vector q(this->dynamicModel->getDofPosition());
	rl::math::Vector qd(this->dynamicModel->getDof());
	rl::math::Vector qdd(this->dynamicModel->getDof());
	rl::math::Vector torque(this->dynamicModel->getDof());
	rl::math::Matrix M(this->dynamicModel->getDof(), this->dynamicModel->getDof());
	
#if 1
	for (int i = 0; i < this->simulationStepsPerFrame; ++i)
	{
		this->dynamicModel->setTorque(this->externalTorque);
		this->dynamicModel->forwardDynamics();
#if 1
		q = this->dynamicModel->getPosition();
		qd = this->dynamicModel->getVelocity();
		qdd = this->dynamicModel->getAcceleration();
		qdd -= this->simulationDampingValue * qd / this->simulationTimeStep;
		// TODO: this is a very simple integration, slightly better than Euler, could be replaced by a *functional-style* Runge-Kutta
		q += this->simulationTimeStep * (qd + qdd * this->simulationTimeStep / 2);
		qd += this->simulationTimeStep * qdd;
		this->dynamicModel->setPosition(q);
		this->dynamicModel->setVelocity(qd);
		this->dynamicModel->setAcceleration(qdd);
#else
		this->dynamicModel->rungeKuttaNystrom(simulationTimeStep); // its side effects do not work with damping
#endif
	}
	
	q = this->dynamicModel->getPosition();
	qd = this->dynamicModel->getVelocity();
	qdd = this->dynamicModel->getAcceleration();
	torque = this->dynamicModel->getTorque();
	this->dynamicModel->calculateMassMatrix(M);
	this->dynamicModel->setPosition(q);
	this->dynamicModel->setVelocity(qd);
	this->dynamicModel->setAcceleration(qdd);
	this->dynamicModel->setTorque(torque);
#else
	for (int i = 0; i < simulationStepsPerFrame; ++i)
	{
		this->dynamicModel->getPosition(q);
		this->dynamicModel->getVelocity(qd);
		this->dynamicModel->getAcceleration(qdd);
		this->dynamicModel->getTorque(torque);
		
		// V
		rl::math::Vector V(this->dynamicModel->getDof());
		this->dynamicModel->setPosition(q);
		this->dynamicModel->setVelocity(qd);
		this->dynamicModel->calculateCentrifugalCoriolis(V);
		
		// G
		rl::math::Vector G(this->dynamicModel->getDof());
		this->dynamicModel->setPosition(q);
		this->dynamicModel->calculateGravity(G);
		
		// M^{-1}
		rl::math::Matrix invM(this->dynamicModel->getDof(), this->dynamicModel->getDof());
		this->dynamicModel->calculateMassMatrixInverse(invM);
		
		// M^{-1} * ( tau - V - G )
		//qdd = invM * (torque - V - G);
		qdd = invM * (- V - G);
		
		qdd -= simulationDampingValue * qd / simulationTimeStep;
		
		// simple Euler-Cauchy integration
		this->dynamicModel->setAcceleration(qdd);
		q += simulationTimeStep * qd;
		qd += simulationTimeStep * qdd;
		this->dynamicModel->setPosition(q);
		this->dynamicModel->setVelocity(qd);
	}
#endif
	
	this->dynamicModel->forwardPosition();
	std::chrono::steady_clock::time_point stopTime = std::chrono::steady_clock::now();
	this->simulationTimeElapsed += this->simulationStepsPerFrame * this->simulationTimeStep;
	this->simulationTime->setText(QString::number(this->simulationTimeElapsed));
	this->simulationResultsTime->setText(QString::number(std::chrono::duration_cast<std::chrono::duration<double>>(stopTime - startTime).count()));
	rl::math::Real energy = static_cast<rl::math::Real>(0.5) * qd.transpose() * M * qd;
	this->simulationResultsEnergy->setText(QString::number(energy));
	this->positionModel->setData(q);
	this->velocityModel->setData(qd);
	this->accelerationModel->setData(qdd);
}

void
MainWindow::saveImage()
{
	QImage image = static_cast<QGLWidget*>(this->viewer->getGLWidget())->grabFrameBuffer(true);
	image.save("rlSimulator-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".png", "PNG");
}

void
MainWindow::saveScene()
{
	SoOutput output;
	
	if (!output.openFile(QString("rlSimulator-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".wrl").toStdString().c_str()))
	{
		return;
	}
	
	output.setHeaderString("#VRML V2.0 utf8");
	
	SoWriteAction writeAction(&output);
	writeAction.apply(this->scene->root);
	
	output.closeFile();
}
