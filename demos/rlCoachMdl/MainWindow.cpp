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
#include <QGLWidget>
#include <QHeaderView>
#include <boost/make_shared.hpp>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/Qt/SoQt.h>
#include <rl/mdl/XmlFactory.h>

#include "ConfigurationDelegate.h"
#include "ConfigurationModel.h"
#include "MainWindow.h"
#include "OperationalDelegate.h"
#include "OperationalModel.h"
#include "Server.h"

MainWindow::MainWindow(QWidget* parent, Qt::WFlags f) :
	QMainWindow(parent, f),
	configurationModels(),
	scene(),
	configurationDelegates(),
	configurationDockWidget(new QDockWidget(this)),
	configurationTabWidget(new QTabWidget(this)),
	configurationViews(),
	operationalDelegates(),
	operationalDockWidget(new QDockWidget(this)),
	operationalTabWidget(new QTabWidget(this)),
	operationalViews(),
	saveImageAction(new QAction(this)),
	saveSceneAction(new QAction(this)),
	server(new Server(this)),
	viewer(NULL)
{
	MainWindow::singleton = this;
	
	SoQt::init(this);
	SoDB::init();
	
	this->scene = boost::make_shared< rl::sg::so::Scene >();
	this->scene->load(QApplication::arguments()[1].toStdString());
	
	rl::mdl::XmlFactory factory;
	
	for (int i = 2; i < QApplication::arguments().size(); ++i)
	{
		this->geometryModels.push_back(this->scene->getModel(i - 2));
		boost::shared_ptr< rl::mdl::Model > kinematicModel;
		kinematicModel.reset(factory.create(QApplication::arguments()[i].toStdString()));
		this->kinematicModels.push_back(kinematicModel);
	}
	
	for (std::size_t i = 0; i < this->kinematicModels.size(); ++i)
	{
		ConfigurationDelegate* configurationDelegate = new ConfigurationDelegate(this);
		configurationDelegate->id = i;
		this->configurationDelegates.push_back(configurationDelegate);
		
		ConfigurationModel* configurationModel = new ConfigurationModel(this);
		configurationModel->id = i;
		this->configurationModels.push_back(configurationModel);
		
		QTableView* configurationView = new QTableView(this);
		configurationView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
		configurationView->horizontalHeader()->hide();
		configurationView->setAlternatingRowColors(true);
		configurationView->setItemDelegate(configurationDelegate);
		configurationView->setModel(configurationModel);
		configurationView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
		this->configurationViews.push_back(configurationView);
		
		this->configurationTabWidget->addTab(configurationView, QString::number(i));
		
		OperationalDelegate* operationalDelegate = new OperationalDelegate(this);
		this->operationalDelegates.push_back(operationalDelegate);
		
		OperationalModel* operationalModel = new OperationalModel(this);
		operationalModel->id = i;
		this->operationalModels.push_back(operationalModel);
		
		QTableView* operationalView = new QTableView(this);
		operationalView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
		operationalView->setAlternatingRowColors(true);
		operationalView->setItemDelegate(operationalDelegate);
		operationalView->setModel(operationalModel);
		operationalView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
		this->operationalViews.push_back(operationalView);
		
		this->operationalTabWidget->addTab(operationalView, QString::number(i));
		
		QObject::connect(
			configurationModel,
			SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
			operationalModel,
			SLOT(configurationChanged(const QModelIndex&, const QModelIndex&))
		);
		
		QObject::connect(
			operationalModel,
			SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
			configurationModel,
			SLOT(operationalChanged(const QModelIndex&, const QModelIndex&))
		);
		
		rl::math::Vector q(this->kinematicModels[i]->getDof());
		q.setZero();
		configurationModel->setData(q);
	}
	
	this->addDockWidget(Qt::LeftDockWidgetArea, configurationDockWidget);
	this->configurationDockWidget->resize(160, 320);
	this->configurationDockWidget->setWidget(this->configurationTabWidget);
	this->configurationDockWidget->setWindowTitle("Configuration");
	
	this->addDockWidget(Qt::LeftDockWidgetArea, operationalDockWidget);
	this->operationalDockWidget->resize(160, 320);
	this->operationalDockWidget->setWidget(this->operationalTabWidget);
	this->operationalDockWidget->setWindowTitle("Operational");
	
	this->viewer = new SoQtExaminerViewer(this, NULL, true, SoQtFullViewer::BUILD_POPUP);
	this->viewer->setSceneGraph(this->scene->root);
	this->viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	this->viewer->viewAll();
	
	this->resize(1024, 768);
	this->setCentralWidget(this->viewer->getWidget());
	this->setWindowIconText("rlCoachMdl");
	this->setWindowTitle("rlCoachMdl");
	
	this->init();
	
	this->server->listen(QHostAddress::Any, 11235);
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
MainWindow::init()
{
	this->configurationDockWidget->toggleViewAction()->setShortcut(QKeySequence("F5"));
	this->addAction(this->configurationDockWidget->toggleViewAction());
	
	this->saveImageAction->setShortcut(QKeySequence("Return"));
	QObject::connect(this->saveImageAction, SIGNAL(triggered()), this, SLOT(saveImage()));
	this->addAction(this->saveImageAction);
	
	this->saveSceneAction->setShortcut(QKeySequence("Ctrl+Return"));
	QObject::connect(this->saveSceneAction, SIGNAL(triggered()), this, SLOT(saveScene()));
	this->addAction(this->saveSceneAction);
}

void
MainWindow::saveImage()
{
	QImage image = static_cast< QGLWidget* >(this->viewer->getGLWidget())->grabFrameBuffer(true);
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
