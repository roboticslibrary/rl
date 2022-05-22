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
#include <Inventor/SoPickedPoint.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <rl/sg/Body.h>
#include <rl/sg/DepthScene.h>
#include <rl/sg/DistanceScene.h>
#include <rl/sg/Model.h>
#include <rl/sg/Shape.h>
#include <rl/sg/SimpleScene.h>
#include <rl/sg/XmlFactory.h>
#include <rl/sg/so/Body.h>
#include <rl/sg/so/Shape.h>

#if QT_VERSION >= 0x050200
#include <QCommandLineParser>
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

#include "BodyDelegate.h"
#include "BodyModel.h"
#include "MainWindow.h"
#include "SoGradientBackground.h"
#include "SoMaterialHighlightRenderAction.h"

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags f) :
	QMainWindow(parent, f),
	collisionScene(),
	viewScene(),
	bodyModel(new BodyModel(this)),
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
	highlightRenderAction(nullptr),
	root(nullptr),
	selected(nullptr),
	selection(nullptr),
	simpleLabel(new QLabel(this)),
	view2collision(),
	viewer(nullptr)
{
	MainWindow::singleton = this;
	
	SoQt::init(this);
	SoDB::init();
	SoGradientBackground::initClass();
	SoMaterialHighlightRenderAction::initClass();
	
	this->root = new SoSeparator();
	this->root->ref();
	
	this->parseCommandLine();
	
	if (this->filename.isEmpty())
	{
		this->filename = QFileDialog::getOpenFileName(this, "", this->filename, "All Formats (*.xml)");
		
		if (this->filename.isEmpty())
		{
			exit(EXIT_FAILURE);
		}
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
	
	if (this->palette().color(QPalette::Window).lightness() < 128)
	{
		this->gradientBackground->color0.setValue(0.0f, 0.0f, 0.0f);
		this->gradientBackground->color1.setValue(0.2f, 0.2f, 0.2f);
	}
	else
	{
		this->gradientBackground->color0.setValue(0.8f, 0.8f, 0.8f);
		this->gradientBackground->color1.setValue(1.0f, 1.0f, 1.0f);
	}
	
	this->root->insertChild(this->gradientBackground, 0);
	
	this->selection = new SoSelection();
	this->selection->addSelectionCallback(MainWindow::selectionCallback, this);
	this->selection->addDeselectionCallback(MainWindow::deselectionCallback, this);
	this->selection->setPickFilterCallback(MainWindow::pickFilterCallback, this);
	this->selection->policy = SoSelection::SINGLE;
	this->selection->addChild(this->viewScene->root);
	this->root->addChild(this->selection);
	
	this->highlightRenderAction = new SoMaterialHighlightRenderAction();
	
	this->viewer = new SoQtExaminerViewer(this, nullptr, true, SoQtFullViewer::BUILD_POPUP);
	this->viewer->setFeedbackVisibility(true);
	this->viewer->setGLRenderAction(this->highlightRenderAction);
	this->viewer->setSceneGraph(this->root);
	this->viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	
	this->setCentralWidget(this->viewer->getWidget());
	this->setFocusProxy(this->viewer->getWidget());
	
	this->setWindowTitle(filename + " - " + this->engine.toUpper() + " - rlCollisionDemo");
	
	rl::sg::XmlFactory factory;
	factory.load(this->filename.toStdString(), this->collisionScene.get());
	factory.load(this->filename.toStdString(), this->viewScene.get());
	
	for (rl::sg::Scene::Iterator i = this->viewScene->begin(), j = this->collisionScene->begin(); i != this->viewScene->end(); ++i, ++j)
	{
		for (rl::sg::Model::Iterator k = (*i)->begin(), l = (*j)->begin(); k != (*i)->end(); ++k, ++l)
		{
			this->view2collision[*k] = *l;
		}
	}
	
	this->viewer->viewAll();
	
	BodyDelegate* bodyDelegate = new BodyDelegate(this);
	
	QTableView* bodyView = new QTableView(this);
#if QT_VERSION >= 0x050000
	bodyView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else // QT_VERSION
	bodyView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif // QT_VERSION
	bodyView->horizontalHeader()->hide();
	bodyView->setAlternatingRowColors(true);
	bodyView->setItemDelegate(bodyDelegate);
	bodyView->setModel(this->bodyModel);
	
	QDockWidget* bodyDockWidget = new QDockWidget(this);
	bodyDockWidget->resize(160, 240);
	bodyDockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
	bodyDockWidget->setFloating(true);
	bodyDockWidget->setWidget(bodyView);
	bodyDockWidget->setWindowTitle("Body");
	
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
	
	if (this->viewScene->getNumModels() > 0)
	{
		rl::sg::Model* model = this->viewScene->getModel(0);
		
		if (model->getNumBodies() > 0)
		{
			rl::sg::Body* body = model->getBody(0);
			this->selection->select(dynamic_cast<rl::sg::so::Body*>(body)->root);
		}
	}
	
	this->test();
}

MainWindow::~MainWindow()
{
	this->root->unref();
	MainWindow::singleton = nullptr;
}

void
MainWindow::changeEvent(QEvent* event)
{
	if (QEvent::PaletteChange == event->type())
	{
		if (this->palette().color(QPalette::Window).lightness() < 128)
		{
			this->gradientBackground->color0.setValue(0.0f, 0.0f, 0.0f);
			this->gradientBackground->color1.setValue(0.2f, 0.2f, 0.2f);
		}
		else
		{
			this->gradientBackground->color0.setValue(0.8f, 0.8f, 0.8f);
			this->gradientBackground->color1.setValue(1.0f, 1.0f, 1.0f);
		}
	}
	
	QMainWindow::changeEvent(event);
}

void
MainWindow::deselectionCallback(void* data, SoPath* path)
{
	MainWindow* mainWindow = static_cast<MainWindow*>(data);
	mainWindow->bodyModel->setBody(nullptr, nullptr);
	mainWindow->selected = nullptr;
	mainWindow->selection->touch();
	mainWindow->test();
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
	
#if QT_VERSION >= 0x050200
	QCommandLineOption engineOption(QStringList("engine"), "Sets collision engine.", engines.join("|"));
	
	QCommandLineParser parser;
	parser.addOption(engineOption);
	const QCommandLineOption helpOption = parser.addHelpOption();
	parser.addPositionalArgument("filename", "", "[filename]");
	
	parser.process(QCoreApplication::arguments());
	
	if (parser.isSet(engineOption))
	{
		QString engine = parser.value(engineOption);
		
		if (!engines.contains(engine, Qt::CaseInsensitive))
		{
			parser.showHelp();
		}
		
		this->engine = engine;
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
	QRegExp engineRegExp("--engine=(" + engines.join("|") + ")");
	QRegExp helpRegExp("--help");
	
	for (int i = 1; i < QApplication::arguments().size(); ++i)
	{
		if (-1 != engineRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->engine = engineRegExp.cap(1);
		}
		else if (-1 != helpRegExp.indexIn(QApplication::arguments()[i]))
		{
			QMessageBox::information(this, "Usage", "rlCollisionDemo [--engine=<" + engines.join("|") + ">] [--help] [filename]");
			exit(0);
		}
		else
		{
			this->filename = QApplication::arguments()[i];
		}
	}
#endif
}

SoPath*
MainWindow::pickFilterCallback(void* data, const SoPickedPoint* pick)
{
	SoFullPath* path = static_cast<SoFullPath*>(pick->getPath());
	path->pop();
	path->pop();
	path->pop();
	return path;
}

void
MainWindow::selectionCallback(void* data, SoPath* path)
{
	MainWindow* mainWindow = static_cast<MainWindow*>(data);
	
	if (path->getTail()->isOfType(SoVRMLTransform::getClassTypeId()))
	{
		SoVRMLTransform* vrmlTransform = static_cast<SoVRMLTransform*>(path->getTail());
		
		if (nullptr != vrmlTransform->getUserData())
		{
			rl::sg::so::Body* view = static_cast<rl::sg::so::Body*>(vrmlTransform->getUserData());
			rl::sg::Body* collision = dynamic_cast<rl::sg::Body*>(mainWindow->view2collision[view]);
			mainWindow->bodyModel->setBody(collision, view);
			mainWindow->selected = view;
		}
	}
	
	mainWindow->selection->touch();
	mainWindow->test();
}

void
MainWindow::test()
{
	this->distanceCoordinate->point.setNum(0);
	this->distanceLineSet->coordIndex.setNum(0);
	this->distanceCoordinate->point.setNum(0);
	
	this->depthCoordinate->point.setNum(0);
	this->depthLineSet->coordIndex.setNum(0);
	this->depthCoordinate->point.setNum(0);
	
	rl::sg::Base* collision = this->view2collision[this->selected];
	
	if (nullptr == this->selected || nullptr == collision)
	{
		this->simpleLabel->hide();
		this->distanceLabel->hide();
		this->depthLabel->hide();
		return;
	}
	
	rl::sg::Body* body = dynamic_cast<rl::sg::Body*>(collision);
	
	if (rl::sg::SimpleScene* simpleScene = dynamic_cast<rl::sg::SimpleScene*>(this->collisionScene.get()))
	{
		std::size_t collisions = 0;
		
		for (rl::sg::Scene::Iterator i = this->collisionScene->begin(); i != this->collisionScene->end(); ++i)
		{
			for (rl::sg::Model::Iterator j = (*i)->begin(); j != (*i)->end(); ++j)
			{
				if (body != *j)
				{
					if (simpleScene->areColliding(body, *j))
					{
						++collisions;
					}
				}
			}
		}
		
		this->simpleLabel->show();
		this->simpleLabel->setText("Collisions: " + QString::number(collisions));
		
		if (collisions > 0)
		{
			this->highlightRenderAction->setDiffuseColor(SbColor(0.8f, 0.0f, 0.0f));
			this->highlightRenderAction->setEmissiveColor(SbColor(0.25f, 0.0f, 0.0f));
			this->highlightRenderAction->setSpecularColor(SbColor(1.0f, 0.0f, 0.0f));
		}
		else
		{
			this->highlightRenderAction->setDiffuseColor(SbColor(0.8f, 0.8f, 0.8f));
			this->highlightRenderAction->setEmissiveColor(SbColor(0.0f, 0.0f, 0.25f));
			this->highlightRenderAction->setSpecularColor(SbColor(0.0f, 0.0f, 1.0f));
		}
	}
	
	if (rl::sg::DistanceScene* distanceScene = dynamic_cast<rl::sg::DistanceScene*>(this->collisionScene.get()))
	{
		rl::math::Vector3 point1 = rl::math::Vector3::Zero();
		rl::math::Vector3 point2 = rl::math::Vector3::Zero();
		rl::math::Real distance = std::numeric_limits<rl::math::Real>::max();
		
		for (rl::sg::Scene::Iterator i = this->collisionScene->begin(); i != this->collisionScene->end(); ++i)
		{
			for (rl::sg::Model::Iterator j = (*i)->begin(); j != (*i)->end(); ++j)
			{
				if (body != *j)
				{
					rl::math::Vector3 tmpPoint1;
					rl::math::Vector3 tmpPoint2;
					rl::math::Real tmpDistance = distanceScene->distance(body, *j, tmpPoint1, tmpPoint2);
					
					if (tmpDistance < distance)
					{
						distance = tmpDistance;
						point1 = tmpPoint1;
						point2 = tmpPoint2;
					}
				}
			}
		}
		
		this->distanceLabel->show();
		this->distanceLabel->setText("Distance: " + QString::number(distance));
		
		if (distance > 0 && distance < std::numeric_limits<rl::math::Real>::max())
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
		rl::math::Vector3 point1 = rl::math::Vector3::Zero();
		rl::math::Vector3 point2 = rl::math::Vector3::Zero();
		rl::math::Real depth = 0;
		
		for (rl::sg::Scene::Iterator i = this->collisionScene->begin(); i != this->collisionScene->end(); ++i)
		{
			for (rl::sg::Model::Iterator j = (*i)->begin(); j != (*i)->end(); ++j)
			{
				if (body != *j)
				{
					rl::math::Vector3 tmpPoint1;
					rl::math::Vector3 tmpPoint2;
					rl::math::Real tmpDepth = depthScene->depth(body, *j, tmpPoint1, tmpPoint2);
					
					if (tmpDepth > depth)
					{
						depth = tmpDepth;
						point1 = tmpPoint1;
						point2 = tmpPoint2;
					}
				}
			}
		}
		
		this->depthLabel->show();
		this->depthLabel->setText("Depth: " + QString::number(depth));
		
		if (depth > 0)
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
