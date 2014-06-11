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

#include <QCoreApplication>
#include <QDateTime>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QGLWidget>
#include <QMessageBox>
#include <QMimeData>
#include <QUrl>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Qt/SoQt.h>

#include "MainWindow.h"

MainWindow::MainWindow(QWidget* parent, Qt::WFlags f) :
	QMainWindow(parent, f),
	displayMenu(NULL),
	fileMenu(NULL),
	filename(),
	offscreenRenderer(NULL),
	offscreenRoot(NULL),
	root(NULL),
	viewer(NULL),
	widget(new QWidget(this))
{
	SoQt::init(this);
	SoDB::init();
	
	QGLFormat format;
	format.setAlpha(true);
	format.setSampleBuffers(true);
	QGLFormat::setDefaultFormat(format);
	
	this->setAcceptDrops(true);
	
	this->viewer = new SoQtExaminerViewer(this->widget);
	this->viewer->setDecoration(false);
	this->viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	
	this->offscreenRenderer = new SoOffscreenRenderer(this->viewer->getViewportRegion());
	
	this->setCentralWidget(this->widget);
	this->setFocusProxy(this->viewer->getWidget());
	
	this->init();
	
	this->resize(800, 600);
	
	this->offscreenRoot = new SoSeparator();
	this->offscreenRoot->ref();
	
	if (QCoreApplication::arguments().size() > 1)
	{
		this->filename = QCoreApplication::arguments()[1];
	}
	
	if (!this->filename.isEmpty())
	{
		this->load(this->filename);
	}
	
	this->viewer->viewAll();
}

MainWindow::~MainWindow()
{
	if (NULL != this->root)
	{
		this->root->unref();
	}
	
	if (NULL != this->offscreenRoot)
	{
		this->offscreenRoot->unref();
	}
	
	if (NULL != this->offscreenRenderer)
	{
		delete this->offscreenRenderer;
	}
}

void
MainWindow::dragEnterEvent(QDragEnterEvent* event)
{
	event->acceptProposedAction();
}

void
MainWindow::dropEvent(QDropEvent* event)
{
	if (event->mimeData()->hasUrls())
	{
		if (event->mimeData()->urls().size() > 0)
		{
			QString filename = event->mimeData()->urls()[0].toLocalFile();
			this->load(filename);
			event->acceptProposedAction();
		}
	}
}

void
MainWindow::init()
{
	this->fileMenu = this->menuBar()->addMenu("File");
	
	QAction* openAction = new QAction("Open...", this);
	openAction->setShortcut(QKeySequence::Open);
	QObject::connect(openAction, SIGNAL(triggered()), this, SLOT(open()));
	this->fileMenu->addAction(openAction);
	
	this->fileMenu->addSeparator();
	
	QAction* reloadAction = new QAction("Reload", this);
	reloadAction->setShortcut(QKeySequence(QKeySequence::Refresh));
	QObject::connect(reloadAction, SIGNAL(triggered()), this, SLOT(reload()));
	this->addAction(reloadAction);
	this->fileMenu->addAction(reloadAction);
	
	this->fileMenu->addSeparator();
	
	QAction* saveImageWithoutAlphaAction = new QAction("Save Image w/o Alpha", this);
	saveImageWithoutAlphaAction->setShortcut(QKeySequence("Return"));
	QObject::connect(saveImageWithoutAlphaAction, SIGNAL(triggered()), this, SLOT(saveImageWithoutAlpha()));
	this->addAction(saveImageWithoutAlphaAction);
	this->fileMenu->addAction(saveImageWithoutAlphaAction);
		
	QAction* saveImageWithAlphaAction = new QAction("Save Image w/ Alpha", this);
	saveImageWithAlphaAction->setShortcut(QKeySequence("Shift+Return"));
	QObject::connect(saveImageWithAlphaAction, SIGNAL(triggered()), this, SLOT(saveImageWithAlpha()));
	this->addAction(saveImageWithAlphaAction);
	this->fileMenu->addAction(saveImageWithAlphaAction);
	
	QAction* saveImageOffscreenAction = new QAction("Save Image Offscreen", this);
	saveImageOffscreenAction->setShortcut(QKeySequence("Backspace"));
	QObject::connect(saveImageOffscreenAction, SIGNAL(triggered()), this, SLOT(saveImageOffscreen()));
	this->addAction(saveImageOffscreenAction);
	this->fileMenu->addAction(saveImageOffscreenAction);
	
	this->fileMenu->addSeparator();
	
	QAction* exitAction = new QAction("Exit", this);
	QObject::connect(exitAction, SIGNAL(triggered()), qApp, SLOT(quit()));
	this->fileMenu->addAction(exitAction);
	
	this->viewMenu = this->menuBar()->addMenu("View");
	
	QActionGroup* viewActionGroup = new QActionGroup(this);
	QObject::connect(viewActionGroup, SIGNAL(triggered(QAction*)), this, SLOT(selectView(QAction*)));
	
	QAction* viewTopAction = new QAction("Top", this);
	viewTopAction->setData(VIEW_TOP);
	viewActionGroup->addAction(viewTopAction);
	this->viewMenu->addAction(viewTopAction);
	
	QAction* viewBottomAction = new QAction("Bottom", this);
	viewBottomAction->setData(VIEW_BOTTOM);
	viewActionGroup->addAction(viewBottomAction);
	this->viewMenu->addAction(viewBottomAction);
	
	QAction* viewFrontAction = new QAction("Front", this);
	viewFrontAction->setData(VIEW_FRONT);
	viewActionGroup->addAction(viewFrontAction);
	this->viewMenu->addAction(viewFrontAction);
	
	QAction* viewBackAction = new QAction("Back", this);
	viewBackAction->setData(VIEW_BACK);
	viewActionGroup->addAction(viewBackAction);
	this->viewMenu->addAction(viewBackAction);
	
	QAction* viewLeftAction = new QAction("Left", this);
	viewLeftAction->setData(VIEW_LEFT);
	viewActionGroup->addAction(viewLeftAction);
	this->viewMenu->addAction(viewLeftAction);
	
	QAction* viewRightAction = new QAction("Right", this);
	viewRightAction->setData(VIEW_RIGHT);
	viewActionGroup->addAction(viewRightAction);
	this->viewMenu->addAction(viewRightAction);
	
	QAction* viewTopBackLeftAction = new QAction("Top Back Left", this);
	viewTopBackLeftAction->setData(VIEW_TOP_BACK_LEFT);
	viewActionGroup->addAction(viewTopBackLeftAction);
	this->viewMenu->addAction(viewTopBackLeftAction);
	
	QAction* viewTopBackRightAction = new QAction("Top Back Right", this);
	viewTopBackRightAction->setData(VIEW_TOP_BACK_RIGHT);
	viewActionGroup->addAction(viewTopBackRightAction);
	this->viewMenu->addAction(viewTopBackRightAction);
	
	QAction* viewTopFrontLeftAction = new QAction("Top Front Left", this);
	viewTopFrontLeftAction->setData(VIEW_TOP_FRONT_LEFT);
	viewActionGroup->addAction(viewTopFrontLeftAction);
	this->viewMenu->addAction(viewTopFrontLeftAction);
	
	QAction* viewTopFrontRightAction = new QAction("Top Front Right", this);
	viewTopFrontRightAction->setData(VIEW_TOP_FRONT_RIGHT);
	viewActionGroup->addAction(viewTopFrontRightAction);
	this->viewMenu->addAction(viewTopFrontRightAction);
	
	this->displayMenu = this->menuBar()->addMenu("Display");
	
	QAction* toggleAxisCrossAction = new QAction("Axis Cross", this);
	toggleAxisCrossAction->setCheckable(true);
	toggleAxisCrossAction->setShortcut(QKeySequence("Space"));
	QObject::connect(toggleAxisCrossAction, SIGNAL(triggered()), this, SLOT(toggleAxisCross()));
	this->addAction(toggleAxisCrossAction);
	this->displayMenu->addAction(toggleAxisCrossAction);
	
	this->displayMenu->addSeparator();
	
	QAction* toggleFullScreenAction = new QAction("Full Screen", this);
	toggleFullScreenAction->setShortcut(QKeySequence("Alt+Return"));
	QObject::connect(toggleFullScreenAction, SIGNAL(triggered()), this, SLOT(toggleFullScreen()));
	this->displayMenu->addAction(toggleFullScreenAction);
	this->addAction(toggleFullScreenAction);
	
	this->displayMenu->addSeparator();
	
	QActionGroup* cameraActionGroup = new QActionGroup(this);
	cameraActionGroup->setExclusive(true);
	QObject::connect(cameraActionGroup, SIGNAL(triggered(QAction*)), this, SLOT(selectCamera(QAction*)));
	
	QAction* cameraOrthogonalAction = new QAction("Orthogonal Camera", this);
	cameraOrthogonalAction->setData(CAMERA_ORTHOGONAL);
	cameraActionGroup->addAction(cameraOrthogonalAction);
	this->displayMenu->addAction(cameraOrthogonalAction);
	
	QAction* cameraPerspectiveAction = new QAction("Perspective Camera", this);
	cameraPerspectiveAction->setData(CAMERA_PERSPECTIVE);
	cameraActionGroup->addAction(cameraPerspectiveAction);
	this->displayMenu->addAction(cameraPerspectiveAction);
	
	this->displayMenu->addSeparator();
	
	QActionGroup* backgroundActionGroup = new QActionGroup(this);
	backgroundActionGroup->setExclusive(true);
	QObject::connect(backgroundActionGroup, SIGNAL(triggered(QAction*)), this, SLOT(selectBackground(QAction*)));
	
	QAction* backgroundBlackAction = new QAction("Black Background", this);
	backgroundBlackAction->setData(BACKGROUND_BLACK);
	backgroundActionGroup->addAction(backgroundBlackAction);
	this->displayMenu->addAction(backgroundBlackAction);
	
	QAction* backgroundWhiteAction = new QAction("White Background", this);
	backgroundWhiteAction->setData(BACKGROUND_WHITE);
	backgroundActionGroup->addAction(backgroundWhiteAction);
	this->displayMenu->addAction(backgroundWhiteAction);
	
	this->displayMenu->addSeparator();
	
	QActionGroup* sizeActionGroup = new QActionGroup(this);
	sizeActionGroup->setExclusive(true);
	QObject::connect(sizeActionGroup, SIGNAL(triggered(QAction*)), this, SLOT(selectSize(QAction*)));
	
	QAction* size640x480Action = new QAction("640x480", this);
	size640x480Action->setData(SIZE_640x480);
	sizeActionGroup->addAction(size640x480Action);
	this->displayMenu->addAction(size640x480Action);
	
	QAction* size800x600Action = new QAction("800x600", this);
	size800x600Action->setData(SIZE_800x600);
	sizeActionGroup->addAction(size800x600Action);
	this->displayMenu->addAction(size800x600Action);
	
	QAction* size1024x768Action = new QAction("1024x768", this);
	size1024x768Action->setData(SIZE_1024x768);
	sizeActionGroup->addAction(size1024x768Action);
	this->displayMenu->addAction(size1024x768Action);
	
	QAction* size1024x1024Action = new QAction("1024x1024", this);
	size1024x1024Action->setData(SIZE_1024x1024);
	sizeActionGroup->addAction(size1024x1024Action);
	this->displayMenu->addAction(size1024x1024Action);
	
	QAction* size1280x720Action = new QAction("1280x720", this);
	size1280x720Action->setData(SIZE_1280x720);
	sizeActionGroup->addAction(size1280x720Action);
	this->displayMenu->addAction(size1280x720Action);
	
	QAction* size1280x960Action = new QAction("1280x960", this);
	size1280x960Action->setData(SIZE_1280x960);
	sizeActionGroup->addAction(size1280x960Action);
	this->displayMenu->addAction(size1280x960Action);
	
	QAction* size1200x1200Action = new QAction("1200x1200", this);
	size1200x1200Action->setData(SIZE_1200x1200);
	sizeActionGroup->addAction(size1200x1200Action);
	this->displayMenu->addAction(size1200x1200Action);
	
	QAction* size1600x1200Action = new QAction("1600x1200", this);
	size1600x1200Action->setData(SIZE_1600x1200);
	sizeActionGroup->addAction(size1600x1200Action);
	this->displayMenu->addAction(size1600x1200Action);
	
	QAction* size1920x1080Action = new QAction("1920x1080", this);
	size1920x1080Action->setData(SIZE_1920x1080);
	sizeActionGroup->addAction(size1920x1080Action);
	this->displayMenu->addAction(size1920x1080Action);
	
	QAction* size2400x2400Action = new QAction("2400x2400", this);
	size2400x2400Action->setData(SIZE_2400x2400);
	sizeActionGroup->addAction(size2400x2400Action);
	this->displayMenu->addAction(size2400x2400Action);
}

void
MainWindow::load(const QString filename)
{
	if (!(filename.endsWith(".iv") || filename.endsWith(".wrl") || filename.endsWith(".wrl.gz") || filename.endsWith(".wrz")))
	{
		QMessageBox::critical(this, "Error", "File format not supported.");
		return;
	}
	
	SoInput input;
	
	if (!input.openFile(filename.toStdString().c_str(), true))
	{
		QMessageBox::critical(this, "Error", "File not found.");
		return;
	}
	
	if (NULL != this->root)
	{
		this->root->unref();
		this->root = NULL;
		this->viewer->setSceneGraph(NULL);
		this->filename.clear();
		this->setWindowTitle("wrlview");
	}
	
	this->root = SoDB::readAll(&input);
	
	input.closeFile();
	
	if (NULL == this->root)
	{
		QMessageBox::critical(this, "Error", "Error reading file.");
		return;
	}
	
	this->root->ref();
	this->viewer->setSceneGraph(this->root);
	this->filename = filename;
	this->setWindowTitle(filename + " - wrlview");
}

void
MainWindow::open()
{
	QString filename = QFileDialog::getOpenFileName(this, "", this->filename, "All Formats (*.iv *.wrl *.wrl.gz *.wrz)");
	
	if (!filename.isEmpty())
	{
		this->load(filename);
	}
	
	this->viewer->viewAll();
}

void
MainWindow::reload()
{
	if (!this->filename.isEmpty())
	{
		this->load(this->filename);
	}
}

void
MainWindow::saveImage(bool withAlpha)
{
	QString filename = "wrlview-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".png";
	
	glReadBuffer(GL_FRONT);
	QImage image = static_cast< QGLWidget* >(this->viewer->getGLWidget())->grabFrameBuffer(withAlpha);
	
	QString format = filename.right(filename.length() - filename.lastIndexOf('.') - 1).toUpper();
	
	if (("JFIF" == format) || ("JPE" == format) || ("JPG" == format))
	{
		format = "JPEG";
	}

	if (!image.save(filename, format.toStdString().c_str()))
	{
		QMessageBox::critical(this, "Error", "Error writing " + filename + ".");
	}
}

void
MainWindow::saveImageOffscreen()
{
	QString filename = "wrlview-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".png";
	
	this->offscreenRoot->removeAllChildren();
	
	if (-1 == this->root->findChild(this->viewer->getCamera()))
	{
		this->offscreenRoot->addChild(this->viewer->getCamera());
	}
	
	if (this->viewer->isHeadlight())
	{
		this->offscreenRoot->addChild(this->viewer->getHeadlight());
	}
	
	this->offscreenRoot->addChild(this->root);
	
	this->offscreenRenderer->setBackgroundColor(this->viewer->getBackgroundColor());
	
	SbViewportRegion viewportRegion(this->viewer->getViewportRegion());
	viewportRegion.setWindowSize(viewportRegion.getWindowSize()[0] * 300.0f / 72.0f, viewportRegion.getWindowSize()[1] * 300.0f / 72.0f);
	this->offscreenRenderer->setViewportRegion(viewportRegion);
	
	this->offscreenRenderer->getGLRenderAction()->setNumPasses(8);
	
	if (!this->offscreenRenderer->render(this->offscreenRoot))
	{
		QMessageBox::critical(this, "Error", "Error rendering " + filename + ".");
		return;
	}
	
	if (!this->offscreenRenderer->isWriteSupported("png"))
	{
		QMessageBox::critical(this, "Error", "Filetype PNG not supported.");
		return;
	}
	
	if (!this->offscreenRenderer->writeToFile(filename.toStdString().c_str(), "png"))
	{
		QMessageBox::critical(this, "Error", "Error writing " + filename + ".");
		return;
	}
}

void
MainWindow::saveImageWithAlpha()
{
	this->saveImage(true);
}

void
MainWindow::saveImageWithoutAlpha()
{
	this->saveImage(false);
}

void
MainWindow::selectBackground(QAction* action)
{
	switch (action->data().toInt())
	{
	case BACKGROUND_BLACK:
		this->viewer->setBackgroundColor(SbColor(0, 0, 0));
		break;
	case BACKGROUND_WHITE:
		this->viewer->setBackgroundColor(SbColor(255, 255, 255));
		break;
	default:
		break;
	}
}

void
MainWindow::selectCamera(QAction* action)
{
	switch (action->data().toInt())
	{
	case CAMERA_ORTHOGONAL:
		this->viewer->setCameraType(SoOrthographicCamera::getClassTypeId());
		break;
	case CAMERA_PERSPECTIVE:
		this->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
		break;
	default:
		break;
	}
	
	this->viewer->getCamera()->setToDefaults();
	this->viewer->viewAll();
}

void
MainWindow::selectSize(QAction* action)
{
	switch (action->data().toInt())
	{
	case SIZE_640x480:
		this->widget->setFixedSize(640, 480);
		break;
	case SIZE_800x600:
		this->widget->setFixedSize(800, 600);
		break;
	case SIZE_1024x768:
		this->widget->setFixedSize(1024, 768);
		break;
	case SIZE_1024x1024:
		this->widget->setFixedSize(1024, 1024);
		break;
	case SIZE_1280x720:
		this->widget->setFixedSize(1280, 720);
		break;
	case SIZE_1280x960:
		this->widget->setFixedSize(1280, 960);
		break;
	case SIZE_1200x1200:
		this->widget->setFixedSize(1200, 1200);
		break;
	case SIZE_1600x1200:
		this->widget->setFixedSize(1600, 1200);
		break;
	case SIZE_1920x1080:
		this->widget->setFixedSize(1920, 1080);
		break;
	case SIZE_2400x2400:
		this->widget->setFixedSize(2400, 2400);
		break;
	default:
		break;
	}
	
	this->setFixedSize(this->widget->sizeHint().width(), this->widget->sizeHint().height());
	this->widget->setFixedSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
	this->setFixedSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
}

void
MainWindow::selectView(QAction* action)
{
	this->viewer->getCamera()->setToDefaults();
	
	SbVec3f position;
	SbVec3f upvector(0, 0, 1);
	
	switch (action->data().toInt())
	{
	case VIEW_BACK:
		position.setValue(0, -1, 0);
		break;
	case VIEW_BOTTOM:
		position.setValue(0, 0, -1);
		upvector.setValue(0, -1, 0);
		break;
	case VIEW_FRONT:
		position.setValue(0, 1, 0);
		break;
	case VIEW_LEFT:
		position.setValue(-1, 0, 0);
		break;
	case VIEW_RIGHT:
		position.setValue(1, 0, 0);
		break;
	case VIEW_TOP:
		position.setValue(0, 0, 1);
		upvector.setValue(0, 1, 0);
		break;
	case VIEW_TOP_BACK_LEFT:
		position.setValue(-1, -1, 1);
		break;
	case VIEW_TOP_BACK_RIGHT:
		position.setValue(1, -1, 1);
		break;
	case VIEW_TOP_FRONT_LEFT:
		position.setValue(-1, 1, 1);
		break;
	case VIEW_TOP_FRONT_RIGHT:
		position.setValue(1, 1, 1);
		break;
	default:
		break;
	}
	
	this->viewer->getCamera()->position.setValue(position);
	this->viewer->getCamera()->pointAt(SbVec3f(0, 0, 0), upvector);
	this->viewer->getCamera()->scaleHeight(1);
	this->viewer->viewAll();
}

void
MainWindow::toggleAxisCross()
{
	if (this->viewer->isFeedbackVisible())
	{
		this->viewer->setFeedbackVisibility(false);
	}
	else
	{
		this->viewer->setFeedbackVisibility(true);
	}
}

void
MainWindow::toggleFullScreen()
{
	if (this->isFullScreen())
	{
		this->showNormal();
		this->menuBar()->show();
	}
	else
	{
		this->showFullScreen();
		this->menuBar()->hide();
	}
}
