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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QAction>
#include <QMainWindow>
#include <QMenuBar>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QString>
#include <string>
#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/VRMLnodes/SoVRMLInline.h>

class SoGradientBackground;

class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	MainWindow(QWidget* parent = nullptr, Qt::WindowFlags f = 0);
	
	virtual ~MainWindow();
	
protected:
	void dragEnterEvent(QDragEnterEvent* event);
	
	void dropEvent(QDropEvent* event);
	
private:
	enum BACKGROUND
	{
		BACKGROUND_BLACK,
		BACKGROUND_GRADIENT,
		BACKGROUND_WHITE
	};
	
	enum CAMERA
	{
		CAMERA_ORTHOGONAL,
		CAMERA_PERSPECTIVE
	};
	
	enum ORIGIN
	{
		ORIGIN_NONE,
		ORIGIN_1,
		ORIGIN_1000
	};
	
	enum SIZE
	{
		SIZE_640x480,
		SIZE_800x600,
		SIZE_1024x768,
		SIZE_1024x1024,
		SIZE_1280x720,
		SIZE_1280x960,
		SIZE_1200x1200,
		SIZE_1600x1200,
		SIZE_1920x1080,
		SIZE_2400x2400
	};
	
	enum VIEW
	{
		VIEW_BACK,
		VIEW_BOTTOM,
		VIEW_FRONT,
		VIEW_LEFT,
		VIEW_RIGHT,
		VIEW_TOP,
		VIEW_TOP_BACK_LEFT,
		VIEW_TOP_BACK_RIGHT,
		VIEW_TOP_FRONT_LEFT,
		VIEW_TOP_FRONT_RIGHT
	};
	
	void init();
	
	static void inlineFetchUrlCallback(const SbString& url, void* userData, SoVRMLInline* node);
	
	void load(const QString filename);
	
	void saveImage(bool withAlpha);
	
	void saveImageOffscreen(bool withAlpha);
	
	SoSwitch* backgroundSwitch;
	
	QMenu* displayMenu;
	
	QMenu* fileMenu;
	
	QString filename;
	
	SoGradientBackground* gradientBackground;
	
	SoInput input;
	
	QNetworkAccessManager* manager;
	
	SoOffscreenRenderer* offscreenRenderer;
	
	SoSwitch* origin1Switch;
	
	SoSwitch* origin1000Switch;
	
	SoSeparator* root;
	
	SoQtExaminerViewer* viewer;
	
	QMenu* viewMenu;
	
	QWidget* widget;
	
private slots:
	void open();
	
	void reload();
	
	void replyFinished(QNetworkReply* reply);
	
	void saveImageOffscreenWithAlpha();

	void saveImageOffscreenWithoutAlpha();
	
	void saveImageWithAlpha();

	void saveImageWithoutAlpha();
	
	void selectBackground(QAction* action);
	
	void selectCamera(QAction* action);
	
	void selectOrigin(QAction* action);
	
	void selectSize(QAction* action);
	
	void selectView(QAction* action);
	
	void toggleAxisCross();
	
	void toggleFullScreen();
};

#endif // MAINWINDOW_H
