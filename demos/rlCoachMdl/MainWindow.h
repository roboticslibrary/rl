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

#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <QAction>
#include <QDockWidget>
#include <QMainWindow>
#include <QTableView>
#include <QTabWidget>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <rl/mdl/Model.h>
#include <rl/sg/Model.h>
#include <rl/sg/so/Scene.h>

class ConfigurationDelegate;
class ConfigurationModel;
class OperationalDelegate;
class OperationalModel;
class Server;

class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	virtual ~MainWindow();
	
	static MainWindow* instance();
	
	std::vector< ConfigurationModel* > configurationModels;
	
	std::vector< rl::sg::Model* > geometryModels;
	
	std::vector< boost::shared_ptr< rl::mdl::Model > > kinematicModels;
	
	std::vector< OperationalModel* > operationalModels;
	
	boost::shared_ptr< rl::sg::so::Scene > scene;
	
public slots:
	void saveImage();
	
	void saveScene();
	
protected:
	MainWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);
	
private:
	void init();
	
	std::vector< ConfigurationDelegate* > configurationDelegates;
	
	QDockWidget* configurationDockWidget;
	
	QTabWidget* configurationTabWidget;
	
	std::vector< QTableView* > configurationViews;
	
	std::vector< OperationalDelegate* > operationalDelegates;
	
	QDockWidget* operationalDockWidget;
	
	QTabWidget* operationalTabWidget;
	
	std::vector< QTableView* > operationalViews;
	
	QAction* saveImageAction;
	
	QAction* saveSceneAction;
	
	Server* server;
	
	static MainWindow* singleton;
	
	SoQtExaminerViewer* viewer;
};

#endif // _MAINWINDOW_H_
