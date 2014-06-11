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
#include <QGraphicsView>
#include <QMainWindow>
#include <QMutex>
#include <QTableView>
#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/mdl/Dynamic.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/Optimizer.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Sampler.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/WorkspaceSphereExplorer.h>
#include <rl/sg/Model.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/Scene.h>
#include <rl/sg/so/Scene.h>

class ConfigurationDelegate;
class ConfigurationModel;
class ConfigurationSpaceScene;
class PlannerModel;
class Thread;
class Viewer;

class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	virtual ~MainWindow();
	
	static MainWindow* instance();
	
	ConfigurationModel* configurationModel;
	
	std::vector< boost::shared_ptr< rl::math::Vector3 > > explorerGoals;
	
	std::vector< boost::shared_ptr< rl::plan::WorkspaceSphereExplorer > > explorers;
	
	std::vector< boost::shared_ptr< rl::math::Vector3 > > explorerStarts;
	
	boost::shared_ptr< rl::math::Vector > goal;
	
	boost::shared_ptr< rl::kin::Kinematics > kin;
	
	boost::shared_ptr< rl::kin::Kinematics > kin2;
	
	boost::shared_ptr< rl::mdl::Dynamic > mdl;
	
	boost::shared_ptr< rl::mdl::Dynamic > mdl2;
	
	boost::shared_ptr< rl::plan::DistanceModel > model;
	
	boost::shared_ptr< rl::plan::Model > model2;
	
	QMutex mutex;
	
	boost::shared_ptr< rl::plan::Optimizer > optimizer;
	
	boost::shared_ptr< rl::plan::Planner > planner;
	
	PlannerModel* plannerModel;
	
	boost::shared_ptr< rl::math::Vector > q;
	
	boost::shared_ptr< rl::plan::Sampler > sampler;
	
	boost::shared_ptr< rl::plan::Sampler > sampler2;
	
	boost::shared_ptr< rl::math::Vector > sigma;
	
	boost::shared_ptr< rl::sg::Scene > scene;
	
	boost::shared_ptr< rl::sg::so::Scene > scene2;
	
	rl::sg::Model* sceneModel;
	
	rl::sg::so::Model* sceneModel2;
	
	boost::shared_ptr< rl::math::Vector > start;
	
	Thread* thread;
	
	boost::shared_ptr< rl::plan::Verifier > verifier;
	
	boost::shared_ptr< rl::plan::Verifier > verifier2;
	
	Viewer* viewer;
	
public slots:
	void eval();
	
	void getGoalConfiguration();
	
	void getRandomConfiguration();
	
	void getRandomFreeConfiguration();
	
	void getStartConfiguration();
	
	void open();
	
	void reset();
	
	void saveImage();
	
	void savePdf();
	
	void saveScene();
	
	void setGoalConfiguration();
	
	void setStartConfiguration();
	
	void startThread();
	
	void toggleCamera();
	
	void toggleConfiguration();
	
	void toggleConfigurationSpace();
	
	void togglePlanner();
	
	void toggleView(const bool& doOn);
	
protected:
	MainWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);
	
private:
	void clear();
	
	void connect(const QObject* sender, const QObject* receiver);
	
	void disconnect(const QObject* sender, const QObject* receiver);
	
	void init();
	
	void load(const QString& filename);
	
	ConfigurationDelegate* configurationDelegate;
	
	QDockWidget* configurationDockWidget;
	
	QDockWidget* configurationSpaceDockWidget;
	
	ConfigurationSpaceScene* configurationSpaceScene;
	
	QGraphicsView* configurationSpaceView;
	
	QTableView* configurationView;
	
	QString engine;
	
	QAction* evalAction;
	
	QAction* exitAction;
	
	QString filename;
	
	QAction* getGoalConfigurationAction;
	
	QAction* getRandomConfigurationAction;
	
	QAction* getRandomFreeConfigurationAction;
	
	QAction* getStartConfigurationAction;
	
	QAction* openAction;
	
	QDockWidget* plannerDockWidget;
	
	QTableView* plannerView;
	
	QAction* resetAction;
	
	QAction* saveImageAction;
	
	QAction* savePdfAction;
	
	QAction* saveSceneAction;
	
	QAction* setGoalConfigurationAction;
	
	QAction* setStartConfigurationAction;
	
	static MainWindow* singleton;
	
	QAction* startThreadAction;
	
	QAction* toggleCameraAction;
	
	QAction* toggleConfigurationAction;
	
	QAction* toggleConfigurationEdgesAction;
	
	QAction* toggleConfigurationSpaceAction;
	
	QAction* toggleConfigurationVerticesAction;
	
	QAction* toggleLinesAction;
	
	QAction* togglePlannerAction;
	
	QAction* togglePointsAction;
	
	QAction* toggleSpheresAction;
	
	QAction* toggleViewAction;
	
	QAction* toggleWorkFramesAction;
	
	bool wait;
};

#endif // _MAINWINDOW_H_
