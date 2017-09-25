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

#include <memory>
#include <QAction>
#include <QDockWidget>
#include <QMainWindow>
#include <QMutex>
#include <QTableView>
#include <boost/optional.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/mdl/Dynamic.h>
#include <rl/plan/NearestNeighbors.h>
#include <rl/plan/Optimizer.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/WorkspaceSphereExplorer.h>
#include <rl/sg/Model.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/Scene.h>
#include <rl/sg/so/Scene.h>

class ConfigurationDelegate;
class ConfigurationModel;
class ConfigurationSpaceModel;
class ConfigurationSpaceScene;
class GraphicsView;
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
	
	ConfigurationSpaceModel* configurationSpaceModel;
	
	QString engine;
	
	std::vector<std::shared_ptr<rl::math::Vector3>> explorerGoals;
	
	std::vector<std::shared_ptr<rl::plan::WorkspaceSphereExplorer>> explorers;
	
	std::vector<std::shared_ptr<rl::math::Vector3>> explorerStarts;
	
	std::shared_ptr<rl::math::Vector> goal;
	
	std::shared_ptr<rl::kin::Kinematics> kin;
	
	std::shared_ptr<rl::kin::Kinematics> kin2;
	
	std::shared_ptr<rl::mdl::Dynamic> mdl;
	
	std::shared_ptr<rl::mdl::Dynamic> mdl2;
	
	std::shared_ptr<rl::plan::SimpleModel> model;
	
	std::shared_ptr<rl::plan::Model> model2;
	
	QMutex mutex;
	
	std::vector<std::shared_ptr<rl::plan::NearestNeighbors>> nearestNeighbors;
	
	std::shared_ptr<rl::plan::Optimizer> optimizer;
	
	std::shared_ptr<rl::plan::Planner> planner;
	
	PlannerModel* plannerModel;
	
	std::shared_ptr<rl::math::Vector> q;
	
	std::shared_ptr<rl::plan::Sampler> sampler;
	
	std::shared_ptr<rl::plan::Sampler> sampler2;
	
	std::shared_ptr<rl::math::Vector> sigma;
	
	std::shared_ptr<rl::sg::Scene> scene;
	
	std::shared_ptr<rl::sg::so::Scene> scene2;
	
	rl::sg::Model* sceneModel;
	
	rl::sg::so::Model* sceneModel2;
	
	std::shared_ptr<rl::math::Vector> start;
	
	Thread* thread;
	
	std::shared_ptr<rl::plan::Verifier> verifier;
	
	std::shared_ptr<rl::plan::Verifier> verifier2;
	
	Viewer* viewer;
	
public slots:
	void eval();
	
	void evalDone();
	
	void getGoalConfiguration();
	
	void getRandomConfiguration();
	
	void getRandomFreeConfiguration();
	
	void getStartConfiguration();
	
	void open();
	
	void reset();
	
	void saveImageWithAlpha();
	
	void saveImageWithoutAlpha();
	
	void savePdf();
	
	void saveScene();
	
	void setGoalConfiguration();
	
	void setStartConfiguration();
	
	void startThread();
	
	void toggleAnimation(const bool& doOn);
	
	void toggleCamera();
	
	void toggleConfiguration();
	
	void toggleConfigurationSpace();
	
	void toggleConfigurationSpaceActive(const bool& doOn);
	
	void toggleConfigurationSpaceScene();
	
	void togglePlanner();
	
	void toggleSweptVolume(const bool& doOn);
	
	void toggleViewActive(const bool& doOn);
	
protected:
	MainWindow(QWidget* parent = nullptr, Qt::WindowFlags f = 0);
	
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
	
	QDockWidget* configurationSpaceSceneDockWidget;
	
	GraphicsView* configurationSpaceSceneView;
	
	QTableView* configurationSpaceView;
	
	QTableView* configurationView;
	
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
	
	QAction* saveImageWithAlphaAction;
	
	QAction* saveImageWithoutAlphaAction;
	
	QAction* savePdfAction;
	
	QAction* saveSceneAction;
	
	boost::optional<std::size_t> seed;
	
	QAction* setGoalConfigurationAction;
	
	QAction* setStartConfigurationAction;
	
	static MainWindow* singleton;
	
	QAction* startThreadAction;
	
	QAction* toggleAnimationAction;
	
	QAction* toggleCameraAction;
	
	QAction* toggleConfigurationAction;
	
	QAction* toggleConfigurationEdgesAction;
	
	QAction* toggleConfigurationSpaceAction;
	
	QAction* toggleConfigurationSpaceActiveAction;
	
	QAction* toggleConfigurationSpaceSceneAction;
	
	QAction* toggleConfigurationVerticesAction;
	
	QAction* toggleLinesAction;
	
	QAction* togglePathAction;
	
	QAction* togglePlannerAction;
	
	QAction* togglePointsAction;
	
	QAction* toggleSpheresAction;
	
	QAction* toggleSweptVolumeAction;
	
	QAction* toggleViewActiveAction;
	
	QAction* toggleWorkFramesAction;
	
	bool wait;
};

#endif // MAINWINDOW_H
