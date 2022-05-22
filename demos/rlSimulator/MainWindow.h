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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>
#include <QAction>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QTableView>
#include <QTabWidget>
#include <QTimer>
#include <vector>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <rl/mdl/Dynamic.h>
#include <rl/sg/Model.h>
#include <rl/sg/so/Scene.h>

class ConfigurationDelegate;
class ConfigurationModel;
class PositionDelegate;
class PositionModel;
class VelocityDelegate;
class VelocityModel;
class AccelerationDelegate;
class AccelerationModel;
class TorqueModel;
class TorqueDelegate;
class OperationalDelegate;
class OperationalModel;
class Server;
class SoGradientBackground;

class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	virtual ~MainWindow();
	
	static MainWindow* instance();
	
	AccelerationModel* accelerationModel;
	
	std::shared_ptr<rl::mdl::Dynamic> dynamicModel;
	
	rl::math::Vector externalTorque;
	
	rl::sg::Model* geometryModels;
	
	OperationalModel* operationalModel;
	
	PositionModel* positionModel;
	
	std::shared_ptr<rl::sg::so::Scene> scene;
	
	TorqueModel* torqueModel;
	
	VelocityModel* velocityModel;
	
public slots:
	void changeSimulationGravity(double value);
	
	void changeSimulationDampingFactor(double value);
	
	void clickSimulationPause();
	
	void clickSimulationReset();
	
	void clickSimulationStart();
	
	void saveImage();
	
	void saveScene();
	
protected:
	MainWindow(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
	
	void changeEvent(QEvent* event);
	
	void timerEvent(QTimerEvent *event);
	
private:
	void init();
	
	AccelerationDelegate* accelerationDelegate;
	
	QDockWidget* accelerationDockWidget;
	
	QTableView* accelerationView;
	
	SoGradientBackground* gradientBackground;
	
	OperationalDelegate* operationalDelegate;
	
	QDockWidget* operationalDockWidget;
	
	QTableView* operationalViews;
	
	PositionDelegate* positionDelegate;
	
	QDockWidget* positionDockWidget;
	
	QTableView* positionView;
	
	QAction* saveImageAction;
	
	QAction* saveSceneAction;
	
	Server* server;
	
	QDoubleSpinBox* simulationDampingFactor;
	
	rl::math::Real simulationDampingValue;
	
	QDockWidget* simulationDockWidget;
	
	QDoubleSpinBox* simulationGravity;
		
	rl::math::Real simulationGravityValue;
	
	bool simulationIsRunning;
	
	QPushButton* simulationPause;
	
	QPushButton* simulationReset;
	
	QLineEdit* simulationResultsEnergy;
	
	QLineEdit* simulationResultsTime;
	
	rl::math::Vector simulationResetQ;
	
	rl::math::Vector simulationResetQd;
	
	rl::math::Vector simulationResetQdd;
	
	QPushButton* simulationStart;
	
	int simulationStepsPerFrame;
	
	QLineEdit* simulationTime;
	
	rl::math::Real simulationTimeElapsed;
	
	rl::math::Real simulationTimeStep;
	
	static MainWindow* singleton;
	
	QBasicTimer timer;
	
	TorqueDelegate* torqueDelegate;
	
	QDockWidget* torqueDockWidget;
	
	QTableView* torqueView;
	
	VelocityDelegate* velocityDelegate;
	
	QDockWidget* velocityDockWidget;
	
	QTableView* velocityView;
	
	SoQtExaminerViewer* viewer;
};

#endif // MAINWINDOW_H
