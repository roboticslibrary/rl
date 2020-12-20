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
#include <QMainWindow>
#include <QLabel>
#include <string>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLPointSet.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <rl/sg/Scene.h>
#include <rl/sg/so/Scene.h>

class SoGradientBackground;

class MainWindow : public QMainWindow
{
public:
	virtual ~MainWindow();
	
	static MainWindow* instance();
	
	void test();
	
	std::shared_ptr<rl::sg::Scene> collisionScene;
	
	std::shared_ptr<rl::sg::so::Scene> viewScene;
	
protected:
	MainWindow(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
	
private:
	void parseCommandLine();
	
	std::size_t body;
	
	SoVRMLCoordinate* depthCoordinate;
	
	QLabel* depthLabel;
	
	SoVRMLIndexedLineSet* depthLineSet;
	
	SoVRMLPointSet* depthPointSet;
	
	SoVRMLCoordinate* distanceCoordinate;
	
	QLabel* distanceLabel;
	
	SoVRMLIndexedLineSet* distanceLineSet;
	
	SoVRMLPointSet* distancePointSet;
	
	QString engine;
	
	QString filename;
	
	SoGradientBackground* gradientBackground;
	
	std::size_t model;
	
	QLabel* simpleLabel;
	
	static MainWindow* singleton;
	
	SoQtExaminerViewer* viewer;
};

#endif // MAINWINDOW_H
