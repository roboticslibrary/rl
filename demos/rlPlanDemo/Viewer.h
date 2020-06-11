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

#ifndef VIEWER_H
#define VIEWER_H

#include <QWidget>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLColor.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLPointSet.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLSwitch.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <rl/plan/Model.h>
#include <rl/plan/VectorList.h>
#include <rl/plan/Viewer.h>

class SoGradientBackground;

class Viewer : public QWidget, public rl::plan::Viewer
{
	Q_OBJECT
	
public:
	Viewer(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
	
	virtual ~Viewer();
	
	void setBackgroundColor(const QColor& color);
	
	rl::math::Real delta;
	
	rl::math::Real deltaSwept;
	
	rl::plan::Model* model;
	
	SoVRMLGroup* sceneGroup;
	
	SoQtExaminerViewer* viewer;
	
public slots:
	void drawConfiguration(const rl::math::Vector& q);
	
	void drawConfigurationEdge(const rl::math::Vector& u, const rl::math::Vector& v, const bool& free = true);
	
	void drawConfigurationPath(const rl::plan::VectorList& path);
	
	void drawConfigurationVertex(const rl::math::Vector& q, const bool& free = true);
	
	void drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void drawPoint(const rl::math::Vector& xyz);
	
	void drawSphere(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void drawSweptVolume(const rl::plan::VectorList& path);
	
	void drawWork(const rl::math::Transform& t);
	
	void drawWorkEdge(const rl::math::Vector& u, const rl::math::Vector& v);
	
	void drawWorkPath(const rl::plan::VectorList& path);
	
	void drawWorkVertex(const rl::math::Vector& q);
	
	void reset();
	
	void resetEdges();
	
	void resetLines();
	
	void resetPoints();
	
	void resetSpheres();
	
	void resetVertices();
	
	void saveImage(bool withAlpha);
	
	void saveScene();
	
	void showMessage(const std::string& message);
	
	void toggleConfigurationEdges(const bool& doOn);
	
	void toggleConfigurationVertices(const bool& doOn);
	
	void toggleLines(const bool& doOn);
	
	void togglePath(const bool& doOn);
	
	void togglePoints(const bool& doOn);
	
	void toggleSpheres(const bool& doOn);
	
	void toggleSweptVolume(const bool& doOn);
	
	void toggleWorkFrames(const bool& doOn);
	
protected:
	
private:
	SoVRMLSwitch* background;
	
	SoGradientBackground* backgroundGradientBackground;
	
	SoVRMLSwitch* edges;
	
	SoVRMLSwitch* edgesColliding;
	
	SoVRMLAppearance* edgesCollidingAppearance;
	
	SoVRMLCoordinate* edgesCollidingCoordinate;
	
	SoDrawStyle* edgesCollidingDrawStyle;
	
	SoVRMLIndexedLineSet* edgesCollidingIndexedLineSet;
	
	SoVRMLMaterial* edgesCollidingMaterial;
	
	SoVRMLShape* edgesCollidingShape;
	
	SoVRMLSwitch* edgesFree;
	
	SoVRMLAppearance* edgesFreeAppearance;
	
	SoVRMLCoordinate* edgesFreeCoordinate;
	
	SoDrawStyle* edgesFreeDrawStyle;
	
	SoVRMLIndexedLineSet* edgesFreeIndexedLineSet;
	
	SoVRMLMaterial* edgesFreeMaterial;
	
	SoVRMLShape* edgesFreeShape;
	
	SoVRMLSwitch* edges3;
	
	SoVRMLAppearance* edges3Appearance;
	
	SoVRMLCoordinate* edges3Coordinate;
	
	SoDrawStyle* edges3DrawStyle;
	
	SoVRMLIndexedLineSet* edges3IndexedLineSet;
	
	SoVRMLMaterial* edges3Material;
	
	SoVRMLShape* edges3Shape;
	
	SoVRMLIndexedLineSet* frameIndexedLineSet;
	
	SoVRMLSwitch* lines;
	
	SoVRMLAppearance* linesAppearance;
	
	SoVRMLCoordinate* linesCoordinate;
	
	SoDrawStyle* linesDrawStyle;
	
	SoVRMLIndexedLineSet* linesIndexedLineSet;
	
	SoVRMLMaterial* linesMaterial;
	
	SoVRMLShape* linesShape;
	
	SoVRMLSwitch* path;
	
	SoVRMLAppearance* pathAppearance;
	
	SoVRMLCoordinate* pathCoordinate;
	
	SoDrawStyle* pathDrawStyle;
	
	SoVRMLIndexedLineSet* pathIndexedLineSet;
	
	SoVRMLMaterial* pathMaterial;
	
	SoVRMLShape* pathShape;
	
	SoVRMLSwitch* path3;
	
	SoVRMLAppearance* path3Appearance;
	
	SoVRMLCoordinate* path3Coordinate;
	
	SoDrawStyle* path3DrawStyle;
	
	SoVRMLIndexedLineSet* path3IndexedLineSet;
	
	SoVRMLMaterial* path3Material;
	
	SoVRMLShape* path3Shape;
	
	SoVRMLSwitch* points;
	
	SoVRMLAppearance* pointsAppearance;
	
	SoVRMLCoordinate* pointsCoordinate;
	
	SoDrawStyle* pointsDrawStyle;
	
	SoVRMLPointSet* pointsPointSet;
	
	SoVRMLMaterial* pointsMaterial;
	
	SoVRMLShape* pointsShape;
	
	SoVRMLSwitch* root;
	
	SoVRMLSwitch* scene;
	
	SoDrawStyle* sceneDrawStyle;
	
	SoVRMLSwitch* spheres;
	
	SoVRMLAppearance* spheresAppearance;
	
	SoDrawStyle* spheresDrawStyle;
	
	SoVRMLGroup* spheresGroup;
	
	SoVRMLMaterial* spheresMaterial;
	
	SoVRMLSwitch* swept;
	
	SoVRMLGroup* sweptGroup;
	
	SoVRMLSwitch* vertices;
	
	SoVRMLSwitch* verticesColliding;
	
	SoVRMLAppearance* verticesCollidingAppearance;
	
	SoVRMLColor* verticesCollidingColor;
	
	SoVRMLCoordinate* verticesCollidingCoordinate;
	
	SoDrawStyle* verticesCollidingDrawStyle;
	
	SoVRMLPointSet* verticesCollidingPointSet;
	
	SoVRMLMaterial* verticesCollidingMaterial;
	
	SoVRMLShape* verticesCollidingShape;
	
	SoVRMLSwitch* verticesFree;
	
	SoVRMLAppearance* verticesFreeAppearance;
	
	SoVRMLColor* verticesFreeColor;
	
	SoVRMLCoordinate* verticesFreeCoordinate;
	
	SoDrawStyle* verticesFreeDrawStyle;
	
	SoVRMLPointSet* verticesFreePointSet;
	
	SoVRMLMaterial* verticesFreeMaterial;
	
	SoVRMLShape* verticesFreeShape;
	
	SoVRMLSwitch* work;
	
	SoDrawStyle* workDrawStyle;
	
	SoVRMLTransform* workTransform;
};

#endif // VIEWER_H
