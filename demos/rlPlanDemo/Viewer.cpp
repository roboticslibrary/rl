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

#include <QDateTime>
#include <QEvent>
#include <QMessageBox>
#include <QStatusBar>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLSphere.h>
#include <rl/sg/Body.h>
#include <rl/sg/so/Body.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/so/Shape.h>

#if QT_VERSION >= 0x060000
#include <QOpenGLWindow>
#else
#include <QGLWidget>
#endif

#include "SoGradientBackground.h"
#include "MainWindow.h"
#include "Viewer.h"

Viewer::Viewer(QWidget* parent, Qt::WindowFlags f) :
	QWidget(parent, f),
	delta(1),
	deltaSwept(100),
	model(nullptr),
	sceneGroup(new SoVRMLGroup()),
	viewer(new SoQtExaminerViewer(this, nullptr, true, SoQtFullViewer::BUILD_POPUP)),
	background(new SoVRMLSwitch()),
	backgroundGradientBackground(new SoGradientBackground()),
	edges(new SoVRMLSwitch()),
	edgesColliding(new SoVRMLSwitch()),
	edgesCollidingAppearance(new SoVRMLAppearance()),
	edgesCollidingCoordinate(new SoVRMLCoordinate()),
	edgesCollidingDrawStyle(new SoDrawStyle()),
	edgesCollidingIndexedLineSet(new SoVRMLIndexedLineSet()),
	edgesCollidingMaterial(new SoVRMLMaterial()),
	edgesCollidingShape(new SoVRMLShape()),
	edgesFree(new SoVRMLSwitch()),
	edgesFreeAppearance(new SoVRMLAppearance()),
	edgesFreeCoordinate(new SoVRMLCoordinate()),
	edgesFreeDrawStyle(new SoDrawStyle()),
	edgesFreeIndexedLineSet(new SoVRMLIndexedLineSet()),
	edgesFreeMaterial(new SoVRMLMaterial()),
	edgesFreeShape(new SoVRMLShape()),
	edges3(new SoVRMLSwitch()),
	edges3Appearance(new SoVRMLAppearance()),
	edges3Coordinate(new SoVRMLCoordinate()),
	edges3DrawStyle(new SoDrawStyle()),
	edges3IndexedLineSet(new SoVRMLIndexedLineSet()),
	edges3Material(new SoVRMLMaterial()),
	edges3Shape(new SoVRMLShape()),
	frameIndexedLineSet(new SoVRMLIndexedLineSet()),
	lines(new SoVRMLSwitch()),
	linesAppearance(new SoVRMLAppearance()),
	linesCoordinate(new SoVRMLCoordinate()),
	linesDrawStyle(new SoDrawStyle()),
	linesIndexedLineSet(new SoVRMLIndexedLineSet()),
	linesMaterial(new SoVRMLMaterial()),
	linesShape(new SoVRMLShape()),
	path(new SoVRMLSwitch()),
	pathEdges(new SoVRMLSwitch()),
	pathEdgesAppearance(new SoVRMLAppearance()),
	pathEdgesCoordinate(new SoVRMLCoordinate()),
	pathEdgesDrawStyle(new SoDrawStyle()),
	pathEdgesIndexedLineSet(new SoVRMLIndexedLineSet()),
	pathEdgesMaterial(new SoVRMLMaterial()),
	pathEdgesShape(new SoVRMLShape()),
	pathVertices(new SoVRMLSwitch()),
	pathVerticesAppearance(new SoVRMLAppearance()),
	pathVerticesCoordinate(new SoVRMLCoordinate()),
	pathVerticesDrawStyle(new SoDrawStyle()),
	pathVerticesPointSet(new SoVRMLPointSet()),
	pathVerticesMaterial(new SoVRMLMaterial()),
	pathVerticesShape(new SoVRMLShape()),
	path3(new SoVRMLSwitch()),
	path3Appearance(new SoVRMLAppearance()),
	path3Coordinate(new SoVRMLCoordinate()),
	path3DrawStyle(new SoDrawStyle()),
	path3IndexedLineSet(new SoVRMLIndexedLineSet()),
	path3Material(new SoVRMLMaterial()),
	path3Shape(new SoVRMLShape()),
	points(new SoVRMLSwitch()),
	pointsAppearance(new SoVRMLAppearance()),
	pointsCoordinate(new SoVRMLCoordinate()),
	pointsDrawStyle(new SoDrawStyle()),
	pointsPointSet(new SoVRMLPointSet()),
	pointsMaterial(new SoVRMLMaterial()),
	pointsShape(new SoVRMLShape()),
	root(new SoVRMLSwitch()),
	scene(new SoVRMLSwitch()),
	sceneDrawStyle(new SoDrawStyle()),
	spheres(new SoVRMLSwitch()),
	spheresAppearance(new SoVRMLAppearance()),
	spheresDrawStyle(new SoDrawStyle()),
	spheresGroup(new SoVRMLGroup()),
	spheresMaterial(new SoVRMLMaterial()),
	swept(new SoVRMLSwitch()),
	sweptGroup(new SoVRMLGroup()),
	vertices(new SoVRMLSwitch()),
	verticesColliding(new SoVRMLSwitch()),
	verticesCollidingAppearance(new SoVRMLAppearance()),
	verticesCollidingColor(new SoVRMLColor()),
	verticesCollidingCoordinate(new SoVRMLCoordinate()),
	verticesCollidingDrawStyle(new SoDrawStyle()),
	verticesCollidingPointSet(new SoVRMLPointSet()),
	verticesCollidingMaterial(new SoVRMLMaterial()),
	verticesCollidingShape(new SoVRMLShape()),
	verticesFree(new SoVRMLSwitch()),
	verticesFreeAppearance(new SoVRMLAppearance()),
	verticesFreeColor(new SoVRMLColor()),
	verticesFreeCoordinate(new SoVRMLCoordinate()),
	verticesFreeDrawStyle(new SoDrawStyle()),
	verticesFreePointSet(new SoVRMLPointSet()),
	verticesFreeMaterial(new SoVRMLMaterial()),
	verticesFreeShape(new SoVRMLShape()),
	work(new SoVRMLSwitch()),
	workDrawStyle(new SoDrawStyle()),
	workTransform(new SoVRMLTransform())
{
	this->root->ref();
	
	this->viewer->setSceneGraph(this->root);
	this->viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	
	// background
	
	if (this->palette().color(QPalette::Window).lightness() < 128)
	{
		this->backgroundGradientBackground->color0.setValue(0.0f, 0.0f, 0.0f);
		this->backgroundGradientBackground->color1.setValue(0.2f, 0.2f, 0.2f);
	}
	else
	{
		this->backgroundGradientBackground->color0.setValue(0.8f, 0.8f, 0.8f);
		this->backgroundGradientBackground->color1.setValue(1.0f, 1.0f, 1.0f);
	}
	
	this->background->setName("background");
	this->background->whichChoice = SO_SWITCH_ALL;
	this->background->addChild(this->backgroundGradientBackground);
	
	this->root->addChild(this->background);
	
	// edgesColliding
	
	this->edgesColliding->setName("edgesColliding");
	this->edgesColliding->whichChoice = SO_SWITCH_ALL;
	
	this->edgesCollidingDrawStyle->lineWidth = 1.0f;
	this->edgesCollidingDrawStyle->pointSize = 0.0f;
	this->edgesColliding->addChild(this->edgesCollidingDrawStyle);
	
	this->edgesCollidingMaterial->diffuseColor.setValue(0.5f, 0.5f, 0.5f);
	this->edgesCollidingAppearance->material = this->edgesCollidingMaterial;
	this->edgesCollidingShape->appearance = this->edgesCollidingAppearance;
	
	this->edgesCollidingIndexedLineSet->coord = this->edgesCollidingCoordinate;
	this->edgesCollidingShape->geometry = this->edgesCollidingIndexedLineSet;
	
	this->edgesColliding->addChild(this->edgesCollidingShape);
	
	this->edges->addChild(this->edgesColliding);
	
	// edgesFree
	
	this->edgesFree->setName("edgesFree");
	this->edgesFree->whichChoice = SO_SWITCH_ALL;
	
	this->edgesFreeDrawStyle->lineWidth = 1.0f;
	this->edgesFreeDrawStyle->pointSize = 0.0f;
	this->edgesFree->addChild(this->edgesFreeDrawStyle);
	
	this->edgesFreeMaterial->diffuseColor.setValue(0.5f, 0.5f, 0.5f);
	this->edgesFreeAppearance->material = this->edgesFreeMaterial;
	this->edgesFreeShape->appearance = this->edgesFreeAppearance;
	
	this->edgesFreeIndexedLineSet->coord = this->edgesFreeCoordinate;
	this->edgesFreeShape->geometry = this->edgesFreeIndexedLineSet;
	
	this->edgesFree->addChild(this->edgesFreeShape);
	
	this->edges->addChild(this->edgesFree);
	
	// edges
	
	this->edges->setName("edges");
	this->edges->whichChoice = SO_SWITCH_ALL;
	
	this->root->addChild(this->edges);
	
	// edges3
	
	this->edges3->setName("edges3");
	this->edges3->whichChoice = SO_SWITCH_ALL;
	
	this->edges3DrawStyle->lineWidth = 1.0f;
	this->edges3DrawStyle->pointSize = 0.0f;
	this->edges3->addChild(this->edges3DrawStyle);
	
	this->edges3Material->diffuseColor.setValue(0.5f, 0.5f, 0.5f);
	this->edges3Appearance->material = this->edges3Material;
	this->edges3Shape->appearance = this->edges3Appearance;
	
	this->edges3IndexedLineSet->coord = this->edges3Coordinate;
	this->edges3Shape->geometry = this->edges3IndexedLineSet;
	
	this->edges3->addChild(this->edges3Shape);
	
	this->root->addChild(this->edges3);
	
	// frame
	
	this->frameIndexedLineSet->colorPerVertex = false;
	
	SoVRMLColor* frameColor = new SoVRMLColor();
	this->frameIndexedLineSet->color = frameColor;
	frameColor->color.set1Value(0, 1.0f, 0.0f, 0.0f);
	frameColor->color.set1Value(1, 0.0f, 1.0f, 0.0f);
	frameColor->color.set1Value(2, 0.0f, 0.0f, 1.0f);
	
	this->frameIndexedLineSet->colorIndex.set1Value(0, 0);
	this->frameIndexedLineSet->colorIndex.set1Value(1, 1);
	this->frameIndexedLineSet->colorIndex.set1Value(2, 2);
	
	SoVRMLCoordinate* frameCoordinate = new SoVRMLCoordinate();
	this->frameIndexedLineSet->coord = frameCoordinate;
	frameCoordinate->point.set1Value(0, 0.0f, 0.0f, 0.0f);
	frameCoordinate->point.set1Value(1, 0.25f, 0.0f, 0.0f);
	frameCoordinate->point.set1Value(2, 0.0f, 0.25f, 0.0f);
	frameCoordinate->point.set1Value(3, 0.0f, 0.0f, 0.25f);
	
	this->frameIndexedLineSet->coordIndex.set1Value(0, 0);
	this->frameIndexedLineSet->coordIndex.set1Value(1, 1);
	this->frameIndexedLineSet->coordIndex.set1Value(2, SO_END_FACE_INDEX);
	this->frameIndexedLineSet->coordIndex.set1Value(3, 0);
	this->frameIndexedLineSet->coordIndex.set1Value(4, 2);
	this->frameIndexedLineSet->coordIndex.set1Value(5, SO_END_FACE_INDEX);
	this->frameIndexedLineSet->coordIndex.set1Value(6, 0);
	this->frameIndexedLineSet->coordIndex.set1Value(7, 3);
	this->frameIndexedLineSet->coordIndex.set1Value(8, SO_END_FACE_INDEX);
	
	// lines
	
	this->lines->setName("lines");
	this->lines->whichChoice = SO_SWITCH_ALL;
	
	this->linesDrawStyle->lineWidth = 1.0f;
	this->linesDrawStyle->pointSize = 0.0f;
	this->lines->addChild(this->linesDrawStyle);
	
	this->linesMaterial->diffuseColor.setValue(232.0f / 255.0f, 21.0f / 255.0f, 21.0f / 255.0f);
	this->linesAppearance->material = this->linesMaterial;
	this->linesShape->appearance = this->linesAppearance;
	
	this->linesIndexedLineSet->coord = this->linesCoordinate;
	this->linesShape->geometry = this->linesIndexedLineSet;
	
	this->lines->addChild(this->linesShape);
	
	this->root->addChild(this->lines);
	
	// path
	
	this->path->setName("path");
	this->path->whichChoice = SO_SWITCH_ALL;
	
	this->root->addChild(this->path);
	
	// path edges
	
	this->pathEdges->setName("pathEdges");
	this->pathEdges->whichChoice = SO_SWITCH_ALL;
	
	this->pathEdgesDrawStyle->lineWidth = 3.0f;
	this->pathEdgesDrawStyle->pointSize = 0.0f;
	this->pathEdges->addChild(this->pathEdgesDrawStyle);
	
	this->pathEdgesMaterial->diffuseColor.setValue(55.0f / 255.0f, 176.0f / 255.0f, 55.0f / 255.0f);
	this->pathEdgesAppearance->material = this->pathEdgesMaterial;
	this->pathEdgesShape->appearance = this->pathEdgesAppearance;
	
	this->pathEdgesIndexedLineSet->coord = this->pathEdgesCoordinate;
	this->pathEdgesShape->geometry = this->pathEdgesIndexedLineSet;
	
	this->pathEdges->addChild(this->pathEdgesShape);
	
	this->path->addChild(this->pathEdges);
	
	// path vertices
	
	this->pathVertices->setName("pathVertices");
	this->pathVertices->whichChoice = SO_SWITCH_NONE;
	
	this->pathVerticesDrawStyle->lineWidth = 0.0f;
	this->pathVerticesDrawStyle->pointSize = 8.0f;
	this->pathVertices->addChild(this->pathVerticesDrawStyle);
	
	this->pathVerticesMaterial->emissiveColor.setValue(55.0f / 255.0f, 176.0f / 255.0f, 55.0f / 255.0f);
	this->pathVerticesAppearance->material = this->pathVerticesMaterial;
	this->pathVerticesShape->appearance = this->pathVerticesAppearance;
	
	this->pathVerticesPointSet->coord = this->pathVerticesCoordinate;
	this->pathVerticesShape->geometry = this->pathVerticesPointSet;
	
	this->pathVertices->addChild(this->pathVerticesShape);
	
	this->path->addChild(this->pathVertices);
	
	// path3
	
	this->path3->setName("path3");
	this->path3->whichChoice = SO_SWITCH_ALL;
	
	this->path3DrawStyle->lineWidth = 3.0f;
	this->path3DrawStyle->pointSize = 0.0f;
	this->path3->addChild(this->path3DrawStyle);
	
	this->path3Material->diffuseColor.setValue(55.0f / 255.0f, 176.0f / 255.0f, 55.0f / 255.0f);
	this->path3Appearance->material = this->path3Material;
	this->path3Shape->appearance = this->path3Appearance;
	
	this->path3IndexedLineSet->coord = this->path3Coordinate;
	this->path3Shape->geometry = this->path3IndexedLineSet;
	
	this->path3->addChild(this->path3Shape);
	
	this->root->addChild(this->path3);
	
	// points
	
	this->points->setName("points");
	this->points->whichChoice = SO_SWITCH_ALL;
	
	this->pointsDrawStyle->lineWidth = 0.0f;
	this->pointsDrawStyle->pointSize = 4.0f;
	this->points->addChild(this->pointsDrawStyle);
	
	this->pointsMaterial->emissiveColor.setValue(232.0f / 255.0f, 21.0f / 255.0f, 21.0f / 255.0f);
	this->pointsAppearance->material = this->pointsMaterial;
	this->pointsShape->appearance = this->pointsAppearance;
	
	this->pointsPointSet->coord = this->pointsCoordinate;
	this->pointsShape->geometry = this->pointsPointSet;
	
	this->points->addChild(this->pointsShape);
	
	this->root->addChild(this->points);
	
	// spheres
	
	this->spheres->setName("spheres");
	this->spheres->whichChoice = SO_SWITCH_ALL;
	
	this->spheres->addChild(this->spheresDrawStyle);
	
	this->spheresMaterial->diffuseColor.setValue(55.0f / 255.0f, 176.0f / 255.0f, 55.0f / 255.0f);
	this->spheresMaterial->transparency.setValue(0.9f);
	this->spheresAppearance->material = this->spheresMaterial;
	
	this->spheresAppearance->ref();
	
	this->spheres->addChild(this->spheresGroup);
	
	this->root->addChild(this->spheres);
	
	// swept
	
	this->swept->setName("swept");
	this->swept->whichChoice = SO_SWITCH_ALL;
	
	this->swept->addChild(this->sweptGroup);
	
	this->root->addChild(this->swept);
	
	// verticesColliding
	
	this->verticesColliding->setName("verticesColliding");
	this->verticesColliding->whichChoice = SO_SWITCH_ALL;
	
	this->verticesCollidingDrawStyle->lineWidth = 0.0f;
	this->verticesCollidingDrawStyle->pointSize = 8.0f;
	this->verticesColliding->addChild(this->verticesCollidingDrawStyle);
	
	this->verticesCollidingMaterial->emissiveColor.setValue(232.0f / 255.0f, 21.0f / 255.0f, 21.0f / 255.0f);
	this->verticesCollidingAppearance->material = this->verticesCollidingMaterial;
	this->verticesCollidingShape->appearance = this->verticesCollidingAppearance;
	
	this->verticesCollidingPointSet->coord = this->verticesCollidingCoordinate;
	this->verticesCollidingShape->geometry = this->verticesCollidingPointSet;
	
	this->verticesColliding->addChild(this->verticesCollidingShape);
	
	this->vertices->addChild(this->verticesColliding);
	
	// verticesFree
	
	this->verticesFree->setName("verticesFree");
	this->verticesFree->whichChoice = SO_SWITCH_ALL;
	
	this->verticesFreeDrawStyle->lineWidth = 0.0f;
	this->verticesFreeDrawStyle->pointSize = 8.0f;
	this->verticesFree->addChild(this->verticesFreeDrawStyle);
	
	this->verticesFreeMaterial->emissiveColor.setValue(55.0f / 255.0f, 176.0f / 255.0f, 55.0f / 255.0f);
	this->verticesFreeAppearance->material = this->verticesFreeMaterial;
	this->verticesFreeShape->appearance = this->verticesFreeAppearance;
	
	this->verticesFreePointSet->coord = this->verticesFreeCoordinate;
	this->verticesFreeShape->geometry = this->verticesFreePointSet;
	
	this->verticesFree->addChild(this->verticesFreeShape);
	
	this->vertices->addChild(this->verticesFree);
	
	// vertices
	
	this->vertices->setName("vertices");
	this->vertices->whichChoice = SO_SWITCH_NONE;
	
	this->root->addChild(this->vertices);
	
	// work
	
	this->work->setName("work");
	this->work->whichChoice = SO_SWITCH_NONE;
	
	this->work->addChild(this->workDrawStyle);
	
	this->workTransform->addChild(this->frameIndexedLineSet);
	
	this->work->addChild(this->workTransform);
	
	this->root->addChild(this->work);
	
	// scene
	
	this->scene->setName("scene");
	this->scene->whichChoice = SO_SWITCH_ALL;
	
	this->scene->addChild(this->sceneDrawStyle);
	
	this->scene->addChild(this->sceneGroup);
	
	this->root->addChild(this->scene);
	
	// root
	
	this->root->setName("root");
	this->root->whichChoice = SO_SWITCH_ALL;
}

Viewer::~Viewer()
{
	this->spheresAppearance->unref();
	this->root->unref();
}

void
Viewer::changeEvent(QEvent* event)
{
	if (QEvent::PaletteChange == event->type())
	{
		if (this->palette().color(QPalette::Window).lightness() < 128)
		{
			this->backgroundGradientBackground->color0.setValue(0.0f, 0.0f, 0.0f);
			this->backgroundGradientBackground->color1.setValue(0.2f, 0.2f, 0.2f);
		}
		else
		{
			this->backgroundGradientBackground->color0.setValue(0.8f, 0.8f, 0.8f);
			this->backgroundGradientBackground->color1.setValue(1.0f, 1.0f, 1.0f);
		}
	}
	
	QWidget::changeEvent(event);
}

void
Viewer::drawConfiguration(const rl::math::Vector& q)
{
	this->model->setPosition(q);
	this->model->updateFrames();
}

void
Viewer::drawConfigurationEdge(const rl::math::Vector& u, const rl::math::Vector& v, const bool& free)
{
	SoVRMLCoordinate* coordinate = nullptr;
	SoVRMLIndexedLineSet* indexedLineSet = nullptr;
	
	if (free)
	{
		coordinate = this->edgesFreeCoordinate;
		indexedLineSet = this->edgesFreeIndexedLineSet;
	}
	else
	{
		coordinate = this->edgesCollidingCoordinate;
		indexedLineSet = this->edgesCollidingIndexedLineSet;
	}
	
	this->edges->enableNotify(false);
	
	rl::math::Vector inter(this->model->getDofPosition());
	
	rl::math::Real steps = std::ceil(this->model->distance(u, v) / this->delta);
	
	if (steps > 0)
	{
		for (std::size_t l = 0; l < this->model->getOperationalDof(); ++l)
		{
			for (std::size_t i = 0; i < steps + 1; ++i)
			{
				this->model->interpolate(u, v, i / steps, inter);
				
				this->model->setPosition(inter);
				this->model->updateFrames(false);
				
				coordinate->point.set1Value(
					coordinate->point.getNum(),
					this->model->forwardPosition(l)(0, 3),
					this->model->forwardPosition(l)(1, 3),
					this->model->forwardPosition(l)(2, 3)
				);
				
				indexedLineSet->coordIndex.set1Value(
					indexedLineSet->coordIndex.getNum(),
					coordinate->point.getNum() - 1
				);
			}
			
			indexedLineSet->coordIndex.set1Value(
				indexedLineSet->coordIndex.getNum(),
				SO_END_FACE_INDEX
			);
		}
	}
	
	this->edges->enableNotify(true);
	
	this->edges->touch();
}

void
Viewer::drawConfigurationPath(const rl::plan::VectorList& path)
{
	this->path->enableNotify(false);
	
	this->pathEdgesCoordinate->point.setNum(0);
	this->pathEdgesIndexedLineSet->coordIndex.setNum(0);
	
	this->pathVerticesCoordinate->point.setNum(0);
	
	rl::math::Vector inter(this->model->getDofPosition());
	
	for (std::size_t l = 0; l < this->model->getOperationalDof(); ++l)
	{
		rl::plan::VectorList::const_iterator i = path.begin();
		rl::plan::VectorList::const_iterator j = ++path.begin();
		
		if (i != path.end() && j != path.end())
		{
			this->model->setPosition(*i);
			this->model->updateFrames();
			
			this->pathEdgesCoordinate->point.set1Value(
				this->pathEdgesCoordinate->point.getNum(),
				this->model->forwardPosition(l)(0, 3),
				this->model->forwardPosition(l)(1, 3),
				this->model->forwardPosition(l)(2, 3)
			);
			
			this->pathEdgesIndexedLineSet->coordIndex.set1Value(
				this->pathEdgesIndexedLineSet->coordIndex.getNum(),
				this->pathEdgesCoordinate->point.getNum() - 1
			);
			
			this->pathVerticesCoordinate->point.set1Value(
				this->pathVerticesCoordinate->point.getNum(),
				this->model->forwardPosition(l)(0, 3),
				this->model->forwardPosition(l)(1, 3),
				this->model->forwardPosition(l)(2, 3)
			);
		}
		
		for (; i != path.end() && j != path.end(); ++i, ++j)
		{
			rl::math::Real steps = std::ceil(this->model->distance(*i, *j) / this->delta);
			
			for (std::size_t k = 1; k < steps + 1; ++k)
			{
				this->model->interpolate(*i, *j, k / steps, inter);
				
				this->model->setPosition(inter);
				this->model->updateFrames(false);
				
				this->pathEdgesCoordinate->point.set1Value(
					this->pathEdgesCoordinate->point.getNum(),
					this->model->forwardPosition(l)(0, 3),
					this->model->forwardPosition(l)(1, 3),
					this->model->forwardPosition(l)(2, 3)
				);
				
				this->pathEdgesIndexedLineSet->coordIndex.set1Value(
					this->pathEdgesIndexedLineSet->coordIndex.getNum(),
					this->pathEdgesCoordinate->point.getNum() - 1
				);
			}
			
			this->pathVerticesCoordinate->point.set1Value(
				this->pathVerticesCoordinate->point.getNum(),
				this->model->forwardPosition(l)(0, 3),
				this->model->forwardPosition(l)(1, 3),
				this->model->forwardPosition(l)(2, 3)
			);
		}
		
		this->pathEdgesIndexedLineSet->coordIndex.set1Value(
			this->pathEdgesIndexedLineSet->coordIndex.getNum(),
			SO_END_FACE_INDEX
		);
	}
	
	this->path->enableNotify(true);
	
	this->path->touch();
}

void
Viewer::drawConfigurationVertex(const rl::math::Vector& q, const bool& free)
{
	SoVRMLCoordinate* coordinate = nullptr;
	
	if (free)
	{
		coordinate = this->verticesFreeCoordinate;
	}
	else
	{
		coordinate = this->verticesCollidingCoordinate;
	}
	
	this->vertices->enableNotify(false);
	
	this->model->setPosition(q);
	this->model->updateFrames(false);
	
	for (std::size_t l = 0; l < this->model->getOperationalDof(); ++l)
	{
		coordinate->point.set1Value(
			coordinate->point.getNum(),
			this->model->forwardPosition(l)(0, 3),
			this->model->forwardPosition(l)(1, 3),
			this->model->forwardPosition(l)(2, 3)
		);
	}
	
	this->vertices->enableNotify(true);
	
	this->vertices->touch();
}

void
Viewer::drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1)
{
	this->linesCoordinate->point.set1Value(
		this->linesCoordinate->point.getNum(),
		xyz0(0),
		xyz0(1),
		xyz0(2)
	);
	
	this->linesIndexedLineSet->coordIndex.set1Value(
		this->linesIndexedLineSet->coordIndex.getNum(),
		this->linesCoordinate->point.getNum() - 1
	);
	
	this->linesCoordinate->point.set1Value(
		this->linesCoordinate->point.getNum(),
		xyz1(0),
		xyz1(1),
		xyz1(2)
	);
	
	this->linesIndexedLineSet->coordIndex.set1Value(
		this->linesIndexedLineSet->coordIndex.getNum(),
		this->linesCoordinate->point.getNum() - 1
	);
	
	this->linesIndexedLineSet->coordIndex.set1Value(
		this->linesIndexedLineSet->coordIndex.getNum(),
		SO_END_FACE_INDEX
	);
}

void
Viewer::drawPoint(const rl::math::Vector& xyz)
{
	this->pointsCoordinate->point.set1Value(
		this->pointsCoordinate->point.getNum(),
		xyz(0),
		xyz(1),
		xyz(2)
	);
}

void
Viewer::drawSphere(const rl::math::Vector& center, const rl::math::Real& radius)
{
	SoVRMLTransform* transform = new SoVRMLTransform();
	transform->translation.setValue(center(0), center(1), center(2));
	
	SoVRMLShape* shape = new SoVRMLShape();
	
	shape->appearance = this->spheresAppearance;
	
	SoVRMLSphere* sphere = new SoVRMLSphere();
	sphere->radius = radius;
	
	shape->geometry = sphere;
	
	transform->addChild(shape);
	
	this->spheresGroup->addChild(transform);
}

void
Viewer::drawSweptVolume(const rl::plan::VectorList& path)
{
	this->sweptGroup->enableNotify(false);
	
	this->sweptGroup->removeAllChildren();
	
	rl::math::Real length = 0;
	
	rl::plan::VectorList::const_iterator i = path.begin();
	rl::plan::VectorList::const_iterator j = ++path.begin();
	
	for (; i != path.end() && j != path.end(); ++i, ++j)
	{
		length += this->model->distance(*i, *j);
	}
	
	rl::math::Real delta = length / static_cast<std::size_t>(std::ceil(length / this->deltaSwept));
	
	rl::math::Vector inter(this->model->getDofPosition());
	
	rl::math::Real x0 = 0;
	rl::math::Real x1 = x0;
	rl::math::Real x = 0;
	
	i = path.begin();
	j = ++path.begin();
	
	for (; i != path.end() && j != path.end(); ++i, ++j)
	{
		x1 += this->model->distance(*i, *j);
		
		for (; x < x1; x += delta)
		{
			this->model->interpolate(*i, *j, (x - x0) / (x1 - x0), inter);
			
			this->model->setPosition(inter);
			this->model->updateFrames();
			
			SoVRMLGroup* model = new SoVRMLGroup();
			
			for (std::size_t k = 0; k < this->model->model->getNumBodies(); ++k)
			{
				SoVRMLTransform* frame = new SoVRMLTransform();
				frame->copyFieldValues(static_cast<rl::sg::so::Body*>(this->model->model->getBody(k))->root);
				
				for (std::size_t l = 0; l < this->model->model->getBody(k)->getNumShapes(); ++l)
				{
					SoVRMLTransform* transform = new SoVRMLTransform();
					transform->copyFieldValues(static_cast<rl::sg::so::Shape*>(this->model->model->getBody(k)->getShape(l))->root);
					transform->addChild(static_cast<rl::sg::so::Shape*>(this->model->model->getBody(k)->getShape(l))->shape);
					frame->addChild(transform);
				}
				
				model->addChild(frame);
			}
			
			this->sweptGroup->addChild(model);
		}
		
		x0 = x1;
		
		if (j == --path.end())
		{
			this->model->interpolate(*i, *j, 1, inter);
			
			this->model->setPosition(inter);
			this->model->updateFrames();
			
			SoVRMLGroup* model = new SoVRMLGroup();
			
			for (std::size_t k = 0; k < this->model->model->getNumBodies(); ++k)
			{
				SoVRMLTransform* frame = new SoVRMLTransform();
				frame->copyFieldValues(static_cast<rl::sg::so::Body*>(this->model->model->getBody(k))->root);
				
				for (std::size_t l = 0; l < this->model->model->getBody(k)->getNumShapes(); ++l)
				{
					SoVRMLTransform* transform = new SoVRMLTransform();
					transform->copyFieldValues(static_cast<rl::sg::so::Shape*>(this->model->model->getBody(k)->getShape(l))->root);
					transform->addChild(static_cast<rl::sg::so::Shape*>(this->model->model->getBody(k)->getShape(l))->shape);
					frame->addChild(transform);
				}
				
				model->addChild(frame);
			}
			
			this->sweptGroup->addChild(model);
		}
	}
	
	this->sweptGroup->enableNotify(true);
	
	this->sweptGroup->touch();
}

void
Viewer::drawWork(const rl::math::Transform& t)
{
	SbMatrix matrix;
	
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			matrix[i][j] = static_cast<float>(t(j, i));
		}
	}
	
	this->workTransform->setMatrix(matrix);
}

void
Viewer::drawWorkEdge(const rl::math::Vector& u, const rl::math::Vector& v)
{
	this->edges3Coordinate->point.set1Value(
		this->edges3Coordinate->point.getNum(),
		u(0),
		u(1),
		u(2)
	);
	
	this->edges3IndexedLineSet->coordIndex.set1Value(
		this->edges3IndexedLineSet->coordIndex.getNum(),
		this->edges3Coordinate->point.getNum() - 1
	);
	
	this->edges3Coordinate->point.set1Value(
		this->edges3Coordinate->point.getNum(),
		v(0),
		v(1),
		v(2)
	);
	
	this->edges3IndexedLineSet->coordIndex.set1Value(
		this->edges3IndexedLineSet->coordIndex.getNum(),
		this->edges3Coordinate->point.getNum() - 1
	);
	
	this->edges3IndexedLineSet->coordIndex.set1Value(
		this->edges3IndexedLineSet->coordIndex.getNum(),
		SO_END_FACE_INDEX
	);
}

void
Viewer::drawWorkPath(const rl::plan::VectorList& path)
{
	this->path3->enableNotify(false);
	
	this->path3Coordinate->point.setNum(0);
	this->path3IndexedLineSet->coordIndex.setNum(0);
	
	for (rl::plan::VectorList::const_iterator i = path.begin(); i != path.end(); ++i)
	{
		this->path3Coordinate->point.set1Value(
			this->path3Coordinate->point.getNum(),
			(*i)(0),
			(*i)(1),
			(*i)(2)
		);
		
		this->path3IndexedLineSet->coordIndex.set1Value(
			this->path3IndexedLineSet->coordIndex.getNum(),
			this->path3Coordinate->point.getNum() - 1
		);
	}
	
	this->path3IndexedLineSet->coordIndex.set1Value(
		this->path3IndexedLineSet->coordIndex.getNum(),
		SO_END_FACE_INDEX
	);
	
	this->path3->enableNotify(true);
	
	this->path3->touch();
}

void
Viewer::drawWorkVertex(const rl::math::Vector& q)
{
}

void
Viewer::reset()
{
	this->resetEdges();
	this->resetLines();
	this->resetPath();
	this->resetPath3();
	this->resetPoints();
	this->resetSpheres();
	this->resetVertices();
	this->sweptGroup->removeAllChildren();
	this->workTransform->setMatrix(SbMatrix::identity());
}

void
Viewer::resetEdges()
{
	this->edgesCollidingCoordinate->point.setNum(0);
	this->edgesCollidingIndexedLineSet->coordIndex.setNum(0);
	this->edgesFreeCoordinate->point.setNum(0);
	this->edgesFreeIndexedLineSet->coordIndex.setNum(0);
	this->edges3Coordinate->point.setNum(0);
	this->edges3IndexedLineSet->coordIndex.setNum(0);
}

void
Viewer::resetLines()
{
	this->linesCoordinate->point.setNum(0);
	this->linesIndexedLineSet->coordIndex.setNum(0);
}

void
Viewer::resetPath()
{
	this->pathEdgesCoordinate->point.setNum(0);
	this->pathEdgesIndexedLineSet->coordIndex.setNum(0);
	this->pathVerticesCoordinate->point.setNum(0);
}

void
Viewer::resetPath3()
{
	this->path3Coordinate->point.setNum(0);
	this->path3IndexedLineSet->coordIndex.setNum(0);
}

void
Viewer::resetPoints()
{
	this->pointsCoordinate->point.setNum(0);
}

void
Viewer::resetSpheres()
{
	this->spheresGroup->removeAllChildren();
}

void
Viewer::resetVertices()
{
	this->verticesCollidingCoordinate->point.setNum(0);
	this->verticesFreeCoordinate->point.setNum(0);
}

void
Viewer::saveImage(bool withAlpha)
{
	QString filename = "rlPlanDemo-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".png";
	
	SoSFInt32 whichChoice = this->background->whichChoice;
	
	if (withAlpha)
	{
		this->background->whichChoice = SO_SWITCH_NONE;
		this->viewer->render();
	}
	
	glReadBuffer(GL_FRONT);
	
#if QT_VERSION >= 0x060000
	QOpenGLWindow* window = this->viewer->getGLWidget()->property("SoQtGLArea").value<QOpenGLWindow*>();
	QSurfaceFormat surfaceFormat = window->format();
	
	if (withAlpha != surfaceFormat.hasAlpha())
	{
		surfaceFormat.setAlphaBufferSize(withAlpha ? 8 : 0);
		window->setFormat(surfaceFormat);
	}
	
	QImage image = window->grabFramebuffer();
#else
	QImage image = static_cast<QGLWidget*>(this->viewer->getGLWidget())->grabFrameBuffer(withAlpha);
#endif
	
	if (withAlpha)
	{
		this->background->whichChoice = whichChoice;
		this->viewer->render();
	}
	
	QString format = filename.right(filename.length() - filename.lastIndexOf('.') - 1).toUpper();
	
	if (("JFIF" == format) || ("JPE" == format) || ("JPG" == format))
	{
		format = "JPEG";
	}

	if (!image.save(filename, format.toStdString().c_str()))
	{
		QMessageBox::critical(this, "Error", "Error writing " + filename + ".");
	}
	
	MainWindow::instance()->statusBar()->showMessage("Successfully saved image in '" + filename + "'.", 1000);
}

void
Viewer::saveScene()
{
	QString filename = "rlPlanDemo-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".wrl";
	
	SoOutput output;
	
	if (!output.openFile(filename.toStdString().c_str()))
	{
		return;
	}
	
	output.setHeaderString("#VRML V2.0 utf8");
	
	SoWriteAction writeAction(&output);
	writeAction.apply(this->root);
	
	output.closeFile();
	
	MainWindow::instance()->statusBar()->showMessage("Successfully saved scene in '" + filename + "'.", 1000);
}

void
Viewer::setBackgroundColor(const QColor& color)
{
	this->background->whichChoice = SO_SWITCH_NONE;
	this->viewer->setBackgroundColor(SbColor(color.redF(), color.greenF(), color.blueF()));
}

void
Viewer::showMessage(const std::string& message)
{
	MainWindow::instance()->statusBar()->showMessage(QString::fromStdString(message));
}

void
Viewer::toggleConfigurationEdges(const bool& doOn)
{
	if (doOn)
	{
		this->edges->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->edges->whichChoice = SO_SWITCH_NONE;
	}
}

void
Viewer::toggleConfigurationVertices(const bool& doOn)
{
	if (doOn)
	{
		this->vertices->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->vertices->whichChoice = SO_SWITCH_NONE;
	}
}

void
Viewer::toggleLines(const bool& doOn)
{
	if (doOn)
	{
		this->lines->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->lines->whichChoice = SO_SWITCH_NONE;
	}
}

void
Viewer::togglePathEdges(const bool& doOn)
{
	if (doOn)
	{
		this->pathEdges->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->pathEdges->whichChoice = SO_SWITCH_NONE;
	}
}

void
Viewer::togglePathVertices(const bool& doOn)
{
	if (doOn)
	{
		this->pathVertices->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->pathVertices->whichChoice = SO_SWITCH_NONE;
	}
}

void
Viewer::togglePoints(const bool& doOn)
{
	if (doOn)
	{
		this->points->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->points->whichChoice = SO_SWITCH_NONE;
	}
}

void
Viewer::toggleSpheres(const bool& doOn)
{
	if (doOn)
	{
		this->spheres->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->spheres->whichChoice = SO_SWITCH_NONE;
	}
}

void
Viewer::toggleSweptVolume(const bool& doOn)
{
	if (doOn)
	{
		this->swept->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->swept->whichChoice = SO_SWITCH_NONE;
	}
}

void
Viewer::toggleWorkFrames(const bool& doOn)
{
	if (doOn)
	{
		this->work->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->work->whichChoice = SO_SWITCH_NONE;
	}
}
