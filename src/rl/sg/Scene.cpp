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

#include <algorithm>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include <Inventor/VRMLnodes/SoVRMLCone.h>
#include <Inventor/VRMLnodes/SoVRMLCylinder.h>
#include <Inventor/VRMLnodes/SoVRMLGeometry.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLSphere.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>

#include "Body.h"
#include "Exception.h"
#include "Model.h"
#include "Scene.h"
#include "Shape.h"

namespace rl
{
	namespace sg
	{
		Scene::Scene() :
			isScalingSupported(true),
			models(),
			name()
		{
		}
		
		Scene::~Scene()
		{
		}
		
		void
		Scene::add(Model* model)
		{
			this->models.push_back(model);
		}
		
		Scene::Iterator
		Scene::begin()
		{
			return this->models.begin();
		}
		
		Scene::Iterator
		Scene::end()
		{
			return this->models.end();
		}
		
		Model*
		Scene::getModel(const ::std::size_t& i) const
		{
			return this->models[i];
		}
		
		::std::string
		Scene::getName() const
		{
			return this->name;
		}
		
		::std::size_t
		Scene::getNumModels() const
		{
			return this->models.size();
		}
		
		void
		Scene::load(const ::std::string& filename, const bool& doBoundingBoxPoints, const bool& doPoints)
		{
			::rl::xml::DomParser parser;
			
			::rl::xml::Document document = parser.readFile(filename, "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
			document.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
			
			::rl::xml::Path path(document);
			
			::rl::xml::NodeSet scenes = path.eval("(/rl/sg|/rlsg)/scene").getValue< ::rl::xml::NodeSet>();
			
			for (int i = 0; i < ::std::min(1, scenes.size()); ++i)
			{
				::SoInput input;
				
				if (!input.openFile(scenes[i].getLocalPath(scenes[i].getProperty("href")).c_str() ,true))
				{
					throw Exception("::rl::sg::Scene::load() - failed to open file");
				}
				
				::SoVRMLGroup* root = SoDB::readAllVRML(&input);
				
				if (nullptr == root)
				{
					throw Exception("::rl::sg::Scene::load() - failed to read file");
				}
				
				::SbViewportRegion viewportRegion;
				
				root->ref();
				
				// model
				
				::rl::xml::NodeSet models = ::rl::xml::Path(document, scenes[i]).eval("model").getValue< ::rl::xml::NodeSet>();
				
				for (int j = 0; j < models.size(); ++j)
				{
					::rl::xml::Path path(document, models[j]);
					
					::SoSearchAction modelSearchAction;
					modelSearchAction.setName(models[j].getProperty("name").c_str());
					modelSearchAction.apply(root);
					
					if (nullptr == modelSearchAction.getPath())
					{
						continue;
					}
					
					Model* model = this->create();
					
					model->setName(models[j].getProperty("name"));
					
					// body
					
					::rl::xml::NodeSet bodies = path.eval("body").getValue< ::rl::xml::NodeSet>();
					
					for (int k = 0; k < bodies.size(); ++k)
					{
						::SoSearchAction bodySearchAction;
						bodySearchAction.setName(bodies[k].getProperty("name").c_str());
						bodySearchAction.apply(static_cast< ::SoFullPath*>(modelSearchAction.getPath())->getTail());
						
						if (nullptr == bodySearchAction.getPath())
						{
							continue;
						}
						
						Body* body = model->create();
						
						body->setName(bodies[k].getProperty("name"));
						
						::SoSearchAction pathSearchAction;
						pathSearchAction.setNode(static_cast< ::SoFullPath*>(bodySearchAction.getPath())->getTail());
						pathSearchAction.apply(root);
						
						::SoGetMatrixAction bodyGetMatrixAction(viewportRegion);
						bodyGetMatrixAction.apply(static_cast< ::SoFullPath*>(pathSearchAction.getPath()));
						::SbMatrix bodyMatrix = bodyGetMatrixAction.getMatrix();
						
						if (!this->isScalingSupported)
						{
							::SbVec3f bodyTranslation;
							::SbRotation bodyRotation;
							::SbVec3f bodyScaleFactor;
							::SbRotation bodyScaleOrientation;
							::SbVec3f bodyCenter;
							bodyMatrix.getTransform(bodyTranslation, bodyRotation, bodyScaleFactor, bodyScaleOrientation, bodyCenter);
							
							for (int l = 0; l < 3; ++l)
							{
								if (::std::abs(bodyScaleFactor[l] - 1.0f) > 1.0e-6f)
								{
									throw Exception("::rl::sg::Scene::load() - bodyScaleFactor not supported");
								}
							}
						}
						
						::rl::math::Transform frame;
						
						for (int m = 0; m < 4; ++m)
						{
							for (int n = 0; n < 4; ++n)
							{
								frame(m, n) = bodyMatrix[n][m];
							}
						}
						
						body->setFrame(frame);
						
						if (static_cast< ::SoFullPath*>(bodySearchAction.getPath())->getTail()->isOfType(::SoVRMLTransform::getClassTypeId()))
						{
							::SoVRMLTransform* bodyVrmlTransform = static_cast< ::SoVRMLTransform*>(static_cast< ::SoFullPath*>(bodySearchAction.getPath())->getTail());
							
							for (int l = 0; l < 3; ++l)
							{
								body->center(l) = bodyVrmlTransform->center.getValue()[l];
							}
						}
						
						::SoPathList pathList;
						
						// shape
						
						::SoSearchAction shapeSearchAction;
						shapeSearchAction.setInterest(::SoSearchAction::ALL);
						shapeSearchAction.setType(::SoVRMLShape::getClassTypeId());
						shapeSearchAction.apply(static_cast< ::SoFullPath*>(bodySearchAction.getPath())->getTail());
						
						for (int l = 0; l < shapeSearchAction.getPaths().getLength(); ++l)
						{
							::SoFullPath* path = static_cast< ::SoFullPath*>(shapeSearchAction.getPaths()[l]);
							
							if (path->getLength() > 1)
							{
								path = static_cast< ::SoFullPath*>(shapeSearchAction.getPaths()[l]->copy(1, static_cast< ::SoFullPath*>(shapeSearchAction.getPaths()[l])->getLength() - 1));
							}
							
							pathList.append(path);
							
							::SoGetMatrixAction shapeGetMatrixAction(viewportRegion);
							shapeGetMatrixAction.apply(path);
							::SbMatrix shapeMatrix = shapeGetMatrixAction.getMatrix();
							
							if (!this->isScalingSupported)
							{
								::SbVec3f shapeTranslation;
								::SbRotation shapeRotation;
								::SbVec3f shapeScaleFactor;
								::SbRotation shapeScaleOrientation;
								::SbVec3f shapeCenter;
								shapeMatrix.getTransform(shapeTranslation, shapeRotation, shapeScaleFactor, shapeScaleOrientation, shapeCenter);
								
								for (int m = 0; m < 3; ++m)
								{
									if (::std::abs(shapeScaleFactor[m] - 1.0f) > 1.0e-6f)
									{
										throw Exception("::rl::sg::Scene::load() - shapeScaleFactor not supported");
									}
								}
							}
							
							::SoVRMLShape* shapeVrmlShape = static_cast< ::SoVRMLShape*>(static_cast< ::SoFullPath*>(shapeSearchAction.getPaths()[l])->getTail());
							
							Shape* shape = body->create(shapeVrmlShape);
							
							shape->setName(shapeVrmlShape->getName().getString());
							
							::rl::math::Transform transform;
							
							for (int m = 0; m < 4; ++m)
							{
								for (int n = 0; n < 4; ++n)
								{
									transform(m, n) = shapeMatrix[n][m];
								}
							}
							
							shape->setTransform(transform);
						}
						
						// bounding box
						
						if (doBoundingBoxPoints)
						{
							::SoGetBoundingBoxAction getBoundingBoxAction(viewportRegion);
							getBoundingBoxAction.apply(pathList);
							::SbBox3f boundingBox = getBoundingBoxAction.getBoundingBox();
							
							for (int l = 0; l < 3; ++l)
							{
								body->max(l) = boundingBox.getMax()[l];
								body->min(l) = boundingBox.getMin()[l];
							}
						}
						
						// convex hull
						
						if (doPoints)
						{
							::SoCallbackAction callbackAction;
							callbackAction.addTriangleCallback(::SoVRMLGeometry::getClassTypeId(), Scene::triangleCallback, &body->points);
							callbackAction.apply(pathList);
						}
					}
				}
				
				root->unref();
			}
		}
		
		void
		Scene::remove(Model* model)
		{
			Iterator found = ::std::find(this->models.begin(), this->models.end(), model);
			
			if (found != this->models.end())
			{
				this->models.erase(found);
			}
		}
		
		void
		Scene::setName(const ::std::string& name)
		{
			this->name = name;
		}
		
		void
		Scene::triangleCallback(void* userData, SoCallbackAction* action, const SoPrimitiveVertex* v1, const SoPrimitiveVertex* v2, const SoPrimitiveVertex* v3)
		{
			::std::vector< ::rl::math::Vector3>* points = static_cast< ::std::vector< ::rl::math::Vector3>*>(userData);
			
			::rl::math::Vector3 p1;
			p1(0) = v1->getPoint()[0];
			p1(1) = v1->getPoint()[1];
			p1(2) = v1->getPoint()[2];
			
			points->push_back(p1);
			
			::rl::math::Vector3 p2;
			p2(0) = v2->getPoint()[0];
			p2(1) = v2->getPoint()[1];
			p2(2) = v2->getPoint()[2];
			
			points->push_back(p2);
			
			::rl::math::Vector3 p3;
			p3(0) = v3->getPoint()[0];
			p3(1) = v3->getPoint()[1];
			p3(2) = v3->getPoint()[2];
			
			points->push_back(p3);
		}
	}
}
