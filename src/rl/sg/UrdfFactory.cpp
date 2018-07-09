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

#include <iostream> // TODO
#include <unordered_map>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/annex/ForeignFiles/SoSTLFileKit.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include <Inventor/VRMLnodes/SoVRMLCylinder.h>
#include <Inventor/VRMLnodes/SoVRMLGeometry.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLSphere.h>
#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
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
#include "SimpleScene.h"
#include "UrdfFactory.h"

namespace rl
{
	namespace sg
	{
		UrdfFactory::UrdfFactory()
		{
		}
		
		UrdfFactory::~UrdfFactory()
		{
		}
		
		void
		UrdfFactory::load(const ::std::string& filename, Scene* scene)
		{
			::rl::xml::DomParser parser;
			
			::rl::xml::Document document = parser.readFile(filename, "", XML_PARSE_NOENT);
			document.substitute(XML_PARSE_NOENT);
			
			::rl::xml::Path path(document);
			
			::rl::xml::NodeSet robots = path.eval("/robot").getValue< ::rl::xml::NodeSet>();
			
			if (robots.empty())
			{
				throw Exception("URDF is missing robot node");
			}
			
			for (int i = 0; i < robots.size(); ++i)
			{
				::rl::xml::Path path(document, robots[i]);
				
				Model* model = scene->create();
				
				// name
				
				model->setName(path.eval("string(@name)").getValue< ::std::string>());
::std::cout << model->getName() << ::std::endl;
				
				// materials
				
				::rl::xml::NodeSet materials = path.eval("material").getValue< ::rl::xml::NodeSet>();
				
				::std::unordered_map< ::std::string, ::SoVRMLMaterial*> name2material;
				
				for (int j = 0; j < materials.size(); ++j)
				{
::std::cout << "material: " << j << ::std::endl;
					::rl::xml::Path path(document, materials[j]);
					
					if (path.eval("count(color/@rgba) > 0").getValue<bool>())
					{
						::std::vector< ::std::string> rgba;
						::std::string tmp = path.eval("string(color/@rgba)").getValue< ::std::string>();
						::boost::split(rgba, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
::std::cout << "\trgba: " << rgba[0] << " " << rgba[1] << " " << rgba[2] << " " << rgba[3] << ::std::endl;
						
						::SoVRMLMaterial* vrmlMaterial = new ::SoVRMLMaterial();
						
						vrmlMaterial->diffuseColor.setValue(
							::boost::lexical_cast< ::rl::math::Real>(rgba[0]),
							::boost::lexical_cast< ::rl::math::Real>(rgba[1]),
							::boost::lexical_cast< ::rl::math::Real>(rgba[2])
						);
						vrmlMaterial->transparency.setValue(
							1.0f - ::boost::lexical_cast< ::rl::math::Real>(rgba[3])
						);
						
						name2material[path.eval("string(@name)").getValue< ::std::string>()] = vrmlMaterial;
::std::cout << "\tname: " << path.eval("string(@name)").getValue< ::std::string>() << ::std::endl;
					}
				}
				
				// links
				
				::rl::xml::NodeSet links = path.eval("link").getValue< ::rl::xml::NodeSet>();
				
				for (int j = 0; j < links.size(); ++j)
				{
::std::cout << "link: " << j << ::std::endl;
					::rl::xml::Path path(document, links[j]);
					
					Body* body = model->create();
					
					body->setName(path.eval("string(@name)").getValue< ::std::string>());
::std::cout << "\tname: " << body->getName() << ::std::endl;
					
					if (SimpleScene* simple = dynamic_cast<SimpleScene*>(scene))
					{
						if (path.eval("count(collision) > 0").getValue<bool>())
						{
							path.setNode(path.eval("collision").getValue< ::rl::xml::NodeSet>()[0]);
						}
						else if (path.eval("count(visual) > 0").getValue<bool>())
						{
							path.setNode(path.eval("visual").getValue< ::rl::xml::NodeSet>()[0]);
						}
						else
						{
							continue;
						}
					}
					else
					{
						if (path.eval("count(visual) > 0").getValue<bool>())
						{
							path.setNode(path.eval("visual").getValue< ::rl::xml::NodeSet>()[0]);
						}
						else if (path.eval("count(collision) > 0").getValue<bool>())
						{
							path.setNode(path.eval("collision").getValue< ::rl::xml::NodeSet>()[0]);
						}
						else
						{
							continue;
						}
					}
					
					::SoVRMLShape* vrmlShape = new ::SoVRMLShape();
					vrmlShape->ref();
					
					::SoVRMLAppearance* vrmlAppearance = new ::SoVRMLAppearance();
					vrmlShape->appearance = vrmlAppearance;
					
					if (path.eval("count(material/color/@rgba) > 0").getValue<bool>())
					{
						::std::vector< ::std::string> rgba;
						::std::string tmp = path.eval("string(material/color/@rgba)").getValue< ::std::string>();
						::boost::split(rgba, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
::std::cout << "\tmaterial rgba: " << rgba[0] << " " << rgba[1] << " " << rgba[2] << " " << rgba[3] << ::std::endl;
						
						::SoVRMLMaterial* vrmlMaterial = new ::SoVRMLMaterial();
						
						vrmlMaterial->diffuseColor.setValue(
							::boost::lexical_cast< ::rl::math::Real>(rgba[0]),
							::boost::lexical_cast< ::rl::math::Real>(rgba[1]),
							::boost::lexical_cast< ::rl::math::Real>(rgba[2])
						);
						vrmlMaterial->transparency.setValue(
							1.0f - ::boost::lexical_cast< ::rl::math::Real>(rgba[3])
						);
						
						vrmlAppearance->material = vrmlMaterial;
					}
					else if (path.eval("count(material/@name) > 0").getValue<bool>())
					{
::std::cout << "\tmaterial name: " << path.eval("string(material/@name)").getValue< ::std::string>() << ::std::endl;
						vrmlAppearance->material = name2material[path.eval("string(material/@name)").getValue< ::std::string>()];
					}
					else
					{
						::SoVRMLMaterial* vrmlMaterial = new ::SoVRMLMaterial();
						vrmlAppearance->material = vrmlMaterial;
					}
					
					::rl::xml::NodeSet shapes = path.eval("geometry/box|geometry/cylinder|geometry/mesh|geometry/sphere").getValue< ::rl::xml::NodeSet>();
					
					for (int k = 0; k < shapes.size(); ++k)
					{
						if ("box" == shapes[k].getName())
						{
							::SoVRMLBox* box = new ::SoVRMLBox();
							
							if (!shapes[k].getProperty("size").empty())
							{
								::std::vector< ::std::string> size;
								::std::string tmp = shapes[k].getProperty("size");
								::boost::split(size, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
								
								box->size.setValue(
									::boost::lexical_cast< ::rl::math::Real>(size[0]),
									::boost::lexical_cast< ::rl::math::Real>(size[1]),
									::boost::lexical_cast< ::rl::math::Real>(size[2])
								);
							}
							
							vrmlShape->geometry = box;
::std::cout << "\tbox: " << box->size.getValue()[0] << " " << box->size.getValue()[1] << " " << box->size.getValue()[2] << ::std::endl;
						}
						else if ("cylinder" == shapes[k].getName())
						{
							::SoVRMLCylinder* cylinder = new ::SoVRMLCylinder();
							
							if (!shapes[k].getProperty("length").empty())
							{
								cylinder->height.setValue(
									::boost::lexical_cast< ::rl::math::Real>(shapes[k].getProperty("length"))
								);
							}
							
							if (!shapes[k].getProperty("radius").empty())
							{
								cylinder->radius.setValue(
									::boost::lexical_cast< ::rl::math::Real>(shapes[k].getProperty("radius"))
								);
							}
							
							vrmlShape->geometry = cylinder;
::std::cout << "\tcylinder: " << cylinder->height.getValue() << " " << cylinder->radius.getValue() << ::std::endl;
						}
						else if ("mesh" == shapes[k].getName())
						{
							if (!shapes[k].getProperty("filename").empty())
							{
								::std::string filename = shapes[k].getLocalPath(shapes[k].getProperty("filename"));
::std::cout << "\tmesh: " << filename << ::std::endl;
								
								if (!boost::iequals("stl", filename.substr(filename.length() - 3, 3)))
								{
									throw Exception("Only STL meshes currently supported");
								}
								
								::SoSTLFileKit* stlFileKit = new ::SoSTLFileKit();
								stlFileKit->ref();
								stlFileKit->readFile(filename.c_str());
								::SoSeparator* stl = stlFileKit->convert();
								stl->ref();
								stlFileKit->unref();
								
								::SoToVRML2Action toVrml2Action;
								toVrml2Action.apply(stl);
								::SoVRMLGroup* vrml2 = toVrml2Action.getVRML2SceneGraph();
								vrml2->ref();
								stl->unref();
								
								::SoSearchAction searchAction;
								searchAction.setInterest(::SoSearchAction::ALL);
								searchAction.setType(::SoVRMLShape::getClassTypeId());
								searchAction.apply(vrml2);
								
								for (int l = 0; l < searchAction.getPaths().getLength(); ++l)
								{
									vrmlShape->geometry = static_cast< ::SoVRMLShape*>(static_cast< ::SoFullPath*>(searchAction.getPaths()[l])->getTail())->geometry;
									
									if (vrmlShape->geometry.getValue()->isOfType(::SoVRMLIndexedFaceSet::getClassTypeId()))
									{
										SoVRMLIndexedFaceSet* vrmlIndexedFaceSet = static_cast<SoVRMLIndexedFaceSet*>(vrmlShape->geometry.getValue());
										vrmlIndexedFaceSet->convex.setValue(false);
									}
								}
								
								vrml2->unref();
							}
						}
						else if ("sphere" == shapes[k].getName())
						{
							::SoVRMLSphere* sphere = new ::SoVRMLSphere();
							
							if (!shapes[k].getProperty("radius").empty())
							{
								sphere->radius.setValue(
									::boost::lexical_cast< ::rl::math::Real>(shapes[k].getProperty("radius"))
								);
							}
							
							vrmlShape->geometry = sphere;
::std::cout << "\tsphere: " << sphere->radius.getValue() << ::std::endl;
						}
					}
					
					if (nullptr != vrmlShape->geometry.getValue())
					{
						Shape* shape = body->create(vrmlShape);
						
						::rl::math::Transform origin = ::rl::math::Transform::Identity();
						
						if (path.eval("count(origin/@rpy) > 0").getValue<bool>())
						{
							::std::vector< ::std::string> rpy;
							::std::string tmp = path.eval("string(origin/@rpy)").getValue< ::std::string>();
							::boost::split(rpy, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
::std::cout << "\trpy: " << rpy[0] << " " << rpy[1] << " " << rpy[2] << ::std::endl;
							
							origin.linear() = ::rl::math::AngleAxis(
								::boost::lexical_cast< ::rl::math::Real>(rpy[2]),
								 ::rl::math::Vector3::UnitZ()
							) * ::rl::math::AngleAxis(
								::boost::lexical_cast< ::rl::math::Real>(rpy[1]),
								::rl::math::Vector3::UnitY()
							) * ::rl::math::AngleAxis(
								::boost::lexical_cast< ::rl::math::Real>(rpy[0]),
								::rl::math::Vector3::UnitX()
							).toRotationMatrix();
						}
						
						if (vrmlShape->geometry.getValue()->isOfType(::SoVRMLCylinder::getClassTypeId()))
						{
							origin *= ::rl::math::AngleAxis(90.0f * ::rl::math::DEG2RAD, ::rl::math::Vector3::UnitX());
						}
						
						if (path.eval("count(origin/@xyz) > 0").getValue<bool>())
						{
							::std::vector< ::std::string> xyz;
							::std::string tmp = path.eval("string(origin/@xyz)").getValue< ::std::string>();
							::boost::split(xyz, tmp, ::boost::algorithm::is_space(), ::boost::algorithm::token_compress_on);
::std::cout << "\txyz: " << xyz[0] << " " << xyz[1] << " " << xyz[2] << ::std::endl;
							
							origin.translation().x() = ::boost::lexical_cast< ::rl::math::Real>(xyz[0]);
							origin.translation().y() = ::boost::lexical_cast< ::rl::math::Real>(xyz[1]);
							origin.translation().z() = ::boost::lexical_cast< ::rl::math::Real>(xyz[2]);
						}
::std::cout << "\torigin:" << ::std::endl << origin.matrix() << ::std::endl;
						
						shape->setTransform(origin);
					}
					
					vrmlShape->unref();
				}
			}
		}
	}
}
