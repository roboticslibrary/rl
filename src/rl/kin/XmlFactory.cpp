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

#include <unordered_map>
#include <rl/math/Constants.h>
#include <rl/math/Rotation.h>
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>
#include <rl/xml/Stylesheet.h>

#include "Exception.h"
#include "Joint.h"
#include "Kinematics.h"
#include "Link.h"
#include "Prismatic.h"
#include "Puma.h"
#include "Revolute.h"
#include "Rhino.h"
#include "World.h"
#include "XmlFactory.h"

namespace rl
{
	namespace kin
	{
		XmlFactory::XmlFactory()
		{
		}
		
		XmlFactory::~XmlFactory()
		{
		}
		
		::std::shared_ptr<Kinematics>
		XmlFactory::create(const ::std::string& filename)
		{
			::std::shared_ptr<Kinematics> kinematics;
			
			::rl::xml::DomParser parser;
			
			::rl::xml::Document document = parser.readFile(filename, "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
			document.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
			
			if ("stylesheet" == document.getRootElement().getName() || "transform" == document.getRootElement().getName())
			{
				if ("1.0" == document.getRootElement().getProperty("version"))
				{
					if (document.getRootElement().hasNamespace() && "http://www.w3.org/1999/XSL/Transform" == document.getRootElement().getNamespace().getHref())
					{
						::rl::xml::Stylesheet stylesheet(document);
						document = stylesheet.apply();
					}
				}
			}
			
			::rl::xml::Path path(document);
			
			::rl::xml::NodeSet instances = path.eval("(/rl/kin|/rlkin)/kinematics|(/rl/kin|/rlkin)/puma|(/rl/kin|/rlkin)/rhino").getValue<::rl::xml::NodeSet>();
			
			if (instances.empty())
			{
				throw Exception("rl::kin::XmlFactory::create() - No models found in file " + filename);
			}
			
			for (int i = 0; i < ::std::min(1, instances.size()); ++i)
			{
				::rl::xml::Path path(document, instances[i]);
				
				if ("puma" == instances[i].getName())
				{
					kinematics = ::std::make_shared<Puma>();
				}
				else if ("rhino" == instances[i].getName())
				{
					kinematics = ::std::make_shared<Rhino>();
				}
				else if ("kinematics" == instances[i].getName())
				{
					kinematics = ::std::make_shared<Kinematics>();
				}
				else
				{
					continue;
				}
				
				// manufacturer
				
				kinematics->setManufacturer(path.eval("string(manufacturer)").getValue<::std::string>());
				
				// name
				
				kinematics->setName(path.eval("string(name)").getValue<::std::string>());
				
				// frames
				
				::rl::xml::NodeSet frames = path.eval("frame|link|world").getValue<::rl::xml::NodeSet>();
				
				::std::unordered_map<::std::string, Frame*> id2frame;
				
				for (int j = 0; j < frames.size(); ++j)
				{
					::rl::xml::Path path(document, frames[j]);
					
					Frame* frame = nullptr;
					
					if ("frame" == frames[j].getName())
					{
						::std::shared_ptr<Frame> f = ::std::make_shared<Frame>();
						
						kinematics->add(f);
						
						f->frame.linear() = ::rl::math::AngleAxis(
							path.eval("number(rotation/z)").getValue<::rl::math::Real>(0) * ::rl::math::constants::deg2rad,
							::rl::math::Vector3::UnitZ()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/y)").getValue<::rl::math::Real>(0) * ::rl::math::constants::deg2rad,
							::rl::math::Vector3::UnitY()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/x)").getValue<::rl::math::Real>(0) * ::rl::math::constants::deg2rad,
							::rl::math::Vector3::UnitX()
						).toRotationMatrix();
						
						f->frame.translation().x() = path.eval("number(translation/x)").getValue<::rl::math::Real>(0);
						f->frame.translation().y() = path.eval("number(translation/y)").getValue<::rl::math::Real>(0);
						f->frame.translation().z() = path.eval("number(translation/z)").getValue<::rl::math::Real>(0);
						
						frame = f.get();
					}
					else if ("link" == frames[j].getName())
					{
						::std::shared_ptr<Link> l = ::std::make_shared<Link>();
						
						kinematics->add(l);
						
						frame = l.get();
					}
					else if ("world" == frames[j].getName())
					{
						::std::shared_ptr<World> w = ::std::make_shared<World>();
						
						kinematics->add(w);
						
						w->frame.linear() = ::rl::math::AngleAxis(
							path.eval("number(rotation/z)").getValue<::rl::math::Real>(0) * ::rl::math::constants::deg2rad,
							::rl::math::Vector3::UnitZ()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/y)").getValue<::rl::math::Real>(0) * ::rl::math::constants::deg2rad,
							::rl::math::Vector3::UnitY()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/x)").getValue<::rl::math::Real>(0) * ::rl::math::constants::deg2rad,
							::rl::math::Vector3::UnitX()
						).toRotationMatrix();
						
						w->frame.translation().x() = path.eval("number(translation/x)").getValue<::rl::math::Real>(0);
						w->frame.translation().y() = path.eval("number(translation/y)").getValue<::rl::math::Real>(0);
						w->frame.translation().z() = path.eval("number(translation/z)").getValue<::rl::math::Real>(0);
						
						frame = w.get();
					}
					
					if (nullptr != frame)
					{
						frame->name = path.eval("string(@id)").getValue<::std::string>();
						
						if (id2frame.find(frame->name) != id2frame.end())
						{
							throw Exception("rl::kin::XmlFactory::create() - Frame with ID '" + frame->name + "' not unique in file '" + filename + "'");
						}
						
						id2frame[frame->name] = frame;
					}
				}
				
				for (int j = 0; j < frames.size(); ++j)
				{
					::rl::xml::Path path(document, frames[j]);
					
					if ("link" == frames[j].getName())
					{
						::std::string f1Id = path.eval("string(@id)").getValue<::std::string>();
						
						if (id2frame.find(f1Id) == id2frame.end())
						{
							throw Exception("rl::kin::XmlFactory::create() - Link with ID '" + f1Id + "' not found in file '" + filename + "'");
						}
						
						Link* l1 = dynamic_cast<Link*>(id2frame[f1Id]);
						
						::rl::xml::NodeSet ignores = path.eval("ignore").getValue<::rl::xml::NodeSet>();
						
						for (int k = 0; k < ignores.size(); ++k)
						{
							if (!ignores[k].getProperty("idref").empty())
							{
								::std::string f2IdRef = ignores[k].getProperty("idref");
								
								if (id2frame.find(f2IdRef) == id2frame.end())
								{
									throw Exception("rl::kin::XmlFactory::create() - Link with IDREF '" + f2IdRef + "' in Link with ID '" + f1Id + "' not found in file '" + filename + "'");
								}
								
								Link* l2 = dynamic_cast<Link*>(id2frame[f2IdRef]);
								
								l1->selfcollision.insert(l2);
								l2->selfcollision.insert(l1);
							}
							else
							{
								l1->collision = false;
							}
						}
					}
				}
				
				// transforms
				
				::rl::xml::NodeSet transforms = path.eval("prismatic|revolute|transform").getValue<::rl::xml::NodeSet>();
				
				for (int j = 0; j < transforms.size(); ++j)
				{
					::rl::xml::Path path(document, transforms[j]);
					
					::std::string name = path.eval("string(@id)").getValue<::std::string>();
					
					::std::string aIdRef = path.eval("string(frame/a/@idref)").getValue<::std::string>();
					
					if (id2frame.find(aIdRef) == id2frame.end())
					{
						throw Exception("rl::kin::XmlFactory::create() - Frame A with IDREF '" + aIdRef + "' in Transform with ID '" + name + "' not found in file '" + filename + "'");
					}
					
					Frame* a = id2frame[aIdRef];
					
					::std::string bIdRef = path.eval("string(frame/b/@idref)").getValue<::std::string>();
					
					if (id2frame.find(bIdRef) == id2frame.end())
					{
						throw Exception("rl::kin::XmlFactory::create() - Frame B with IDREF '" + bIdRef + "' in Transform with ID '" + name + "' not found in file '" + filename + "'");
					}
					
					Frame* b = id2frame[bIdRef];
					
					Transform* transform = nullptr;
					
					if ("prismatic" == transforms[j].getName())
					{
						::std::shared_ptr<Prismatic> p = ::std::make_shared<Prismatic>();
						
						kinematics->add(p, a, b);
						
						p->a = path.eval("number(dh/a)").getValue<::rl::math::Real>(0);
						p->alpha = path.eval("number(dh/alpha)").getValue<::rl::math::Real>(0);
						p->d = path.eval("number(dh/d)").getValue<::rl::math::Real>(0);
						p->max = path.eval("number(max)").getValue<::rl::math::Real>(::std::numeric_limits<::rl::math::Real>::max());
						p->min = path.eval("number(min)").getValue<::rl::math::Real>(-::std::numeric_limits<::rl::math::Real>::max());
						p->offset = path.eval("number(offset)").getValue<::rl::math::Real>(0);
						p->speed = path.eval("number(speed)").getValue<::rl::math::Real>(0);
						p->theta = path.eval("number(dh/theta)").getValue<::rl::math::Real>(0);
						p->wraparound = path.eval("count(wraparound) > 0").getValue<bool>();
						
						p->alpha *= ::rl::math::constants::deg2rad;
						p->theta *= ::rl::math::constants::deg2rad;
						
						transform = p.get();
					}
					else if ("revolute" == transforms[j].getName())
					{
						::std::shared_ptr<Revolute> r = ::std::make_shared<Revolute>();
						
						kinematics->add(r, a, b);
						
						r->a = path.eval("number(dh/a)").getValue<::rl::math::Real>(0);
						r->alpha = path.eval("number(dh/alpha)").getValue<::rl::math::Real>(0);
						r->d = path.eval("number(dh/d)").getValue<::rl::math::Real>(0);
						r->max = path.eval("number(max)").getValue<::rl::math::Real>(::std::numeric_limits<::rl::math::Real>::max());
						r->min = path.eval("number(min)").getValue<::rl::math::Real>(-::std::numeric_limits<::rl::math::Real>::max());
						r->offset = path.eval("number(offset)").getValue<::rl::math::Real>(0);
						r->speed = path.eval("number(speed)").getValue<::rl::math::Real>(0);
						r->theta = path.eval("number(dh/theta)").getValue<::rl::math::Real>(0);
						r->wraparound = path.eval("count(wraparound) > 0").getValue<bool>();
						
						r->alpha *= ::rl::math::constants::deg2rad;
						r->max *= ::rl::math::constants::deg2rad;
						r->min *= ::rl::math::constants::deg2rad;
						r->offset *= ::rl::math::constants::deg2rad;
						r->speed *= ::rl::math::constants::deg2rad;
						r->theta *= ::rl::math::constants::deg2rad;
						
						transform = r.get();
					}
					else if ("transform" == transforms[j].getName())
					{
						::std::shared_ptr<Transform> t = ::std::make_shared<Transform>();
						
						kinematics->add(t, a, b);
						
						t->transform.setIdentity();
						
						t->transform = ::rl::math::AngleAxis(
							path.eval("number(rotation/z)").getValue<::rl::math::Real>(0) * ::rl::math::constants::deg2rad,
							::rl::math::Vector3::UnitZ()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/y)").getValue<::rl::math::Real>(0) * ::rl::math::constants::deg2rad,
							::rl::math::Vector3::UnitY()
						) * ::rl::math::AngleAxis(
							path.eval("number(rotation/x)").getValue<::rl::math::Real>(0) * ::rl::math::constants::deg2rad,
							::rl::math::Vector3::UnitX()
						);
						
						t->transform.translation().x() = path.eval("number(translation/x)").getValue<::rl::math::Real>(0);
						t->transform.translation().y() = path.eval("number(translation/y)").getValue<::rl::math::Real>(0);
						t->transform.translation().z() = path.eval("number(translation/z)").getValue<::rl::math::Real>(0);
						
						transform = t.get();
					}
					
					if (nullptr != transform)
					{
						transform->name = name;
					}
				}
				
				kinematics->update();
			}
			
			return kinematics;
		}
	}
}
